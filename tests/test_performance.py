"""Regression checks for the fast paths, timing boundaries, and disposable cache."""
from concurrent.futures import ThreadPoolExecutor
from functools import partial
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import mcap.reader
from mcap.writer import IndexType, Writer as McapWriter
from mcap_ros2.writer import Writer

from autoware_diag import incident, quick
from autoware_diag.cache import Cache, source_identity
from autoware_diag.errors import AnalysisError
from autoware_diag.events import detect_aeb_events, detect_mrm_events
from test_cli import AEB_DEFINITION, BASE_NS, BASE_SECONDS, CLI, LOG_DEFINITION, write_fixture

SHADOW = '/pix/AebInfoShadow'
EMERGENCY = '/control/command/emergency_cmd'


def unwrap(result):
    return json.loads(result['content'][0]['text'])


def aeb_data(on, fcw=False):
    return dict(aeb_status=1, fcw_status=1, aeb_req_filter=int(on), fcw_req_filter=int(fcw),
                aeb_req_no_filter=int(on), fcw_req_no_filter=int(fcw))


def make_window_fixture(path, *, indexed):
    if indexed:
        options = {}
    else:
        options = dict(index_types=IndexType.NONE, repeat_channels=False, repeat_schemas=False,
                       use_statistics=False, use_summary_offsets=False)
    with path.open('wb') as stream, patch('mcap_ros2.writer.McapWriter', partial(McapWriter, **options)):
        w = Writer(stream, chunk_size=1)
        shadow = w.register_msgdef('aeb_interfaces/msg/AebInfoShadow', AEB_DEFINITION)
        emergency = w.register_msgdef('fixture/msg/Emergency', 'bool emergency\n')
        unrelated = w.register_msgdef('fixture/msg/Other', 'string data\n')
        w.write_message('/unmonitored', unrelated, {'data': 'lead'}, BASE_NS, BASE_NS + 15)
        # Deliberately physically out of order. publish_time is inside the query
        # for out-of-window signals and outside it for in-window signals.
        for delta, on, fcw in ((9, False, True), (10, True, False), (20, True, False),
                               (15, False, False), (21, True, True)):
            w.write_message(SHADOW, shadow, aeb_data(on, fcw), BASE_NS + delta,
                            BASE_NS + (15 if delta in (9, 21) else 50))
        w.write_message(EMERGENCY, emergency, {'emergency': True}, BASE_NS + 21, BASE_NS + 15)
        w.write_message('/unmonitored', unrelated, {'data': 'tail'}, BASE_NS + 30, BASE_NS + 15)
        w.finish()


class PerformanceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.cache_dir = self.root / 'cache'
        env = patch.dict(os.environ, {'AUTOWARE_DIAG_CACHE_DIR': str(self.cache_dir)})
        env.start()
        self.addCleanup(env.stop)
        self.bag = self.root / 'fixture.mcap'
        write_fixture(self.bag)
        self.args = {'file_path': str(self.bag)}

    def invoke(self, *args):
        result = subprocess.run([sys.executable, '-B', str(CLI), *map(str, args)],
                                capture_output=True, timeout=20, check=True)
        return json.loads(result.stdout), len(result.stdout)

    def test_inspect_only_summary_and_cache_hit(self):
        with patch.object(mcap.reader.SeekingReader, 'iter_messages', side_effect=AssertionError('scanned')):
            cold = quick.inspect_bag(self.args)
            with patch.object(quick, '_open_mcap', side_effect=AssertionError('opened on hit')):
                hit = quick.inspect_bag(self.args)
        self.assertEqual(cold['cache']['status'], 'miss')
        self.assertEqual(hit['cache']['status'], 'hit')
        self.assertEqual(cold['basic_info']['total_messages'], 18)
        self.assertEqual(cold['basic_info']['unique_topics'], 6)
        self.assertEqual(cold['basic_info']['time_range']['start_ns'], BASE_NS)
        self.assertTrue(cold['schemas'])

    def test_inspect_without_summary_never_scans(self):
        make_window_fixture(self.bag, indexed=False)
        with patch.object(mcap.reader.SeekingReader, 'iter_messages', side_effect=AssertionError('scanned')):
            result = quick.inspect_bag({**self.args, 'no_cache': True})
        self.assertFalse(result['index_available'])
        self.assertFalse(result['summary_available'])
        self.assertIsNone(result['basic_info']['time_range'])
        self.assertIn('no message scan', result['note'])
        self.assertFalse(self.cache_dir.exists())

    def test_triage_iterates_once_and_hit_does_not_scan(self):
        original = mcap.reader.SeekingReader.iter_messages
        with patch.object(mcap.reader.SeekingReader, 'iter_messages', autospec=True, side_effect=original) as iterate:
            first = quick.triage_bag(self.args)
            self.assertEqual(iterate.call_count, 1)
            hit = quick.triage_bag({**self.args, 'details': True})
            self.assertEqual(iterate.call_count, 1)
        self.assertEqual(hit['cache']['status'], 'hit')
        self.assertEqual(first['scope']['message_iterations'], 1)
        self.assertEqual(hit['scope']['message_iterations'], 0)
        self.assertEqual(first['scope']['messages_visited'], hit['scope']['evidence_messages_visited'])
        self.assertEqual(first['basic_info']['total_messages'], 18)
        self.assertEqual(first['logs']['total'], 2)
        self.assertEqual(first['diagnostics']['non_ok_status_count'], 2)
        self.assertEqual(first['aeb']['total'], unwrap(detect_aeb_events(self.args))['total_aeb_events'])
        self.assertEqual(first['mrm']['total'], unwrap(detect_mrm_events(self.args))['total_mrm_events'])

    def test_counts_survive_representative_limits_and_large_payload_is_not_decoded(self):
        with self.bag.open('wb') as stream:
            w = Writer(stream, chunk_size=512)
            aeb = w.register_msgdef('fixture/msg/Aeb', AEB_DEFINITION)
            emg = w.register_msgdef('fixture/msg/Emergency', 'bool emergency\n')
            cloud = w.register_msgdef('fixture/msg/Cloud', 'string data\n')
            rosout = w.register_msgdef('rcl_interfaces/msg/Log', LOG_DEFINITION)
            for i in range(130):
                w.write_message(SHADOW, aeb, aeb_data(bool(i % 2)), BASE_NS+i)
                w.write_message(EMERGENCY, emg, {'emergency': bool(i % 2)}, BASE_NS+i)
                w.write_message('/rosout', rosout, {'stamp': {'sec': BASE_SECONDS, 'nanosec': i},
                                'level': 40, 'name': '车辆' * 200, 'msg': 'DO_NOT_PERSIST_RAW_LOG',
                                'file': '', 'function': '', 'line': 0}, BASE_NS+i)
            w.write_message('/sensing/lidar/top/pointcloud_raw', cloud, {'data': 'x' * 10000}, BASE_NS)
            w.finish()
        from autoware_diag.mcap_support import Ros2DecoderFactory
        original = Ros2DecoderFactory.decoder_for
        with patch.object(Ros2DecoderFactory, 'decoder_for', autospec=True, side_effect=original) as decode:
            compact = quick.triage_bag(self.args)
            self.assertNotIn('fixture/msg/Cloud', [c.args[2].name for c in decode.call_args_list])
        result, nbytes = self.invoke('triage', self.bag)
        detailed, _ = self.invoke('triage', self.bag, '--details')
        self.assertEqual(result['cache']['status'], 'hit')
        self.assertLessEqual(nbytes, 16384)
        for section in ('logs', 'aeb', 'mrm'):
            self.assertGreater(compact[section]['total'], 100)
            self.assertLessEqual(len(result[section]['examples']), 10)
            self.assertEqual(len(detailed[section]['examples']), 100)
            self.assertEqual(compact[section]['total'], result[section]['omitted'] + len(result[section]['examples']))
        self.assertEqual(result['aeb']['total'], unwrap(detect_aeb_events(self.args))['total_aeb_events'])
        self.assertEqual(result['mrm']['total'], unwrap(detect_mrm_events(self.args))['total_mrm_events'])
        self.assertEqual(result['basic_info']['total_messages'], 391)
        self.assertNotIn('DO_NOT_PERSIST_RAW_LOG', next(self.cache_dir.glob('bag-*.json')).read_text())

    def assert_window(self, result, mode):
        self.assertEqual(result['scope']['mode'], mode)
        self.assertEqual(result['scope']['message_iterations'], 1)
        self.assertEqual(result['scope']['file_time_range']['start_ns'], BASE_NS)
        self.assertEqual(result['scope']['file_time_range']['end_ns'], BASE_NS + 30)
        self.assertEqual(result['time_range']['start_ns'], BASE_NS + 10)
        self.assertEqual(result['time_range']['end_ns'], BASE_NS + 20)
        self.assertEqual(result['topic_presence'][SHADOW]['messages_in_window'], 3)
        self.assertEqual(result['topic_presence'][EMERGENCY]['messages_in_window'], 0)
        events = [a for a in result['alerts'] if a['type'] == 'aeb_request']
        self.assertEqual([e['log_time_ns'] for e in events], [BASE_NS + 10, BASE_NS + 20])
        self.assertTrue(events[0]['already_active_at_window_start'])
        self.assertFalse(events[1]['already_active_at_window_start'])
        self.assertFalse(any(a['type'] == 'fcw_request' for a in result['alerts']))

    def test_indexed_incident_pushes_exact_closed_log_time_bounds(self):
        make_window_fixture(self.bag, indexed=True)
        original = mcap.reader.SeekingReader.iter_messages
        with patch.object(mcap.reader.SeekingReader, 'iter_messages', autospec=True, side_effect=original) as iterate, \
                patch.object(mcap.reader, 'breakup_chunk', wraps=mcap.reader.breakup_chunk) as chunks:
            result = unwrap(incident.investigate_incident({**self.args,
                'start_iso': '2027-01-15T08:00:00.000000010Z',
                'end_iso': '2027-01-15T08:00:00.000000020Z'}))
        self.assertEqual(iterate.call_count, 1)
        self.assertEqual(iterate.call_args.kwargs['start_time'], BASE_NS + 10)
        self.assertEqual(iterate.call_args.kwargs['end_time'], BASE_NS + 21)
        self.assertEqual(chunks.call_count, 3)  # three in-range chunks out of eight
        self.assert_window(result, 'indexed_window')

    def test_unindexed_incident_and_triage_single_streaming_iteration(self):
        make_window_fixture(self.bag, indexed=False)
        original = mcap.reader.NonSeekingReader.iter_messages
        with patch.object(mcap.reader.NonSeekingReader, 'iter_messages', autospec=True, side_effect=original) as iterate:
            result = unwrap(incident.investigate_incident({**self.args,
                'start': f'{BASE_SECONDS}.000000010', 'end': f'{BASE_SECONDS}.000000020'}))
            self.assertEqual(iterate.call_count, 1)
            self.assertFalse(iterate.call_args.kwargs['log_time_order'])
            triage = quick.triage_bag(self.args)
            self.assertEqual(iterate.call_count, 2)
            self.assertFalse(iterate.call_args.kwargs['log_time_order'])
        self.assert_window(result, 'linear_fallback')
        self.assertEqual(result['scope']['messages_visited'], 8)
        self.assertEqual(triage['basic_info']['total_messages'], 8)
        self.assertEqual(triage['scope']['mode'], 'linear_fallback')
        self.assertEqual(triage['aeb']['by_event']['AEB_ACTIVATED'], 2)
        self.assertEqual(triage['mrm']['by_event']['EMERGENCY_CMD_ON'], 1)
        self.assertEqual(next(r for r in triage['health']['topic_statistics'] if r['topic'] == SHADOW)['max_gap_ns'], 5)

    def test_cli_point_precision_and_cache_management(self):
        make_window_fixture(self.bag, indexed=True)
        result, _ = self.invoke('incident', self.bag, '--timestamp', f'{BASE_SECONDS}.000000015',
                                '--window-seconds', '0.000000005')
        self.assert_window(result, 'indexed_window')
        self.invoke('inspect', self.bag)
        self.assertEqual(self.invoke('cache', 'info')[0]['bags'], 1)
        before = self.bag.read_bytes()
        unrelated = self.cache_dir / 'keep.mcap'
        unrelated.write_bytes(b'bystander')
        self.assertEqual(self.invoke('cache', 'clear')[0]['removed_bags'], 1)
        self.assertEqual(self.bag.read_bytes(), before)
        self.assertEqual(unrelated.read_bytes(), b'bystander')

    def test_incomplete_or_mutated_scan_is_not_cached(self):
        original = mcap.reader.SeekingReader.iter_messages
        def broken(reader, **filters):
            yield next(original(reader, **filters))
            raise OSError('simulated incomplete scan')
        with patch.object(mcap.reader.SeekingReader, 'iter_messages', broken):
            result = quick.triage_bag(self.args)
        self.assertFalse(result['scope']['complete'])
        self.assertIn('incomplete scan', result['scope']['scan_error'])
        self.assertEqual(Cache().info()['bags'], 0)
        def change(path):
            data = quick._triage(path)
            with open(path, 'ab') as stream:
                stream.write(b'x')
            return data
        result = quick._cached(self.args, 'triage', change)
        self.assertTrue(result['scope']['source_changed'])
        self.assertEqual(result['cache']['reason'], 'source_changed')
        self.assertEqual(Cache().info()['bags'], 0)

    def test_bad_cdr_cannot_be_mistaken_for_complete_content_evidence(self):
        with self.bag.open('wb') as stream:
            writer = McapWriter(stream)
            writer.start()
            schema = writer.register_schema('fixture/msg/Emergency', 'ros2msg', b'bool emergency\n')
            chan = writer.register_channel(EMERGENCY, 'cdr', schema)
            writer.add_message(chan, BASE_NS, b'\x00', BASE_NS)
            writer.finish()
        result = quick.triage_bag(self.args)
        self.assertEqual(result['scope']['decode_failures'][EMERGENCY], 1)
        self.assertTrue(any('decode gaps' in note for note in result['next_steps']))
        self.assertEqual(result['cache']['reason'], 'incomplete_evidence')
        self.assertEqual(Cache().info()['bags'], 0)

    def test_cache_default_uses_current_home_and_respects_override(self):
        with patch.dict(os.environ, {'HOME': str(self.root), 'AUTOWARE_DIAG_CACHE_DIR': ''}):
            default = Cache().directory
            self.assertEqual(default, self.root / '.pix-mcap-cache' / '.autoware-diag-cache')
            self.assertFalse(default.exists())
        self.assertEqual(Cache().directory, self.cache_dir)

    def test_cache_bypass_and_failure_still_analyze(self):
        with patch.object(Cache, '_lock', side_effect=PermissionError('cache directory not writable')) as lock:
            bypass = quick.triage_bag({**self.args, 'no_cache': True})
            lock.assert_not_called()
            result = quick.triage_bag(self.args)
        self.assertEqual(bypass['cache']['status'], 'bypass')
        self.assertEqual(result['cache']['status'], 'unavailable')
        self.assertIn('not writable', result['cache']['warning'])
        self.assertEqual(result['logs']['total'], bypass['logs']['total'])

    def test_cache_corruption_replacement_and_rules_version(self):
        quick.triage_bag(self.args)
        entry = next(self.cache_dir.glob('bag-*.json'))
        envelope = json.loads(entry.read_text())
        envelope['results']['triage']['aeb']['total'] = -1  # valid JSON, wrong checksum
        entry.write_text(json.dumps(envelope))
        repaired = quick.triage_bag(self.args)
        self.assertEqual(repaired['cache']['status'], 'miss')
        self.assertIn('integrity', repaired['cache']['warning'])
        self.assertEqual(quick.triage_bag(self.args)['cache']['status'], 'hit')
        with patch('autoware_diag.cache.ANALYSIS_VERSION', 999):
            self.assertEqual(quick.triage_bag(self.args)['cache']['reason'], 'source_or_rules_changed')
        before = self.bag.stat()
        copy = self.root / 'copy.mcap'
        copy.write_bytes(self.bag.read_bytes())
        os.utime(copy, ns=(before.st_atime_ns, before.st_mtime_ns))
        os.replace(copy, self.bag)
        self.assertEqual(quick.triage_bag(self.args)['cache']['reason'], 'source_or_rules_changed')
        self.bag.unlink()
        with self.assertRaises(AnalysisError):
            quick.triage_bag(self.args)

    def test_concurrent_cache_writers_merge_variants(self):
        with ThreadPoolExecutor(max_workers=4) as executor:
            jobs = [executor.submit(self.invoke, command, self.bag)
                    for command in ('triage', 'inspect', 'triage', 'inspect')]
            for job in jobs:
                self.assertIn(job.result()[0]['cache']['status'], ('hit', 'miss'))
        self.assertEqual(Cache().info()['bags'], 1)
        self.assertEqual(quick.triage_bag(self.args)['cache']['status'], 'hit')
        self.assertEqual(quick.inspect_bag(self.args)['cache']['status'], 'hit')
        self.assertFalse(list(self.cache_dir.glob('.autoware-diag-write-*.tmp')))

    def test_cache_lru_bag_limit_byte_limit_and_entry_size(self):
        files = [self.root / f'{i}.mcap' for i in range(3)]
        for file in files:
            file.write_bytes(self.bag.read_bytes())
        keys = list(map(source_identity, files))
        for directory, limits in [('bags', dict(max_bags=2)), ('bytes', dict(max_bytes=2400))]:
            with self.subTest(limit=directory):
                cache = Cache(self.root / directory, **limits)
                row = {'example': 'x' * 400}
                for key in keys[:2]:
                    self.assertTrue(cache.put(key, 'inspect', row)['stored'])
                os.utime(cache._path(keys[0]), ns=(1, 1))
                os.utime(cache._path(keys[1]), ns=(2, 2))
                self.assertEqual(cache.get(keys[0], 'inspect')[1]['status'], 'hit')
                self.assertTrue(cache.put(keys[2], 'inspect', row)['stored'])
                self.assertEqual(cache.info()['bags'], 2)
                self.assertEqual(cache.get(keys[1], 'inspect')[1]['status'], 'miss')
                self.assertTrue(all(file.read_bytes() == self.bag.read_bytes() for file in files))
        cache = Cache(self.root / 'max_entry', max_entry_bytes=700)
        self.assertEqual(cache.put(keys[0], 'triage', {'sample': 'x' * 1000})['reason'], 'entry_size_limit')
        self.assertEqual(cache.info()['bags'], 0)


if __name__ == '__main__':
    unittest.main()
