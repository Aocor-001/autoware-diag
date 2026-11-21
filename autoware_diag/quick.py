"""Indexed inspection and a single-pass, bounded diagnostic first response."""
from __future__ import annotations

from array import array
from collections import Counter, defaultdict
import copy
import heapq
import json
import os
from pathlib import Path

from .cache import Cache, source_identity
from .core import _decoding_status, _open_mcap, _require_file_path, _time_str
from .event_state import update_legacy, update_mrm, update_shadow
from .errors import AnalysisError, INTERNAL_ERROR
from .mcap_support import HAVE_ROS2, Ros2DecoderFactory
from .noise import _is_rosout_noise
from .parsers import _extract_engage, _parse_aeb_shadow, _val
from .topics import TOPIC_ALL_HEALTH, TOPIC_MRM, TOPIC_AEB_SHADOW

LEGACY_AEB = '/pix_robobus/av_aeb_ctrl'
LOG_TOPIC = '/rosout'
DIAG_TOPIC = '/diagnostics'
SAFETY_TOPICS = TOPIC_AEB_SHADOW + [LEGACY_AEB] + TOPIC_MRM
WATCH_TOPICS = sorted(set(TOPIC_ALL_HEALTH + SAFETY_TOPICS + [LOG_TOPIC, DIAG_TOPIC]))
SIG_TYPES = {'/api/autoware/get/engage': 'engage',
             '/api/autoware/get/emergency': 'emergency_flag',
             '/control/command/emergency_cmd': 'emergency_cmd'}
DECODE_TOPICS = set(TOPIC_AEB_SHADOW + [LEGACY_AEB, LOG_TOPIC, DIAG_TOPIC]) | SIG_TYPES.keys()
WATCH_SET = set(WATCH_TOPICS)
MAX_OUTPUT_BYTES = 16 * 1024
DETAIL_LIMIT = 100


class EventBuffer:
    """Exact counters with bounded, severity-prioritized evidence examples."""
    def __init__(self, topic=None, limit=DETAIL_LIMIT):
        self.topic = topic
        self.limit = limit
        self.total = 0
        self.counts = Counter()
        self.severities = Counter()
        self._heap = []
        self.timestamp = 0

    def append(self, event):
        self.total += 1
        self.counts[event.get('event', 'unknown')] += 1
        severity = event.get('severity', 'info')
        self.severities[severity] += 1
        event = dict(event)
        if self.topic:
            event['topic'] = self.topic
        priority = {'critical': 4, 'error': 3, 'warning': 2, 'info': 1}.get(severity, 0)
        event['log_time_ns'] = self.timestamp
        item = (priority, -self.timestamp, -self.total, event)
        if len(self._heap) < self.limit:
            heapq.heappush(self._heap, item)
        elif item[:3] > self._heap[0][:3]:
            heapq.heapreplace(self._heap, item)

    def result(self):
        rows = [item[3] for item in sorted(self._heap, reverse=True)]
        return {'total': self.total, 'by_event': dict(self.counts),
                'by_severity': dict(self.severities), 'examples': rows,
                'omitted': self.total - len(rows)}


def summary_info(file_path, summary):
    """Read only objects in the Summary. Never iterates MCAP message records."""
    stats = getattr(summary, 'statistics', None)
    channels = getattr(summary, 'channels', {}) or {}
    schemas = getattr(summary, 'schemas', {}) or {}
    counts = getattr(stats, 'channel_message_counts', None)
    topics = {}
    for cid, channel in channels.items():
        row = topics.setdefault(channel.topic, {'topic': channel.topic, 'message_count': 0 if counts is not None else None,
                                                'schemas': [], 'encodings': []})
        if counts is not None:
            row['message_count'] += counts.get(cid, 0)
        name = schemas[channel.schema_id].name if channel.schema_id in schemas else None
        if name is not None and name not in row['schemas']:
            row['schemas'].append(name)
        if channel.message_encoding not in row['encodings']:
            row['encodings'].append(channel.message_encoding)
    time_range = None
    if stats is not None and stats.message_count:
        start, end = stats.message_start_time, stats.message_end_time
        time_range = {'start_ns': start, 'end_ns': end,
                      'start_iso': _time_str(start), 'end_iso': _time_str(end),
                      'duration_seconds': (end - start) / 1e9}
    return {
        'file_path': str(Path(file_path).resolve()),
        'file_size_mb': round(os.path.getsize(file_path) / (1024 * 1024), 2),
        'summary_available': summary is not None,
        'index_available': bool(getattr(summary, 'chunk_indexes', None)),
        'statistics_available': stats is not None,
        'basic_info': {'total_messages': stats.message_count if stats else None,
                       'unique_topics': len(topics) if summary is not None else None,
                       'channels': len(channels) if summary is not None else None,
                       'schemas': len(schemas) if summary is not None else None,
                       'time_range': time_range},
        'topics': sorted(topics.values(), key=lambda r: r['topic']),
        'schemas': [{'name': schema.name, 'encoding': schema.encoding, 'definition_bytes': len(schema.data)}
                    for _, schema in sorted(schemas.items())],
        'note': ('Summary/index unavailable; no message scan was performed.' if summary is None
                 else 'Chunk index unavailable; only Summary metadata was read.' if not summary.chunk_indexes
                 else None),
    }


def _inspect(file_path):
    reader, fp = _open_mcap(file_path)
    try:
        return summary_info(file_path, reader.get_summary())
    except Exception as exc:
        raise AnalysisError(INTERNAL_ERROR, f'Cannot read MCAP Summary: {exc}') from exc
    finally:
        fp.close()


def _triage(file_path):
    reader, fp = _open_mcap(file_path)
    stats = {}
    decoded_counts, decode_failures = Counter(), Counter()
    log_events, mrm_events = EventBuffer(LOG_TOPIC), EventBuffer()
    aeb_buffers = {t: EventBuffer(t) for t in TOPIC_AEB_SHADOW + [LEGACY_AEB]}
    aeb_states = {t: {'aeb_req': None, 'fcw_req': None, 'aeb_raw': None, 'fcw_raw': None,
                      'aeb_fault': None, 'fcw_fault': None} for t in aeb_buffers}
    mrm_state = {}
    log_levels, diag_levels = Counter(), Counter()
    diag_groups = {}
    diag_statuses = diag_non_ok = diag_unaggregated = noise_filtered = 0
    scan_error = None
    scanned = 0
    total_all = 0
    first_all = last_all = None
    schema_ids, scanned_topics = set(), set()
    pending_safety = []
    linear_times = defaultdict(lambda: array('Q'))

    def safety(topic, ns, data):
        if topic in aeb_buffers:
            buffer = aeb_buffers[topic]
            buffer.timestamp = ns
            update = update_legacy if topic == LEGACY_AEB else update_shadow
            update(data, ns, aeb_states[topic], buffer, None)
        else:
            mrm_events.timestamp = ns
            update_mrm(SIG_TYPES[topic], ns, data, mrm_state, mrm_events)

    try:
        summary = reader.get_summary()
        metadata = summary_info(file_path, summary)
        # If statistics are unavailable, one linear pass also obtains global counts/range.
        indexed = metadata['index_available']
        all_topics = not metadata['statistics_available']
        mode = ('indexed_full' if all_topics else 'indexed_topics') if indexed else 'linear_fallback'
        decoder = Ros2DecoderFactory() if HAVE_ROS2 else None
        decode_functions = {}
        try:
            for schema, channel, message in reader.iter_messages(
                    topics=None if all_topics else WATCH_SET, log_time_order=indexed):
                scanned += 1
                topic, ns = channel.topic, message.log_time
                if all_topics:
                    total_all += 1
                    first_all = ns if first_all is None else min(first_all, ns)
                    last_all = ns if last_all is None else max(last_all, ns)
                    scanned_topics.add(topic)
                    if schema:
                        schema_ids.add(schema.id)
                if topic not in WATCH_SET:
                    continue
                row = stats.setdefault(topic, {'topic': topic, 'count': 0, 'first_ns': ns,
                                                'last_ns': ns, 'max_gap_ns': 0})
                if indexed and row['count']:
                    row['max_gap_ns'] = max(row['max_gap_ns'], ns - row['last_ns'])
                if not indexed:
                    linear_times[topic].append(ns)
                row['count'] += 1
                row['first_ns'] = min(row['first_ns'], ns)
                row['last_ns'] = max(row['last_ns'], ns)
                if topic not in DECODE_TOPICS:
                    continue
                decoded = None
                key = (channel.message_encoding, schema.id if schema else None)
                if key not in decode_functions:
                    try:
                        decode_functions[key] = decoder.decoder_for(channel.message_encoding, schema) if decoder and schema else None
                    except Exception:
                        decode_functions[key] = None
                try:
                    function = decode_functions[key]
                    decoded = function(message.data) if function else None
                except Exception:
                    pass
                if decoded is None:
                    decode_failures[topic] += 1
                    continue
                decoded_counts[topic] += 1
                if topic in aeb_buffers or topic in SIG_TYPES:
                    if topic in SIG_TYPES:
                        value = _extract_engage(decoded) if SIG_TYPES[topic] == 'engage' else _val(decoded, 'emergency')
                        value = bool(value) if value is not None and SIG_TYPES[topic] != 'engage' else value
                    elif topic == LEGACY_AEB:
                        fields = ('acu_aeb_active_request', 'acu_aeb_brake_en',
                                  'acu_fcw_active_request', 'acu_aeb_brake_pedal_target')
                        value = {field: item for field in fields if (item := _val(decoded, field)) is not None}
                    else:
                        value = _parse_aeb_shadow(decoded) if not indexed else decoded
                    if indexed:
                        safety(topic, ns, value)
                    else:
                        # NonSeekingReader would otherwise retain all raw payloads to sort.
                        pending_safety.append((ns, topic, value))
                if topic == LOG_TOPIC:
                    level = int(_val(decoded, 'level') or 0)
                    log_levels[level] += 1
                    if level >= 30:
                        name, text = str(_val(decoded, 'name') or '?'), str(_val(decoded, 'msg') or '')
                        if _is_rosout_noise(name, text):
                            noise_filtered += 1
                        else:
                            log_events.timestamp = ns
                            log_events.append({'time': _time_str(ns),
                                               'event': 'ROSOUT', 'level': level,
                                               'severity': 'error' if level >= 40 else 'warning',
                                               'node': name[:160]})
                elif topic == DIAG_TOPIC:
                    for status in _val(decoded, 'status') or []:
                        diag_statuses += 1
                        level = int(_val(status, 'level') or 0)
                        diag_levels[level] += 1
                        if not level:
                            continue
                        diag_non_ok += 1
                        name = str(_val(status, 'name') or '?')
                        key = (name, level)
                        item = diag_groups.get(key)
                        if item is None and len(diag_groups) < 1000:
                            item = diag_groups[key] = {
                                'name': name[:160], 'level': level, 'count': 0,
                                'topic': topic, 'first_log_time_ns': ns, 'last_log_time_ns': ns,
                            }
                        if item is None:
                            diag_unaggregated += 1
                        else:
                            item['count'] += 1
                            item['first_log_time_ns'] = min(item['first_log_time_ns'], ns)
                            item['last_log_time_ns'] = max(item['last_log_time_ns'], ns)
        except Exception as exc:
            scan_error = f'{type(exc).__name__}: {exc}'[:500]
    except Exception as exc:
        raise AnalysisError(INTERNAL_ERROR, f'Cannot read MCAP Summary: {exc}') from exc
    finally:
        fp.close()

    if not indexed:
        for ns, topic, value in sorted(pending_safety, key=lambda signal: signal[0]):
            safety(topic, ns, value)
        for topic, ticks in linear_times.items():
            ordered = sorted(ticks)
            stats[topic]['max_gap_ns'] = max((b-a for a, b in zip(ordered, ordered[1:])), default=0)
    if all_topics:
        metadata['basic_info'].update(total_messages=total_all, unique_topics=len(scanned_topics),
                                      schemas=len(schema_ids))
        if first_all is not None:
            metadata['basic_info']['time_range'] = {
                'start_ns': first_all, 'end_ns': last_all,
                'start_iso': _time_str(first_all), 'end_iso': _time_str(last_all),
                'duration_seconds': (last_all-first_all)/1e9}
    mismatched_topics = [row['topic'] for row in metadata['topics']
                         if row['topic'] in WATCH_SET and row['message_count'] is not None
                         and row['message_count'] != stats.get(row['topic'], {}).get('count', 0)]
    if scan_error is None and mismatched_topics:
        scan_error = 'Observed topic counts do not match the MCAP Summary.'
    source = next((t for t in TOPIC_AEB_SHADOW + [LEGACY_AEB] if stats.get(t, {}).get('count')), None)
    aeb = aeb_buffers[source].result() if source else EventBuffer().result()
    aeb.update(source_topic=source, message_count=stats.get(source, {}).get('count', 0),
               decoded_messages=decoded_counts[source], decode_failures=decode_failures[source])
    mrm = mrm_events.result()
    for event in mrm['examples']:
        event['topic'] = ('/api/autoware/get/engage' if event['event'] in ('ENGAGE', 'DISENGAGE')
                          else '/control/command/emergency_cmd' if event['event'].startswith('EMERGENCY_CMD_')
                          else '/api/autoware/get/emergency')
    mrm.update(message_count=sum(stats.get(t, {}).get('count', 0) for t in SIG_TYPES),
               decoded_messages=sum(decoded_counts[t] for t in SIG_TYPES),
               decode_failures=sum(decode_failures[t] for t in SIG_TYPES))
    frequencies = []
    for topic, row in sorted(stats.items()):
        duration = (row['last_ns']-row['first_ns'])/1e9
        frequencies.append({**row, 'avg_frequency_hz': round(row['count']/duration, 2) if duration else 0,
                            'max_gap_seconds': row['max_gap_ns']/1e9})
    diagnostics = sorted(diag_groups.values(), key=lambda r: (-r['level'], -r['count'], r['name']))
    suggestions = []
    if aeb['total'] or mrm['total']:
        suggestions.append('incident: align the reported safety-event time with planning/control and chassis evidence')
    if diag_non_ok:
        suggestions.append('diagnostics: inspect the affected module/status around the symptom time')
    if log_events.total:
        suggestions.append('rosout: check the original log text for the reported node/severity and log time')
    if decode_failures or scan_error:
        suggestions.append('Resolve scan/decode gaps before drawing conclusions about event absence')
    if not suggestions:
        suggestions.append('Provide a symptom and timestamp for a targeted incident/domain investigation; this is not a full vehicle-health certification')
    return {
        'file_path': metadata['file_path'], 'file_size_mb': metadata['file_size_mb'],
        'ros2_decoding': _decoding_status(), 'basic_info': metadata['basic_info'],
        'scope': {'mode': mode, 'complete': scan_error is None, 'scan_error': scan_error,
                  'message_iterations': 1, 'messages_visited': scanned, 'evidence_messages_visited': scanned,
                  'summary_count_mismatches': mismatched_topics,
                  'all_topics_selected': all_topics,
                  'topics_requested': WATCH_TOPICS, 'time_basis': 'log_time',
                  'time_range': metadata['basic_info']['time_range'],
                  'decode_target_topics': sorted(DECODE_TOPICS),
                  'decode_failures': dict(decode_failures), 'decoded_messages': sum(decoded_counts.values()),
                  'note': 'No point-cloud/image/object payload decoding. Counts are observations, not proof of event absence when coverage or decoding is incomplete.'},
        'health': {'monitored_topics': len(WATCH_TOPICS), 'present_topics': len(stats),
                   'missing_topics': [t for t in WATCH_TOPICS if t not in stats],
                   'note': 'Missing within this file/scan is not necessarily a vehicle fault; alternative topic names exist.',
                   'topic_statistics': frequencies, 'topic_statistics_omitted': 0},
        'logs': {**log_events.result(), 'message_count': stats.get(LOG_TOPIC, {}).get('count', 0),
                 'levels': dict(log_levels), 'noise_filtered': noise_filtered,
                 'decode_failures': decode_failures[LOG_TOPIC]},
        'diagnostics': {'message_count': stats.get(DIAG_TOPIC, {}).get('count', 0),
                        'status_count': diag_statuses, 'non_ok_status_count': diag_non_ok,
                        'levels': dict(diag_levels), 'groups': diagnostics[:DETAIL_LIMIT],
                        'groups_omitted': max(0, len(diagnostics)-DETAIL_LIMIT),
                        'unaggregated_status_count': diag_unaggregated,
                        'decode_failures': decode_failures[DIAG_TOPIC]},
        'aeb': aeb, 'mrm': mrm, 'next_steps': suggestions,
    }


def _bounded_output(result, details=False):
    result = copy.deepcopy(result)
    # Cached aggregates always retain the bounded detailed variant (up to 100 examples).
    limit = DETAIL_LIMIT if details else 10
    for name in ('logs', 'aeb', 'mrm'):
        section = result[name]
        section['examples'] = section['examples'][:limit]
        section['omitted'] = section['total'] - len(section['examples'])
    section = result['diagnostics']
    available = len(section['groups']) + section['groups_omitted']
    section['groups'] = section['groups'][:limit]
    section['groups_omitted'] = available - len(section['groups'])
    if details:
        return result
    health = result['health']
    rows = health['topic_statistics']
    health['topic_statistics'] = sorted(rows, key=lambda row: (-row['max_gap_ns'], row['topic']))[:limit]
    health['topic_statistics_omitted'] = len(rows) - len(health['topic_statistics'])
    # Match CLI pretty-print byte accounting, not Unicode character count.
    def size():
        return len(json.dumps(result, ensure_ascii=False, indent=2, default=str).encode('utf-8')) + 1
    while size() > MAX_OUTPUT_BYTES:
        candidates = [(len(result[name]['examples']), result[name]['examples'], result[name], 'omitted')
                      for name in ('logs','aeb','mrm')]
        candidates.append((len(result['diagnostics']['groups']), result['diagnostics']['groups'],
                           result['diagnostics'], 'groups_omitted'))
        candidates.append((len(result['health']['topic_statistics']), result['health']['topic_statistics'],
                           result['health'], 'topic_statistics_omitted'))
        count, rows, section, omitted_key = max(candidates, key=lambda item: item[0])
        if not count:
            # Long source paths/error strings are bounded by the filesystem and parser;
            # retain coverage/counters rather than silently truncate valid JSON.
            raise AnalysisError(INTERNAL_ERROR, 'Triage metadata exceeds compact output budget; use --details')
        rows.pop()
        section[omitted_key] += 1
    return result


def _cached(args, kind, compute):
    file_path = _require_file_path(args)
    identity = source_identity(file_path)  # Always reopen source before accepting cached evidence.
    cache = None if args.get('no_cache') else Cache()
    if cache is None:
        result, state = None, {'status': 'bypass'}
    else:
        result, state = cache.get(identity, kind)
    if result is not None:
        current = source_identity(file_path)
        if current != identity:
            identity = current
            result, state = None, {'status': 'miss', 'reason': 'source_changed'}
    if result is None:
        result = compute(identity['path'])
        # Incomplete scans are useful partial evidence, but never successful cached results.
        scope = result.get('scope', {})
        complete = scope.get('complete', True) and not scope.get('decode_failures')
        try:
            stable = source_identity(file_path) == identity
        except (OSError, AnalysisError):
            stable = False
        if not stable:
            result.setdefault('scope', {}).update(complete=False, source_changed=True)
            state.update(stored=False, reason='source_changed')
        elif cache is not None and state['status'] != 'unavailable':
            if complete:
                state.update(cache.put(identity, kind, result))
                if state.get('reason') in ('source_changed', 'source_unavailable_or_changed'):
                    result.setdefault('scope', {}).update(complete=False, source_changed=True)
            else:
                state.update(stored=False, reason='incomplete_evidence')
    result = copy.deepcopy(result)
    result['source_identity'] = identity
    result['cache'] = state
    if kind == 'triage':
        if state['status'] == 'hit':
            result['scope']['message_iterations'] = 0
            result['scope']['messages_visited'] = 0
        return _bounded_output(result, args.get('details', False))
    return result


def inspect_bag(args):
    return _cached(args, 'inspect', _inspect)


def triage_bag(args):
    return _cached(args, 'triage', _triage)
