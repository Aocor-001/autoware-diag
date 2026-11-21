"""Bounded, disposable analysis cache. Never writes to or removes source recordings."""
from __future__ import annotations

from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import tempfile
import time

from .errors import AnalysisError, INVALID_PARAMS

CACHE_VERSION = 1
ANALYSIS_VERSION = 2  # Bump when evidence interpretation or aggregate semantics change.
DEFAULT_CACHE_DIR = Path('~/.pix-mcap-cache/.autoware-diag-cache')
MAX_BYTES = 256 * 1024 * 1024
MAX_BAGS = 500
MAX_ENTRY_BYTES = 1024 * 1024
_OWNED = re.compile(r'bag-[a-f0-9]{64}\.json\Z')
_TEMP_PREFIX = '.autoware-diag-write-'
_TEMP_SUFFIX = '.tmp'


def source_identity(file_path):
    """Constant-size fingerprint, including fstat checks against concurrent changes."""
    try:
        path = Path(file_path).resolve(strict=True)
        with path.open('rb') as stream:
            before = os.fstat(stream.fileno())
            if not stat.S_ISREG(before.st_mode) or not before.st_size:
                raise ValueError('source must be a nonempty regular file')
            head = stream.read(65536)
            stream.seek(max(0, before.st_size - 65536))
            tail = stream.read(65536)
            after = os.fstat(stream.fileno())
        fields = lambda s: (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns)
        if fields(before) != fields(after) or fields(after) != fields(path.stat()):
            raise ValueError('source changed while fingerprinting')
        return {
            'path': str(path), 'size': after.st_size, 'mtime_ns': after.st_mtime_ns,
            'ctime_ns': after.st_ctime_ns, 'device': after.st_dev, 'inode': after.st_ino,
            'head_sha256': hashlib.sha256(head).hexdigest(),
            'tail_sha256': hashlib.sha256(tail).hexdigest(),
        }
    except (OSError, ValueError) as exc:
        raise AnalysisError(INVALID_PARAMS, f'Cannot identify source recording: {exc}') from exc


class Cache:
    def __init__(self, directory=None, *, max_bytes=MAX_BYTES, max_bags=MAX_BAGS,
                 max_entry_bytes=MAX_ENTRY_BYTES):
        configured = directory or os.environ.get('AUTOWARE_DIAG_CACHE_DIR') or DEFAULT_CACHE_DIR
        self.directory = Path(configured).expanduser()
        self.max_bytes = max_bytes
        self.max_bags = max_bags
        self.max_entry_bytes = max_entry_bytes

    @contextmanager
    def _lock(self):
        # A symlinked cache directory could redirect clear/write outside the intended root.
        if self.directory.is_symlink():
            raise OSError('cache directory must not be a symlink')
        self.directory.mkdir(parents=True, exist_ok=True, mode=0o700)
        fd = os.open(self.directory / '.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        try:
            deadline = time.monotonic() + 0.5
            while True:
                try:
                    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    break
                except BlockingIOError:
                    if time.monotonic() >= deadline:
                        raise OSError('cache lock timed out')
                    time.sleep(0.01)
            yield
        finally:
            os.close(fd)

    def _path(self, identity):
        key = hashlib.sha256(identity['path'].encode()).hexdigest()
        return self.directory / f'bag-{key}.json'

    def _entries(self):
        with os.scandir(self.directory) as items:
            return [(Path(entry.path), entry.stat(follow_symlinks=False)) for entry in items
                    if _OWNED.fullmatch(entry.name) and entry.is_file(follow_symlinks=False)]

    def _clean_temporary_files(self):
        # A process killed before replace may leave an orphan. All writers hold
        # the same lock, so no active writer owns these files while we clean up.
        with os.scandir(self.directory) as items:
            for entry in items:
                if (entry.name.startswith(_TEMP_PREFIX) and entry.name.endswith(_TEMP_SUFFIX)
                        and entry.is_file(follow_symlinks=False)):
                    os.unlink(entry.path)

    def _prune(self):
        entries = sorted(self._entries(), key=lambda item: (item[1].st_mtime_ns, item[0].name))
        total = sum(s.st_size for _, s in entries)
        count = len(entries)
        removed = 0
        for path, info in entries:
            if total <= self.max_bytes and count <= self.max_bags:
                break
            path.unlink()
            total -= info.st_size
            count -= 1
            removed += 1
        return removed

    def _read(self, path):
        if path.is_symlink() or not path.is_file():
            raise ValueError('non-regular cache entry ignored')
        with path.open('rb') as stream:
            raw = stream.read(self.max_entry_bytes + 1)
        if len(raw) > self.max_entry_bytes:
            raise ValueError('oversized cache entry ignored')
        data = json.loads(raw)
        if not isinstance(data, dict) or not isinstance(data.get('results'), dict):
            raise ValueError('invalid cache envelope')
        payload = json.dumps(data['results'], ensure_ascii=False, separators=(',', ':')).encode()
        if data.get('results_sha256') != hashlib.sha256(payload).hexdigest():
            raise ValueError('cache integrity check failed')
        return data

    def _valid(self, data, identity):
        return (data.get('cache_version') == CACHE_VERSION
                and data.get('analysis_version') == ANALYSIS_VERSION
                and data.get('source_identity') == identity)

    def get(self, identity, kind):
        try:
            with self._lock():
                path = self._path(identity)
                if not path.exists():
                    return None, {'status': 'miss'}
                try:
                    data = self._read(path)
                except (ValueError, RecursionError) as exc:
                    return None, {'status': 'miss', 'warning': str(exc)}
                if not self._valid(data, identity):
                    return None, {'status': 'miss', 'reason': 'source_or_rules_changed'}
                value = data['results'].get(kind)
                if not isinstance(value, dict):
                    return None, {'status': 'miss'}
                os.utime(path, None)  # LRU; does not touch source file metadata.
                return value, {'status': 'hit'}
        except OSError as exc:
            return None, {'status': 'unavailable', 'warning': str(exc)}

    def put(self, identity, kind, result):
        """Source validation occurs outside the short metadata-management lock."""
        try:
            if source_identity(identity['path']) != identity:
                return {'stored': False, 'reason': 'source_changed'}
        except (OSError, AnalysisError):
            return {'stored': False, 'reason': 'source_unavailable_or_changed'}
        try:
            with self._lock():
                self._clean_temporary_files()
                path = self._path(identity)
                data = {'cache_version': CACHE_VERSION, 'analysis_version': ANALYSIS_VERSION,
                        'source_identity': identity, 'results': {}}
                if path.exists():
                    try:
                        previous = self._read(path)
                        if self._valid(previous, identity):
                            data['results'] = previous['results']
                    except (ValueError, RecursionError):
                        pass
                data['results'][kind] = result
                payload = json.dumps(data['results'], ensure_ascii=False, separators=(',', ':')).encode()
                data['results_sha256'] = hashlib.sha256(payload).hexdigest()
                raw = json.dumps(data, ensure_ascii=False, separators=(',', ':')).encode()
                if len(raw) > min(self.max_entry_bytes, self.max_bytes):
                    # Per-bag cap includes all result variants; do not grow indefinitely.
                    return {'stored': False, 'reason': 'entry_size_limit'}
                temporary = None
                try:
                    with tempfile.NamedTemporaryFile(dir=self.directory, prefix=_TEMP_PREFIX,
                                                     suffix=_TEMP_SUFFIX, delete=False) as stream:
                        temporary = Path(stream.name)
                        stream.write(raw)
                        stream.flush()
                        os.fsync(stream.fileno())
                    os.replace(temporary, path)
                finally:
                    if temporary is not None and temporary.exists():
                        temporary.unlink()
                evicted = self._prune()
                return {'stored': path.exists(), 'evicted_bags': evicted}
        except OSError as exc:
            return {'status': 'unavailable', 'stored': False, 'warning': str(exc)}

    def info(self):
        try:
            with self._lock():
                self._clean_temporary_files()
                self._prune()
                entries = self._entries()
                return {'directory': str(self.directory), 'bags': len(entries),
                        'bytes': sum(s.st_size for _, s in entries),
                        'max_bytes': self.max_bytes, 'max_bags': self.max_bags,
                        'max_entry_bytes': self.max_entry_bytes}
        except OSError as exc:
            return {'directory': str(self.directory), 'status': 'unavailable', 'warning': str(exc)}

    def clear(self):
        try:
            with self._lock():
                self._clean_temporary_files()
                entries = self._entries()
                for path, _ in entries:
                    path.unlink()
                return {'directory': str(self.directory), 'removed_bags': len(entries),
                        'removed_bytes': sum(s.st_size for _, s in entries)}
        except OSError as exc:
            return {'directory': str(self.directory), 'status': 'unavailable', 'warning': str(exc)}
