"""MCAP open/scan helpers."""
from __future__ import annotations

import datetime as dt_mod
import json
import os
import traceback
from collections import Counter, defaultdict
from typing import Any, Callable, Dict, List, Optional, Tuple

from .log import log
from .mcap_support import HAVE_MCAP, HAVE_ROS2, Ros2DecoderFactory, make_reader
from .errors import INTERNAL_ERROR, INVALID_PARAMS, MAX_RESPONSE_CHARS, AnalysisError, _truncate_text

def _open_mcap(file_path: str):
    """Open an MCAP file and return (reader, file_object).

    调用方负责关闭 file_object（用 try/finally），避免句柄泄漏。
    文件以流式方式读取，不会整文件加载进内存。
    """
    if not HAVE_MCAP:
        raise AnalysisError(INTERNAL_ERROR, "mcap library not installed (pip install mcap)")
    if not os.path.isfile(file_path):
        raise AnalysisError(INVALID_PARAMS, f"File not found: {file_path}")
    file_size = os.path.getsize(file_path)
    if file_size == 0:
        raise AnalysisError(INVALID_PARAMS, f"File is empty: {file_path}")
    f = None
    try:
        f = open(file_path, "rb")
        decoder_factories = [Ros2DecoderFactory()] if HAVE_ROS2 else []
        reader = make_reader(f, decoder_factories=decoder_factories)
        return reader, f
    except AnalysisError:
        if f is not None:
            f.close()
        raise
    except Exception as e:
        if f is not None:
            f.close()
        log.error("Failed to open MCAP file %s: %s", file_path, traceback.format_exc())
        raise AnalysisError(INTERNAL_ERROR, f"Failed to read MCAP file: {e}")


def _require_file_path(args: Dict) -> str:
    """Validate and return the file_path argument (clean error instead of KeyError)."""
    fp = args.get("file_path")
    if not fp or not isinstance(fp, str):
        raise AnalysisError(INVALID_PARAMS, "Missing required parameter: file_path (string)")
    if not os.path.isfile(fp):
        raise AnalysisError(INVALID_PARAMS, f"File not found: {fp}")
    return fp


def _decoding_status() -> str:
    """Surface whether ROS2 message decoding is available (silent false-negatives otherwise)."""
    if HAVE_ROS2:
        return "enabled"
    return (
        "DISABLED (mcap-ros2-support not installed) — messages are NOT decoded; "
        "content analysis and event detection may report false negatives"
    )


def _get_summary(reader) -> tuple:
    """Get summary info from reader. Returns (summary, channel->topic map)."""
    summary = reader.get_summary()
    ch_topic: Dict[int, str] = {}
    if summary and summary.channels:
        for cid, ch in summary.channels.items():
            ch_topic[cid] = ch.topic
    return summary, ch_topic


def _time_str(ns: int) -> str:
    """Convert nanoseconds to ISO time string (local time, with UTC offset).

    带时区偏移（如 +08:00），避免跨时区分析时时间对不上；与
    dateutil_parser.parse 的解析结果可精确互转。
    """
    return dt_mod.datetime.fromtimestamp(ns / 1e9).astimezone().isoformat()


def _dictify(obj, max_depth: int = 3, _depth: int = 0) -> Any:
    """Convert a ROS2 decoded object to a JSON-serializable dict."""
    if _depth >= max_depth:
        return str(obj) if obj is not None else None
    if obj is None:
        return None
    if isinstance(obj, (int, float, bool, str)):
        return obj
    if isinstance(obj, bytes):
        # Cap hex dump — pointcloud / undecoded payloads can be megabytes
        if len(obj) > 64:
            return obj[:64].hex() + f"... ({len(obj)} bytes total)"
        return obj.hex()
    if isinstance(obj, (list, tuple)):
        if len(obj) > 20:
            return [_dictify(x, max_depth, _depth + 1) for x in obj[:20]] + [
                f"... ({len(obj) - 20} more)"
            ]
        return [_dictify(x, max_depth, _depth + 1) for x in obj]
    # Collect field names: __dict__ + __slots__ + dir() fallback
    field_names = set()
    if hasattr(obj, "__dict__"):
        field_names.update(k for k in obj.__dict__ if not k.startswith("_"))
    if hasattr(obj, "__slots__"):
        for slot in obj.__slots__:
            if not slot.startswith("_"):
                field_names.add(slot)
    if not field_names:
        for name in dir(obj):
            if not name.startswith("_"):
                try:
                    val = getattr(obj, name)
                    if not callable(val):
                        field_names.add(name)
                except Exception:
                    pass
    if field_names:
        result = {}
        for name in sorted(field_names):
            try:
                val = getattr(obj, name)
                result[name] = _dictify(val, max_depth, _depth + 1)
            except Exception:
                result[name] = "<error>"
        return result
    return str(obj)


def _get_per_topic_messages(
    file_path: str, topic_filter: Optional[List[str]] = None,
    max_per_topic: int = 0,
):
    """Iterate decoded messages from an MCAP file, grouped by topic.
    Uses iter_messages (fast) + on-demand decode for matching topics.
    When max_per_topic > 0, stops early after that many samples per topic.
    When max_per_topic == 0 and topic_filter is given, scans the WHOLE file
    (used by event detectors which must not miss transitions).
    Yields (topic, decoded_msg, publish_time_ns, log_time_ns).
    """
    reader, fp = _open_mcap(file_path)
    try:
        summary, ch_topic = _get_summary(reader)
        topic_set = set(topic_filter) if topic_filter else None

        if topic_set:
            # Fast path: iter_messages (no decode) + decode only matching topics
            counts = {t: 0 for t in topic_set}
            decoder = Ros2DecoderFactory() if HAVE_ROS2 else None

            for _schema, channel, msg in reader.iter_messages(log_time_order=False, topics=topic_filter):

                topic = ch_topic.get(channel.id, "unknown") if channel else "unknown"
                if topic not in topic_set:
                    continue
                if max_per_topic > 0 and counts[topic] >= max_per_topic:
                    continue
                counts[topic] += 1

                # Decode on demand
                decoded = None
                if decoder and _schema:
                    try:
                        dec_fn = decoder.decoder_for(
                            channel.message_encoding if channel else "cdr",
                            _schema or None,
                        )
                        if dec_fn:
                            decoded = dec_fn(msg.data)
                    except Exception:
                        pass

                yield topic, decoded if decoded is not None else msg.data, msg.publish_time, msg.log_time
                if max_per_topic > 0 and all(c >= max_per_topic for c in counts.values()):
                    return
        else:
            # No filter: decoded iterator (decodes all topics)
            for _schema, channel, msg, decoded in reader.iter_decoded_messages(
                log_time_order=False, topics=topic_filter
            ):
                topic = ch_topic.get(channel.id, "unknown") if channel else "unknown"
                yield topic, decoded, msg.publish_time, msg.log_time
    finally:
        fp.close()


def _scan_mcap(
    reader,
    watch_topics: Optional[List[str]] = None,
    ts_cap: int = 20000,
    gap_threshold_s: float = 1.0,
    topics: Optional[List[str]] = None,
) -> Dict[str, Any]:
    """Single streaming pass over all messages. O(1) memory per topic —
    不整文件读入内存，不为每个话题保存全量时间戳。

    :param topics: 传给 mcap reader.iter_messages(topics=...)，仅解压包含这些话题
        的 chunk，大幅降内存/提速（点云等大块直接跳过）。None = 全文件。

    Returns dict with:
      - stats: {topic: {count, first_log_time, last_log_time, schema}}
      - total_messages / unique_schemas / first_log_time / last_log_time
      - timestamps: {topic: [log_time_ns, ...]} — 仅 watch_topics，每话题封顶 ts_cap 条
      - gaps: {topic: {total_gaps, max_gap_s, gaps(前10条)}}
      - scan_error: 迭代中途出错时的错误信息（此时统计为部分数据）
    """
    topic_stats: Dict[str, Dict] = {}
    schemas: Dict[int, str] = {}
    message_count = 0
    first_log_time = None
    last_log_time = None
    watch = set(watch_topics) if watch_topics else None
    timestamps: Dict[str, List[int]] = defaultdict(list)
    gap_state: Dict[str, Dict] = {}
    scan_error = None

    try:
        for schema, channel, message in reader.iter_messages(log_time_order=False, topics=topics):
            if schema and schema.id not in schemas:
                schemas[schema.id] = schema.name
            topic = channel.topic if channel else "unknown"
            log_time_ns = message.log_time

            ts = topic_stats.get(topic)
            if ts is None:
                ts = topic_stats[topic] = {
                    "count": 0,
                    "first_log_time": log_time_ns,
                    "last_log_time": log_time_ns,
                    "schema": schema.name if schema else "unknown",
                }
            ts["count"] += 1
            ts["first_log_time"] = min(ts["first_log_time"], log_time_ns)
            ts["last_log_time"] = max(ts["last_log_time"], log_time_ns)

            message_count += 1
            if first_log_time is None or log_time_ns < first_log_time:
                first_log_time = log_time_ns
            if last_log_time is None or log_time_ns > last_log_time:
                last_log_time = log_time_ns

            # 采样时间戳（检查在 append 之前，cap 真实生效）
            if watch and topic in watch and len(timestamps[topic]) < ts_cap:
                timestamps[topic].append(log_time_ns)

            # O(1) gap 跟踪。依赖「同一话题在文件中按时间顺序存储」
            # （ROS 录制器成立）；乱序产生的负 gap 直接忽略。
            gs = gap_state.get(topic)
            if gs is None:
                gs = gap_state[topic] = {
                    "prev": None, "total_gaps": 0, "max_gap_s": 0.0, "gaps": [],
                }
            prev = gs["prev"]
            if prev is not None:
                gap_s = (log_time_ns - prev) / 1e9
                if gap_s > gap_threshold_s:
                    gs["total_gaps"] += 1
                    gs["max_gap_s"] = max(gs["max_gap_s"], gap_s)
                    if len(gs["gaps"]) < 10:
                        gs["gaps"].append({
                            "from_seconds": round(prev / 1e9, 3),
                            "to_seconds": round(log_time_ns / 1e9, 3),
                            "gap_seconds": round(gap_s, 3),
                            "from_iso": _time_str(prev),
                            "to_iso": _time_str(log_time_ns),
                        })
            gs["prev"] = log_time_ns
    except Exception as e:
        log.error("_scan_mcap: iter_messages failed: %s", e)
        scan_error = f"Error reading MCAP messages (partial results): {e}"

    return {
        "stats": topic_stats,
        "total_messages": message_count,
        "unique_schemas": len(schemas),
        "first_log_time": first_log_time,
        "last_log_time": last_log_time,
        "timestamps": dict(timestamps),
        "gaps": gap_state,
        "scan_error": scan_error,
    }


def _format_stats(scan: Dict[str, Any]) -> Dict[str, Any]:
    """Format _scan_mcap output into the analyze_mcap_file-style stats dict."""
    topic_stats = scan["stats"]
    first_log_time = scan["first_log_time"]
    last_log_time = scan["last_log_time"]

    total_duration_ns = (
        (last_log_time - first_log_time) if first_log_time and last_log_time else 0
    )
    total_duration_s = total_duration_ns / 1e9 if total_duration_ns else 0

    topic_details = {}
    for topic, ts in topic_stats.items():
        duration = ts["last_log_time"] - ts["first_log_time"]
        duration_s = duration / 1e9 if duration else 0
        freq = ts["count"] / duration_s if duration_s > 0 else 0
        topic_details[topic] = {
            "message_count": ts["count"],
            "first_log_time_ns": ts["first_log_time"],
            "last_log_time_ns": ts["last_log_time"],
            "duration_seconds": round(duration_s, 3),
            "avg_frequency_hz": round(freq, 2),
            "schema": ts["schema"],
        }

    result = {
        "total_messages": scan["total_messages"],
        "unique_topics": len(topic_stats),
        "unique_schemas": scan["unique_schemas"],
        "time_range_ns": {
            "start": first_log_time,
            "end": last_log_time,
            "duration_seconds": round(total_duration_s, 3),
        },
        "topics": topic_details,
    }
    if scan.get("scan_error"):
        result["scan_error"] = scan["scan_error"]
    return result


def _build_anomalies(
    scan: Dict[str, Any],
    gap_threshold_s: float,
    min_frequency_hz: float,
) -> List[Dict[str, Any]]:
    """Build anomaly list (gaps, low frequency) from a _scan_mcap result."""
    anomalies: List[Dict[str, Any]] = []
    gap_state = scan["gaps"]

    for topic, ts in scan["stats"].items():
        count = ts["count"]

        if count < 2:
            anomalies.append(
                {
                    "topic": topic,
                    "type": "insufficient_data",
                    "severity": "warning",
                    "detail": f"Topic '{topic}' has only {count} message(s)",
                }
            )
            continue

        duration_s = (ts["last_log_time"] - ts["first_log_time"]) / 1e9
        freq = count / duration_s if duration_s > 0 else float("inf")

        if freq < min_frequency_hz and duration_s > 1:
            anomalies.append(
                {
                    "topic": topic,
                    "type": "low_frequency",
                    "severity": "warning",
                    "detail": (
                        f"Topic '{topic}' has low frequency: {freq:.2f} Hz "
                        f"({count} msgs over {duration_s:.1f}s)"
                    ),
                }
            )

        gs = gap_state.get(topic)
        if gs and gs["total_gaps"]:
            anomalies.append(
                {
                    "topic": topic,
                    "type": "message_gap",
                    "severity": "error" if gs["max_gap_s"] > 5.0 else "warning",
                    "detail": (
                        f"Topic '{topic}' has {gs['total_gaps']} gap(s), "
                        f"max gap: {gs['max_gap_s']:.1f}s, threshold: {gap_threshold_s}s"
                    ),
                    "gaps": gs["gaps"],
                    "total_gaps": gs["total_gaps"],
                }
            )

    return anomalies


def _collect_topic_data(
    file_path: str,
    topics: List[str],
    max_per_topic: int = 5,
) -> Dict[str, list]:
    """Collect decoded messages for given topics (SAMPLED — 每话题最多 max_per_topic 条).
    Returns {topic: [{"time": ns, "data": decoded, ...}]}
    ⚠️ 采样数据只能用于内容概览；事件/跳变检测必须用 _collect_topic_data_full。
    """
    result: Dict[str, list] = defaultdict(list)
    for topic, decoded, pub_time, log_time in _get_per_topic_messages(
        file_path, topics, max_per_topic=max_per_topic
    ):
        result[topic].append(
            {
                "publish_time_ns": pub_time,
                "log_time_ns": log_time,
                "data": decoded,
            }
        )
    return dict(result)


def _collect_topic_data_full(file_path: str, topics: List[str]) -> Dict[str, list]:
    """Collect ALL messages for given topics (无采样上限).
    事件检测器必须用这个 —— 采样会漏掉录制中后期的事件。
    每话题消息按 log_time 排序，保证跳变检测时序正确。
    """
    result: Dict[str, list] = defaultdict(list)
    for topic, decoded, pub_time, log_time in _get_per_topic_messages(
        file_path, topics, max_per_topic=0
    ):
        result[topic].append(
            {
                "publish_time_ns": pub_time,
                "log_time_ns": log_time,
                "data": decoded,
            }
        )
    for msgs in result.values():
        msgs.sort(key=lambda m: m["log_time_ns"])
    return dict(result)


def _cap_timeline(items: list, limit: int = 200) -> list:
    """Cap a timeline for payload safety: keep first/last halves + omission marker.
    只截断返回给 LLM 的 payload；事件检测在此之前已基于全量数据完成。
    """
    if len(items) <= limit:
        return items
    half = limit // 2
    return (
        items[:half]
        + [{"note": f"... {len(items) - limit} messages omitted (middle of timeline) ..."}]
        + items[-half:]
    )


class _BoundedList:
    """首 head 条 + 尾 tail 条的有界缓冲。

    长录制下高频话题（如新版 AEB 反馈）的 timeline 若全量驻内存会随录制
    时长线性增长；此缓冲把内存固定在 head+tail 条，事件检测仍基于全量流。
    """

    def __init__(self, head: int = 50, tail: int = 50):
        self._head = head
        self._tail = tail
        self._items: List[Any] = []
        self.total = 0

    def add(self, item: Any) -> None:
        self.total += 1
        cap = self._head + self._tail
        if len(self._items) < cap:
            self._items.append(item)
        else:
            del self._items[self._head]
            self._items.append(item)

    def to_list(self) -> List[Any]:
        if self.total <= len(self._items):
            return self._items
        omitted = self.total - len(self._items)
        out = list(self._items[: self._head])
        out.append({"note": f"... {omitted} messages omitted (middle of timeline) ..."})
        out.extend(self._items[self._head:])
        return out


def _freq_analysis(
    timestamps_ns: List[int],
    expected_hz: Optional[float] = None,
) -> Dict[str, Any]:
    """Analyze frequency and gaps from a list of timestamps."""
    if len(timestamps_ns) < 2:
        return {"count": len(timestamps_ns), "note": "insufficient_data"}

    times = sorted(timestamps_ns)
    start_ns = times[0]
    end_ns = times[-1]
    duration_s = (end_ns - start_ns) / 1e9
    actual_hz = len(times) / duration_s if duration_s > 0 else 0

    intervals_s = [(times[i + 1] - times[i]) / 1e9 for i in range(len(times) - 1)]
    avg_interval_ms = (sum(intervals_s) / len(intervals_s)) * 1000 if intervals_s else 0
    max_interval_ms = max(intervals_s) * 1000 if intervals_s else 0
    min_interval_ms = min(intervals_s) * 1000 if intervals_s else 0

    # Detect gaps > 2x expected interval
    expected_interval_ms = 1000 / expected_hz if expected_hz and expected_hz > 0 else avg_interval_ms
    gap_count = sum(1 for i in intervals_s if i * 1000 > expected_interval_ms * 2)
    max_gap_ms = max(intervals_s) * 1000 if intervals_s else 0

    return {
        "count": len(times),
        "start_time": _time_str(start_ns),
        "end_time": _time_str(end_ns),
        "duration_seconds": round(duration_s, 2),
        "actual_frequency_hz": round(actual_hz, 2),
        "expected_frequency_hz": expected_hz,
        "avg_interval_ms": round(avg_interval_ms, 2),
        "max_interval_ms": round(max_interval_ms, 2),
        "min_interval_ms": round(min_interval_ms, 2),
        "large_gap_count": gap_count,
        "max_gap_ms": round(max_gap_ms, 2),
        "healthy": gap_count == 0 or expected_hz is None,
    }


def _cap_snapshot_data(data: Any, max_chars: int) -> Any:
    """Cap snapshot per-topic data size（感知话题最近消息可达上百 KB，
    会撑爆响应预算；只保留预览与截断标记）。"""
    if max_chars <= 0:
        return {"note": "data omitted (response budget reduction)"}
    s = json.dumps(data, ensure_ascii=False, default=str)
    if len(s) <= max_chars:
        return data
    last_nl = s[:max_chars].rfind("\n")
    if last_nl > max_chars // 2:
        cut = s[:last_nl]
    else:
        cut = s[:max_chars]
    return {
        "note": f"data truncated ({len(s)} chars total, >{max_chars} chars budget)",
        "preview": cut[:400],
    }


def _read_topic_timestamps(
    file_path: str, topics: List[str], ts_cap: int = 20000
) -> Dict[str, List[int]]:
    """Read only timestamps (log_time) for given topics (no decoding, fast).
    使用 log_time（MCAP 记录时间）而非 publish_time，确保时间线准确。
    每话题封顶 ts_cap 条（20000 条 ≈ 50Hz × 400s，足够频率/间隙分析，
    同时防止超长录制撑爆内存）。全文件级 gap 检测请用 detect_anomalies_local。
    """
    reader, fp = _open_mcap(file_path)
    try:
        scan = _scan_mcap(reader, watch_topics=topics, ts_cap=ts_cap, topics=topics)
        return scan["timestamps"]
    finally:
        fp.close()


def detect_anomalies_local(args: Dict) -> Dict:
    file_path = _require_file_path(args)

    file_size = os.path.getsize(file_path)
    if file_size > 500 * 1024 * 1024:
        log.warning("Large file detected for anomaly detection: %s (%.0f MB)", file_path, file_size / 1024 / 1024)

    gap_threshold_s = args.get("gap_threshold_s", 1.0)
    min_frequency_hz = args.get("min_frequency_hz", 0.1)

    # 流式单遍扫描：统计 + gap 检测一次完成（旧版整文件读内存且遍历两遍）
    reader, fp = _open_mcap(file_path)
    try:
        scan = _scan_mcap(reader, gap_threshold_s=gap_threshold_s)
    finally:
        fp.close()

    stats = _format_stats(scan)
    anomalies = _build_anomalies(scan, gap_threshold_s, min_frequency_hz)

    return {
        "content": [
            {
                "type": "text",
                "text": json.dumps(
                    {
                        "file_path": file_path,
                        "file_size_mb": round(file_size / (1024 * 1024), 2),
                        "statistics": stats,
                        "anomalies": anomalies,
                        "total_anomalies": len(anomalies),
                        "anomalies_by_severity": {
                            severity: len(
                                [a for a in anomalies if a.get("severity") == severity]
                            )
                            for severity in ["error", "warning", "info"]
                        },
                    },
                    ensure_ascii=False,
                    indent=2,
                    default=str,
                ),
            }
        ],
    }

