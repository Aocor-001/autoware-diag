#!/usr/bin/env python3
"""
Autoware Diagnostic MCP Server
用于排查自动驾驶车辆的感知、规划控制、MRM/AEB 事件及底盘问题。

Usage:
  python3 server.py

MCP 协议: JSON-RPC 2.0 over stdio (2025-03-26)
"""

import datetime as dt_mod
import json
import logging
import os
import sys
import traceback
from collections import Counter, defaultdict

from dateutil import parser as dateutil_parser
from typing import Any, Callable, Dict, List, Optional

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.WARNING,
    format="[autoware-diag] %(levelname)s %(message)s",
    stream=sys.stderr,
)
log = logging.getLogger("autoware-diag")

# ---------------------------------------------------------------------------
# MCAP + ROS2 support
# ---------------------------------------------------------------------------

HAVE_MCAP = False
try:
    from mcap.reader import make_reader
    from mcap.decoder import DecoderFactory as McapDecoderFactory
    from mcap.records import Schema as McapSchema
    HAVE_MCAP = True
except ImportError:
    pass

HAVE_ROS2 = False
try:
    from mcap_ros2.decoder import generate_dynamic

    HAVE_ROS2 = True
except ImportError:
    pass


class Ros2DecoderFactory(McapDecoderFactory):
    """Decode ROS2 CDR-encoded MCAP messages using mcap-ros2-support."""

    def __init__(self):
        self._decoders: Dict[str, Dict[str, Callable]] = {}

    def decoder_for(self, message_encoding: str, schema: Optional[McapSchema]):
        if message_encoding != "cdr":
            return None
        if not schema or not schema.data:
            return None
        schema_name = schema.name
        schema_text = schema.data.decode("utf-8")

        if schema_name not in self._decoders:
            try:
                decoders = generate_dynamic(schema_name, schema_text)
                self._decoders[schema_name] = decoders
            except Exception:
                return None

        decoders = self._decoders.get(schema_name, {})
        return decoders.get(schema_name)


def _open_mcap(file_path: str):
    """Open an MCAP file and return (reader, file_object).

    调用方负责关闭 file_object（用 try/finally），避免句柄泄漏。
    文件以流式方式读取，不会整文件加载进内存。
    """
    if not HAVE_MCAP:
        raise McpError(INTERNAL_ERROR, "mcap library not installed (pip install mcap)")
    if not os.path.isfile(file_path):
        raise McpError(INVALID_PARAMS, f"File not found: {file_path}")
    file_size = os.path.getsize(file_path)
    if file_size == 0:
        raise McpError(INVALID_PARAMS, f"File is empty: {file_path}")
    f = None
    try:
        f = open(file_path, "rb")
        decoder_factories = [Ros2DecoderFactory()] if HAVE_ROS2 else []
        reader = make_reader(f, decoder_factories=decoder_factories)
        return reader, f
    except McpError:
        if f is not None:
            f.close()
        raise
    except Exception as e:
        if f is not None:
            f.close()
        log.error("Failed to open MCAP file %s: %s", file_path, traceback.format_exc())
        raise McpError(INTERNAL_ERROR, f"Failed to read MCAP file: {e}")


def _require_file_path(args: Dict) -> str:
    """Validate and return the file_path argument (clean error instead of KeyError)."""
    fp = args.get("file_path")
    if not fp or not isinstance(fp, str):
        raise McpError(INVALID_PARAMS, "Missing required parameter: file_path (string)")
    if not os.path.isfile(fp):
        raise McpError(INVALID_PARAMS, f"File not found: {fp}")
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


# ---------------------------------------------------------------------------
# MCAP analysis helpers
# ---------------------------------------------------------------------------


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


# ---------------------------------------------------------------------------
# Bag structure / rosout / diagnostics helpers
# ---------------------------------------------------------------------------

_ROSOUT_LEVEL_LABELS = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}

# 话题顶层命名空间 → 模块语义（用于 overview 分组，未知前缀进 other）
_NS_MODULE_HINTS = {
    "sensing": "sensing",
    "perception": "perception",
    "planning": "planning",
    "control": "control",
    "localization": "localization",
    "map": "map",
    "system": "system",
    "vehicle": "vehicle",
    "api": "api",
    "external": "external",
    "pix": "vehicle_interface",
    "pix_robobus": "vehicle_interface",
    "autoware": "autoware_core",
    "electronic_rearview_mirror": "camera",
    "diagnostics": "diagnostics",
    "rosout": "logging",
    "tf": "tf",
    "tf_static_relay": "tf",
}


def _read_bag_metadata(file_path: str, reader=None) -> List[Dict[str, Any]]:
    """Read MCAP Metadata records (rosbag2 ROS_DISTRO 等)。

    优先用 mcap reader.iter_metadata()；失败时回退到 Footer→summary 手工扫描。
    map 编码是「字节长度前缀」而非条目数。
    """
    import struct

    # mcap.Opcode: METADATA=12(0x0C), FOOTER=2(0x02)
    OP_METADATA = 0x0C
    OP_FOOTER = 0x02
    MAGIC = b"\x89MCAP0\r\n"

    def _parse_metadata_body(body: bytes) -> Dict[str, Any]:
        off = 0
        nlen = struct.unpack_from("<I", body, off)[0]
        off += 4
        name = body[off : off + nlen].decode("utf-8", "replace")
        off += nlen
        meta_len = struct.unpack_from("<I", body, off)[0]
        off += 4
        meta_end = off + meta_len
        fields: Dict[str, str] = {}
        while off < meta_end and off + 4 <= len(body):
            klen = struct.unpack_from("<I", body, off)[0]
            off += 4
            if off + klen > len(body):
                break
            k = body[off : off + klen].decode("utf-8", "replace")
            off += klen
            if off + 4 > len(body):
                break
            vlen = struct.unpack_from("<I", body, off)[0]
            off += 4
            if off + vlen > len(body):
                break
            v = body[off : off + vlen].decode("utf-8", "replace")
            off += vlen
            fields[k] = v[:2000]
        return {"name": name, "metadata": fields}

    # Path 1: official iter_metadata
    try:
        out: List[Dict[str, Any]] = []
        r = reader
        fp = None
        if r is None:
            r, fp = _open_mcap(file_path)
        try:
            for metadata in r.iter_metadata():
                out.append(
                    {
                        "name": getattr(metadata, "name", None),
                        "metadata": dict(getattr(metadata, "metadata", {}) or {}),
                    }
                )
        finally:
            if fp is not None:
                fp.close()
        if out:
            return out
    except Exception as e:
        log.debug("iter_metadata failed, fallback to footer scan: %s", e)

    # Path 2: Footer → summary scan
    try:
        size = os.path.getsize(file_path)
        with open(file_path, "rb") as f:
            if size < 37:
                return [{"error": "file too small"}]
            f.seek(size - 8)
            if f.read(8) != MAGIC:
                return [{"error": "missing MCAP magic at EOF"}]
            # Footer record immediately before magic: 9 + 20 body
            f.seek(size - 8 - 9 - 20)
            header = f.read(9)
            if len(header) < 9 or header[0] != OP_FOOTER:
                return [{"error": "footer record not found at expected offset"}]
            length = struct.unpack("<Q", header[1:9])[0]
            body = f.read(min(length, 20))
            summary_start = struct.unpack_from("<Q", body, 0)[0]
            start = summary_start if 0 < summary_start < size else max(0, size - 2 * 1024 * 1024)
            f.seek(start)
            scanned = 0
            out = []
            while f.tell() < size - 8 and scanned < 200000:
                pos = f.tell()
                header = f.read(9)
                if len(header) < 9:
                    break
                opcode = header[0]
                length = struct.unpack("<Q", header[1:9])[0]
                if length > 100 * 1024 * 1024:
                    break
                body = f.read(length)
                scanned += 1
                if opcode == OP_METADATA:
                    try:
                        out.append(_parse_metadata_body(body))
                    except Exception as e:
                        out.append(
                            {"name": f"@offset_{pos}", "error": f"parse failed: {e}"}
                        )
                if opcode == OP_FOOTER:
                    break
            return out
    except Exception as e:
        return [{"error": f"metadata read failed: {e}"}]


def _classify_topics_by_namespace(topic_details: Dict[str, Dict]) -> Dict[str, Any]:
    """Group topic stats by top-level namespace and semantic module."""
    ns_stats: Dict[str, Dict[str, Any]] = {}
    module_stats: Dict[str, Dict[str, Any]] = {}
    zero_message: List[str] = []
    for topic, info in topic_details.items():
        parts = topic.strip("/").split("/")
        ns = parts[0] if parts and parts[0] else topic
        mod = _NS_MODULE_HINTS.get(ns, "other")
        for bucket, key in ((ns_stats, ns), (module_stats, mod)):
            b = bucket.setdefault(
                key, {"topic_count": 0, "message_count": 0, "topics": []}
            )
            b["topic_count"] += 1
            b["message_count"] += info.get("message_count", 0)
            b["topics"].append(
                {
                    "topic": topic,
                    "message_count": info.get("message_count", 0),
                    "avg_frequency_hz": info.get("avg_frequency_hz"),
                    "schema": info.get("schema"),
                }
            )
        if info.get("message_count", 0) == 0:
            zero_message.append(topic)
    for bucket in (ns_stats, module_stats):
        for b in bucket.values():
            b["topics"].sort(key=lambda x: -x.get("message_count", 0))
    return {
        "namespaces": dict(sorted(ns_stats.items(), key=lambda kv: -kv[1]["topic_count"])),
        "modules": dict(sorted(module_stats.items(), key=lambda kv: kv[0])),
        "zero_message_topics": sorted(zero_message),
    }


def _service_like_topics(topics: List[str]) -> List[str]:
    """Detect service-related topics (rosbag2 通常不录 service call，多为空)."""
    keys = ("/_service", "/service", "srv", "/event")
    out = []
    for t in topics:
        low = t.lower()
        if any(k in low for k in keys):
            out.append(t)
    return sorted(out)


def _infer_nodes(
    rosout_nodes: Counter,
    diag_hardware_ids: Counter,
    topic_details: Dict[str, Dict],
) -> Dict[str, Any]:
    """Infer graph nodes from rosout names, diagnostic hardware_ids, topic prefixes.

    MCAP/rosbag2 默认不写 node 图；这是尽力推断，不是权威 ROS graph。
    """
    sources: Dict[str, Dict[str, List[str]]] = {
        "rosout": {},
        "diagnostics": {},
        "topic_prefix": {},
    }
    for name, cnt in rosout_nodes.most_common():
        if name:
            sources["rosout"][name] = [f"rosout_messages={cnt}"]
    for hw, cnt in diag_hardware_ids.most_common():
        if hw and hw not in ("none",):
            sources["diagnostics"][hw] = [f"diagnostic_status_samples={cnt}"]

    # 话题前缀启发式：/a/b/c 的前 2 段常对应节点/组件
    prefix_counts: Counter = Counter()
    for topic, info in topic_details.items():
        parts = topic.strip("/").split("/")
        if len(parts) >= 2 and parts[0] in _NS_MODULE_HINTS:
            # 跳过纯传感器/中间件话题
            if parts[0] in ("tf", "rosout", "diagnostics", "electronic_rearview_mirror"):
                continue
            prefix = "/" + "/".join(parts[:2])
            prefix_counts[prefix] += info.get("message_count", 0)
    for prefix, cnt in prefix_counts.most_common(40):
        sources["topic_prefix"][prefix] = [f"implied_message_count={cnt}"]

    all_names = sorted(
        set(sources["rosout"]) | set(sources["diagnostics"]) | set(sources["topic_prefix"])
    )
    return {
        "note": (
            "Nodes inferred from /rosout names, /diagnostics hardware_id, and topic "
            "prefixes — rosbag2/MCAP does not store the ROS graph authoritatively."
        ),
        "inferred_node_count": len(all_names),
        "by_source": {
            "rosout_nodes": sorted(sources["rosout"].keys()),
            "diagnostics_hardware_ids": sorted(sources["diagnostics"].keys()),
            "topic_prefixes": sorted(sources["topic_prefix"].keys()),
        },
        "details": {
            name: {
                "rosout": sources["rosout"].get(name),
                "diagnostics": sources["diagnostics"].get(name),
                "topic_prefix": sources["topic_prefix"].get(name),
            }
            for name in all_names
        },
    }


def _analyze_rosout_stream(
    file_path: str,
    max_events: int = 200,
    min_level: int = 30,
) -> Dict[str, Any]:
    """Full-stream /rosout analysis: node histogram + WARN/ERROR/FATAL events."""
    if not HAVE_ROS2:
        return {
            "error": "ROS2 decoding disabled (mcap-ros2-support not installed)",
            "ros2_decoding": _decoding_status(),
        }
    nodes: Counter = Counter()
    levels: Counter = Counter()
    events: List[Dict[str, Any]] = []
    total = 0
    suppressed = 0
    reader, fp = _open_mcap(file_path)
    try:
        for schema, channel, msg, decoded in reader.iter_decoded_messages(
            topics=["/rosout"], log_time_order=True
        ):
            total += 1
            if decoded is None:
                continue
            name = _val(decoded, "name") or "?"
            level = _val(decoded, "level")
            try:
                level = int(level) if level is not None else 0
            except Exception:
                level = 0
            msg_text = str(_val(decoded, "msg") or "")
            nodes[name] += 1
            levels[_ROSOUT_LEVEL_LABELS.get(level, f"LEVEL{level}")] += 1
            if level >= min_level:
                if len(events) < max_events:
                    events.append(
                        {
                            "time": _time_str(msg.log_time),
                            "log_time_ns": msg.log_time,
                            "level": _ROSOUT_LEVEL_LABELS.get(level, level),
                            "node": name,
                            "message": msg_text[:400],
                            "function": _val(decoded, "function"),
                            "line": _val(decoded, "line"),
                        }
                    )
                else:
                    suppressed += 1
    finally:
        fp.close()
    return {
        "file_path": file_path,
        "topic": "/rosout",
        "ros2_decoding": _decoding_status(),
        "total_messages": total,
        "level_histogram": dict(levels),
        "nodes": [
            {"name": n, "message_count": c} for n, c in nodes.most_common()
        ],
        "min_level_filter": min_level,
        "events": events,
        "events_suppressed": suppressed,
        "has_rosout": total > 0,
    }


def _analyze_diagnostics_stream(
    file_path: str,
    max_status_details: int = 100,
) -> Dict[str, Any]:
    """Full-stream /diagnostics: hardware_ids, status names, non-OK aggregation."""
    if not HAVE_ROS2:
        return {
            "error": "ROS2 decoding disabled (mcap-ros2-support not installed)",
            "ros2_decoding": _decoding_status(),
        }
    hw_ids: Counter = Counter()
    status_names: Counter = Counter()
    level_hist: Counter = Counter()
    # (name, level) -> {count, first_ns, last_ns, message, hardware_id}
    non_ok: Dict[tuple, Dict[str, Any]] = {}
    total_msgs = 0
    total_status = 0
    sample_keys: set = set()
    reader, fp = _open_mcap(file_path)
    try:
        for schema, channel, msg, decoded in reader.iter_decoded_messages(
            topics=["/diagnostics"], log_time_order=True
        ):
            total_msgs += 1
            statuses = _val(decoded, "status") or []
            try:
                statuses = list(statuses)
            except Exception:
                statuses = []
            for st in statuses:
                total_status += 1
                name = _val(st, "name") or "?"
                hw = _val(st, "hardware_id") or ""
                try:
                    level = int(_val(st, "level") or 0)
                except Exception:
                    level = 0
                message = str(_val(st, "message") or "")[:200]
                hw_ids[hw] += 1
                status_names[name] += 1
                level_hist[_DIAG_LEVEL_LABELS.get(level, f"LEVEL{level}")] += 1
                if level:
                    key = (name, level)
                    e = non_ok.get(key)
                    if e is None:
                        non_ok[key] = {
                            "name": name,
                            "level": level,
                            "level_label": _DIAG_LEVEL_LABELS.get(level, str(level)),
                            "hardware_id": hw,
                            "message": message,
                            "count": 1,
                            "first_time": _time_str(msg.log_time),
                            "last_time": _time_str(msg.log_time),
                            "first_log_time_ns": msg.log_time,
                            "last_log_time_ns": msg.log_time,
                        }
                    else:
                        e["count"] += 1
                        e["last_time"] = _time_str(msg.log_time)
                        e["last_log_time_ns"] = msg.log_time
                # sample diagnostic keys for content discovery
                if len(sample_keys) < 80:
                    for kv in list(_val(st, "values") or [])[:5]:
                        k = _val(kv, "key")
                        if k:
                            sample_keys.add(str(k))
    finally:
        fp.close()

    non_ok_list = sorted(
        non_ok.values(),
        key=lambda x: (-x["level"], -x["count"], x["name"]),
    )
    return {
        "file_path": file_path,
        "topic": "/diagnostics",
        "ros2_decoding": _decoding_status(),
        "message_count": total_msgs,
        "status_count": total_status,
        "level_histogram": dict(level_hist),
        "unique_status_names": len(status_names),
        "hardware_ids": [
            {"hardware_id": hw or "(empty)", "count": c}
            for hw, c in hw_ids.most_common(40)
        ],
        "top_status_names": [
            {"name": n, "count": c} for n, c in status_names.most_common(30)
        ],
        "non_ok_statuses": non_ok_list[:max_status_details],
        "non_ok_total": len(non_ok_list),
        "sample_keys": sorted(sample_keys),
        "has_diagnostics": total_msgs > 0,
    }


# ---------------------------------------------------------------------------
# Autoware-specific diagnostic helpers
# ---------------------------------------------------------------------------

# Key topics for each diagnostic area
TOPIC_PERCEPTION = [
    "/perception/object_recognition/detection/bevfusion/objects",
    "/perception/object_recognition/tracking/objects",
    "/perception/object_recognition/objects",
    "/sensing/lidar/concatenated/pointcloud_cloudini",
]

TOPIC_PLANNING_CONTROL = [
    "/control/command/control_cmd",
    "/control/command/actuation_cmd",
    "/control/trajectory_follower/control_cmd",
    "/control/trajectory_follower/lateral/diagnostic",
    "/control/trajectory_follower/longitudinal/diagnostic",
    "/planning/scenario_planning/scenario",
    "/control/current_gate_mode",
    "/control/command/emergency_cmd",
    "/control/command/hazard_lights_cmd",
    "/control/command/turn_indicators_cmd",
    "/control/command/gear_cmd",
    "/control/shift_decider/gear_cmd",
    "/planning/planning_validator/validation_status",
    "/autoware/state",
    "/api/autoware/get/engage",
    "/api/autoware/get/emergency",
    "/system/operation_mode/state",
]

TOPIC_MRM = [
    "/control/command/emergency_cmd",
    "/api/autoware/get/emergency",
    "/api/autoware/get/engage",
    "/autoware/engage",
    "/control/current_gate_mode",
    "/vehicle/status/control_mode",
]

# AEB/FCW 反馈话题 — 新旧录音兼容：
#   新版：/pix/AebInfoShadow（aeb_interfaces/msg/AebInfoShadow），话题名存在
#         Info/lnfo 两种拼写，运行时探测实际存在的那个。
#   旧版：/pix_robobus/av_aeb_ctrl（pix_robobus_driver_msgs/msg/AvAebCtrl）。
TOPIC_AEB_SHADOW = [
    "/pix/AebInfoShadow",
    "/pix/AeblnfoShadow",
]

TOPIC_AEB = [
    "/pix_robobus/av_aeb_ctrl",
    "/control/autonomous_emergency_braking/debug/markers",
    "/control/autonomous_emergency_braking/debug/rss_distance",
    "/control/command/emergency_cmd",
] + TOPIC_AEB_SHADOW

# AEB 反馈话题组：新旧任一存在即视为存在（替代关系，避免互相误报缺失）
AEB_FEEDBACK_GROUP = ["/pix_robobus/av_aeb_ctrl"] + TOPIC_AEB_SHADOW

# AebInfoShadow 字段语义标签（对照 aeb_interfaces/msg/AebInfoShadow.msg 官方注释）
# aeb_status/fcw_status — 功能状态: 0=OFF(关闭/无效) 1=ON(开启/激活) 2=FAULT(错误)
_AEB_STATUS_LABELS = {0: "OFF", 1: "ON", 2: "FAULT"}
# *_req_no_filter/*_req_filter — 请求状态: 0=无请求 1=请求
#   no_filter=滤波之前的原始请求, filter=滤波之后的生效请求
_AEB_REQ_LABELS = {0: "NO_REQ", 1: "REQ"}
# *_sensitivity_auto/local — 灵敏度: 1=低 2=中 3=高
#   auto=自动驾驶模式, local=手动驾驶模式
_AEB_SENSITIVITY_LABELS = {1: "low", 2: "mid", 3: "high"}

TOPIC_LOCALIZATION = [
    "/localization/kinematic_state",
    "/localization/acceleration",
    "/localization/twist_estimator/twist_with_covariance",
    "/localization/twist_estimator/gyro_twist",
    "/sensing/vehicle_velocity_converter/twist_with_covariance",
    "/sensing/gnss/fix",
    "/sensing/gnss/imu",
    "/sensing/gnss/heading",
    "/autoware_orientation",
]

TOPIC_CHASSIS = [
    "/pix_robobus/av_drive_ctrl",
    "/pix_robobus/av_brake_ctrl",
    "/pix_robobus/va_drive_sta_fb",
    "/pix_robobus/va_brake_sta_fb",
    "/pix_robobus/va_steer_sta_fb",
    "/pix_robobus/va_chassis_wheel_rpm_fb",
    "/pix_robobus/av_aeb_ctrl",
    "/vehicle/status/actuation_status",
    "/vehicle/status/velocity_status",
    "/vehicle/status/steering_status",
    "/control/command/actuation_cmd",
    "/control/command/control_cmd",
] + TOPIC_AEB_SHADOW

# 规划轨迹话题 — investigate_incident 排查「规划轨迹消失/异常」时使用。
# 轨迹消息体大（数百个点），采集时用 _reduce_trajectory 精简。
TOPIC_PLANNING_TRAJECTORY = [
    "/planning/scenario_planning/trajectory",
    "/planning/scenario_planning/lane_driving/trajectory",
    "/planning/scenario_planning/scenario_selector/trajectory",
    "/planning/scenario_planning/velocity_smoother/trajectory",
    "/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory",
    "/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory",
    "/planning/scenario_planning/lane_driving/motion_planning/elastic_band_smoother/output/traj",
]

TOPIC_ALL_HEALTH = list(
    set(
        TOPIC_PERCEPTION
        + TOPIC_PLANNING_CONTROL
        + TOPIC_MRM
        + TOPIC_AEB
        + TOPIC_LOCALIZATION
        + TOPIC_CHASSIS
        + [
            "/diagnostics",
            "/tf",
            "/vehicle/status/velocity_status",
            "/vehicle/status/steering_status",
        ]
    )
)


def _val(d: Any, *names: str) -> Any:
    """Tolerant field read — 兼容解码对象与 dictify 后的 dict，字段缺失返回 None."""
    if d is None:
        return None
    if isinstance(d, dict):
        for n in names:
            v = d.get(n)
            if v is not None:
                return v
        return None
    for n in names:
        v = getattr(d, n, None)
        if v is not None:
            return v
    return None


def _parse_aeb_shadow(d: Any) -> Dict[str, Any]:
    """Normalize an AebInfoShadow message into labeled fields.

    字段语义（官方注释）：status=功能状态(off/on/fault)，与请求状态相互独立；
    req_no_filter=滤波前原始请求，req_filter=滤波后生效请求；
    sensitivity 1/2/3=低/中/高（auto=自动驾驶模式，local=手动驾驶模式）。
    字段缺失/未知时返回 None，事件检测基于能读到的字段，不会因个别字段
    缺失而崩溃。
    """
    aeb_status = _val(d, "aeb_status")
    fcw_status = _val(d, "fcw_status")
    return {
        "aeb_status": aeb_status,
        "aeb_status_label": _AEB_STATUS_LABELS.get(aeb_status, "?"),
        "fcw_status": fcw_status,
        "fcw_status_label": _AEB_STATUS_LABELS.get(fcw_status, "?"),
        "aeb_req_filter": _val(d, "aeb_req_filter"),
        "fcw_req_filter": _val(d, "fcw_req_filter"),
        "aeb_req_no_filter": _val(d, "aeb_req_no_filter"),
        "fcw_req_no_filter": _val(d, "fcw_req_no_filter"),
        "aeb_sensitivity_auto": _val(d, "aeb_sensitivity_auto"),
        "aeb_sensitivity_local": _val(d, "aeb_sensitivity_local"),
        "fcw_sensitivity_auto": _val(d, "fcw_sensitivity_auto"),
        "fcw_sensitivity_local": _val(d, "fcw_sensitivity_local"),
    }


def _extract_engage(d: Any) -> Optional[bool]:
    """Unified engage extraction（对象或 dict；兼容包装 dict 与 int/str 形式）。"""
    val = _val(d, "engage", "engage_flag")
    if isinstance(val, dict):
        val = val.get("data", val.get("engage"))
    if isinstance(val, (int, str)) and not isinstance(val, bool):
        val = str(val) not in ("0", "False", "false")
    return val


def _group_has_data(topic_health: Dict[str, Dict], group: List[str]) -> bool:
    """组内任一话题有消息即视为组存在（新旧话题替代关系）。"""
    return any(topic_health.get(t, {}).get("count", 0) > 0 for t in group)


# ---------------------------------------------------------------------------
# 自定义消息字段语义（相对原版 Autoware 的差异 / Pix Robobus 私有接口）
# ---------------------------------------------------------------------------

_GATE_MODE_LABELS = {0: "AUTO", 1: "EXTERNAL"}
_GEAR_CMD_LABELS = {
    0: "NONE", 1: "NEUTRAL", 2: "DRIVE", 20: "REVERSE", 22: "PARK", 23: "LOW"
}
_CONTROL_MODE_LABELS = {
    0: "NO_COMMAND", 1: "AUTONOMOUS", 2: "AUTONOMOUS_STEER_ONLY",
    3: "AUTONOMOUS_VELOCITY_ONLY", 4: "MANUAL", 5: "DISENGAGED", 6: "NOT_READY",
}
_AUTOWARE_STATE_LABELS = {
    1: "INITIALIZING", 2: "WAITING_FOR_ROUTE", 3: "PLANNING",
    4: "WAITING_FOR_ENGAGE", 5: "DRIVING", 6: "ARRIVED_GOAL", 7: "FINALIZING",
}
_OPERATION_MODE_LABELS = {
    0: "UNKNOWN", 1: "STOP", 2: "AUTONOMOUS", 3: "LOCAL", 4: "REMOTE",
}
_AEB_OBSTACLE_SOURCE_LABELS = {
    0: "POINTCLOUD", 1: "PREDICTED_OBJECT", 2: "ASSOCIATED", 3: "UNKNOWN",
}

# 字段级说明：name → {upstream_diff, fields, notes}
CUSTOM_MESSAGE_CATALOG: Dict[str, Dict[str, Any]] = {
    "aeb_interfaces/msg/AebInfoShadow": {
        "origin": "Pix 私有（原版 Autoware 无此消息）",
        "upstream_diff": "非 Autoware upstream；AEB/FCW 影子状态总线",
        "fields": {
            "aeb_status": "0=OFF 1=ON 2=FAULT",
            "fcw_status": "0=OFF 1=ON 2=FAULT",
            "aeb_req_no_filter": "滤波前 AEB 请求 0/1",
            "fcw_req_no_filter": "滤波前 FCW 请求 0/1",
            "aeb_req_filter": "滤波后 AEB 请求（生效）0/1",
            "fcw_req_filter": "滤波后 FCW 请求（生效）0/1",
            "aeb_sensitivity_auto/local": "灵敏度 1=低 2=中 3=高",
            "fcw_sensitivity_auto/local": "灵敏度 1=低 2=中 3=高",
        },
        "notes": "no_filter=1 且 filter=0 → REQUEST_FILTERED（该触发未触发）",
    },
    "aeb_interfaces/msg/AebObstacleDebugData": {
        "origin": "Pix 私有",
        "fields": {
            "closest_object_distance": "m",
            "closest_object_velocity": "m/s",
            "closest_object_source": "0=POINTCLOUD 1=PREDICTED 2=ASSOCIATED 3=UNKNOWN",
            "associated_object_count": "",
            "suppressed_pointcloud_object_count": "点云目标被抑制数量",
        },
    },
    "pix_robobus_driver_msgs/msg/AvAebCtrl": {
        "origin": "Pix Robobus CAN 上行（旧版 AEB 反馈）",
        "fields": {
            "acu_aeb_work_sta": "0..2 工作状态",
            "acu_aeb_active_request": "bool",
            "acu_fcw_active_request": "bool",
            "acu_aeb_brake_en": "bool",
            "acu_aeb_brake_pedal_target": "% 0-100",
        },
    },
    "pix_robobus_driver_msgs/msg/AvDriveCtrl": {
        "origin": "Pix Robobus 自动驾驶驱动指令（ACU→VCU）",
        "fields": {
            "acu_chassis_driver_en_ctrl": "bool 使能",
            "acu_chassis_driver_mode_ctrl": "0..3",
            "acu_chassis_gear_ctrl": "0..3 挡位",
            "acu_chassis_speed_ctrl": "m/s 0-50",
            "acu_chassis_throttle_pdl_target": "% 0-100",
            "acu_drive_life_sig": "生命信号 0-15（周期跳变，停更=通讯异常）",
        },
    },
    "pix_robobus_driver_msgs/msg/AvBrakeCtrl": {
        "origin": "Pix Robobus 制动指令（ACU→VCU）",
        "fields": {
            "acu_chassis_brake_en": "bool",
            "acu_chassis_aeb_ctrl": "bool AEB 使能透传",
            "acu_chassis_brake_pdl_target": "% 0-100",
            "acu_chassis_epb_ctrl": "0..2",
            "acu_brake_life_sig": "生命信号 0-15",
        },
    },
    "pix_robobus_driver_msgs/msg/VaDriveStaFb": {
        "origin": "Pix Robobus 驱动反馈（VCU→）",
        "fields": {
            "vcu_chassis_driver_en_sta": "bool",
            "vcu_chassis_diver_slopover": "bool 溜坡/接管相关",
            "vcu_chassis_gear_fb": "0..3 实际挡位",
            "vcu_chassis_speed_fb": "m/s 实际车速",
            "vcu_chassis_throttle_pald_fb": "%（注意拼写 pald）",
            "vcu_chassis_acceleration_fb": "m/s²",
            "vcu_drive_life": "生命信号",
        },
    },
    "pix_robobus_driver_msgs/msg/VaBrakeStaFb": {
        "origin": "Pix Robobus 制动反馈",
        "fields": {
            "vcu_chassis_brake_en_sta": "bool",
            "vcu_chassis_epb_fb": "0..3 EPB",
            "vcu_chassis_brake_communication_lost": "bool CAN 丢帧标志",
            "vcu_chassis_brake_padl_fb": "% 踏板",
            "vcu_aeb_en_sta_fb": "bool AEB 使能反馈",
            "vcu_aeb_trigger_sta_fb": "bool AEB 触发反馈",
            "vcu_chassis_brake_pressure_fb": "bar",
            "vcu_brake_life": "生命信号",
        },
    },
    "pix_robobus_driver_msgs/msg/VaSteerStaFb": {
        "origin": "Pix Robobus 转向反馈 — 单位是度(°)，不是 Autoware 的 rad",
        "upstream_diff": "字段名/单位与 autoware_vehicle_msgs/SteeringReport (rad) 不同",
        "fields": {
            "vcu_chassis_steer_en_sta": "bool",
            "vcu_chassis_steer_work_mode": "0..2",
            "vcu_chassis_steer_mode_fb": "0..4",
            "vcu_chassis_steer_communication_lost": "bool",
            "vcu_chassis_steer_angle_fb": "deg 前轮，int，范围 ±500",
            "vcu_chassis_steer_angle_rear_fb": "deg 后轮",
            "vcu_chassis_steer_angle_speed_fb": "deg/s",
        },
    },
    "pix_robobus_driver_msgs/msg/VaChassisWheelRpmFb": {
        "origin": "Pix Robobus 四轮轮速",
        "fields": {
            "vcu_chassis_wheel_rpm_lf/rf/lr/rr": "rpm ±1000",
        },
    },
    "autoware_control_msgs/msg/Control": {
        "origin": "Autoware upstream（嵌套 lateral/longitudinal）",
        "upstream_diff": "旧代码若读 top-level steering 会取空；真实字段在 lateral.steering_tire_angle",
        "fields": {
            "lateral.steering_tire_angle": "rad，左正右负",
            "lateral.steering_tire_rotation_rate": "rad/s",
            "longitudinal.velocity": "m/s",
            "longitudinal.acceleration": "m/s²",
            "longitudinal.jerk": "m/s³",
        },
    },
    "tier4_control_msgs/msg/GateMode": {
        "origin": "tier4",
        "fields": {"data": "0=AUTO 1=EXTERNAL"},
    },
    "autoware_internal_planning_msgs/msg/Scenario": {
        "origin": "Autoware internal",
        "fields": {
            "current_scenario": "Empty | LaneDriving | Parking",
            "activating_scenarios": "string[]",
        },
    },
    "point_cloud_interfaces/msg/CompressedPointCloud2": {
        "origin": "CTU CloudCompress（非 sensor_msgs/PointCloud2）",
        "upstream_diff": "点云是压缩体，format 如 cloudini；不能当 XYZ 直接读",
        "fields": {
            "format": "压缩格式标识",
            "compressed_data": "uint8[]",
            "point_step/fields": "解压后布局",
        },
    },
    "autoware_planning_validator/msg/PlanningValidatorStatus": {
        "origin": "Autoware planning_validator",
        "fields": {
            "is_valid_*": "一串布尔校验；任一 false 对应诊断 ERROR",
            "invalid_count": "累计无效次数",
            "max_curvature / velocity_deviation 等": "数值度量",
        },
    },
    "autoware_adapi_v1_msgs/msg/OperationModeState": {
        "origin": "AD API v1",
        "fields": {
            "mode": "0=UNKNOWN 1=STOP 2=AUTONOMOUS 3=LOCAL 4=REMOTE",
            "is_autoware_control_enabled": "bool",
        },
    },
    "tier4_vehicle_msgs/msg/ActuationCommandStamped": {
        "origin": "tier4",
        "fields": {
            "actuation.accel_cmd": "",
            "actuation.brake_cmd": "",
            "actuation.steer_cmd": "",
        },
    },
    "tier4_vehicle_msgs/msg/VehicleEmergencyStamped": {
        "origin": "tier4（与 autoware_adapi Emergency 同形）",
        "fields": {"emergency": "bool"},
    },
}


def _extract_control_summary(d: Any) -> Optional[Dict[str, Any]]:
    """Extract nested Control fields (lateral/longitudinal). 兼容 dictify 结果."""
    if d is None:
        return None
    dd = d if isinstance(d, dict) else _dictify(d, max_depth=4)
    if not isinstance(dd, dict):
        return None
    lat = dd.get("lateral") if isinstance(dd.get("lateral"), dict) else {}
    lon = dd.get("longitudinal") if isinstance(dd.get("longitudinal"), dict) else {}
    return {
        "steering_tire_angle_rad": lat.get("steering_tire_angle"),
        "steering_tire_rotation_rate": lat.get("steering_tire_rotation_rate"),
        "velocity_mps": lon.get("velocity"),
        "acceleration_mps2": lon.get("acceleration"),
        "jerk_mps3": lon.get("jerk"),
    }


def _labelize(value: Any, labels: Dict[int, str]) -> Dict[str, Any]:
    """Attach a semantic label to an integer enum field."""
    out = {"value": value}
    try:
        if value is not None:
            out["label"] = labels.get(int(value), f"UNKNOWN({value})")
    except Exception:
        pass
    return out


def _parse_pix_drive_fb(d: Any) -> Optional[Dict[str, Any]]:
    dd = d if isinstance(d, dict) else _dictify(d, max_depth=3)
    if not isinstance(dd, dict):
        return None
    return {
        "driver_en": _val(dd, "vcu_chassis_driver_en_sta"),
        "driver_mode": _val(dd, "vcu_chassis_driver_mode_sta"),
        "gear_fb": _labelize(_val(dd, "vcu_chassis_gear_fb"), {0: "NONE", 1: "NEUTRAL", 2: "DRIVE", 3: "REVERSE"}),
        "speed_fb_mps": _val(dd, "vcu_chassis_speed_fb"),
        "throttle_pdl_fb_pct": _val(dd, "vcu_chassis_throttle_pald_fb"),
        "acceleration_fb_mps2": _val(dd, "vcu_chassis_acceleration_fb"),
        "life": _val(dd, "vcu_drive_life"),
    }


def _parse_pix_brake_fb(d: Any) -> Optional[Dict[str, Any]]:
    dd = d if isinstance(d, dict) else _dictify(d, max_depth=3)
    if not isinstance(dd, dict):
        return None
    return {
        "brake_en": _val(dd, "vcu_chassis_brake_en_sta"),
        "epb_fb": _val(dd, "vcu_chassis_epb_fb"),
        "comm_lost": _val(dd, "vcu_chassis_brake_communication_lost"),
        "pedal_fb_pct": _val(dd, "vcu_chassis_brake_padl_fb"),
        "aeb_en_fb": _val(dd, "vcu_aeb_en_sta_fb"),
        "aeb_trigger_fb": _val(dd, "vcu_aeb_trigger_sta_fb"),
        "brake_pressure_bar": _val(dd, "vcu_chassis_brake_pressure_fb"),
        "life": _val(dd, "vcu_brake_life"),
    }


def _parse_pix_steer_fb(d: Any) -> Optional[Dict[str, Any]]:
    dd = d if isinstance(d, dict) else _dictify(d, max_depth=3)
    if not isinstance(dd, dict):
        return None
    steer_deg = _val(dd, "vcu_chassis_steer_angle_fb")
    rear_deg = _val(dd, "vcu_chassis_steer_angle_rear_fb")
    return {
        "steer_en": _val(dd, "vcu_chassis_steer_en_sta"),
        "steer_work_mode": _val(dd, "vcu_chassis_steer_work_mode"),
        "steer_mode_fb": _val(dd, "vcu_chassis_steer_mode_fb"),
        "comm_lost": _val(dd, "vcu_chassis_steer_communication_lost"),
        "steer_angle_deg": steer_deg,
        "steer_angle_rad": round(steer_deg * 3.141592653589793 / 180.0, 5)
        if isinstance(steer_deg, (int, float))
        else None,
        "rear_steer_angle_deg": rear_deg,
        "steer_angle_speed_dps": _val(dd, "vcu_chassis_steer_angle_speed_fb"),
        "life": _val(dd, "vcu_steer_life"),
    }


def _parse_pix_drive_cmd(d: Any) -> Optional[Dict[str, Any]]:
    dd = d if isinstance(d, dict) else _dictify(d, max_depth=3)
    if not isinstance(dd, dict):
        return None
    return {
        "driver_en": _val(dd, "acu_chassis_driver_en_ctrl"),
        "driver_mode": _val(dd, "acu_chassis_driver_mode_ctrl"),
        "gear_cmd": _labelize(
            _val(dd, "acu_chassis_gear_ctrl"),
            {0: "NONE", 1: "NEUTRAL", 2: "DRIVE", 3: "REVERSE"},
        ),
        "speed_cmd_mps": _val(dd, "acu_chassis_speed_ctrl"),
        "throttle_target_pct": _val(dd, "acu_chassis_throttle_pdl_target"),
        "life_sig": _val(dd, "acu_drive_life_sig"),
    }


def _parse_pix_brake_cmd(d: Any) -> Optional[Dict[str, Any]]:
    dd = d if isinstance(d, dict) else _dictify(d, max_depth=3)
    if not isinstance(dd, dict):
        return None
    return {
        "brake_en": _val(dd, "acu_chassis_brake_en"),
        "aeb_ctrl": _val(dd, "acu_chassis_aeb_ctrl"),
        "brake_pdl_target_pct": _val(dd, "acu_chassis_brake_pdl_target"),
        "epb_ctrl": _val(dd, "acu_chassis_epb_ctrl"),
        "life_sig": _val(dd, "acu_brake_life_sig"),
    }


def _parse_aeb_obstacle_debug(d: Any) -> Optional[Dict[str, Any]]:
    dd = d if isinstance(d, dict) else _dictify(d, max_depth=3)
    if not isinstance(dd, dict):
        return None

    def _num(v: Any) -> Any:
        # float32 NaN 会污染 JSON；无目标时服务端常见 NaN，转成 null
        if isinstance(v, float) and (v != v):  # NaN
            return None
        return v

    src = _val(dd, "closest_object_source")
    return {
        "closest_object_distance_m": _num(_val(dd, "closest_object_distance")),
        "closest_object_velocity_mps": _num(_val(dd, "closest_object_velocity")),
        "closest_pointcloud_velocity_mps": _num(
            _val(dd, "closest_pointcloud_object_velocity")
        ),
        "closest_predicted_velocity_mps": _num(
            _val(dd, "closest_predicted_object_velocity")
        ),
        "closest_object_source": _labelize(src, _AEB_OBSTACLE_SOURCE_LABELS),
        "associated_object_count": _val(dd, "associated_object_count"),
        "pointcloud_only_object_count": _val(dd, "pointcloud_only_object_count"),
        "predicted_only_object_count": _val(dd, "predicted_only_object_count"),
        "suppressed_pointcloud_object_count": _val(
            dd, "suppressed_pointcloud_object_count"
        ),
    }


def _extract_scenario_summary(d: Any) -> Optional[Dict[str, Any]]:
    dd = d if isinstance(d, dict) else _dictify(d, max_depth=3)
    if not isinstance(dd, dict):
        return None
    return {
        "current_scenario": _val(dd, "current_scenario"),
        "activating_scenarios": _val(dd, "activating_scenarios"),
    }


def _extract_gate_mode(d: Any) -> Optional[Dict[str, Any]]:
    dd = d if isinstance(d, dict) else _dictify(d, max_depth=3)
    if not isinstance(dd, dict):
        return None
    raw = _val(dd, "data", "gate_mode", "mode")
    return _labelize(raw, _GATE_MODE_LABELS)


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


_DIAG_LEVEL_LABELS = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}


def _reduce_diagnostics(d: Any) -> Optional[List[Dict[str, Any]]]:
    """Extract non-OK statuses from a DiagnosticArray.

    /diagnostics 频率可达数百 Hz 且每条含上百个 status，全量 dictify 会撑爆内存，
    因此只保留非 OK 的 status。全部 OK（或无法解码）时返回 None。
    """
    if d is None or not hasattr(d, "status"):
        return None
    try:
        out = []
        for st in d.status:
            level = getattr(st, "level", 0)
            if level:
                out.append(
                    {
                        "name": getattr(st, "name", "?"),
                        "level": level,
                        "level_label": _DIAG_LEVEL_LABELS.get(level, str(level)),
                        "message": str(getattr(st, "message", ""))[:200],
                        "hardware_id": getattr(st, "hardware_id", ""),
                    }
                )
        return out or None
    except Exception:
        return None


def _reduce_trajectory(d: Any) -> Any:
    """Compact summary of a Trajectory message (points 数组可能数百个，全量太大）."""
    if d is None:
        return {"note": "undecoded (mcap-ros2-support missing or decode failed)"}
    try:
        pts = getattr(d, "points", None)
        if pts is None:
            return _dictify(d, max_depth=3)
        n = len(pts)
        return {
            "num_points": n,
            "first_point": _dictify(pts[0], max_depth=4) if n else None,
            "last_point": _dictify(pts[-1], max_depth=4) if n else None,
        }
    except Exception:
        return {"note": "failed to summarize trajectory"}


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


# ---------------------------------------------------------------------------
# MCP Protocol Implementation (JSON-RPC 2.0 over stdio)
# ---------------------------------------------------------------------------

PARSE_ERROR = -32700
INVALID_REQUEST = -32600
METHOD_NOT_FOUND = -32601
INVALID_PARAMS = -32602
INTERNAL_ERROR = -32603

# 最大响应输出长度（字符数），超过此长度截断
MAX_RESPONSE_CHARS = 100_000


def _truncate_text(text: str, max_chars: int = MAX_RESPONSE_CHARS, label: str = "response") -> str:
    """Truncate text at max_chars with a notice, keeping JSON valid-ish."""
    if len(text) <= max_chars:
        return text
    cut = text[:max_chars]
    # Try to cut at last newline before limit so partial lines don't break display
    last_nl = cut.rfind("\n")
    if last_nl > max_chars // 2:
        cut = text[:last_nl]
    return (
        cut
        + f'\n\n--- [TRUNCATED: {label} exceeded {max_chars} chars '
        f'({len(text)} total); {len(text) - max_chars} chars omitted] ---\n'
    )


class McpError(Exception):
    def __init__(self, code: int, message: str, data: Any = None):
        self.code = code
        self.message = message
        self.data = data
        super().__init__(message)


class McpServer:
    """Minimal MCP server implementing JSON-RPC 2.0 over stdio."""

    def __init__(
        self,
        name: str = "autoware-diag-mcp",
        version: str = "1.0.0",
        instructions: str = "",
    ):
        self.name = name
        self.version = version
        self.instructions = instructions
        self._handlers: Dict[str, Callable] = {}
        self._initialized = False
        self._register("initialize", self._handle_initialize)
        # MCP 客户端会定期 ping 保活，必须应答空 result
        self._register("ping", lambda params: {})

    def _register(self, method: str, handler: Callable):
        self._handlers[method] = handler

    def tool(self, name: str, description: str = "", input_schema: Optional[Dict] = None):
        """Decorator to register a tool handler."""
        def decorator(func):
            self._handlers[f"tools/call:{name}"] = func
            if not hasattr(self, "_tools"):
                self._tools: List[Dict] = []
            self._tools.append(
                {
                    "name": name,
                    "description": description,
                    "inputSchema": input_schema
                    or {"type": "object", "properties": {}},
                }
            )
            return func
        return decorator

    def _handle_initialize(self, params: Dict) -> Dict:
        self._initialized = True
        return {
            "protocolVersion": "2025-03-26",
            "capabilities": {
                "tools": {
                    "listChanged": False,
                },
            },
            "serverInfo": {
                "name": self.name,
                "version": self.version,
            },
            "instructions": self.instructions,
        }

    def _send_message(self, msg: Dict):
        line = json.dumps(msg, ensure_ascii=False, default=str)
        sys.stdout.write(line + "\n")
        sys.stdout.flush()

    def _handle_request(self, request: Dict):
        msg_id = request.get("id")
        method = request.get("method", "")
        params = request.get("params", {}) or {}

        try:
            if method == "tools/list":
                tools = getattr(self, "_tools", [])
                result = {"tools": tools}
                self._send_message(
                    {
                        "jsonrpc": "2.0",
                        "id": msg_id,
                        "result": result,
                    }
                )

            elif method == "tools/call":
                tool_name = params.get("name", "")
                arguments = params.get("arguments", {})
                handler_key = f"tools/call:{tool_name}"

                if handler_key not in self._handlers:
                    self._send_message(
                        {
                            "jsonrpc": "2.0",
                            "id": msg_id,
                            "error": {
                                "code": METHOD_NOT_FOUND,
                                "message": f"Unknown tool: {tool_name}",
                            },
                        }
                    )
                    return

                try:
                    result = self._handlers[handler_key](arguments)
                    if isinstance(result, dict) and "content" in result:
                        # 对已存在的 content 做大小截断
                        content = result.get("content", [])
                        truncated_content = []
                        for item in content:
                            if isinstance(item, dict) and item.get("type") == "text":
                                item["text"] = _truncate_text(
                                    item["text"], label=f"tool {tool_name}"
                                )
                            truncated_content.append(item)
                        result["content"] = truncated_content
                        self._send_message(
                            {
                                "jsonrpc": "2.0",
                                "id": msg_id,
                                "result": result,
                            }
                        )
                    else:
                        text = json.dumps(result, ensure_ascii=False, default=str)
                        text = _truncate_text(text, label=f"tool {tool_name}")
                        self._send_message(
                            {
                                "jsonrpc": "2.0",
                                "id": msg_id,
                                "result": {
                                    "content": [
                                        {
                                            "type": "text",
                                            "text": text,
                                        }
                                    ],
                                },
                            }
                        )
                except McpError as e:
                    self._send_message(
                        {
                            "jsonrpc": "2.0",
                            "id": msg_id,
                            "result": {
                                "content": [{"type": "text", "text": e.message}],
                                "isError": True,
                            },
                        }
                    )
                except Exception as e:
                    self._send_message(
                        {
                            "jsonrpc": "2.0",
                            "id": msg_id,
                            "result": {
                                "content": [
                                    {
                                        "type": "text",
                                        "text": f"Error: {str(e)}\n{traceback.format_exc()}",
                                    }
                                ],
                                "isError": True,
                            },
                        }
                    )

            elif method in self._handlers:
                result = self._handlers[method](params)
                self._send_message(
                    {
                        "jsonrpc": "2.0",
                        "id": msg_id,
                        "result": result,
                    }
                )

            else:
                self._send_message(
                    {
                        "jsonrpc": "2.0",
                        "id": msg_id,
                        "error": {
                            "code": METHOD_NOT_FOUND,
                            "message": f"Method not found: {method}",
                        },
                    }
                )

        except McpError as e:
            self._send_message(
                {
                    "jsonrpc": "2.0",
                    "id": msg_id,
                    "error": {"code": e.code, "message": e.message, "data": e.data},
                }
            )
        except Exception as e:
            self._send_message(
                {
                    "jsonrpc": "2.0",
                    "id": msg_id,
                    "error": {
                        "code": INTERNAL_ERROR,
                        "message": str(e),
                        "data": {"traceback": traceback.format_exc()},
                    },
                }
            )

    def _handle_notification(self, notification: Dict):
        method = notification.get("method", "")
        if method == "notifications/initialized":
            self._initialized = True
        elif method in ("notifications/cancelled", "$/cancellation"):
            # MCP 规范名为 notifications/cancelled；$/cancellation 是 LSP 写法，容忍并存
            pass

    def _process_message(self, raw: str):
        if not raw.strip():
            return
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            self._send_message(
                {
                    "jsonrpc": "2.0",
                    "id": None,
                    "error": {"code": PARSE_ERROR, "message": "Parse error"},
                }
            )
            return
        if isinstance(msg, list):
            for item in msg:
                self._process_single(item)
            return
        self._process_single(msg)

    def _process_single(self, msg: Dict):
        if "id" in msg and msg["id"] is not None:
            self._handle_request(msg)
        else:
            self._handle_notification(msg)

    def run(self):
        sys.stderr.write(f"Autoware Diag MCP server starting (protocol 2025-03-26)...\n")
        sys.stderr.flush()
        # stdio 传输强制 UTF-8，避免跟随 locale 编码
        for stream in (sys.stdin, sys.stdout):
            try:
                stream.reconfigure(encoding="utf-8", errors="replace")
            except Exception:
                pass
        # MCP stdio 是 NDJSON：每条消息一行完整的 JSON-RPC，不允许内嵌换行。
        # 直接逐行解析 —— 不要按括号配平累积（字符串里含不配平括号会吞掉合法请求）。
        for line in sys.stdin:
            try:
                raw = line.strip()
                if not raw:
                    # Skip empty lines (keepalive / whitespace)
                    continue
                if len(raw) > 64 * 1024 * 1024:  # 64MB safety limit
                    self._send_message(
                        {
                            "jsonrpc": "2.0",
                            "id": None,
                            "error": {
                                "code": INVALID_REQUEST,
                                "message": "Message too large (>64MB)",
                            },
                        }
                    )
                    continue
                self._process_message(raw)
            except SystemExit:
                break
            except Exception:
                sys.stderr.write(
                    f"Error processing message: {traceback.format_exc()}\n"
                )
                sys.stderr.flush()


# ---------------------------------------------------------------------------
# Build the MCP server with all tools
# ---------------------------------------------------------------------------


def create_server() -> McpServer:
    server = McpServer(
        name="autoware-diag-mcp",
        version="1.0.0",
        instructions=(
            "【Autoware 自动驾驶诊断 MCP 服务器 — 使用规范】\n\n"
            "## ⚠️ 必须遵守的严谨规则\n\n"
            "你是一个严谨的自动驾驶诊断工具，使用本服务器所有工具时必须遵守以下规则：\n\n"
            "### 1. 时间参数必须精确\n"
            "- 调用 investigate_incident 时，传入的时间必须与用户指定的时间完全一致。\n"
            "- 用户说「看看 xxxxxx 时刻」→ 必须把该时刻原样填入 timestamp/timestamp_iso。\n"
            "- 用户说「从 A 到 B 时间段」→ 必须把 A 填 start/start_iso，B 填 end/end_iso。\n"
            "- **严禁自行篡改、偏移、猜测时间值**。如果你不确定时间格式，请求用户明确。\n"
            "- 调用后必须检查返回的 time_range 是否与用户要求一致；若不一致，向用户说明偏差原因。\n\n"
            "### 2. 文件路径必须真实\n"
            "- file_path 必须是用户提供的实际 MCAP 文件路径。\n"
            "- **严禁自行编造、猜测文件路径**。如果用户未提供路径，先问用户。\n\n"
            "### 3. 一次只做一个分析\n"
            "- 每次工具调用只分析一个 MCAP 文件的一个方面。\n"
            "- 不要在一次调用中请求过多数据（如 window_seconds 不要超过 30 秒，除非用户明确要求更长时间范围）。\n\n"
            "### 4. 结果必须可追溯\n"
            "- 每个工具返回结果中都包含 file_path 和时间范围，你必须核对它们是否对应你想要查的数据。\n"
            "- 如果结果中显示的时间范围与你要求的不符，立即指出并重新调用。\n\n"
            "### 5. 错误处理\n"
            "- 如果工具返回错误，仔细阅读错误信息。\n"
            "- 常见的「No messages found」可能是因为时间戳格式有误或时间范围不在录音范围内。\n"
            "- 不要盲目重试同样的参数——先检查时间值是否正确。\n\n"
            "### 6. 推荐排查顺序\n"
            "- 全局图景：analyze_bag_overview（话题分组/推断节点/服务/元数据）\n"
            "- 节点日志：analyze_rosout（/rosout WARN/ERROR）\n"
            "- 模块健康：analyze_diagnostics 或 analyze_vehicle_health\n"
            "- 指定时刻：investigate_incident\n\n"
            "---\n"
            "可用工具列表如下，每个工具都有详细说明。\n"
        ),
    )

    # ===================================================================
    # analyze_mcap_file
    # ===================================================================
    @server.tool(
        name="analyze_mcap_file",
        description="分析 MCAP 文件的内容统计信息（话题、频率、消息数）。file_path 必须使用用户提供的真实路径，不可编造。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须与用户提供的路径一致）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _analyze_mcap_file(args: Dict):
        file_path = _require_file_path(args)

        file_size = os.path.getsize(file_path)
        large_file_warn = ""
        if file_size > 500 * 1024 * 1024:
            large_file_warn = f" [WARNING: large file ({file_size / 1024 / 1024:.0f} MB), analysis may be slow]"

        # 流式单遍扫描，不整文件读入内存
        reader, fp = _open_mcap(file_path)
        try:
            scan = _scan_mcap(reader)
        finally:
            fp.close()
        stats = _format_stats(scan)

        # bag 元数据 + 命名空间分组（不额外全量解码）
        try:
            reader2, fp2 = _open_mcap(file_path)
            try:
                bag_meta = _read_bag_metadata(file_path, reader2)
            finally:
                fp2.close()
        except Exception:
            bag_meta = []
        grouping = _classify_topics_by_namespace(stats.get("topics", {}))

        return {
            "content": [
                {
                    "type": "text",
                    "text": json.dumps(
                        {
                            "file_path": file_path,
                            "file_size_mb": round(file_size / (1024 * 1024), 2),
                            "warning": large_file_warn.strip() or None,
                            "bag_metadata": bag_meta,
                            "namespaces": {
                                ns: {
                                    "topic_count": v["topic_count"],
                                    "message_count": v["message_count"],
                                }
                                for ns, v in grouping["namespaces"].items()
                            },
                            "zero_message_topic_count": len(
                                grouping["zero_message_topics"]
                            ),
                            **stats,
                            "next_steps": [
                                "analyze_bag_overview — 节点/服务/模块总览",
                                "analyze_rosout — 节点日志",
                                "analyze_diagnostics — 诊断聚合",
                            ],
                        },
                        ensure_ascii=False,
                        indent=2,
                        default=str,
                    ),
                }
            ],
        }

    # ===================================================================
    # detect_anomalies_local
    # ===================================================================
    @server.tool(
        name="detect_anomalies_local",
        description="检测 MCAP 文件中的时间异常（消息间隙、低频率）。file_path 必须使用用户提供的真实路径。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须与用户提供的路径一致）",
                },
                "gap_threshold_s": {
                    "type": "number",
                    "description": "消息间隔阈值（秒），超过视为间隙 (默认 1.0)",
                },
                "min_frequency_hz": {
                    "type": "number",
                    "description": "最低期望频率 Hz (默认 0.1)",
                },
            },
            "required": ["file_path"],
        },
    )
    def _detect_anomalies_local(args: Dict):
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

    # ===================================================================
    # analyze_perception
    # ===================================================================
    @server.tool(
        name="analyze_perception",
        description="【严谨模式】分析感知模块 — BEVFusion、跟踪、预测、LiDAR 点云。file_path 必须与用户提供的一致，不可编造。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _analyze_perception(args: Dict):
        file_path = _require_file_path(args)

        # Frequency analysis
        timestamps = _read_topic_timestamps(file_path, TOPIC_PERCEPTION)
        freq_results = {}
        expected_freqs = {
            "/perception/object_recognition/detection/bevfusion/objects": 10,
            "/perception/object_recognition/tracking/objects": 10,
            "/perception/object_recognition/objects": 10,
            "/sensing/lidar/concatenated/pointcloud_cloudini": 10,
        }
        for topic in TOPIC_PERCEPTION:
            ts = timestamps.get(topic, [])
            freq_results[topic] = _freq_analysis(
                ts, expected_freqs.get(topic)
            )

        # Content analysis - collect samples
        content_data = _collect_topic_data(
            file_path, TOPIC_PERCEPTION, max_per_topic=5
        )

        # Summarize perception content
        perception_summary = {}
        for topic, msgs in content_data.items():
            if not msgs:
                perception_summary[topic] = {"note": "no data"}
                continue

            samples = []
            for m in msgs[:3]:
                d = m["data"]
                if hasattr(d, "objects"):
                    objs = d.objects
                    labels = Counter()
                    confidences = []
                    for obj in objs:
                        if hasattr(obj, "classification") and obj.classification:
                            for c in obj.classification:
                                label = getattr(c, "label", None)
                                prob = getattr(c, "probability", 0)
                                labels[label] += 1
                                confidences.append(prob)
                        elif hasattr(obj, "existence_probability"):
                            confidences.append(obj.existence_probability)

                    samples.append(
                        {
                            "time": _time_str(m["publish_time_ns"]),
                            "object_count": len(objs),
                            "classification_counts": dict(labels),
                            "confidence_range": (
                                {
                                    "min": round(min(confidences), 3),
                                    "max": round(max(confidences), 3),
                                    "avg": round(
                                        sum(confidences) / len(confidences), 3
                                    ),
                                }
                                if confidences
                                else None
                            ),
                        }
                    )
                elif hasattr(d, "header"):
                    samples.append(
                        {
                            "time": _time_str(m["publish_time_ns"]),
                            "type": type(d).__name__,
                            "header": {
                                "stamp": str(d.header.stamp) if hasattr(d.header, "stamp") else None,
                                "frame_id": getattr(d.header, "frame_id", None),
                            }
                            if hasattr(d, "header")
                            else None,
                        }
                    )

            perception_summary[topic] = {
                "total_messages": freq_results.get(topic, {}).get("count", 0),
                "samples": samples,
            }

        # Anomaly detection
        anomalies = []
        for topic, freq in freq_results.items():
            if not freq.get("healthy", True):
                anomalies.append(
                    {
                        "topic": topic,
                        "type": "frequency_anomaly",
                        "severity": "warning",
                        "detail": (
                            f"Topic {topic}: "
                            f"expected {freq.get('expected_frequency_hz','?')} Hz, "
                            f"got {freq.get('actual_frequency_hz','?')} Hz, "
                            f"{freq.get('large_gap_count', 0)} large gaps detected"
                        ),
                    }
                )
            if freq.get("count", 0) == 0:
                anomalies.append(
                    {
                        "topic": topic,
                        "type": "missing_topic",
                        "severity": "error",
                        "detail": f"Topic {topic} has no messages!",
                    }
                )

        return {
            "content": [
                {
                    "type": "text",
                    "text": json.dumps(
                        {
                            "file_path": file_path,
                            "frequency_analysis": freq_results,
                            "perception_content": perception_summary,
                            "anomalies": anomalies,
                            "total_anomalies": len(anomalies),
                        },
                        ensure_ascii=False,
                        indent=2,
                        default=str,
                    ),
                }
            ],
        }

    # ===================================================================
    # analyze_chassis
    # ===================================================================
    @server.tool(
        name="analyze_chassis",
        description="【严谨模式】分析车辆底盘状态 — Pix Robobus 执行命令与反馈、轮速。file_path 必须与用户提供的一致，不可编造。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _analyze_chassis(args: Dict):
        file_path = _require_file_path(args)

        # Frequency analysis
        timestamps = _read_topic_timestamps(file_path, TOPIC_CHASSIS)
        freq_results = {}
        expected_freqs = {
            "/pix_robobus/av_drive_ctrl": 50,
            "/pix_robobus/av_brake_ctrl": 50,
            "/pix_robobus/va_drive_sta_fb": 50,
            "/pix_robobus/va_brake_sta_fb": 50,
            "/pix_robobus/va_steer_sta_fb": 50,
            "/pix_robobus/va_chassis_wheel_rpm_fb": 50,
            "/vehicle/status/actuation_status": 50,
            "/vehicle/status/velocity_status": 50,
            "/vehicle/status/steering_status": 50,
        }
        for topic in TOPIC_CHASSIS:
            ts = timestamps.get(topic, [])
            freq_results[topic] = _freq_analysis(
                ts, expected_freqs.get(topic)
            )

        # Content analysis
        content_data = _collect_topic_data(
            file_path, TOPIC_CHASSIS, max_per_topic=20
        )

        # Analyze actuation command vs feedback
        actuation_analysis = {}
        ctrl_msgs = content_data.get("/control/command/actuation_cmd", [])[:10]
        actu_status_msgs = content_data.get("/vehicle/status/actuation_status", [])[:10]
        actuation_analysis["actuation_command_sample"] = [
            _dictify(m["data"]) for m in ctrl_msgs
        ]
        actuation_analysis["actuation_status_sample"] = [
            _dictify(m["data"]) for m in actu_status_msgs
        ]

        # Control command analysis
        control_cmd_msgs = content_data.get("/control/command/control_cmd", [])[:10]
        actuation_analysis["control_command_sample"] = [
            _dictify(m["data"]) for m in control_cmd_msgs
        ]
        # 嵌套 Control 字段摘要（lateral/longitudinal）
        actuation_analysis["control_command_parsed"] = [
            {
                "time": _time_str(m["publish_time_ns"]),
                **(_extract_control_summary(m["data"]) or {}),
            }
            for m in control_cmd_msgs[:5]
        ]

        # Velocity status
        vel_msgs = content_data.get("/vehicle/status/velocity_status", [])[:5]
        velocity_samples = []
        for m in vel_msgs:
            d = m["data"]
            velocity_samples.append(
                {
                    "time": _time_str(m["publish_time_ns"]),
                    "data": _dictify(d),
                }
            )

        # Pix feedback analysis — 语义字段 + 原始 dump
        pix_drive = content_data.get("/pix_robobus/va_drive_sta_fb", [])[:5]
        pix_brake = content_data.get("/pix_robobus/va_brake_sta_fb", [])[:5]
        pix_steer = content_data.get("/pix_robobus/va_steer_sta_fb", [])[:5]
        pix_wheel = content_data.get("/pix_robobus/va_chassis_wheel_rpm_fb", [])[:3]
        pix_drive_cmd = content_data.get("/pix_robobus/av_drive_ctrl", [])[:5]
        pix_brake_cmd = content_data.get("/pix_robobus/av_brake_ctrl", [])[:5]

        # Anomalies
        anomalies = []
        for topic, freq in freq_results.items():
            if freq.get("count", 0) == 0:
                if topic in AEB_FEEDBACK_GROUP and _group_has_data(
                    freq_results, AEB_FEEDBACK_GROUP
                ):
                    continue  # AEB 反馈新旧话题是替代关系，另一个存在即可
                anomalies.append(
                    {
                        "topic": topic,
                        "type": "missing_topic",
                        "severity": "error",
                        "detail": f"Topic {topic} has no messages!",
                    }
                )
            elif not freq.get("healthy", True):
                anomalies.append(
                    {
                        "topic": topic,
                        "type": "frequency_anomaly",
                        "severity": "warning",
                        "detail": (
                            f"Topic {topic}: "
                            f"expected {freq.get('expected_frequency_hz','?')} Hz, "
                            f"actual {freq.get('actual_frequency_hz','?')} Hz, "
                            f"{freq.get('large_gap_count', 0)} large gaps"
                        ),
                    }
                )

        # Pix CAN 生命信号停更 / 通讯丢失告警
        for label, parsed_list, topic in (
            ("drive", [_parse_pix_drive_fb(m["data"]) for m in pix_drive], "/pix_robobus/va_drive_sta_fb"),
            ("brake", [_parse_pix_brake_fb(m["data"]) for m in pix_brake], "/pix_robobus/va_brake_sta_fb"),
            ("steer", [_parse_pix_steer_fb(m["data"]) for m in pix_steer], "/pix_robobus/va_steer_sta_fb"),
        ):
            lives = [p.get("life") for p in parsed_list if p and p.get("life") is not None]
            if len(lives) >= 3 and len(set(lives[-3:])) == 1:
                anomalies.append(
                    {
                        "topic": topic,
                        "type": "life_signal_stuck",
                        "severity": "warning",
                        "detail": f"Pix {label} life signal stuck at {lives[-1]} in last samples",
                    }
                )
            for p in parsed_list:
                if p and p.get("comm_lost"):
                    anomalies.append(
                        {
                            "topic": topic,
                            "type": "communication_lost",
                            "severity": "error",
                            "detail": f"Pix {label} feedback reports communication_lost=True",
                        }
                    )
                    break

        return {
            "content": [
                {
                    "type": "text",
                    "text": json.dumps(
                        {
                            "file_path": file_path,
                            "message_semantics": {
                                "note": "Pix Robobus CAN 字段为私有接口；转向角单位为度(deg)，与 Autoware rad 不同",
                                "catalog_ref": list(
                                    k for k in CUSTOM_MESSAGE_CATALOG if k.startswith("pix_")
                                ),
                            },
                            "frequency_analysis": freq_results,
                            "actuation_analysis": actuation_analysis,
                            "velocity_samples": velocity_samples,
                            "pix_drive_cmd_parsed": [
                                _parse_pix_drive_cmd(m["data"]) for m in pix_drive_cmd
                            ],
                            "pix_brake_cmd_parsed": [
                                _parse_pix_brake_cmd(m["data"]) for m in pix_brake_cmd
                            ],
                            "pix_drive_feedback": [
                                _dictify(m["data"]) for m in pix_drive
                            ],
                            "pix_drive_feedback_parsed": [
                                _parse_pix_drive_fb(m["data"]) for m in pix_drive
                            ],
                            "pix_brake_feedback": [
                                _dictify(m["data"]) for m in pix_brake
                            ],
                            "pix_brake_feedback_parsed": [
                                _parse_pix_brake_fb(m["data"]) for m in pix_brake
                            ],
                            "pix_steer_feedback": [
                                _dictify(m["data"]) for m in pix_steer
                            ],
                            "pix_steer_feedback_parsed": [
                                _parse_pix_steer_fb(m["data"]) for m in pix_steer
                            ],
                            "pix_wheel_rpm_raw": [
                                _dictify(m["data"]) for m in pix_wheel
                            ],
                            "anomalies": anomalies,
                            "total_anomalies": len(anomalies),
                        },
                        ensure_ascii=False,
                        indent=2,
                        default=str,
                    ),
                }
            ],
        }

    # ===================================================================
    # analyze_localization
    # ===================================================================
    @server.tool(
        name="analyze_localization",
        description="【严谨模式】分析定位模块 — GNSS、IMU、里程计、姿态。file_path 必须与用户提供的一致，不可编造。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _analyze_localization(args: Dict):
        file_path = _require_file_path(args)

        timestamps = _read_topic_timestamps(file_path, TOPIC_LOCALIZATION)
        freq_results = {}
        expected_freqs = {
            "/localization/kinematic_state": 50,
            "/localization/acceleration": 50,
            "/localization/twist_estimator/twist_with_covariance": 50,
            "/localization/twist_estimator/gyro_twist": 50,
            "/sensing/vehicle_velocity_converter/twist_with_covariance": 50,
            "/sensing/gnss/fix": 100,
            "/sensing/gnss/imu": 100,
            "/sensing/gnss/heading": 100,
            "/autoware_orientation": 100,
        }
        for topic in TOPIC_LOCALIZATION:
            ts = timestamps.get(topic, [])
            freq_results[topic] = _freq_analysis(ts, expected_freqs.get(topic))

        content_data = _collect_topic_data(file_path, TOPIC_LOCALIZATION, max_per_topic=10)

        # Kinematic state analysis
        kinematic_samples = []
        for m in content_data.get("/localization/kinematic_state", []):
            d = _dictify(m["data"], max_depth=5)
            pose = d.get("pose", {}).get("pose", {})
            twist = d.get("twist", {}).get("twist", {})
            pos = pose.get("position", {})
            linear = twist.get("linear", {})
            kinematic_samples.append({
                "time": _time_str(m["publish_time_ns"]),
                "position": {"x": pos.get("x"), "y": pos.get("y"), "z": pos.get("z")},
                "velocity": {"x": linear.get("x"), "y": linear.get("y"), "z": linear.get("z")},
            })

        # GNSS fix analysis
        gnss_samples = []
        for m in content_data.get("/sensing/gnss/fix", [])[:5]:
            d = _dictify(m["data"], max_depth=5)
            gnss_samples.append({
                "time": _time_str(m["publish_time_ns"]),
                "latitude": d.get("latitude"),
                "longitude": d.get("longitude"),
                "altitude": d.get("altitude"),
                "position_covariance_type": d.get("position_covariance_type"),
                "status": d.get("status"),
            })

        # Accel samples
        accel_samples = []
        for m in content_data.get("/localization/acceleration", [])[:5]:
            d = _dictify(m["data"], max_depth=5)
            accel = d.get("accel", {}).get("accel", d.get("accel", {}))
            if isinstance(accel, dict):
                linear_a = accel.get("linear", {})
                accel_samples.append({
                    "time": _time_str(m["publish_time_ns"]),
                    "linear_x": linear_a.get("x"),
                    "linear_y": linear_a.get("y"),
                    "linear_z": linear_a.get("z"),
                })

        # IMU samples
        imu_samples = []
        for m in content_data.get("/sensing/gnss/imu", [])[:5]:
            d = _dictify(m["data"], max_depth=5)
            imu_samples.append({
                "time": _time_str(m["publish_time_ns"]),
                "orientation": d.get("orientation"),
                "angular_velocity": d.get("angular_velocity"),
                "linear_acceleration": d.get("linear_acceleration"),
            })

        # Velocity source comparison
        vel_sources = {}
        for src in ["/sensing/vehicle_velocity_converter/twist_with_covariance",
                     "/localization/twist_estimator/twist_with_covariance",
                     "/localization/twist_estimator/gyro_twist"]:
            msgs = content_data.get(src, [])[:3]
            if msgs:
                vel_sources[src] = []
                for m in msgs:
                    d = _dictify(m["data"], max_depth=5)
                    twist = d.get("twist", {}).get("twist", d)
                    linear = twist.get("linear", {})
                    vel_sources[src].append({
                        "time": _time_str(m["publish_time_ns"]),
                        "speed_x": linear.get("x"),
                        "speed_y": linear.get("y"),
                    })

        # Anomalies
        anomalies = []
        for topic, freq in freq_results.items():
            if freq.get("count", 0) == 0:
                anomalies.append({"topic": topic, "type": "missing_topic", "severity": "error",
                                  "detail": f"Topic {topic} has no messages!"})
            elif not freq.get("healthy", True):
                anomalies.append({"topic": topic, "type": "frequency_anomaly", "severity": "warning",
                                  "detail": f"{topic}: expected {freq.get('expected_frequency_hz','?')} Hz, actual {freq.get('actual_frequency_hz','?')} Hz"})

        return {
            "content": [{
                "type": "text",
                "text": json.dumps({
                    "file_path": file_path,
                    "frequency_analysis": freq_results,
                    "gnss_samples": gnss_samples,
                    "imu_samples": imu_samples,
                    "kinematic_samples": kinematic_samples,
                    "acceleration_samples": accel_samples,
                    "velocity_source_comparison": vel_sources,
                    "anomalies": anomalies,
                    "total_anomalies": len(anomalies),
                }, ensure_ascii=False, indent=2, default=str),
            }],
        }

    # ===================================================================
    # analyze_planning_control
    # ===================================================================
    @server.tool(
        name="analyze_planning_control",
        description="【严谨模式】分析规划控制 — 控制指令、场景、控制模式、诊断数据。file_path 必须与用户提供的一致，不可编造。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _analyze_planning_control(args: Dict):
        file_path = _require_file_path(args)

        # Frequency
        timestamps = _read_topic_timestamps(file_path, TOPIC_PLANNING_CONTROL)
        freq_results = {}
        expected_freqs = {
            "/control/command/control_cmd": 33,
            "/control/command/actuation_cmd": 33,
            "/control/trajectory_follower/control_cmd": 33,
            "/control/trajectory_follower/lateral/diagnostic": 33,
            "/control/trajectory_follower/longitudinal/diagnostic": 33,
            "/planning/scenario_planning/scenario": 10,
            "/control/current_gate_mode": 10,
        }
        for topic in TOPIC_PLANNING_CONTROL:
            ts = timestamps.get(topic, [])
            freq_results[topic] = _freq_analysis(
                ts, expected_freqs.get(topic)
            )

        # Content
        content_data = _collect_topic_data(
            file_path, TOPIC_PLANNING_CONTROL, max_per_topic=10
        )

        # Control command timeline
        ctrl_cmd_msgs = content_data.get("/control/command/control_cmd", [])
        actuation_cmd_msgs = content_data.get("/control/command/actuation_cmd", [])

        # Scenario analysis
        scenario_msgs = content_data.get("/planning/scenario_planning/scenario", [])
        scenario_timeline = []
        for m in scenario_msgs:
            d = m["data"]
            scenario_timeline.append(
                {
                    "time": _time_str(m["publish_time_ns"]),
                    "data": _dictify(d, max_depth=4),
                    "parsed": _extract_scenario_summary(d),
                }
            )

        # Gate mode analysis
        gate_msgs = content_data.get("/control/current_gate_mode", [])
        gate_timeline = []
        for m in gate_msgs:
            d = m["data"]
            gate_timeline.append(
                {
                    "time": _time_str(m["publish_time_ns"]),
                    "data": _dictify(d),
                    "parsed": _extract_gate_mode(d),
                }
            )

        # Planning validator
        pv_msgs = content_data.get("/planning/planning_validator/validation_status", [])
        pv_parsed = []
        for m in pv_msgs[:5]:
            dd = _dictify(m["data"], max_depth=3)
            invalid_flags = {}
            if isinstance(dd, dict):
                for k, v in dd.items():
                    if isinstance(k, str) and k.startswith("is_valid_") and v is False:
                        invalid_flags[k] = False
            pv_parsed.append(
                {
                    "time": _time_str(m["publish_time_ns"]),
                    "invalid_flags": invalid_flags or None,
                    "invalid_count": dd.get("invalid_count") if isinstance(dd, dict) else None,
                    "raw": dd,
                }
            )

        # Anomalies
        anomalies = []
        for topic, freq in freq_results.items():
            if freq.get("count", 0) == 0:
                anomalies.append(
                    {
                        "topic": topic,
                        "type": "missing_topic",
                        "severity": "error",
                        "detail": f"Topic {topic} has no messages!",
                    }
                )
            elif not freq.get("healthy", True):
                anomalies.append(
                    {
                        "topic": topic,
                        "type": "frequency_anomaly",
                        "severity": "warning",
                        "detail": (
                            f"Topic {topic}: "
                            f"expected {freq.get('expected_frequency_hz','?')} Hz, "
                            f"actual {freq.get('actual_frequency_hz','?')} Hz"
                        ),
                    }
                )

        # Scenario / gate mode 变化
        prev_scn = None
        for item in scenario_timeline:
            cur = (item.get("parsed") or {}).get("current_scenario")
            if cur and prev_scn and cur != prev_scn:
                anomalies.append(
                    {
                        "topic": "/planning/scenario_planning/scenario",
                        "type": "scenario_change",
                        "severity": "info",
                        "detail": f"Scenario changed: {prev_scn} -> {cur} @ {item['time']}",
                    }
                )
            if cur:
                prev_scn = cur
        prev_gate = None
        for item in gate_timeline:
            cur = (item.get("parsed") or {}).get("label")
            if cur and prev_gate and cur != prev_gate:
                anomalies.append(
                    {
                        "topic": "/control/current_gate_mode",
                        "type": "gate_mode_change",
                        "severity": "warning",
                        "detail": f"GateMode changed: {prev_gate} -> {cur} @ {item['time']}",
                    }
                )
            if cur:
                prev_gate = cur

        return {
            "content": [
                {
                    "type": "text",
                    "text": json.dumps(
                        {
                            "file_path": file_path,
                            "message_semantics": {
                                "control_cmd": CUSTOM_MESSAGE_CATALOG.get(
                                    "autoware_control_msgs/msg/Control"
                                ),
                                "scenario": CUSTOM_MESSAGE_CATALOG.get(
                                    "autoware_internal_planning_msgs/msg/Scenario"
                                ),
                                "gate_mode": CUSTOM_MESSAGE_CATALOG.get(
                                    "tier4_control_msgs/msg/GateMode"
                                ),
                            },
                            "frequency_analysis": freq_results,
                            "control_command_timeline": [
                                _dictify(m["data"]) for m in ctrl_cmd_msgs
                            ],
                            "control_command_parsed": [
                                {
                                    "time": _time_str(m["publish_time_ns"]),
                                    **(_extract_control_summary(m["data"]) or {}),
                                }
                                for m in ctrl_cmd_msgs[:15]
                            ],
                            "actuation_command_timeline": [
                                _dictify(m["data"]) for m in actuation_cmd_msgs
                            ],
                            "scenario_timeline": scenario_timeline,
                            "gate_mode_timeline": gate_timeline,
                            "planning_validator": pv_parsed,
                            "anomalies": anomalies,
                            "total_anomalies": len(anomalies),
                        },
                        ensure_ascii=False,
                        indent=2,
                        default=str,
                    ),
                }
            ],
        }

    # ===================================================================
    # detect_mrm_events
    # ===================================================================
    @server.tool(
        name="detect_mrm_events",
        description="【严谨模式】检测 MRM (Minimum Risk Maneuver) 事件 — Emergency、Engage、GateMode。file_path 必须与用户提供的一致，不可编造。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _detect_mrm_events(args: Dict):
        file_path = _require_file_path(args)

        # 全量扫描（不用采样）—— 采样会漏掉录制中后期的事件
        content_data = _collect_topic_data_full(file_path, TOPIC_MRM)

        # Emergency command timeline（57Hz 持续发布，含 emergency 布尔字段）
        emergency_cmd_msgs = content_data.get("/control/command/emergency_cmd", [])
        emergency_timeline = []
        for m in emergency_cmd_msgs:
            emergency_timeline.append(
                {
                    "time": _time_str(m["log_time_ns"]),
                    "data": _dictify(m["data"], max_depth=4),
                }
            )

        # Emergency flag
        api_emergency_msgs = content_data.get("/api/autoware/get/emergency", [])
        emergency_flag_timeline = []
        for m in api_emergency_msgs:
            is_emergency = getattr(m["data"], "emergency", None)
            emergency_flag_timeline.append(
                {
                    "time": _time_str(m["log_time_ns"]),
                    "emergency": bool(is_emergency) if is_emergency is not None else None,
                }
            )

        # Engage status
        engage_msgs = content_data.get("/api/autoware/get/engage", [])
        engage_timeline = []
        for m in engage_msgs:
            engage_timeline.append(
                {
                    "time": _time_str(m["log_time_ns"]),
                    "data": _dictify(m["data"]),
                }
            )

        # Gate mode
        gate_msgs = content_data.get("/control/current_gate_mode", [])
        gate_timeline = []
        for m in gate_msgs:
            gate_timeline.append(
                {
                    "time": _time_str(m["log_time_ns"]),
                    "data": _dictify(m["data"]),
                }
            )

        # Detect MRM events (state transitions) — 基于全量、按 log_time 排序的信号
        events = []
        prev_engage = None
        prev_emergency = None
        prev_emg_cmd = None

        # Combine all time-ordered signals（用 log_time，与过滤基准一致）
        all_signals = []
        for m in engage_msgs:
            val = _extract_engage(m["data"])
            all_signals.append(("engage", m["log_time_ns"], val))
        for m in api_emergency_msgs:
            val = getattr(m["data"], "emergency", None)
            all_signals.append(
                ("emergency_flag", m["log_time_ns"], bool(val) if val is not None else None)
            )
        for m in emergency_cmd_msgs:
            val = getattr(m["data"], "emergency", None)
            all_signals.append(
                ("emergency_cmd", m["log_time_ns"], bool(val) if val is not None else None)
            )

        all_signals.sort(key=lambda x: x[1])
        for sig_type, ts_ns, val in all_signals:
            if val is None:
                continue
            if sig_type == "engage":
                if prev_engage is not None and prev_engage != val:
                    if val is False:
                        events.append(
                            {
                                "time": _time_str(ts_ns),
                                "event": "DISENGAGE",
                                "detail": "Autoware disengaged (MRM triggered)",
                                "severity": "error",
                            }
                        )
                    else:
                        events.append(
                            {
                                "time": _time_str(ts_ns),
                                "event": "ENGAGE",
                                "detail": "Autoware engaged",
                                "severity": "info",
                            }
                        )
                prev_engage = val
            elif sig_type == "emergency_flag":
                if val is True and prev_emergency is not True:
                    events.append(
                        {
                            "time": _time_str(ts_ns),
                            "event": "EMERGENCY_ON",
                            "detail": "Emergency flag activated"
                            + (" (already active at recording start)" if prev_emergency is None else ""),
                            "severity": "error",
                        }
                    )
                elif val is False and prev_emergency is True:
                    events.append(
                        {
                            "time": _time_str(ts_ns),
                            "event": "EMERGENCY_OFF",
                            "detail": "Emergency flag deactivated",
                            "severity": "info",
                        }
                    )
                prev_emergency = val
            elif sig_type == "emergency_cmd":
                if val is True and prev_emg_cmd is not True:
                    events.append(
                        {
                            "time": _time_str(ts_ns),
                            "event": "EMERGENCY_CMD_ON",
                            "detail": "Emergency control command activated (vehicle_cmd.emergency=True)"
                            + (" (already active at recording start)" if prev_emg_cmd is None else ""),
                            "severity": "error",
                        }
                    )
                elif val is False and prev_emg_cmd is True:
                    events.append(
                        {
                            "time": _time_str(ts_ns),
                            "event": "EMERGENCY_CMD_OFF",
                            "detail": "Emergency control command cleared",
                            "severity": "info",
                        }
                    )
                prev_emg_cmd = val

        return {
            "content": [
                {
                    "type": "text",
                    "text": json.dumps(
                        {
                            "file_path": file_path,
                            "ros2_decoding": _decoding_status(),
                            "mrm_events": events,
                            "total_mrm_events": len(events),
                            # timeline 只截断返回 payload，事件检测已基于全量数据完成
                            "emergency_command_timeline": _cap_timeline(emergency_timeline, 100),
                            "emergency_flag_timeline": _cap_timeline(emergency_flag_timeline, 100),
                            "engage_timeline": _cap_timeline(engage_timeline, 100),
                            "gate_mode_timeline": _cap_timeline(gate_timeline, 100),
                            "summary": {
                                "has_emergency_command": len(emergency_cmd_msgs) > 0,
                                "emergency_command_count": len(emergency_cmd_msgs),
                                "disengage_events": len(
                                    [e for e in events if e["event"] == "DISENGAGE"]
                                ),
                                "emergency_activations": len(
                                    [e for e in events if e["event"] == "EMERGENCY_ON"]
                                ),
                                "emergency_cmd_activations": len(
                                    [e for e in events if e["event"] == "EMERGENCY_CMD_ON"]
                                ),
                            },
                        },
                        ensure_ascii=False,
                        indent=2,
                        default=str,
                    ),
                }
            ],
        }

    # ===================================================================
    # detect_aeb_events
    # ===================================================================
    @server.tool(
        name="detect_aeb_events",
        description="【严谨模式】检测 AEB (Autonomous Emergency Braking) 事件。file_path 必须与用户提供的一致，不可编造。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _detect_aeb_events(args: Dict):
        file_path = _require_file_path(args)

        # 单遍快扫（不解码）：确认 AEB 反馈话题存在性与消息计数。
        # 只扫小消息话题（AebInfoShadow / av_aeb_ctrl / rss / emergency_cmd），
        # 跳过 debug/markers 等大消息，避免全量解码 MarkerArray 撑爆内存。
        rss_topic = "/control/autonomous_emergency_braking/debug/rss_distance"
        scan_topics = TOPIC_AEB_SHADOW + [
            "/pix_robobus/av_aeb_ctrl",
            rss_topic,
            "/control/command/emergency_cmd",
        ]
        reader, fp = _open_mcap(file_path)
        try:
            scan = _scan_mcap(reader, topics=scan_topics)
        finally:
            fp.close()
        stats = scan["stats"]

        # 新版 /pix/AebInfoShadow 优先，旧版 /pix_robobus/av_aeb_ctrl 兼容
        shadow_topic = next(
            (t for t in TOPIC_AEB_SHADOW if stats.get(t, {}).get("count", 0) > 0),
            None,
        )
        has_legacy = stats.get("/pix_robobus/av_aeb_ctrl", {}).get("count", 0) > 0
        source_topic = shadow_topic or ("/pix_robobus/av_aeb_ctrl" if has_legacy else None)

        rss_count = stats.get(rss_topic, {}).get("count", 0)
        emergency_count = stats.get("/control/command/emergency_cmd", {}).get("count", 0)

        aeb_events: List[Dict] = []
        aeb_timeline = _BoundedList()
        rss_timeline = _BoundedList()
        total_aeb_msgs = 0

        if shadow_topic:
            # ---- 新版：AebInfoShadow（aeb_interfaces）----
            # 事件以「滤波后请求」(aeb_req_filter/fcw_req_filter) 为准；
            # no_filter 仅用于对比/滤波诊断；status=2 为 FAULT。
            content = _collect_topic_data_full(file_path, [shadow_topic, rss_topic])
            aeb_msgs = content.get(shadow_topic, [])
            total_aeb_msgs = len(aeb_msgs)

            prev = {"aeb_req": None, "fcw_req": None, "aeb_raw": None, "fcw_raw": None,
                    "aeb_fault": None, "fcw_fault": None}
            for m in aeb_msgs:
                p = _parse_aeb_shadow(m["data"])
                aeb_req = bool(p["aeb_req_filter"]) if p["aeb_req_filter"] is not None else None
                fcw_req = bool(p["fcw_req_filter"]) if p["fcw_req_filter"] is not None else None
                aeb_raw = bool(p["aeb_req_no_filter"]) if p["aeb_req_no_filter"] is not None else None
                fcw_raw = bool(p["fcw_req_no_filter"]) if p["fcw_req_no_filter"] is not None else None
                aeb_status = p["aeb_status"]
                fcw_status = p["fcw_status"]

                # AEB 触发（滤波后请求）
                if aeb_req and prev["aeb_req"] is not True:
                    aeb_events.append({
                        "time": _time_str(m["log_time_ns"]),
                        "event": "AEB_ACTIVATED",
                        "severity": "error",
                        "detail": (
                            "AEB request activated (filtered) "
                            f"[no_filter={int(bool(aeb_raw))}, status={p['aeb_status_label']}]"
                            + (" (already active at recording start)" if prev["aeb_req"] is None else "")
                        ),
                    })
                elif aeb_req is False and prev["aeb_req"] is True:
                    aeb_events.append({
                        "time": _time_str(m["log_time_ns"]),
                        "event": "AEB_CLEARED",
                        "severity": "info",
                        "detail": "AEB request cleared",
                    })

                # FCW 触发（滤波后请求）
                if fcw_req and prev["fcw_req"] is not True:
                    aeb_events.append({
                        "time": _time_str(m["log_time_ns"]),
                        "event": "FCW_ACTIVATED",
                        "severity": "warning",
                        "detail": (
                            "FCW request activated (filtered) "
                            f"[no_filter={int(bool(fcw_raw))}, status={p['fcw_status_label']}]"
                            + (" (already active at recording start)" if prev["fcw_req"] is None else "")
                        ),
                    })
                elif fcw_req is False and prev["fcw_req"] is True:
                    aeb_events.append({
                        "time": _time_str(m["log_time_ns"]),
                        "event": "FCW_CLEARED",
                        "severity": "info",
                        "detail": "FCW request cleared",
                    })

                # 原始请求被滤波拦截 — 诊断「AEB 该触发未触发」的关键线索。
                # 结合官方语义区分两种根因：
                #   功能 OFF(0) → 请求未生效属设计预期（功能未开启）；
                #   功能 ON(1) 却被拦 → 疑似滤波逻辑问题，需深查。
                if aeb_raw and not aeb_req and prev["aeb_raw"] is not True:
                    aeb_events.append({
                        "time": _time_str(m["log_time_ns"]),
                        "event": "AEB_REQUEST_FILTERED",
                        "severity": "warning",
                        "detail": (
                            "AEB raw request (no_filter=1) was filtered out (filter=0) — AEB did not engage"
                            + (
                                " [aeb_status=OFF: feature disabled, request not executed by design]"
                                if aeb_status == 0
                                else (
                                    " [aeb_status=ON: feature enabled but request filtered — check filter logic]"
                                    if aeb_status == 1
                                    else ""
                                )
                            )
                            + (" (already the case at recording start)" if prev["aeb_raw"] is None else "")
                        ),
                    })
                if fcw_raw and not fcw_req and prev["fcw_raw"] is not True:
                    aeb_events.append({
                        "time": _time_str(m["log_time_ns"]),
                        "event": "FCW_REQUEST_FILTERED",
                        "severity": "warning",
                        "detail": (
                            "FCW raw request (no_filter=1) was filtered out (filter=0) — FCW did not engage"
                            + (
                                " [fcw_status=OFF: feature disabled, request not executed by design]"
                                if fcw_status == 0
                                else (
                                    " [fcw_status=ON: feature enabled but request filtered — check filter logic]"
                                    if fcw_status == 1
                                    else ""
                                )
                            )
                            + (" (already the case at recording start)" if prev["fcw_raw"] is None else "")
                        ),
                    })

                # status 故障（0=OFF 1=ON 2=FAULT）
                if aeb_status == 2 and prev["aeb_fault"] is not True:
                    aeb_events.append({
                        "time": _time_str(m["log_time_ns"]),
                        "event": "AEB_STATUS_FAULT",
                        "severity": "error",
                        "detail": "AEB status=FAULT"
                        + (" (already fault at recording start)" if prev["aeb_fault"] is None else ""),
                    })
                elif aeb_status is not None and aeb_status != 2 and prev["aeb_fault"] is True:
                    aeb_events.append({
                        "time": _time_str(m["log_time_ns"]),
                        "event": "AEB_STATUS_RECOVERED",
                        "severity": "info",
                        "detail": "AEB status recovered from FAULT",
                    })
                if fcw_status == 2 and prev["fcw_fault"] is not True:
                    aeb_events.append({
                        "time": _time_str(m["log_time_ns"]),
                        "event": "FCW_STATUS_FAULT",
                        "severity": "error",
                        "detail": "FCW status=FAULT"
                        + (" (already fault at recording start)" if prev["fcw_fault"] is None else ""),
                    })
                elif fcw_status is not None and fcw_status != 2 and prev["fcw_fault"] is True:
                    aeb_events.append({
                        "time": _time_str(m["log_time_ns"]),
                        "event": "FCW_STATUS_RECOVERED",
                        "severity": "info",
                        "detail": "FCW status recovered from FAULT",
                    })

                prev.update({
                    "aeb_req": aeb_req, "fcw_req": fcw_req,
                    "aeb_raw": aeb_raw, "fcw_raw": fcw_raw,
                    "aeb_fault": aeb_status == 2, "fcw_fault": fcw_status == 2,
                })

                aeb_timeline.add({
                    "time": _time_str(m["log_time_ns"]),
                    "aeb_status": p["aeb_status_label"],
                    "fcw_status": p["fcw_status_label"],
                    "aeb_req_filter": _AEB_REQ_LABELS.get(
                        p["aeb_req_filter"], p["aeb_req_filter"]
                    ),
                    "fcw_req_filter": _AEB_REQ_LABELS.get(
                        p["fcw_req_filter"], p["fcw_req_filter"]
                    ),
                    "aeb_req_no_filter": _AEB_REQ_LABELS.get(
                        p["aeb_req_no_filter"], p["aeb_req_no_filter"]
                    ),
                    "fcw_req_no_filter": _AEB_REQ_LABELS.get(
                        p["fcw_req_no_filter"], p["fcw_req_no_filter"]
                    ),
                    "aeb_sensitivity_auto": _AEB_SENSITIVITY_LABELS.get(
                        p["aeb_sensitivity_auto"], p["aeb_sensitivity_auto"]
                    ),
                    "aeb_sensitivity_local": _AEB_SENSITIVITY_LABELS.get(
                        p["aeb_sensitivity_local"], p["aeb_sensitivity_local"]
                    ),
                    "fcw_sensitivity_auto": _AEB_SENSITIVITY_LABELS.get(
                        p["fcw_sensitivity_auto"], p["fcw_sensitivity_auto"]
                    ),
                    "fcw_sensitivity_local": _AEB_SENSITIVITY_LABELS.get(
                        p["fcw_sensitivity_local"], p["fcw_sensitivity_local"]
                    ),
                })

            # RSS distance（与 AEB 触发交叉验证）
            for m in content.get(rss_topic, []):
                rss_timeline.add({
                    "time": _time_str(m["log_time_ns"]),
                    "data": _dictify(m["data"]),
                })

        elif has_legacy:
            # ---- 旧版：AvAebCtrl（老录音兼容）----
            content = _collect_topic_data_full(
                file_path, ["/pix_robobus/av_aeb_ctrl", rss_topic]
            )
            aeb_msgs = content.get("/pix_robobus/av_aeb_ctrl", [])
            total_aeb_msgs = len(aeb_msgs)

            prev_aeb_active = None
            prev_fcw_active = None
            for m in aeb_msgs:
                d = m["data"]
                aeb_active = bool(
                    getattr(d, "acu_aeb_active_request", False)
                    or getattr(d, "acu_aeb_brake_en", False)
                )
                aeb_brake_en = getattr(d, "acu_aeb_brake_en", False)
                fcw_active = bool(getattr(d, "acu_fcw_active_request", False))
                brake_pedal = getattr(d, "acu_aeb_brake_pedal_target", 0)

                # Detect transitions（None 表示第一条消息：若已激活则标注录制开始即激活）
                if aeb_active and prev_aeb_active is not True:
                    aeb_events.append(
                        {
                            "time": _time_str(m["log_time_ns"]),
                            "event": "AEB_ACTIVATED",
                            "brake_pedal_target": brake_pedal,
                            "severity": "error",
                            "detail": "AEB activated"
                            + (" (already active at recording start)" if prev_aeb_active is None else ""),
                        }
                    )
                if fcw_active and prev_fcw_active is not True:
                    aeb_events.append(
                        {
                            "time": _time_str(m["log_time_ns"]),
                            "event": "FCW_ACTIVATED",
                            "severity": "warning",
                            "detail": "FCW activated"
                            + (" (already active at recording start)" if prev_fcw_active is None else ""),
                        }
                    )

                prev_aeb_active = aeb_active
                prev_fcw_active = fcw_active

                aeb_timeline.add(
                    {
                        "time": _time_str(m["log_time_ns"]),
                        "aeb_active": aeb_active,
                        "aeb_brake_enabled": aeb_brake_en,
                        "fcw_active": fcw_active,
                        "brake_pedal_target": brake_pedal,
                        "work_status": getattr(d, "acu_aeb_work_sta", None),
                    }
                )

            # RSS distance
            for m in content.get(rss_topic, []):
                rss_timeline.add(
                    {
                        "time": _time_str(m["log_time_ns"]),
                        "data": _dictify(m["data"]),
                    }
                )

        # AEB obstacle debug (aeb_interfaces) — 与触发距离交叉验证
        obstacle_topic = "/control/autonomous_emergency_braking/debug/obstacle_data"
        obstacle_samples = []
        try:
            obst = _collect_topic_data(file_path, [obstacle_topic], max_per_topic=5)
            for m in obst.get(obstacle_topic, [])[:5]:
                obstacle_samples.append(
                    {
                        "time": _time_str(m["log_time_ns"]),
                        "parsed": _parse_aeb_obstacle_debug(m["data"]),
                    }
                )
        except Exception:
            pass

        return {
            "content": [
                {
                    "type": "text",
                    "text": json.dumps(
                        {
                            "file_path": file_path,
                            "ros2_decoding": _decoding_status(),
                            "source_topic": source_topic,
                            "data_source": (
                                "AebInfoShadow (new)"
                                if shadow_topic
                                else ("AvAebCtrl (legacy)" if has_legacy else "none")
                            ),
                            "message_semantics": {
                                "aeb_info_shadow": CUSTOM_MESSAGE_CATALOG.get(
                                    "aeb_interfaces/msg/AebInfoShadow"
                                ),
                                "obstacle_debug": CUSTOM_MESSAGE_CATALOG.get(
                                    "aeb_interfaces/msg/AebObstacleDebugData"
                                ),
                            },
                            "aeb_events": aeb_events,
                            "total_aeb_events": len(aeb_events),
                            # timeline 只截断返回 payload，事件检测已基于全量数据完成
                            "aeb_timeline": aeb_timeline.to_list(),
                            "rss_distance_timeline": rss_timeline.to_list(),
                            "obstacle_debug_samples": obstacle_samples,
                            "summary": {
                                "aeb_activations": len(
                                    [e for e in aeb_events if e["event"] == "AEB_ACTIVATED"]
                                ),
                                "fcw_activations": len(
                                    [e for e in aeb_events if e["event"] == "FCW_ACTIVATED"]
                                ),
                                "aeb_status_faults": len(
                                    [e for e in aeb_events if e["event"] == "AEB_STATUS_FAULT"]
                                ),
                                "fcw_status_faults": len(
                                    [e for e in aeb_events if e["event"] == "FCW_STATUS_FAULT"]
                                ),
                                "requests_filtered": len(
                                    [
                                        e for e in aeb_events
                                        if e["event"].endswith("REQUEST_FILTERED")
                                    ]
                                ),
                                "total_aeb_messages": total_aeb_msgs,
                                "total_rss_messages": rss_count,
                                "total_emergency_commands": emergency_count,
                            },
                        },
                        ensure_ascii=False,
                        indent=2,
                        default=str,
                    ),
                }
            ],
        }

    # ===================================================================
    # analyze_vehicle_health
    # ===================================================================
    @server.tool(
        name="analyze_vehicle_health",
        description="【严谨模式】综合健康检查 — 一次性检查感知/规划控制/MRM/AEB/底盘所有模块。file_path 必须与用户提供的一致，不可编造。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _analyze_vehicle_health(args: Dict):
        file_path = _require_file_path(args)

        # 流式单遍扫描：基本统计 + 目标话题时间戳一次完成（旧版整文件读内存 ×2）
        reader, fp = _open_mcap(file_path)
        try:
            scan = _scan_mcap(reader, watch_topics=TOPIC_ALL_HEALTH)
        finally:
            fp.close()

        # 1. Basic stats
        stats = _format_stats(scan)

        # 2. Topic presence and frequency health
        timestamps = scan["timestamps"]

        expected_freqs_all = {
            "/perception/object_recognition/detection/bevfusion/objects": 10,
            "/perception/object_recognition/tracking/objects": 10,
            "/perception/object_recognition/objects": 10,
            "/sensing/lidar/concatenated/pointcloud_cloudini": 10,
            "/control/command/control_cmd": 33,
            "/control/command/actuation_cmd": 33,
            "/control/trajectory_follower/control_cmd": 33,
            "/planning/scenario_planning/scenario": 10,
            "/control/current_gate_mode": 10,
            "/pix_robobus/av_drive_ctrl": 50,
            "/pix_robobus/av_brake_ctrl": 50,
            "/pix_robobus/va_drive_sta_fb": 50,
            "/pix_robobus/va_brake_sta_fb": 50,
            "/pix_robobus/va_steer_sta_fb": 50,
            "/pix_robobus/va_chassis_wheel_rpm_fb": 50,
            "/vehicle/status/actuation_status": 50,
            "/vehicle/status/velocity_status": 50,
            "/vehicle/status/steering_status": 50,
            "/sensing/gnss/fix": 100,
            "/sensing/gnss/imu": 100,
            "/localization/kinematic_state": 50,
            "/diagnostics": 840,
            "/tf": 160,
        }

        topic_health = {}
        topic_categories = {
            "perception": TOPIC_PERCEPTION,
            "planning_control": TOPIC_PLANNING_CONTROL,
            "chassis": TOPIC_CHASSIS,
            "localization": TOPIC_LOCALIZATION,
            "mrm": TOPIC_MRM,
            "aeb": TOPIC_AEB,
        }

        for topic in TOPIC_ALL_HEALTH:
            ts = timestamps.get(topic, [])
            freq = _freq_analysis(ts, expected_freqs_all.get(topic))
            topic_health[topic] = freq

        # 3. Collect anomalies from all subsystems
        all_anomalies = defaultdict(list)

        for topic, freq in topic_health.items():
            if freq.get("count", 0) == 0:
                if topic in AEB_FEEDBACK_GROUP and _group_has_data(
                    topic_health, AEB_FEEDBACK_GROUP
                ):
                    continue  # AEB 反馈新旧话题是替代关系，另一个存在即可
                all_anomalies["missing"].append(topic)
            elif not freq.get("healthy", True):
                all_anomalies["frequency"].append(
                    {
                        "topic": topic,
                        "actual_hz": freq.get("actual_frequency_hz"),
                        "expected_hz": freq.get("expected_frequency_hz"),
                        "gaps": freq.get("large_gap_count"),
                    }
                )

        # 4. Scoring
        # AEB 反馈新旧话题算作 1 个逻辑话题，避免老/新录音互相拉低分母
        total_topics = len(TOPIC_ALL_HEALTH)
        group_in_set = [t for t in AEB_FEEDBACK_GROUP if t in TOPIC_ALL_HEALTH]
        if len(group_in_set) > 1:
            total_topics -= len(group_in_set) - 1

        present_topics = sum(
            1 for t in TOPIC_ALL_HEALTH if topic_health.get(t, {}).get("count", 0) > 0
        )
        healthy_topics = sum(
            1
            for t in TOPIC_ALL_HEALTH
            if topic_health.get(t, {}).get("healthy", False)
        )
        # 组内多个话题同时存在（新旧切换期录音）时只算 1 个逻辑话题，
        # 否则覆盖率/健康分可能超过 100%
        group_present = sum(
            1 for t in AEB_FEEDBACK_GROUP if topic_health.get(t, {}).get("count", 0) > 0
        )
        group_healthy = sum(
            1 for t in AEB_FEEDBACK_GROUP if topic_health.get(t, {}).get("healthy", False)
        )
        present_topics -= max(0, group_present - 1)
        healthy_topics -= max(0, group_healthy - 1)

        coverage_score = round(present_topics / total_topics * 100, 1) if total_topics else 0
        health_score = round(healthy_topics / total_topics * 100, 1) if total_topics else 0

        # 5. Category-level health
        category_health = {}
        for cat_name, cat_topics in topic_categories.items():
            cat_group = [t for t in cat_topics if t in AEB_FEEDBACK_GROUP]
            base_topics = [t for t in cat_topics if t not in AEB_FEEDBACK_GROUP]
            base_present = sum(
                1
                for t in base_topics
                if topic_health.get(t, {}).get("count", 0) > 0
            )
            group_present = (
                1
                if any(topic_health.get(t, {}).get("count", 0) > 0 for t in cat_group)
                else 0
            )
            cat_present = base_present + group_present
            cat_total = len(base_topics) + (1 if cat_group else 0)
            category_health[cat_name] = {
                "topics_present": f"{cat_present}/{cat_total}",
                "health_score": round(cat_present / cat_total * 100, 1) if cat_total else 0,
            }

        return {
            "content": [
                {
                    "type": "text",
                    "text": json.dumps(
                        {
                            "file_path": file_path,
                            "file_size_mb": round(os.path.getsize(file_path) / (1024 * 1024), 2),
                            "ros2_decoding": _decoding_status(),
                            "basic_info": {
                                "total_messages": stats.get("total_messages"),
                                "unique_topics": stats.get("unique_topics"),
                                "time_range": stats.get("time_range_ns"),
                            },
                            "health_summary": {
                                "overall_health_score": health_score,
                                "topic_coverage_score": coverage_score,
                                "total_monitored_topics": total_topics,
                                "present_topics": present_topics,
                                "healthy_topics": healthy_topics,
                            },
                            "category_health": category_health,
                            "all_topic_health": topic_health,
                            "anomalies_summary": {
                                "missing_topics": all_anomalies["missing"],
                                "frequency_anomalies": all_anomalies["frequency"],
                                "total_anomalies": len(all_anomalies["missing"])
                                + len(all_anomalies["frequency"]),
                            },
                        },
                        ensure_ascii=False,
                        indent=2,
                        default=str,
                    ),
                }
            ],
        }

    # ===================================================================
    # investigate_incident — 时间参数解析
    # ===================================================================
    def _parse_time_value(val: Any, label: str = "time") -> int:
        """Parse a time value (Unix seconds float or ISO string) to nanoseconds."""
        if val is None:
            raise McpError(INVALID_PARAMS, f"Missing required parameter: {label}")
        if isinstance(val, (int, float)):
            return int(val * 1e9)
        if isinstance(val, str):
            return int(dateutil_parser.parse(val.strip()).timestamp() * 1e9)
        raise McpError(INVALID_PARAMS, f"Cannot parse {label}={val!r} (expected Unix seconds or ISO string)")

    # ===================================================================
    # investigate_incident — 时间点/时间段分析
    # ===================================================================
    @server.tool(
        name="investigate_incident",
        description="【严谨模式】分析某个时间点或时间段的车辆状态 — 排查规划/感知/底盘异常。\n"
        "⚠️ 使用规则：\n"
        "1. 时间参数必须与用户指定的时间完全一致，严禁篡改。\n"
        "2. 单点模式：传 timestamp(Unix秒) 或 timestamp_iso(如'2026-07-20 10:04:30')，可配合 window_seconds（默认2秒）。\n"
        "3. 时间段模式：传 start/end（Unix秒）或 start_iso/end_iso（ISO字符串），必须成对使用。\n"
        "4. 调用后必须核对返回的 time_range 是否与用户要求一致，不一致需向用户说明。\n"
        "5. window_seconds 建议不超过30秒。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须与用户提供的路径一致，不可编造）",
                },
                "timestamp": {
                    "type": "number",
                    "description": "单点模式：目标时间点（Unix 秒）。必须与用户指定时刻完全一致，不可偏移。示例：1718000000",
                },
                "timestamp_iso": {
                    "type": "string",
                    "description": "单点模式：ISO 时间字符串。必须与用户指定时刻完全一致，不可偏移。格式：'2026-07-20 10:04:30' 或 '2026-07-20T10:04:30'",
                },
                "window_seconds": {
                    "type": "number",
                    "description": "单点模式：前后分析窗口（秒），默认 2.0。建议不超过 30 秒。",
                },
                "start": {
                    "type": "number",
                    "description": "时间段模式：开始时间（Unix 秒）。必须与用户指定的开始时间一致。",
                },
                "end": {
                    "type": "number",
                    "description": "时间段模式：结束时间（Unix 秒）。必须与用户指定的结束时间一致。",
                },
                "start_iso": {
                    "type": "string",
                    "description": "时间段模式：开始 ISO 时间。必须与用户指定的开始时间一致。格式：'2026-07-20 10:04:30'",
                },
                "end_iso": {
                    "type": "string",
                    "description": "时间段模式：结束 ISO 时间。必须与用户指定的结束时间一致。格式：'2026-07-20 10:05:00'",
                },
            },
            "required": ["file_path"],
        },
    )
    def _investigate_incident(args: Dict):
        file_path = _require_file_path(args)

        # Determine mode: time point vs time range
        is_range = bool(
            args.get("start") is not None or args.get("start_iso") is not None
            or args.get("end") is not None or args.get("end_iso") is not None
        )
        is_point = bool(
            args.get("timestamp") is not None or args.get("timestamp_iso") is not None
        )

        if is_range:
            # Time range mode — 直接取值，避免 args[key] 的 KeyError 风险
            ts_start_ns = _parse_time_value(
                args.get("start", args.get("start_iso")),
                label="start/start_iso",
            )
            ts_end_ns = _parse_time_value(
                args.get("end", args.get("end_iso")),
                label="end/end_iso",
            )
            if ts_end_ns <= ts_start_ns:
                raise McpError(INVALID_PARAMS, "end must be after start")
            ts_ns = None  # no center point
        elif is_point:
            # Single point mode
            ts_ns = _parse_time_value(
                args.get("timestamp", args.get("timestamp_iso")),
                label="timestamp/timestamp_iso",
            )
            window = args.get("window_seconds", 2.0)
            if window <= 0:
                raise McpError(INVALID_PARAMS, "window_seconds must be positive")
            ts_start_ns = ts_ns - int(window * 1e9)
            ts_end_ns = ts_ns + int(window * 1e9)
        else:
            raise McpError(
                INVALID_PARAMS,
                "Provide either timestamp/timestamp_iso (point mode) "
                "or start/end or start_iso/end_iso (range mode)",
            )

        # Topics to investigate — 包含 rosout（节点崩溃日志）、/diagnostics（模块健康）
        # 和规划轨迹话题（排查「规划轨迹消失/异常」）
        investigate_topics = list(set(
            TOPIC_PERCEPTION + TOPIC_PLANNING_CONTROL + TOPIC_MRM
            + TOPIC_AEB + TOPIC_LOCALIZATION + TOPIC_CHASSIS
            + TOPIC_PLANNING_TRAJECTORY
            + ["/rosout", "/diagnostics"]
        ))

        # Collect messages within the time window
        reader, fp = _open_mcap(file_path)
        summary, ch_topic = _get_summary(reader)

        window_data: Dict[str, list] = defaultdict(list)
        topic_counts: Counter = Counter()       # 窗口内真实消息数（不受精简影响）
        topic_time_range: Dict[str, list] = {}  # 窗口内每话题 [min,max] log_time
        file_first_log = None                   # 全文件时间范围（用于空窗口提示，
        file_last_log = None                    #  避免二次读文件）
        decoder = Ros2DecoderFactory() if HAVE_ROS2 else None

        try:
            for _schema, channel, msg in reader.iter_messages(log_time_order=False, topics=investigate_topics):
                lt = msg.log_time
                if file_first_log is None or lt < file_first_log:
                    file_first_log = lt
                if file_last_log is None or lt > file_last_log:
                    file_last_log = lt

                topic = ch_topic.get(channel.id, "unknown") if channel else "unknown"
                if topic not in investigate_topics:
                    continue
                # 使用 log_time（MCAP 记录时间）作为时间过滤依据，比 publish_time（节点时钟）更可靠
                if lt < ts_start_ns or lt > ts_end_ns:
                    continue

                topic_counts[topic] += 1
                tr = topic_time_range.get(topic)
                if tr is None:
                    topic_time_range[topic] = [lt, lt]
                else:
                    if lt < tr[0]:
                        tr[0] = lt
                    if lt > tr[1]:
                        tr[1] = lt

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

                # 大体消息内联精简，控制内存与 payload：
                # - /diagnostics（可达数百 Hz × 上百 status）：只保留非 OK status，全 OK 不存储
                # - 规划轨迹（数百个点）：只保留点数 + 首尾点
                if topic == "/diagnostics":
                    reduced = _reduce_diagnostics(decoded)
                    if not reduced:
                        continue  # 计数已记录，无异常内容可存
                    data = {"non_ok_statuses": reduced}
                elif topic in TOPIC_PLANNING_TRAJECTORY:
                    data = _reduce_trajectory(decoded)
                else:
                    data = _dictify(decoded if decoded is not None else msg.data, max_depth=4)

                window_data[topic].append({
                    "publish_time_ns": msg.publish_time,
                    "publish_time_iso": _time_str(msg.publish_time),
                    "log_time_ns": lt,
                    "log_time_iso": _time_str(lt),
                    "data": data,
                })
        except Exception as e:
            log.error("investigate_incident: iter_messages failed: %s", traceback.format_exc())
            raise McpError(INTERNAL_ERROR, f"Error reading messages from MCAP file: {e}")
        finally:
            fp.close()

        # Sort each topic by log_time for temporal consistency
        for topic in window_data:
            window_data[topic].sort(key=lambda x: x["log_time_ns"])

        # Check if window has any data（用真实计数，不经精简）
        total_in_window = sum(topic_counts.values())
        if total_in_window == 0:
            # 文件整体时间范围在主扫描中已顺带记录，直接给出提示
            file_range_info = ""
            if file_first_log is not None and file_last_log is not None:
                file_range_info = (
                    f" File time range: {_time_str(file_first_log)} ~ {_time_str(file_last_log)}. "
                    f"Your window: {_time_str(ts_start_ns)} ~ {_time_str(ts_end_ns)}."
                )
            raise McpError(
                INVALID_PARAMS,
                f"No messages found in the specified time window.{file_range_info} "
                "Check that the timestamp is within the recording's time range.",
            )

        # Point mode: snapshot (closest to target) — 基于全量数据，用 log_time 选最近
        snapshot = None
        if is_point and ts_ns is not None:
            snapshot = {}
            for topic, msgs in window_data.items():
                if not msgs:
                    continue
                closest = min(msgs, key=lambda m: abs(m["log_time_ns"] - ts_ns))
                snapshot[topic] = {
                    "log_time_iso": closest["log_time_iso"],
                    "publish_time_iso": closest["publish_time_iso"],
                    "time_offset_ms": round((closest["log_time_ns"] - ts_ns) / 1e6, 1),
                    "data": closest["data"],
                }

        # Build topic presence summary — 用采集阶段的真实计数与时间范围
        topic_presence = {}
        for topic in investigate_topics:
            entry = {"messages_in_window": topic_counts.get(topic, 0)}
            tr = topic_time_range.get(topic)
            if tr:
                entry["first_time"] = _time_str(tr[0])
                entry["last_time"] = _time_str(tr[1])
            topic_presence[topic] = entry

        # Detect anomalies / events in the time range
        # ⚠️ 全部基于未截断的 window_data；告警用「状态跳变」而非逐条消息
        # （emergency_cmd 57Hz、aeb 11Hz 持续发布，逐条报会刷屏且淹没关键信息）
        alerts = []

        # Emergency commands — emergency 字段 False->True / True->False 跳变
        prev_emg = None
        for m in window_data.get("/control/command/emergency_cmd", []):
            emg = m["data"].get("emergency") if isinstance(m["data"], dict) else None
            if emg is None:
                continue
            if emg and prev_emg is not True:
                alerts.append({
                    "time": m["log_time_iso"],
                    "topic": "/control/command/emergency_cmd",
                    "type": "emergency_command",
                    "severity": "error",
                    "detail": "Emergency control command activated"
                    + (" (already active at window start)" if prev_emg is None else ""),
                })
            elif not emg and prev_emg is True:
                alerts.append({
                    "time": m["log_time_iso"],
                    "topic": "/control/command/emergency_cmd",
                    "type": "emergency_cleared",
                    "severity": "info",
                    "detail": "Emergency control command cleared",
                })
            prev_emg = emg

        # AEB/FCW events — 跳变检测
        prev_aeb = None
        prev_fcw = None
        for m in window_data.get("/pix_robobus/av_aeb_ctrl", []):
            d = m["data"]
            if not isinstance(d, dict):
                continue
            aeb = bool(d.get("acu_aeb_active_request") or d.get("acu_aeb_brake_en"))
            fcw = bool(d.get("acu_fcw_active_request"))
            if aeb and prev_aeb is not True:
                alerts.append({
                    "time": m["log_time_iso"],
                    "topic": "/pix_robobus/av_aeb_ctrl",
                    "type": "aeb_active",
                    "severity": "error",
                    "detail": f"AEB active: brake_en={d.get('acu_aeb_brake_en')}, pedal_target={d.get('acu_aeb_brake_pedal_target')}"
                    + (" (already active at window start)" if prev_aeb is None else ""),
                })
            if fcw and prev_fcw is not True:
                alerts.append({
                    "time": m["log_time_iso"],
                    "topic": "/pix_robobus/av_aeb_ctrl",
                    "type": "fcw_active",
                    "severity": "warning",
                    "detail": "FCW active"
                    + (" (already active at window start)" if prev_fcw is None else ""),
                })
            prev_aeb = aeb
            prev_fcw = fcw

        # AebInfoShadow（新版 AEB/FCW 反馈）— 跳变检测（以滤波后请求为准）
        prev_shadow = {"aeb_req": None, "fcw_req": None, "aeb_fault": None, "fcw_fault": None}
        for shadow_t in TOPIC_AEB_SHADOW:
            for m in window_data.get(shadow_t, []):
                d = m["data"]
                if not isinstance(d, dict):
                    continue
                p = _parse_aeb_shadow(d)
                aeb_req = bool(p["aeb_req_filter"]) if p["aeb_req_filter"] is not None else None
                fcw_req = bool(p["fcw_req_filter"]) if p["fcw_req_filter"] is not None else None
                aeb_status = p["aeb_status"]
                fcw_status = p["fcw_status"]

                if aeb_req and prev_shadow["aeb_req"] is not True:
                    alerts.append({
                        "time": m["log_time_iso"],
                        "topic": shadow_t,
                        "type": "aeb_request",
                        "severity": "error",
                        "detail": f"AEB request activated (filtered) [status={p['aeb_status_label']}]"
                        + (" (already active at window start)" if prev_shadow["aeb_req"] is None else ""),
                    })
                if fcw_req and prev_shadow["fcw_req"] is not True:
                    alerts.append({
                        "time": m["log_time_iso"],
                        "topic": shadow_t,
                        "type": "fcw_request",
                        "severity": "warning",
                        "detail": f"FCW request activated (filtered) [status={p['fcw_status_label']}]"
                        + (" (already active at window start)" if prev_shadow["fcw_req"] is None else ""),
                    })
                if aeb_status == 2 and prev_shadow["aeb_fault"] is not True:
                    alerts.append({
                        "time": m["log_time_iso"],
                        "topic": shadow_t,
                        "type": "aeb_fault",
                        "severity": "error",
                        "detail": "AEB status=FAULT"
                        + (" (already fault at window start)" if prev_shadow["aeb_fault"] is None else ""),
                    })
                if fcw_status == 2 and prev_shadow["fcw_fault"] is not True:
                    alerts.append({
                        "time": m["log_time_iso"],
                        "topic": shadow_t,
                        "type": "fcw_fault",
                        "severity": "error",
                        "detail": "FCW status=FAULT"
                        + (" (already fault at window start)" if prev_shadow["fcw_fault"] is None else ""),
                    })

                prev_shadow["aeb_req"] = aeb_req
                prev_shadow["fcw_req"] = fcw_req
                prev_shadow["aeb_fault"] = aeb_status == 2
                prev_shadow["fcw_fault"] = fcw_status == 2

        # MRM / Engage changes — 跳变检测（窗口开始即 disengaged 也报一次）
        prev_eng = None
        for m in window_data.get("/api/autoware/get/engage", []):
            engaged = _extract_engage(m["data"])
            if engaged is None:
                continue
            if engaged is False and prev_eng is not False:
                alerts.append({
                    "time": m["log_time_iso"],
                    "topic": "/api/autoware/get/engage",
                    "type": "disengaged",
                    "severity": "error",
                    "detail": "Autoware disengaged (engage=False)"
                    + (" (already disengaged at window start)" if prev_eng is None else ""),
                })
            elif engaged is True and prev_eng is False:
                alerts.append({
                    "time": m["log_time_iso"],
                    "topic": "/api/autoware/get/engage",
                    "type": "engaged",
                    "severity": "info",
                    "detail": "Autoware engaged",
                })
            prev_eng = engaged

        # Emergency flag — 跳变检测
        prev_eflag = None
        for m in window_data.get("/api/autoware/get/emergency", []):
            d = m["data"]
            eflag = bool(d.get("emergency")) if isinstance(d, dict) else None
            if eflag is None:
                continue
            if eflag and prev_eflag is not True:
                alerts.append({
                    "time": m["log_time_iso"],
                    "topic": "/api/autoware/get/emergency",
                    "type": "emergency_flag",
                    "severity": "error",
                    "detail": "Emergency flag ON"
                    + (" (already ON at window start)" if prev_eflag is None else ""),
                })
            prev_eflag = eflag

        # Gate mode changes
        prev_gate = None
        for m in window_data.get("/control/current_gate_mode", []):
            d = m["data"]
            mode = d.get("data", d.get("gate_mode", d.get("mode"))) if isinstance(d, dict) else None
            if mode is not None and prev_gate is not None and mode != prev_gate:
                alerts.append({
                    "time": m["log_time_iso"],
                    "topic": "/control/current_gate_mode",
                    "type": "gate_mode_change",
                    "severity": "info",
                    "detail": f"Gate mode changed: {prev_gate} -> {mode}",
                })
            if mode is not None:
                prev_gate = mode

        # Control command changes (detect abrupt steering)
        # autoware_control_msgs/Control: lateral.steering_tire_angle (rad)
        prev_steer = None
        for m in window_data.get("/control/command/control_cmd", []):
            d = m["data"]
            if not isinstance(d, dict):
                continue
            lat = d.get("lateral") if isinstance(d.get("lateral"), dict) else {}
            steer = lat.get("steering_tire_angle")
            if steer is None:
                steer = d.get("steering", d.get("steering_angle"))
            if steer is not None and prev_steer is not None and abs(steer - prev_steer) > 0.3:
                alerts.append({
                    "time": m["log_time_iso"],
                    "topic": "/control/command/control_cmd",
                    "type": "abrupt_steering",
                    "severity": "warning",
                    "detail": f"Abrupt steering change: {prev_steer:.3f} -> {steer:.3f}",
                })
            if steer is not None:
                prev_steer = steer

        # Diagnostics — 基于采集阶段精简的非 OK status，按 (name,level,message) 去重计数
        # level 语义：0=OK, 1=WARN, 2=ERROR, 3=STALE（旧版 ==1 把 WARN 误报为 ERROR 且漏掉真 ERROR）
        diag_seen: Dict[tuple, Dict] = {}
        for m in window_data.get("/diagnostics", []):
            for st in m["data"].get("non_ok_statuses", []):
                key = (st["name"], st["level"], st["message"])
                e = diag_seen.get(key)
                if e:
                    e["count"] += 1
                else:
                    diag_seen[key] = {"first": m["log_time_iso"], "count": 1}
        for (name, level, message), e in sorted(
            diag_seen.items(), key=lambda kv: (-kv[0][1], kv[0][0])
        ):
            label = _DIAG_LEVEL_LABELS.get(level, f"LEVEL{level}")
            alerts.append({
                "time": e["first"],
                "topic": "/diagnostics",
                "type": f"diagnostic_{label.lower()}",
                "severity": "error" if level >= 2 else "warning",
                "detail": f"Diag {label}: {name} - {message[:100]} (x{e['count']})",
            })

        # Rosout 错误日志 — 捕获节点崩溃 / 致命错误 (ROS level >= 40 = ERROR, >= 50 = FATAL)
        rosout_alert_count = 0
        rosout_suppressed = 0
        for m in window_data.get("/rosout", []):
            d = m["data"]
            if not isinstance(d, dict):
                continue
            level = d.get("level", 0)
            if level < 40:
                continue
            if rosout_alert_count >= 100:
                rosout_suppressed += 1
                continue
            name = d.get("name", "?")
            msg_text = str(d.get("msg", ""))[:200]
            alerts.append({
                "time": m["log_time_iso"],
                "topic": "/rosout",
                "type": "rosout_fatal" if level >= 50 else "rosout_error",
                "severity": "error" if level >= 50 else "warning",
                "detail": f"[{'FATAL' if level >= 50 else 'ERROR'}] {name}: {msg_text}",
            })
            rosout_alert_count += 1
        if rosout_suppressed:
            alerts.append({
                "time": None,
                "topic": "/rosout",
                "type": "rosout_suppressed",
                "severity": "info",
                "detail": f"{rosout_suppressed} more rosout error/fatal messages suppressed (cap 100)",
            })

        # Deduplicate alerts (same time + same type)
        seen_alerts = set()
        unique_alerts = []
        for a in alerts:
            key = (a["time"], a["type"])
            if key not in seen_alerts:
                seen_alerts.add(key)
                unique_alerts.append(a)

        TL_PLANNING = [
            "/planning/scenario_planning/trajectory",
            "/planning/scenario_planning/lane_driving/trajectory",
            "/planning/scenario_planning/scenario",
            "/control/trajectory_follower/lateral/diagnostic",
            "/control/trajectory_follower/longitudinal/diagnostic",
        ]
        TL_STATE = [
            "/vehicle/status/velocity_status", "/vehicle/status/steering_status",
            "/vehicle/status/actuation_status", "/control/command/control_cmd",
            "/control/current_gate_mode", "/api/autoware/get/engage",
        ]

        def _build_payload(tl_cap: int, snap_chars: int) -> Dict[str, Any]:
            # payload 截断只发生在这里（timeline 每话题最多 tl_cap 条；
            # snapshot 每话题数据最多 snap_chars 字符）；
            # 告警/presence/计数均基于全量数据，不受降级影响
            timeline = {}
            if tl_cap > 0:
                for tl_topic in TL_PLANNING + TL_STATE:
                    msgs = window_data.get(tl_topic, [])
                    if msgs:
                        timeline[tl_topic] = [
                            {"t": m["log_time_iso"], "d": m["data"]} for m in msgs[:tl_cap]
                        ]
            snapshot_payload = None
            if snapshot:
                snapshot_payload = {
                    t: {
                        "log_time_iso": s["log_time_iso"],
                        "publish_time_iso": s["publish_time_iso"],
                        "time_offset_ms": s["time_offset_ms"],
                        "data": _cap_snapshot_data(s["data"], snap_chars),
                    }
                    for t, s in snapshot.items()
                }
            return {
                "mode": "range" if is_range else "point",
                "ros2_decoding": _decoding_status(),
                "time_range": {
                    "start_iso": _time_str(ts_start_ns),
                    "end_iso": _time_str(ts_end_ns),
                    "duration_seconds": round((ts_end_ns - ts_start_ns) / 1e9, 2),
                },
                "snapshot": snapshot_payload,
                "timeline": timeline,
                "topic_presence": topic_presence,
                "alerts": unique_alerts,
                "total_alerts": len(unique_alerts),
                "summary": {
                    "topics_with_data": len([t for t, v in topic_presence.items() if v["messages_in_window"] > 0]),
                    "total_topics_monitored": len(investigate_topics),
                    "total_messages_in_window": sum(v["messages_in_window"] for v in topic_presence.values()),
                    "alerts_found": len(unique_alerts),
                },
            }

        # timeline/snapshot 体积随窗口时长和消息大小变化大，逐步降级以保证返回完整 JSON
        budget = MAX_RESPONSE_CHARS - 15000
        text = ""
        for tl_cap, snap_chars in ((30, 6000), (10, 3000), (4, 1500), (1, 800), (0, 0)):
            payload = _build_payload(tl_cap, snap_chars)
            if tl_cap != 30 or snap_chars != 6000:
                payload["summary"]["timeline_entries_per_topic"] = tl_cap
                payload["summary"]["snapshot_data_chars_per_topic"] = snap_chars
            text = json.dumps(payload, ensure_ascii=False, indent=2, default=str)
            if len(text) <= budget:
                break

        return {
            "content": [
                {
                    "type": "text",
                    "text": text,
                }
            ],
        }

    # ===================================================================
    # describe_message_schema — 自定义消息 schema / 字段语义
    # ===================================================================
    @server.tool(
        name="describe_message_schema",
        description=(
            "【严谨模式】查询消息 schema：topic 或 schema 名。返回 MCAP 内嵌 .msg/.idl "
            "定义，并附 Pix/私有相对原版 Autoware 的字段差异说明。"
            "感知/规划控制/pix 消息优先用这个理解字段。"
            "file_path 必须与用户提供的一致。"
        ),
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
                "topic": {
                    "type": "string",
                    "description": "话题名，如 /pix/AebInfoShadow（与 schema 二选一）",
                },
                "schema_name": {
                    "type": "string",
                    "description": "完整 schema 名，如 pix_robobus_driver_msgs/msg/VaSteerStaFb",
                },
                "include_schema_text": {
                    "type": "boolean",
                    "description": "是否返回完整 schema 文本（默认 true，可关掉只拿语义摘要）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _describe_message_schema(args: Dict):
        file_path = _require_file_path(args)
        topic = args.get("topic")
        schema_name = args.get("schema_name")
        include_text = args.get("include_schema_text")
        if include_text is None:
            include_text = True
        if not topic and not schema_name:
            raise McpError(
                INVALID_PARAMS, "Provide topic or schema_name"
            )

        reader, fp = _open_mcap(file_path)
        try:
            summary = reader.get_summary()
        finally:
            fp.close()
        if not summary:
            raise McpError(INTERNAL_ERROR, "No summary section in MCAP")

        # Resolve schema
        matched_schemas = []
        topic_info = None
        if topic:
            for ch in summary.channels.values():
                if ch.topic == topic:
                    sch = summary.schemas.get(ch.schema_id)
                    topic_info = {
                        "topic": ch.topic,
                        "encoding": ch.message_encoding,
                        "schema_id": ch.schema_id,
                    }
                    if sch:
                        matched_schemas.append(sch)
                    break
            if topic_info is None:
                raise McpError(INVALID_PARAMS, f"Topic not found in MCAP: {topic}")
        else:
            for sch in summary.schemas.values():
                if sch.name == schema_name or sch.name.endswith("/" + schema_name):
                    matched_schemas.append(sch)
            if not matched_schemas:
                # prefix match
                for sch in summary.schemas.values():
                    if schema_name and schema_name in sch.name:
                        matched_schemas.append(sch)

        results = []
        for sch in matched_schemas[:5]:
            text = sch.data.decode("utf-8", "replace") if sch.data else ""
            top = text.split(
                "================================================================================"
            )[0].strip() if text else ""
            # 话题列表
            topics_using = [
                ch.topic
                for ch in summary.channels.values()
                if ch.schema_id == sch.id
            ]
            entry = {
                "schema_name": sch.name,
                "encoding": sch.encoding,
                "topics_using_schema": topics_using,
                "custom_semantics": CUSTOM_MESSAGE_CATALOG.get(sch.name),
            }
            if include_text:
                # 顶层定义优先；全文过长时截断嵌套部分
                entry["schema_top_level"] = top[:4000]
                if len(text) > 4000:
                    entry["schema_full_length"] = len(text)
                    entry["schema_full_truncated"] = text[:8000]
            results.append(entry)

        # Also list custom catalog hits when schema unknown
        catalog_notes = None
        if not results and schema_name:
            hits = {
                k: v
                for k, v in CUSTOM_MESSAGE_CATALOG.items()
                if schema_name.lower() in k.lower()
            }
            if hits:
                catalog_notes = hits

        return {
            "content": [
                {
                    "type": "text",
                    "text": json.dumps(
                        {
                            "file_path": file_path,
                            "topic": topic,
                            "schema_name_query": schema_name,
                            "topic_info": topic_info,
                            "schemas": results,
                            "catalog_notes": catalog_notes,
                            "custom_catalog_index": sorted(
                                CUSTOM_MESSAGE_CATALOG.keys()
                            ),
                            "note": (
                                "Pix/pix_robobus/aeb_interfaces/point_cloud_interfaces "
                                "为私有或改造消息，字段单位见 custom_semantics"
                            ),
                        },
                        ensure_ascii=False,
                        indent=2,
                        default=str,
                    ),
                }
            ],
        }

    # ===================================================================
    # list_custom_messages — 私有/差异消息一览
    # ===================================================================
    @server.tool(
        name="list_custom_messages",
        description=(
            "列出 MCAP 中相对原版 Autoware 的私有/差异消息（pix_*、aeb_interfaces、"
            "point_cloud_interfaces 等），含字段语义。file_path 必须与用户提供的一致。"
        ),
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _list_custom_messages(args: Dict):
        file_path = _require_file_path(args)
        reader, fp = _open_mcap(file_path)
        try:
            summary = reader.get_summary()
        finally:
            fp.close()
        if not summary:
            raise McpError(INTERNAL_ERROR, "No summary section in MCAP")

        def _is_custom(name: str) -> bool:
            n = (name or "").lower()
            return any(
                x in n
                for x in (
                    "pix_",
                    "pix/",
                    "aeb_interfaces",
                    "point_cloud_interfaces",
                    "failcase",
                )
            )

        found = []
        ch_counts = {}
        try:
            ch_counts = (
                summary.statistics.channel_message_counts
                if summary.statistics and summary.statistics.channel_message_counts
                else {}
            )
        except Exception:
            pass

        for cid, ch in summary.channels.items():
            sch = summary.schemas.get(ch.schema_id)
            sname = sch.name if sch else ""
            if not (_is_custom(sname) or _is_custom(ch.topic)):
                continue
            found.append(
                {
                    "topic": ch.topic,
                    "schema": sname,
                    "message_count": ch_counts.get(cid, 0),
                    "encoding": ch.message_encoding,
                    "semantics": CUSTOM_MESSAGE_CATALOG.get(sname),
                }
            )
        found.sort(key=lambda x: -x.get("message_count", 0))

        # catalog entries present in file even if topic filter missed
        present_schemas = {sch.name for sch in summary.schemas.values()}
        catalog_in_file = {
            k: v for k, v in CUSTOM_MESSAGE_CATALOG.items() if k in present_schemas
        }

        return {
            "content": [
                {
                    "type": "text",
                    "text": json.dumps(
                        {
                            "file_path": file_path,
                            "custom_topics": found,
                            "custom_topic_count": len(found),
                            "catalog_in_file": catalog_in_file,
                            "upstream_diff_summary": [
                                "pix_robobus_driver_msgs/* — Pix CAN 私有接口，转向角单位 deg",
                                "aeb_interfaces/AebInfoShadow — AEB/FCW 影子状态（原版无）",
                                "aeb_interfaces/AebObstacleDebugData — AEB 障碍物调试",
                                "point_cloud_interfaces/CompressedPointCloud2 — 压缩点云，非 PointCloud2",
                                "autoware_control_msgs/Control — 字段在 lateral/longitudinal 嵌套内",
                            ],
                        },
                        ensure_ascii=False,
                        indent=2,
                        default=str,
                    ),
                }
            ],
        }

    # ===================================================================
    # analyze_bag_overview — 包结构总览：话题/节点/服务/元数据
    # ===================================================================
    @server.tool(
        name="analyze_bag_overview",
        description=(
            "【严谨模式】MCAP 包结构总览 — 话题命名空间分组、推断节点、服务话题、"
            "bag 元数据(ROS_DISTRO)、零消息话题。file_path 必须与用户提供的一致。"
            "适合诊断前先建立全局图景；深入模块请再调用 analyze_rosout / "
            "analyze_diagnostics / analyze_vehicle_health。"
        ),
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _analyze_bag_overview(args: Dict):
        file_path = _require_file_path(args)
        # 单遍流式：summary 统计 + 全话题扫描 + rosout/diagnostics 解码合并做
        # （旧实现对大包开 3 次文件，355MB 会明显变慢）
        rosout_nodes: Counter = Counter()
        diag_hw: Counter = Counter()
        reader, fp = _open_mcap(file_path)
        try:
            summary = reader.get_summary()
            bag_meta = _read_bag_metadata(file_path, None)

            topic_stats: Dict[str, Dict[str, Any]] = {}
            first_t = last_t = None
            total_msgs = 0
            schema_ids: set = set()
            decoder = Ros2DecoderFactory() if HAVE_ROS2 else None
            for schema, channel, msg in reader.iter_messages(
                log_time_order=False
            ):
                topic = channel.topic if channel else "unknown"
                if schema is not None:
                    schema_ids.add(schema.id)
                ts = topic_stats.get(topic)
                if ts is None:
                    ts = topic_stats[topic] = {
                        "count": 0,
                        "first_log_time": msg.log_time,
                        "last_log_time": msg.log_time,
                        "schema": schema.name if schema else "unknown",
                    }
                ts["count"] += 1
                ts["first_log_time"] = min(ts["first_log_time"], msg.log_time)
                ts["last_log_time"] = max(ts["last_log_time"], msg.log_time)
                total_msgs += 1
                if first_t is None or msg.log_time < first_t:
                    first_t = msg.log_time
                if last_t is None or msg.log_time > last_t:
                    last_t = msg.log_time

                # 同一遍解码 rosout / diagnostics，避免二次全量扫描
                if decoder and schema and topic in ("/rosout", "/diagnostics"):
                    try:
                        dec_fn = decoder.decoder_for(
                            channel.message_encoding, schema
                        )
                        if dec_fn:
                            decoded = dec_fn(msg.data)
                            if topic == "/rosout":
                                rosout_nodes[_val(decoded, "name") or "?"] += 1
                            else:
                                for st in list(_val(decoded, "status") or [])[:200]:
                                    hw = _val(st, "hardware_id")
                                    if hw:
                                        diag_hw[str(hw)] += 1
                    except Exception:
                        pass
        finally:
            fp.close()

        duration_s = (
            (last_t - first_t) / 1e9 if first_t is not None and last_t is not None else 0
        )
        topic_details: Dict[str, Dict] = {}
        for topic, ts in topic_stats.items():
            dur = (ts["last_log_time"] - ts["first_log_time"]) / 1e9
            topic_details[topic] = {
                "message_count": ts["count"],
                "first_log_time_ns": ts["first_log_time"],
                "last_log_time_ns": ts["last_log_time"],
                "duration_seconds": round(dur, 3),
                "avg_frequency_hz": round(ts["count"] / dur, 2) if dur > 0 else 0,
                "schema": ts["schema"],
            }
        stats = {
            "total_messages": total_msgs,
            "unique_topics": len(topic_stats),
            "unique_schemas": len(schema_ids),
            "time_range_ns": {
                "start": first_t,
                "end": last_t,
                "duration_seconds": round(duration_s, 3),
            },
            "topics": topic_details,
        }

        # 合并 summary channels（含 0 消息话题）与扫描统计，避免漏报
        topic_details = dict(stats.get("topics", {}))
        summary_topic_count = 0
        if summary and summary.channels:
            summary_topic_count = len(summary.channels)
            ch_counts = {}
            try:
                ch_counts = (
                    summary.statistics.channel_message_counts
                    if summary.statistics and summary.statistics.channel_message_counts
                    else {}
                )
            except Exception:
                ch_counts = {}
            schemas = summary.schemas or {}
            for cid, ch in summary.channels.items():
                if ch.topic not in topic_details:
                    n = ch_counts.get(cid, 0)
                    schema_name = (
                        schemas[ch.schema_id].name if ch.schema_id in schemas else "?"
                    )
                    topic_details[ch.topic] = {
                        "message_count": n,
                        "schema": schema_name,
                        "avg_frequency_hz": 0,
                        "note": "present in summary index; no messages in scan",
                    }

        grouping = _classify_topics_by_namespace(topic_details)
        nodes = _infer_nodes(rosout_nodes, diag_hw, topic_details)
        services = _service_like_topics(list(topic_details.keys()))

        # attachments from summary index
        attachments = []
        if summary and getattr(summary, "attachment_indexes", None):
            for a in summary.attachment_indexes[:50]:
                attachments.append(
                    {
                        "name": a.name,
                        "media_type": a.media_type,
                        "length": a.length,
                    }
                )

        duration = stats.get("time_range_ns", {}).get("duration_seconds") or 0
        return {
            "content": [
                {
                    "type": "text",
                    "text": json.dumps(
                        {
                            "file_path": file_path,
                            "file_size_mb": round(
                                os.path.getsize(file_path) / (1024 * 1024), 2
                            ),
                            "ros2_decoding": _decoding_status(),
                            "bag_metadata": bag_meta,
                            "basic_info": {
                                "total_messages": stats.get("total_messages"),
                                "unique_topics_in_scan": stats.get("unique_topics"),
                                "unique_channels_in_summary": summary_topic_count
                                or stats.get("unique_topics"),
                                "unique_schemas": stats.get("unique_schemas"),
                                "time_range": stats.get("time_range_ns"),
                                "duration_seconds": duration,
                            },
                            "topic_groups": {
                                "namespaces": grouping["namespaces"],
                                "modules": grouping["modules"],
                                "zero_message_topics": grouping["zero_message_topics"],
                            },
                            "nodes": nodes,
                            "services": {
                                "note": (
                                    "rosbag2/MCAP 通常不录制 service 调用本身；"
                                    "此处仅列出 service-like 话题名。"
                                ),
                                "count": len(services),
                                "topics": services,
                            },
                            "attachments": attachments,
                            "next_steps": [
                                "analyze_rosout — 节点 stdout/日志与 WARN/ERROR",
                                "analyze_diagnostics — /diagnostics 健康聚合",
                                "analyze_vehicle_health — 关键控制话题健康分",
                                "investigate_incident — 指定时刻/时间段细查",
                            ],
                        },
                        ensure_ascii=False,
                        indent=2,
                        default=str,
                    ),
                }
            ],
        }

    # ===================================================================
    # analyze_rosout — 节点日志 (stdout / /rosout)
    # ===================================================================
    @server.tool(
        name="analyze_rosout",
        description=(
            "【严谨模式】分析 /rosout（节点 stdout 日志）— 节点列表、日志级别分布、"
            "WARN/ERROR/FATAL 事件时间线。file_path 必须与用户提供的一致，不可编造。"
            "排查节点崩溃、CAN 超时、AEB 业务日志、感知重启等首选。"
        ),
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
                "max_events": {
                    "type": "integer",
                    "description": "最多返回的 WARN+ 事件条数（默认 200，最大 1000）",
                },
                "min_level": {
                    "type": "integer",
                    "description": "事件最低级别：10=DEBUG 20=INFO 30=WARN 40=ERROR 50=FATAL（默认 30）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _analyze_rosout_tool(args: Dict):
        file_path = _require_file_path(args)
        max_events = int(args.get("max_events") or 200)
        max_events = max(1, min(max_events, 1000))
        min_level = int(args.get("min_level") or 30)
        min_level = max(0, min(min_level, 50))
        result = _analyze_rosout_stream(
            file_path, max_events=max_events, min_level=min_level
        )
        result["file_path"] = file_path
        return {
            "content": [
                {
                    "type": "text",
                    "text": json.dumps(result, ensure_ascii=False, indent=2, default=str),
                }
            ],
        }

    # ===================================================================
    # analyze_diagnostics — /diagnostics 健康聚合
    # ===================================================================
    @server.tool(
        name="analyze_diagnostics",
        description=(
            "【严谨模式】分析 /diagnostics — hardware_id 节点清单、status 名称、"
            "级别直方图、ERROR/WARN 聚合（按 status 去重计数与起止时间）。"
            "file_path 必须与用户提供的一致，不可编造。"
        ),
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
                "max_status_details": {
                    "type": "integer",
                    "description": "最多返回的非 OK status 明细条数（默认 100，最大 500）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _analyze_diagnostics_tool(args: Dict):
        file_path = _require_file_path(args)
        max_details = int(args.get("max_status_details") or 100)
        max_details = max(1, min(max_details, 500))
        result = _analyze_diagnostics_stream(
            file_path, max_status_details=max_details
        )
        result["file_path"] = file_path
        return {
            "content": [
                {
                    "type": "text",
                    "text": json.dumps(result, ensure_ascii=False, indent=2, default=str),
                }
            ],
        }

    # ===================================================================
    # (Legacy) list_topics — kept for backward compat, reads from local file
    # ===================================================================
    @server.tool(
        name="list_topics",
        description="【严谨模式】列出 MCAP 文件中的所有话题。file_path 必须与用户提供的一致，不可编造。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _list_topics(args: Dict):
        file_path = _require_file_path(args)

        reader, fp = _open_mcap(file_path)
        try:
            summary = reader.get_summary()
        finally:
            fp.close()
        if not summary or not summary.channels:
            return {
                "content": [
                    {
                        "type": "text",
                        "text": json.dumps(
                            {"topics": [], "note": "No summary section found"},
                            indent=2,
                        ),
                    }
                ],
            }

        topics = []
        for ch in summary.channels.values():
            schema_name = (
                summary.schemas[ch.schema_id].name
                if ch.schema_id in summary.schemas
                else "unknown"
            )
            topics.append(
                {
                    "topic": ch.topic,
                    "schema": schema_name,
                    "encoding": ch.message_encoding,
                }
            )

        return {
            "content": [
                {
                    "type": "text",
                    "text": json.dumps(
                        {"topics": topics, "total": len(topics)},
                        ensure_ascii=False,
                        indent=2,
                    ),
                }
            ],
        }

    return server


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main():
    server = create_server()
    server.run()


if __name__ == "__main__":
    main()
