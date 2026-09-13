"""Bag overview helpers and tools."""
from __future__ import annotations

import json
import os
from collections import Counter
from typing import Any, Dict, List, Optional, Tuple

from .core import (
    _decoding_status,
    _format_stats,
    _open_mcap,
    _require_file_path,
    _scan_mcap,
    _time_str,
)
from .mcap_support import HAVE_MCAP, HAVE_ROS2, Ros2DecoderFactory
from .parsers import _val

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


def analyze_bag_overview(args: Dict) -> Dict:
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


def list_topics(args: Dict) -> Dict:
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

