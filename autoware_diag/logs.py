"""/rosout and /diagnostics analysis."""
from __future__ import annotations

import json
from collections import Counter
from typing import Any, Dict, List

from .core import _decoding_status, _open_mcap, _require_file_path, _time_str
from .mcap_support import HAVE_ROS2
from .noise import _is_rosout_noise
from .parsers import _DIAG_LEVEL_LABELS, _val

_ROSOUT_LEVEL_LABELS = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}


def _analyze_rosout_stream(
    file_path: str,
    max_events: int = 200,
    min_level: int = 30,
) -> Dict[str, Any]:
    """Full-stream /rosout analysis: node histogram + WARN/ERROR/FATAL events.

    已知噪音（如未使用的 can1 超时）会从 events 中过滤，仅计入 noise_filtered。
    """
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
    noise_filtered = 0
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
                if _is_rosout_noise(name, msg_text):
                    noise_filtered += 1
                    continue
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
        "noise_filtered": noise_filtered,
        "noise_note": "can1 CAN Receive Timeout 已过滤（实车未使用 can1）",
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


def analyze_rosout(args: Dict) -> Dict:
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


def analyze_diagnostics(args: Dict) -> Dict:
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

