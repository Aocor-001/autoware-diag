"""Incident window investigation."""
from __future__ import annotations

import json
import traceback
from collections import Counter, defaultdict
from typing import Any, Dict, List, Optional

from dateutil import parser as dateutil_parser

from .log import log
from .core import (
    _BoundedList,
    _cap_snapshot_data,
    _decoding_status,
    _dictify,
    _get_summary,
    _open_mcap,
    _read_topic_timestamps,
    _require_file_path,
    _time_str,
    _truncate_text,
)
from .mcap_support import HAVE_ROS2, Ros2DecoderFactory
from .mcp import INVALID_PARAMS, MAX_RESPONSE_CHARS, McpError
from .noise import _is_rosout_noise
from .parsers import (
    _AUTOWARE_STATE_LABELS,
    _CONTROL_MODE_LABELS,
    _DIAG_LEVEL_LABELS,
    _extract_control_summary,
    _extract_engage,
    _extract_gate_mode,
    _extract_scenario_summary,
    _labelize,
    _parse_aeb_shadow,
    _parse_pix_drive_fb,
    _reduce_diagnostics,
    _reduce_trajectory,
    _val,
)
from .topics import (
    TOPIC_AEB,
    TOPIC_AEB_SHADOW,
    TOPIC_CHASSIS,
    TOPIC_LOCALIZATION,
    TOPIC_MRM,
    TOPIC_PERCEPTION,
    TOPIC_PLANNING_CONTROL,
    TOPIC_PLANNING_TRAJECTORY,
)


def _parse_time_value(val: Any, label: str = "time") -> int:
    """Parse a time value (Unix seconds float or ISO string) to nanoseconds."""
    if val is None:
        raise McpError(INVALID_PARAMS, f"Missing required parameter: {label}")
    if isinstance(val, (int, float)):
        return int(val * 1e9)
    if isinstance(val, str):
        return int(dateutil_parser.parse(val.strip()).timestamp() * 1e9)
    raise McpError(INVALID_PARAMS, f"Cannot parse {label}={val!r} (expected Unix seconds or ISO string)")


def investigate_incident(args: Dict) -> Dict:
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
    rosout_noise_filtered = 0
    for m in window_data.get("/rosout", []):
        d = m["data"]
        if not isinstance(d, dict):
            continue
        level = d.get("level", 0)
        if level < 40:
            continue
        name = d.get("name", "?")
        msg_text = str(d.get("msg", ""))[:200]
        if _is_rosout_noise(name, msg_text):
            rosout_noise_filtered += 1
            continue
        if rosout_alert_count >= 100:
            rosout_suppressed += 1
            continue
        alerts.append({
            "time": m["log_time_iso"],
            "topic": "/rosout",
            "type": "rosout_fatal" if level >= 50 else "rosout_error",
            "severity": "error" if level >= 50 else "warning",
            "detail": f"[{'FATAL' if level >= 50 else 'ERROR'}] {name}: {msg_text}",
        })
        rosout_alert_count += 1
    if rosout_noise_filtered:
        alerts.append({
            "time": None,
            "topic": "/rosout",
            "type": "rosout_noise_filtered",
            "severity": "info",
            "detail": (
                f"{rosout_noise_filtered} known-noise rosout messages filtered "
                "(can1 CAN Receive Timeout; vehicle does not use can1)"
            ),
        })
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

