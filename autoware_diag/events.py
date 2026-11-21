"""MRM and AEB event detection."""
from __future__ import annotations

import json
from typing import Any, Dict, List, Optional

from .core import (
    _BoundedList,
    _cap_timeline,
    _collect_topic_data,
    _collect_topic_data_full,
    _decoding_status,
    _dictify,
    _open_mcap,
    _require_file_path,
    _scan_mcap,
    _time_str,
)
from .event_state import update_shadow, update_legacy, update_mrm
from .mcap_support import HAVE_ROS2
from .parsers import (
    CUSTOM_MESSAGE_CATALOG,
    _extract_engage,
    _extract_gate_mode,
    _parse_aeb_obstacle_debug,
    _parse_aeb_shadow,
    _val,
)
from .topics import AEB_FEEDBACK_GROUP, TOPIC_AEB, TOPIC_AEB_SHADOW, TOPIC_MRM

def detect_mrm_events(args: Dict) -> Dict:
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

    mrm_state = {}
    all_signals.sort(key=lambda x: x[1])
    for sig_type, ts_ns, val in all_signals:
        update_mrm(sig_type, ts_ns, val, mrm_state, events)

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


def detect_aeb_events(args: Dict) -> Dict:
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
            update_shadow(m["data"], m["log_time_ns"], prev, aeb_events, aeb_timeline)

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

        legacy_state = {}
        for m in aeb_msgs:
            update_legacy(m["data"], m["log_time_ns"], legacy_state, aeb_events, aeb_timeline)

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


