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
from .mcap_support import HAVE_ROS2
from .parsers import (
    CUSTOM_MESSAGE_CATALOG,
    _AEB_REQ_LABELS,
    _AEB_SENSITIVITY_LABELS,
    _AEB_STATUS_LABELS,
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


