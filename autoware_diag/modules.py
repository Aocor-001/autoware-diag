"""Module analyzers."""
from __future__ import annotations

import json
import os
from collections import Counter
from typing import Any, Dict, List, Optional

from .core import (
    _build_anomalies,
    _cap_timeline,
    _collect_topic_data,
    _collect_topic_data_full,
    _dictify,
    _format_stats,
    _freq_analysis,
    _get_per_topic_messages,
    _open_mcap,
    _read_topic_timestamps,
    _require_file_path,
    _scan_mcap,
    _time_str,
)
from .mcap_support import HAVE_ROS2
from .parsers import (
    CUSTOM_MESSAGE_CATALOG,
    _AEB_OBSTACLE_SOURCE_LABELS,
    _AEB_REQ_LABELS,
    _AEB_SENSITIVITY_LABELS,
    _AEB_STATUS_LABELS,
    _CONTROL_MODE_LABELS,
    _GEAR_CMD_LABELS,
    _extract_control_summary,
    _extract_engage,
    _extract_gate_mode,
    _extract_scenario_summary,
    _is_tl_group_topic,
    _labelize,
    _parse_aeb_obstacle_debug,
    _parse_aeb_shadow,
    _parse_pix_brake_cmd,
    _parse_pix_brake_fb,
    _parse_pix_drive_cmd,
    _parse_pix_drive_fb,
    _parse_pix_steer_fb,
    _parse_traffic_light_message,
    _primary_tl_color,
    _group_has_data,
    _val,
)
from .overview import _classify_topics_by_namespace, _read_bag_metadata
from .topics import (
    AEB_FEEDBACK_GROUP,
    TOPIC_AEB,
    TOPIC_AEB_SHADOW,
    TOPIC_CHASSIS,
    TOPIC_LOCALIZATION,
    TOPIC_PERCEPTION,
    TOPIC_PLANNING_CONTROL,
    TOPIC_PLANNING_TRAJECTORY,
    TOPIC_TAKEOVER,
    TOPIC_TRAFFIC_LIGHT,
    TOPIC_TRAFFIC_LIGHT_FINAL,
)

def analyze_mcap_file(args: Dict) -> Dict:
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


def analyze_perception(args: Dict) -> Dict:
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


def analyze_chassis(args: Dict) -> Dict:
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


def analyze_localization(args: Dict) -> Dict:
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


def analyze_planning_control(args: Dict) -> Dict:
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


def _tl_color_flicker_events(timeline: List[Dict]) -> List[Dict]:
    """Detect RED→UNKNOWN / GREEN↔UNKNOWN / color jumps on a single topic timeline."""
    events: List[Dict] = []
    prev = None
    for item in timeline:
        color = item.get("color")
        if color is None:
            prev = color
            continue
        if prev is not None and color != prev:
            severity = "info"
            etype = "color_change"
            if prev in ("RED", "AMBER", "GREEN") and color in ("UNKNOWN", "UNSET"):
                etype = "color_lost_to_unknown"
                severity = "warning"
            elif prev in ("UNKNOWN", "UNSET") and color == "GREEN":
                etype = "green_recovered"
                severity = "info"
            elif prev == "RED" and color == "GREEN":
                etype = "red_to_green"
                severity = "info"
            elif color == "UNKNOWN" or color == "UNSET":
                etype = "to_unknown"
                severity = "warning"
            events.append(
                {
                    "time": item.get("time"),
                    "from": prev,
                    "to": color,
                    "type": etype,
                    "severity": severity,
                    "confidence": item.get("confidence"),
                }
            )
        prev = color
    return events


def _summarize_tl_topic_timeline(msgs: List[Dict], topic: str) -> Dict[str, Any]:
    """Build a compact color timeline from collected TL messages."""
    prefer_group = _is_tl_group_topic(topic)
    timeline: List[Dict] = []
    parse_errors = 0
    color_counts: Dict[str, int] = {}
    last_state = None
    for m in msgs:
        raw = m["data"]
        parsed = _parse_traffic_light_message(raw, prefer_group=prefer_group)
        if parsed is None:
            parse_errors += 1
            continue
        color = _primary_tl_color(parsed)
        if color is None:
            color = "EMPTY"
        color_counts[color] = color_counts.get(color, 0) + 1
        conf = None
        elements: List[Dict] = []
        for sig in parsed.get("signals") or []:
            elements.extend(sig.get("elements") or [])
        for g in parsed.get("groups") or []:
            elements.extend(g.get("elements") or [])
        if elements:
            confs = [e.get("confidence") for e in elements if e.get("confidence") is not None]
            conf = max(confs) if confs else None
            primary = next(
                (e for e in elements if e.get("color_label") not in (None, "UNKNOWN", "UNSET")),
                elements[0],
            )
        else:
            primary = None
        t = _time_str(m["log_time_ns"])
        if last_state is None or last_state[0] != color:
            timeline.append(
                {
                    "time": t,
                    "log_time_ns": m["log_time_ns"],
                    "color": color,
                    "confidence": conf,
                    "shape": (primary or {}).get("shape_label"),
                    "status": (primary or {}).get("status_label"),
                    "decode": parsed.get("decode"),
                }
            )
            last_state = (color, conf)
        elif last_state[1] is None and conf is not None:
            last_state = (color, conf)
    events = _tl_color_flicker_events(timeline)
    green_segments: List[Dict] = []
    seg_start = None
    for item in timeline + [{"color": None, "time": None, "log_time_ns": None}]:
        c = item.get("color")
        if c == "GREEN":
            if seg_start is None:
                seg_start = item
        else:
            if seg_start is not None and item.get("log_time_ns") is not None:
                dur_ms = (item["log_time_ns"] - seg_start["log_time_ns"]) / 1e6
                green_segments.append(
                    {
                        "start": seg_start.get("time"),
                        "end": item.get("time"),
                        "duration_ms": round(dur_ms, 1),
                    }
                )
            elif seg_start is not None and item.get("color") is None:
                green_segments.append(
                    {
                        "start": seg_start.get("time"),
                        "end": timeline[-1].get("time") if timeline else None,
                        "duration_ms": None,
                        "note": "green until end of bag",
                    }
                )
            seg_start = None
    return {
        "total_messages": len(msgs),
        "parse_errors": parse_errors,
        "color_counts": color_counts,
        "timeline": _cap_timeline(timeline, limit=80),
        "color_events": _cap_timeline(events, limit=60),
        "green_segments": green_segments[:20],
        "last_color": timeline[-1]["color"] if timeline else None,
    }


def analyze_traffic_light(args: Dict) -> Dict:
    """红绿灯链路分析：分类输出 + 组消息灯态 + 精检测失败 + 接管上下文。

    排查「红灯转绿后车辆不走」时，必须把灯态时间线和控制权
    （control_mode / engage / Takeover 日志）对齐，避免把人工接管误判为
    「感知绿灯后 AUTO 未放行」。
    """
    file_path = _require_file_path(args)

    timestamps = _read_topic_timestamps(file_path, TOPIC_TRAFFIC_LIGHT)
    freq_results = {}
    expected_freqs = {
        "/perception/traffic_light_recognition/front_3mm/classification/car/traffic_signals": 5,
        "/perception/traffic_light_recognition/traffic_signals": 5,
        "/perception/traffic_light_recognition/judged/traffic_signals": 5,
        "/perception/traffic_light_recognition/internal/traffic_signals": 5,
        "/perception/traffic_light_recognition/front_3mm/detection/expect/rois": 30,
        "/perception/traffic_light_recognition/front_3mm/detection/rough/rois": 30,
        "/perception/traffic_light_recognition/front_3mm/detection/rois": 5,
    }
    for topic in TOPIC_TRAFFIC_LIGHT:
        ts = timestamps.get(topic, [])
        freq_results[topic] = _freq_analysis(ts, expected_freqs.get(topic))

    signal_topics = [t for t in TOPIC_TRAFFIC_LIGHT if t.endswith("traffic_signals")]
    content = _collect_topic_data_full(file_path, signal_topics)

    topic_summaries = {}
    for topic in signal_topics:
        msgs = content.get(topic, [])
        if not msgs:
            topic_summaries[topic] = {"note": "no data"}
            continue
        topic_summaries[topic] = _summarize_tl_topic_timeline(msgs, topic)

    tl_detector_events: List[Dict] = []
    takeover_events: List[Dict] = []
    for topic, decoded, pub_ns, log_ns in _get_per_topic_messages(
        file_path, ["/rosout"], max_per_topic=0
    ):
        name = str(getattr(decoded, "name", "") or "")
        msg = str(getattr(decoded, "msg", "") or "")
        level = getattr(decoded, "level", None)
        t = _time_str(log_ns)
        if "traffic_light" in name or "TL_FINE" in msg or "TL_YOLOX" in msg or "TL_CLASSIFIER" in msg:
            if "NO_MATCH" in msg or "EMPTY_OUTPUT" in msg or "zero_sized" in msg:
                tl_detector_events.append(
                    {
                        "time": t,
                        "log_time_ns": log_ns,
                        "level": level,
                        "node": name,
                        "kind": "fine_detector_empty",
                        "message": msg[:400],
                    }
                )
            elif "TL_YOLOX" in msg and "output_objects=0" in msg:
                tl_detector_events.append(
                    {
                        "time": t,
                        "log_time_ns": log_ns,
                        "level": level,
                        "node": name,
                        "kind": "yolox_zero_objects",
                        "message": msg[:400],
                    }
                )
        low = msg.lower()
        if (
            "takeover" in low
            or "接管" in msg
            or ("teleop" in low and "event" in low)
            or ("brake" in low and "takeover" in low)
        ):
            takeover_events.append(
                {
                    "time": t,
                    "log_time_ns": log_ns,
                    "level": level,
                    "node": name,
                    "kind": "rosout_takeover",
                    "message": msg[:300],
                }
            )

    tk_content = _collect_topic_data_full(
        file_path,
        [
            "/vehicle/status/control_mode",
            "/api/autoware/get/engage",
            "/control/command/gear_cmd",
        ],
    )
    control_mode_timeline: List[Dict] = []
    prev_mode = None
    for m in tk_content.get("/vehicle/status/control_mode", []):
        mode = _val(m["data"], "mode")
        if mode is None and isinstance(m["data"], dict):
            mode = m["data"].get("mode")
        try:
            mode = int(mode)
        except (TypeError, ValueError):
            mode = None
        label = _CONTROL_MODE_LABELS.get(mode, str(mode)) if mode is not None else None
        if mode != prev_mode:
            control_mode_timeline.append(
                {"time": _time_str(m["log_time_ns"]), "mode": mode, "label": label}
            )
            prev_mode = mode

    gear_timeline: List[Dict] = []
    prev_gear = None
    for m in tk_content.get("/control/command/gear_cmd", []):
        g = _val(m["data"], "command", "data")
        try:
            g = int(g)
        except (TypeError, ValueError):
            g = None
        if g != prev_gear:
            gear_timeline.append(
                {
                    "time": _time_str(m["log_time_ns"]),
                    "gear": g,
                    "label": _GEAR_CMD_LABELS.get(g, str(g)),
                }
            )
            prev_gear = g

    engage_timeline: List[Dict] = []
    prev_eng = None
    for m in tk_content.get("/api/autoware/get/engage", []):
        eng = _extract_engage(m["data"])
        if eng != prev_eng:
            engage_timeline.append({"time": _time_str(m["log_time_ns"]), "engage": eng})
            prev_eng = eng

    anomalies: List[Dict] = []
    for topic, freq in freq_results.items():
        if freq.get("count", 0) == 0 and topic in (
            TOPIC_TRAFFIC_LIGHT_FINAL,
            "/perception/traffic_light_recognition/front_3mm/classification/car/traffic_signals",
        ):
            anomalies.append(
                {
                    "topic": topic,
                    "type": "missing_topic",
                    "severity": "error",
                    "detail": f"Critical TL topic has no messages: {topic}",
                }
            )
        elif freq.get("count", 0) == 0:
            anomalies.append(
                {
                    "topic": topic,
                    "type": "missing_topic",
                    "severity": "warning",
                    "detail": f"Topic {topic} has no messages",
                }
            )
        elif not freq.get("healthy", True):
            anomalies.append(
                {
                    "topic": topic,
                    "type": "frequency_anomaly",
                    "severity": "warning",
                    "detail": (
                        f"{topic}: expected {freq.get('expected_frequency_hz','?')} Hz, "
                        f"got {freq.get('actual_frequency_hz','?')} Hz"
                    ),
                }
            )

    final = topic_summaries.get(TOPIC_TRAFFIC_LIGHT_FINAL, {})
    if final.get("color_counts"):
        unk = final["color_counts"].get("UNKNOWN", 0) + final["color_counts"].get("UNSET", 0)
        total = sum(final["color_counts"].values()) or 1
        if unk / total > 0.3:
            anomalies.append(
                {
                    "topic": TOPIC_TRAFFIC_LIGHT_FINAL,
                    "type": "high_unknown_ratio",
                    "severity": "warning",
                    "detail": (
                        f"final traffic_signals UNKNOWN/UNSET ratio "
                        f"{unk}/{total} ({100 * unk / total:.0f}%) — 规划可能一直按未知灯态压停"
                    ),
                }
            )
    for ev in final.get("color_events") or []:
        if ev.get("type") in ("color_lost_to_unknown", "to_unknown"):
            anomalies.append(
                {
                    "topic": TOPIC_TRAFFIC_LIGHT_FINAL,
                    "type": ev["type"],
                    "severity": "warning",
                    "detail": f"灯态 {ev.get('from')}→{ev.get('to')} @ {ev.get('time')}",
                }
            )
            break

    if tl_detector_events:
        anomalies.append(
            {
                "topic": "/rosout",
                "type": "tl_fine_detector_empty",
                "severity": "warning",
                "detail": (
                    f"{len(tl_detector_events)} fine-detector empty/NO_MATCH events; "
                    f"first={tl_detector_events[0].get('time')} "
                    f"last={tl_detector_events[-1].get('time')}"
                ),
            }
        )

    left_autonomous = [
        e
        for e in control_mode_timeline
        if e.get("label") in ("MANUAL", "DISENGAGED") or e.get("mode") in (4, 5)
    ]
    entered_autonomous = [
        e
        for e in control_mode_timeline
        if e.get("mode") == 1
        or (e.get("label") or "").startswith("AUTONOMOUS")
    ]

    from .core import _decoding_status

    guidance = [
        "灯态时间线在 topic_summaries[/.../traffic_signals]；color_events 标了 RED→UNKNOWN / GREEN 闪烁。",
        "若 final 灯态长时间 UNKNOWN，规划会按未知压停——不要只看 BEV。",
        "control_mode 1→4 或 rosout Takeover 说明是人工接管后起步，不是 AUTO 放行。",
        "绿灯首次出现时间 vs 车速开始上升时间：车速更早升高 → 多半是接管起步。",
    ]

    return {
        "content": [
            {
                "type": "text",
                "text": json.dumps(
                    {
                        "file_path": file_path,
                        "ros2_decoding": _decoding_status(),
                        "semantics": {
                            "final_topic": TOPIC_TRAFFIC_LIGHT_FINAL,
                            "tier4_colors": "RED=1 AMBER=2 GREEN=3 (UNKNOWN=18 in tier4)",
                            "autoware_colors": "UNKNOWN=0 RED=1 AMBER=2 GREEN=3",
                            "note": (
                                "tier4 分类链与 autoware 组消息常量不同；"
                                "tier4 schema 为 ros2idl，解码失败时用 CDR fallback"
                            ),
                        },
                        "frequency_analysis": freq_results,
                        "topic_summaries": topic_summaries,
                        "tl_detector_events": _cap_timeline(tl_detector_events, limit=40),
                        "control_mode_timeline": control_mode_timeline,
                        "engage_timeline": engage_timeline,
                        "gear_timeline": _cap_timeline(gear_timeline, limit=30),
                        "takeover_events": _cap_timeline(takeover_events, limit=20),
                        "takeover_summary": {
                            "entered_autonomous_times": [e["time"] for e in entered_autonomous[:10]],
                            "left_autonomous_times": [e["time"] for e in left_autonomous[:10]],
                            "note": (
                                "left_autonomous 之后的车速上升应解释为人工/遥控起步，"
                                "不能归因于「感知绿灯后 AUTO 未响应」"
                            ),
                        },
                        "anomalies": anomalies,
                        "total_anomalies": len(anomalies),
                        "diagnosis_guidance": guidance,
                    },
                    ensure_ascii=False,
                    indent=2,
                    default=str,
                ),
            }
        ],
    }


