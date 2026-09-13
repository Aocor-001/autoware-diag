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
    _dictify,
    _format_stats,
    _freq_analysis,
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
    _extract_control_summary,
    _extract_engage,
    _extract_gate_mode,
    _extract_scenario_summary,
    _labelize,
    _parse_aeb_obstacle_debug,
    _parse_aeb_shadow,
    _parse_pix_brake_cmd,
    _parse_pix_brake_fb,
    _parse_pix_drive_cmd,
    _parse_pix_drive_fb,
    _parse_pix_steer_fb,
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


