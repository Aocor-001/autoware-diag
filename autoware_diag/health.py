"""Vehicle health aggregation."""
from __future__ import annotations

import json
import os
from collections import defaultdict
from typing import Any, Dict, List

from .core import (
    _decoding_status,
    _format_stats,
    _freq_analysis,
    _open_mcap,
    _require_file_path,
    _scan_mcap,
)
from .mcap_support import HAVE_ROS2
from .parsers import _group_has_data
from .topics import (
    AEB_FEEDBACK_GROUP,
    TOPIC_AEB,
    TOPIC_ALL_HEALTH,
    TOPIC_CHASSIS,
    TOPIC_LOCALIZATION,
    TOPIC_MRM,
    TOPIC_PERCEPTION,
    TOPIC_PLANNING_CONTROL,
)

def analyze_vehicle_health(args: Dict) -> Dict:
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

