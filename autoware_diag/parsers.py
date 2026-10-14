"""Message field parsers and semantic labels."""
from __future__ import annotations

from typing import Any, Dict, List, Optional

from .core import _dictify, _time_str
from .topics import (  # noqa: F401  — re-export for analyzers
    _AEB_REQ_LABELS,
    _AEB_SENSITIVITY_LABELS,
    _AEB_STATUS_LABELS,
)

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


_DIAG_LEVEL_LABELS = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}

# ---------------------------------------------------------------------------
# 红绿灯 — 两套常量表（tier4 分类链 vs autoware 组消息），不可混用
# ---------------------------------------------------------------------------

# tier4_perception_msgs/TrafficLightElement（car/ped 分类链输出）
# IDL: RED=1 AMBER=2 GREEN=3 WHITE=4 … UNKNOWN=18（与 autoware 的 UNKNOWN=0 不同）
_TL_T4_COLOR = {
    0: "UNKNOWN", 1: "RED", 2: "AMBER", 3: "GREEN", 4: "WHITE", 18: "UNKNOWN",
}
_TL_T4_SHAPE = {
    5: "CIRCLE", 6: "LEFT_ARROW", 7: "RIGHT_ARROW", 8: "UP_ARROW",
    9: "UP_LEFT_ARROW", 10: "UP_RIGHT_ARROW", 11: "DOWN_ARROW",
    12: "DOWN_LEFT_ARROW", 13: "DOWN_RIGHT_ARROW", 14: "CROSS",
    15: "SOLID_OFF", 16: "SOLID_ON", 17: "FLASHING", 18: "UNKNOWN",
}
_TL_T4_STATUS = {0: "UNLIT", 1: "LIT", 2: "BLINK"}

# autoware_perception_msgs/TrafficLightElement（internal/judged/final）
_TL_AW_COLOR = {0: "UNKNOWN", 1: "RED", 2: "AMBER", 3: "GREEN", 4: "WHITE"}
_TL_AW_SHAPE = {
    1: "CIRCLE", 2: "LEFT_ARROW", 3: "RIGHT_ARROW", 4: "UP_ARROW",
    5: "UP_LEFT_ARROW", 6: "UP_RIGHT_ARROW", 7: "DOWN_ARROW",
    8: "DOWN_LEFT_ARROW", 9: "DOWN_RIGHT_ARROW", 10: "CROSS",
}
_TL_AW_STATUS = {0: "UNSET", 1: "SOLID_OFF", 2: "SOLID_ON", 3: "FLASHING"}


class _CdrReader:
    """Minimal little-endian CDR reader for TrafficLight messages.

    tier4 TrafficLightArray 的 schema 是 ros2idl，mcap-ros2-support 的
    DecoderFactory 只认 ros2msg，解码会返回 None。这里按 IDL 布局手工解，
    避免红绿灯链路在工具里变成「无数据」假阴性。
    """

    def __init__(self, data: bytes):
        self.b = data
        self.o = 4  # skip encapsulation header

    def _align(self, n: int) -> None:
        while (self.o - 4) % n:
            self.o += 1

    def u8(self) -> int:
        v = self.b[self.o]
        self.o += 1
        return v

    def i32(self) -> int:
        self._align(4)
        v = int.from_bytes(self.b[self.o:self.o + 4], "little", signed=True)
        self.o += 4
        return v

    def u32(self) -> int:
        self._align(4)
        v = int.from_bytes(self.b[self.o:self.o + 4], "little", signed=False)
        self.o += 4
        return v

    def i64(self) -> int:
        self._align(8)
        v = int.from_bytes(self.b[self.o:self.o + 8], "little", signed=True)
        self.o += 8
        return v

    def f32(self) -> float:
        self._align(4)
        import struct
        v = struct.unpack_from("<f", self.b, self.o)[0]
        self.o += 4
        return v

    def f64(self) -> float:
        self._align(8)
        import struct
        v = struct.unpack_from("<d", self.b, self.o)[0]
        self.o += 8
        return v

    def string(self) -> str:
        n = self.u32()
        if n <= 0:
            return ""
        s = self.b[self.o:self.o + n - 1].decode("utf-8", "replace")
        self.o += n
        return s

    def time(self) -> Dict[str, int]:
        return {"sec": self.i32(), "nanosec": self.u32()}


def _tl_element_summary(color: int, shape: int, status: int, conf: float,
                        family: str) -> Dict[str, Any]:
    if family == "tier4":
        cmap, smap, stmap = _TL_T4_COLOR, _TL_T4_SHAPE, _TL_T4_STATUS
    else:
        cmap, smap, stmap = _TL_AW_COLOR, _TL_AW_SHAPE, _TL_AW_STATUS
    return {
        "color": color,
        "color_label": cmap.get(color, str(color)),
        "shape": shape,
        "shape_label": smap.get(shape, str(shape)),
        "status": status,
        "status_label": stmap.get(status, str(status)),
        "confidence": None if conf is None else round(float(conf), 4),
    }


def _parse_tl_array_decoded(d: Any) -> Optional[Dict[str, Any]]:
    """Parse a decoded tier4 TrafficLightArray (ROS2 object)."""
    signals = getattr(d, "signals", None)
    if signals is None:
        return None
    out = []
    for tl in signals:
        tid = getattr(tl, "traffic_light_id", None)
        ttype = getattr(tl, "traffic_light_type", None)
        elements = []
        for el in getattr(tl, "elements", None) or []:
            elements.append(
                _tl_element_summary(
                    getattr(el, "color", None),
                    getattr(el, "shape", None),
                    getattr(el, "status", None),
                    getattr(el, "confidence", None),
                    "tier4",
                )
            )
        out.append(
            {
                "traffic_light_id": tid,
                "traffic_light_type": ttype,
                "traffic_light_type_label": (
                    "CAR" if ttype == 0 else "PEDESTRIAN" if ttype == 1 else str(ttype)
                ),
                "elements": elements,
            }
        )
    header = getattr(d, "header", None)
    stamp = getattr(header, "stamp", None) if header else None
    return {
        "signals": out,
        "header_stamp": (
            {"sec": getattr(stamp, "sec", None), "nanosec": getattr(stamp, "nanosec", None)}
            if stamp is not None
            else None
        ),
    }


def _parse_tl_group_decoded(d: Any) -> Optional[Dict[str, Any]]:
    """Parse a decoded autoware TrafficLightGroupArray (ROS2 object)."""
    groups = getattr(d, "traffic_light_groups", None)
    if groups is None:
        return None
    out = []
    for g in groups:
        gid = getattr(g, "traffic_light_group_id", None)
        elements = []
        for el in getattr(g, "elements", None) or []:
            elements.append(
                _tl_element_summary(
                    getattr(el, "color", None),
                    getattr(el, "shape", None),
                    getattr(el, "status", None),
                    getattr(el, "confidence", None),
                    "aw",
                )
            )
        preds = []
        for p in getattr(g, "predictions", None) or []:
            pels = []
            for el in getattr(p, "simultaneous_elements", None) or []:
                pels.append(
                    _tl_element_summary(
                        getattr(el, "color", None),
                        getattr(el, "shape", None),
                        getattr(el, "status", None),
                        getattr(el, "confidence", None),
                        "aw",
                    )
                )
            preds.append(
                {
                    "reliability": getattr(p, "reliability", None),
                    "information_source": getattr(p, "information_source", None),
                    "elements": pels,
                }
            )
        out.append(
            {
                "traffic_light_group_id": gid,
                "elements": elements,
                "predictions": preds or None,
            }
        )
    return {"groups": out}


def _parse_tl_array_cdr(data: bytes) -> Optional[Dict[str, Any]]:
    """Fallback CDR decode for tier4 TrafficLightArray (ros2idl)."""
    try:
        c = _CdrReader(data)
        stamp = c.time()
        c.string()  # frame_id
        n = c.u32()
        signals = []
        for _ in range(n):
            tid = c.i64()
            ttype = c.u8()
            ne = c.u32()
            elements = []
            for _e in range(ne):
                color = c.u8()
                shape = c.u8()
                status = c.u8()
                conf = c.f64()
                elements.append(
                    _tl_element_summary(color, shape, status, conf, "tier4")
                )
            signals.append(
                {
                    "traffic_light_id": tid,
                    "traffic_light_type": ttype,
                    "traffic_light_type_label": (
                        "CAR" if ttype == 0 else "PEDESTRIAN" if ttype == 1 else str(ttype)
                    ),
                    "elements": elements,
                }
            )
        return {"signals": signals, "header_stamp": stamp, "decode": "cdr_fallback"}
    except Exception:
        return None


def _parse_tl_group_cdr(data: bytes) -> Optional[Dict[str, Any]]:
    """Fallback CDR decode for autoware TrafficLightGroupArray (ros2msg).

    Layout: Time stamp; sequence<TrafficLightGroup>
      TrafficLightGroup: int64 id; sequence<TrafficLightElement> elements;
                         sequence<PredictedTrafficLightState> predictions
      TrafficLightElement: u8 color, u8 shape, u8 status, f32 confidence
      PredictedTrafficLightState: Time predicted_stamp;
        sequence<Tele> simultaneous_elements; f32 reliability; string information_source
    """
    try:
        c = _CdrReader(data)
        c.time()  # stamp
        ng = c.u32()
        groups = []
        for _ in range(ng):
            gid = c.i64()
            ne = c.u32()
            elements = []
            for _e in range(ne):
                color = c.u8()
                shape = c.u8()
                status = c.u8()
                conf = c.f32()
                elements.append(
                    _tl_element_summary(color, shape, status, conf, "aw")
                )
            npred = c.u32()
            preds = []
            for _p in range(npred):
                c.time()
                nse = c.u32()
                pels = []
                for _se in range(nse):
                    color = c.u8()
                    shape = c.u8()
                    status = c.u8()
                    conf = c.f32()
                    pels.append(
                        _tl_element_summary(color, shape, status, conf, "aw")
                    )
                rel = c.f32()
                src = c.string()
                preds.append(
                    {
                        "reliability": rel,
                        "information_source": src,
                        "elements": pels,
                    }
                )
            groups.append(
                {
                    "traffic_light_group_id": gid,
                    "elements": elements,
                    "predictions": preds or None,
                }
            )
        return {"groups": groups, "decode": "cdr_fallback"}
    except Exception:
        return None


def _parse_traffic_light_message(
    data: Any, prefer_group: bool = False
) -> Optional[Dict[str, Any]]:
    """Unified TL parse: try ROS2 object first, fall back to CDR bytes."""
    # Decoded ROS2 object
    if hasattr(data, "signals") or hasattr(data, "traffic_light_groups"):
        if hasattr(data, "traffic_light_groups"):
            return _parse_tl_group_decoded(data) or {
                "note": "decoded object missing traffic_light_groups"
            }
        return _parse_tl_array_decoded(data) or {
            "note": "decoded object missing signals"
        }
    # Raw bytes → CDR fallback
    if isinstance(data, (bytes, bytearray)):
        if prefer_group:
            return _parse_tl_group_cdr(data) or _parse_tl_array_cdr(data)
        return _parse_tl_array_cdr(data) or _parse_tl_group_cdr(data)
    if isinstance(data, dict):
        # already dictified
        return data
    return None


def _primary_tl_color(parsed: Optional[Dict[str, Any]]) -> Optional[str]:
    """Extract a single dominant color label from a parsed TL message."""
    if not parsed:
        return None
    elements: List[Dict[str, Any]] = []
    for sig in parsed.get("signals") or []:
        elements.extend(sig.get("elements") or [])
    for g in parsed.get("groups") or []:
        elements.extend(g.get("elements") or [])
    if not elements:
        return None
    # 优先取非 UNKNOWN；否则返回第一个
    for el in elements:
        if el.get("color_label") not in (None, "UNKNOWN", "UNSET"):
            return el.get("color_label")
    return elements[0].get("color_label")


def _is_tl_group_topic(topic: str) -> bool:
    return any(
        topic.endswith(s)
        for s in (
            "/internal/traffic_signals",
            "/judged/traffic_signals",
            "/traffic_signals",
        )
    ) and "front_3mm" not in topic

