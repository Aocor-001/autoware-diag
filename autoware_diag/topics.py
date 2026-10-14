"""Topic lists used by module analyzers."""

# Key topics for each diagnostic area
TOPIC_PERCEPTION = [
    "/perception/object_recognition/detection/bevfusion/objects",
    "/perception/object_recognition/tracking/objects",
    "/perception/object_recognition/objects",
    "/sensing/lidar/concatenated/pointcloud_cloudini",
]

# 红绿灯识别链路（按数据流）：rough/expect ROI → fine detector → car/ped 分类
#   → internal/judged → final traffic_signals（规划实际消费）
# tier4 TrafficLightArray 为 ros2idl，标准 ROS2 decoder 不支持，需 CDR 兜底。
TOPIC_TRAFFIC_LIGHT = [
    "/perception/traffic_light_recognition/front_3mm/classification/car/traffic_signals",
    "/perception/traffic_light_recognition/front_3mm/classification/pedestrian/traffic_signals",
    "/perception/traffic_light_recognition/front_3mm/classification/traffic_signals",
    "/perception/traffic_light_recognition/internal/traffic_signals",
    "/perception/traffic_light_recognition/judged/traffic_signals",
    "/perception/traffic_light_recognition/traffic_signals",
    "/perception/traffic_light_recognition/front_3mm/detection/rois",
    "/perception/traffic_light_recognition/front_3mm/detection/expect/rois",
    "/perception/traffic_light_recognition/front_3mm/detection/rough/rois",
]

# 规划真正读的最终灯态
TOPIC_TRAFFIC_LIGHT_FINAL = "/perception/traffic_light_recognition/traffic_signals"

# 接管 / 控制权交接（红绿灯后不走时必须核对：是 AUTO 压停还是人工接管）
TOPIC_TAKEOVER = [
    "/vehicle/status/control_mode",
    "/api/autoware/get/engage",
    "/control/current_gate_mode",
    "/control/command/gear_cmd",
    "/system/operation_mode/state",
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

