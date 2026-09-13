# Autoware MCP Server

Autoware 自动驾驶诊断 MCP 服务器，用于分析本地 MCAP 录音文件，排查感知、规划控制、底盘、定位等问题。

## 功能

通过 MCP 协议提供以下工具：

| 工具 | 描述 |
|------|------|
| `list_topics` | 列出 MCAP 文件中的所有话题 |
| `analyze_mcap_file` | 分析本地 MCAP 文件的内容统计信息 + bag 元数据 + 命名空间 |
| `analyze_bag_overview` | 包结构总览 — 话题分组、推断节点、服务、元数据、零消息话题 |
| `analyze_rosout` | 分析 /rosout 节点日志 — 节点列表、级别分布、WARN/ERROR 时间线 |
| `analyze_diagnostics` | 分析 /diagnostics — hardware_id、status、ERROR/WARN 聚合 |
| `describe_message_schema` | 查询话题/schema 的内嵌定义 + Pix 私有字段语义 |
| `list_custom_messages` | 列出相对原版 Autoware 的私有/差异消息（pix/AEB/压缩点云等） |
| `detect_anomalies_local` | 分析本地 MCAP 文件中的异常（消息间隙、低频率） |
| `analyze_perception` | 分析感知模块 — BEVFusion、跟踪、预测、LiDAR 点云 |
| `analyze_chassis` | 分析车辆底盘状态 — Pix Robobus 执行命令与反馈、轮速 |
| `analyze_localization` | 分析定位模块 — GNSS、IMU、里程计、姿态 |
| `analyze_planning_control` | 分析规划控制 — 控制指令、场景、控制模式 |
| `detect_mrm_events` | 检测 MRM (Minimum Risk Maneuver) 事件 |
| `detect_aeb_events` | 检测 AEB/FCW 事件（新版 `/pix/AebInfoShadow`，旧版 `/pix_robobus/av_aeb_ctrl` 自动兼容） |
| `analyze_vehicle_health` | 综合健康检查 |
| `investigate_incident` | 分析某个时间点/时间段的车辆状态 |

### AEB/FCW 反馈话题兼容

- 新版反馈走 `/pix/AebInfoShadow`（`aeb_interfaces/msg/AebInfoShadow`），事件以**滤波后请求**
  （`aeb_req_filter`/`fcw_req_filter`）的跳变为准；
- 旧版录音走 `/pix_robobus/av_aeb_ctrl`（`AvAebCtrl`），自动回退兼容；
- `*_no_filter=1 且 *_filter=0` 时上报 `REQUEST_FILTERED` 事件（诊断「AEB 该触发未触发」的关键线索）；
- `aeb_status`/`fcw_status=2` 上报 `STATUS_FAULT` 事件。

### 异常检测能力

- **消息间隙检测**: 发现话题中超过阈值的消息间隔
- **低频率检测**: 识别发布频率异常低的话题
- **低覆盖率检测**: 计算数据的间隙覆盖率

### 已知噪音过滤

- `socket_can1_receiver` 的 `can1 - CAN Receive Timeout`：实车未使用 can1，`analyze_rosout` / `investigate_incident` 会自动过滤，仅在 `noise_filtered` 计数。

## 安装

```bash
# 安装依赖
pip3 install --user -r requirements.txt
```

## 系统要求

- Python 3.7+
- `mcap` Python 包
- `mcap-ros2-support` Python 包

## 协议

MCP 协议版本: `2025-03-26`
传输方式: stdio (JSON-RPC 2.0)
