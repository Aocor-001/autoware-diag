#!/usr/bin/env python3
"""
Autoware Diagnostic MCP Server

Usage:
  python3 server.py

MCP 协议: JSON-RPC 2.0 over stdio (2025-03-26)
"""

from __future__ import annotations

from typing import Dict

from autoware_diag.core import detect_anomalies_local
from autoware_diag.events import detect_aeb_events, detect_mrm_events
from autoware_diag.health import analyze_vehicle_health
from autoware_diag.incident import investigate_incident
from autoware_diag.log import log  # noqa: F401
from autoware_diag.logs import analyze_diagnostics, analyze_rosout
from autoware_diag.mcp import McpServer
from autoware_diag.modules import (
    analyze_chassis,
    analyze_localization,
    analyze_mcap_file,
    analyze_perception,
    analyze_planning_control,
)
from autoware_diag.overview import analyze_bag_overview, list_topics
from autoware_diag.schema import describe_message_schema, list_custom_messages


def create_server() -> McpServer:
    server = McpServer(
        name="autoware-diag-mcp",
        version="1.0.0",
        instructions=(
            "【Autoware 自动驾驶诊断 MCP 服务器 — 使用规范】\n\n"
            "## ⚠️ 必须遵守的严谨规则\n\n"
            "你是一个严谨的自动驾驶诊断工具，使用本服务器所有工具时必须遵守以下规则：\n\n"
            "### 1. 时间参数必须精确\n"
            "- 调用 investigate_incident 时，传入的时间必须与用户指定的时间完全一致。\n"
            "- 用户说「看看 xxxxxx 时刻」→ 必须把该时刻原样填入 timestamp/timestamp_iso。\n"
            "- 用户说「从 A 到 B 时间段」→ 必须把 A 填 start/start_iso，B 填 end/end_iso。\n"
            "- **严禁自行篡改、偏移、猜测时间值**。如果你不确定时间格式，请求用户明确。\n"
            "- 调用后必须检查返回的 time_range 是否与用户要求一致；若不一致，向用户说明偏差原因。\n\n"
            "### 2. 文件路径必须真实\n"
            "- file_path 必须是用户提供的实际 MCAP 文件路径。\n"
            "- **严禁自行编造、猜测文件路径**。如果用户未提供路径，先问用户。\n\n"
            "### 3. 一次只做一个分析\n"
            "- 每次工具调用只分析一个 MCAP 文件的一个方面。\n"
            "- 不要在一次调用中请求过多数据（如 window_seconds 不要超过 30 秒，除非用户明确要求更长时间范围）。\n\n"
            "### 4. 结果必须可追溯\n"
            "- 每个工具返回结果中都包含 file_path 和时间范围，你必须核对它们是否对应你想要查的数据。\n"
            "- 如果结果中显示的时间范围与你要求的不符，立即指出并重新调用。\n\n"
            "### 5. 错误处理\n"
            "- 如果工具返回错误，仔细阅读错误信息。\n"
            "- 常见的「No messages found」可能是因为时间戳格式有误或时间范围不在录音范围内。\n"
            "- 不要盲目重试同样的参数——先检查时间值是否正确。\n\n"
            "### 6. 推荐排查顺序\n"
            "- 全局图景：analyze_bag_overview（话题分组/推断节点/服务/元数据）\n"
            "- 节点日志：analyze_rosout（/rosout WARN/ERROR）\n"
            "- 模块健康：analyze_diagnostics 或 analyze_vehicle_health\n"
            "- 指定时刻：investigate_incident\n\n"
            "---\n"
            "可用工具列表如下，每个工具都有详细说明。\n"
        ),
    )

    # ===================================================================
    # analyze_mcap_file
    # ===================================================================
    @server.tool(
        name="analyze_mcap_file",
        description="分析 MCAP 文件的内容统计信息（话题、频率、消息数）。file_path 必须使用用户提供的真实路径，不可编造。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须与用户提供的路径一致）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _tool(args: Dict):
        return analyze_mcap_file(args)

    # ===================================================================
    # detect_anomalies_local
    # ===================================================================
    @server.tool(
        name="detect_anomalies_local",
        description="检测 MCAP 文件中的时间异常（消息间隙、低频率）。file_path 必须使用用户提供的真实路径。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须与用户提供的路径一致）",
                },
                "gap_threshold_s": {
                    "type": "number",
                    "description": "消息间隔阈值（秒），超过视为间隙 (默认 1.0)",
                },
                "min_frequency_hz": {
                    "type": "number",
                    "description": "最低期望频率 Hz (默认 0.1)",
                },
            },
            "required": ["file_path"],
        },
    )
    def _tool(args: Dict):
        return detect_anomalies_local(args)

    # ===================================================================
    # analyze_perception
    # ===================================================================
    @server.tool(
        name="analyze_perception",
        description="【严谨模式】分析感知模块 — BEVFusion、跟踪、预测、LiDAR 点云。file_path 必须与用户提供的一致，不可编造。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _tool(args: Dict):
        return analyze_perception(args)

    # ===================================================================
    # analyze_chassis
    # ===================================================================
    @server.tool(
        name="analyze_chassis",
        description="【严谨模式】分析车辆底盘状态 — Pix Robobus 执行命令与反馈、轮速。file_path 必须与用户提供的一致，不可编造。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _tool(args: Dict):
        return analyze_chassis(args)

    # ===================================================================
    # analyze_localization
    # ===================================================================
    @server.tool(
        name="analyze_localization",
        description="【严谨模式】分析定位模块 — GNSS、IMU、里程计、姿态。file_path 必须与用户提供的一致，不可编造。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _tool(args: Dict):
        return analyze_localization(args)

    # ===================================================================
    # analyze_planning_control
    # ===================================================================
    @server.tool(
        name="analyze_planning_control",
        description="【严谨模式】分析规划控制 — 控制指令、场景、控制模式、诊断数据。file_path 必须与用户提供的一致，不可编造。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _tool(args: Dict):
        return analyze_planning_control(args)

    # ===================================================================
    # detect_mrm_events
    # ===================================================================
    @server.tool(
        name="detect_mrm_events",
        description="【严谨模式】检测 MRM (Minimum Risk Maneuver) 事件 — Emergency、Engage、GateMode。file_path 必须与用户提供的一致，不可编造。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _tool(args: Dict):
        return detect_mrm_events(args)

    # ===================================================================
    # detect_aeb_events
    # ===================================================================
    @server.tool(
        name="detect_aeb_events",
        description="【严谨模式】检测 AEB (Autonomous Emergency Braking) 事件。file_path 必须与用户提供的一致，不可编造。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _tool(args: Dict):
        return detect_aeb_events(args)

    # ===================================================================
    # analyze_vehicle_health
    # ===================================================================
    @server.tool(
        name="analyze_vehicle_health",
        description="【严谨模式】综合健康检查 — 一次性检查感知/规划控制/MRM/AEB/底盘所有模块。file_path 必须与用户提供的一致，不可编造。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _tool(args: Dict):
        return analyze_vehicle_health(args)

    # ===================================================================
    # investigate_incident — 时间点/时间段分析
    # ===================================================================
    @server.tool(
        name="investigate_incident",
        description="【严谨模式】分析某个时间点或时间段的车辆状态 — 排查规划/感知/底盘异常。\n"
        "⚠️ 使用规则：\n"
        "1. 时间参数必须与用户指定的时间完全一致，严禁篡改。\n"
        "2. 单点模式：传 timestamp(Unix秒) 或 timestamp_iso(如'2026-07-20 10:04:30')，可配合 window_seconds（默认2秒）。\n"
        "3. 时间段模式：传 start/end（Unix秒）或 start_iso/end_iso（ISO字符串），必须成对使用。\n"
        "4. 调用后必须核对返回的 time_range 是否与用户要求一致，不一致需向用户说明。\n"
        "5. window_seconds 建议不超过30秒。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须与用户提供的路径一致，不可编造）",
                },
                "timestamp": {
                    "type": "number",
                    "description": "单点模式：目标时间点（Unix 秒）。必须与用户指定时刻完全一致，不可偏移。示例：1718000000",
                },
                "timestamp_iso": {
                    "type": "string",
                    "description": "单点模式：ISO 时间字符串。必须与用户指定时刻完全一致，不可偏移。格式：'2026-07-20 10:04:30' 或 '2026-07-20T10:04:30'",
                },
                "window_seconds": {
                    "type": "number",
                    "description": "单点模式：前后分析窗口（秒），默认 2.0。建议不超过 30 秒。",
                },
                "start": {
                    "type": "number",
                    "description": "时间段模式：开始时间（Unix 秒）。必须与用户指定的开始时间一致。",
                },
                "end": {
                    "type": "number",
                    "description": "时间段模式：结束时间（Unix 秒）。必须与用户指定的结束时间一致。",
                },
                "start_iso": {
                    "type": "string",
                    "description": "时间段模式：开始 ISO 时间。必须与用户指定的开始时间一致。格式：'2026-07-20 10:04:30'",
                },
                "end_iso": {
                    "type": "string",
                    "description": "时间段模式：结束 ISO 时间。必须与用户指定的结束时间一致。格式：'2026-07-20 10:05:00'",
                },
            },
            "required": ["file_path"],
        },
    )
    def _tool(args: Dict):
        return investigate_incident(args)

    # ===================================================================
    # describe_message_schema — 自定义消息 schema / 字段语义
    # ===================================================================
    @server.tool(
        name="describe_message_schema",
        description=(
            "【严谨模式】查询消息 schema：topic 或 schema 名。返回 MCAP 内嵌 .msg/.idl "
            "定义，并附 Pix/私有相对原版 Autoware 的字段差异说明。"
            "感知/规划控制/pix 消息优先用这个理解字段。"
            "file_path 必须与用户提供的一致。"
        ),
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
                "topic": {
                    "type": "string",
                    "description": "话题名，如 /pix/AebInfoShadow（与 schema 二选一）",
                },
                "schema_name": {
                    "type": "string",
                    "description": "完整 schema 名，如 pix_robobus_driver_msgs/msg/VaSteerStaFb",
                },
                "include_schema_text": {
                    "type": "boolean",
                    "description": "是否返回完整 schema 文本（默认 true，可关掉只拿语义摘要）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _tool(args: Dict):
        return describe_message_schema(args)

    # ===================================================================
    # list_custom_messages — 私有/差异消息一览
    # ===================================================================
    @server.tool(
        name="list_custom_messages",
        description=(
            "列出 MCAP 中相对原版 Autoware 的私有/差异消息（pix_*、aeb_interfaces、"
            "point_cloud_interfaces 等），含字段语义。file_path 必须与用户提供的一致。"
        ),
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _tool(args: Dict):
        return list_custom_messages(args)

    # ===================================================================
    # analyze_bag_overview — 包结构总览：话题/节点/服务/元数据
    # ===================================================================
    @server.tool(
        name="analyze_bag_overview",
        description=(
            "【严谨模式】MCAP 包结构总览 — 话题命名空间分组、推断节点、服务话题、"
            "bag 元数据(ROS_DISTRO)、零消息话题。file_path 必须与用户提供的一致。"
            "适合诊断前先建立全局图景；深入模块请再调用 analyze_rosout / "
            "analyze_diagnostics / analyze_vehicle_health。"
        ),
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _tool(args: Dict):
        return analyze_bag_overview(args)

    # ===================================================================
    # analyze_rosout — 节点日志 (stdout / /rosout)
    # ===================================================================
    @server.tool(
        name="analyze_rosout",
        description=(
            "【严谨模式】分析 /rosout（节点 stdout 日志）— 节点列表、日志级别分布、"
            "WARN/ERROR/FATAL 事件时间线。file_path 必须与用户提供的一致，不可编造。"
            "排查节点崩溃、AEB 业务日志、感知重启等首选。"
            "已知噪音：can1 CAN Receive Timeout（实车未使用 can1）会被自动过滤。"
        ),
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
                "max_events": {
                    "type": "integer",
                    "description": "最多返回的 WARN+ 事件条数（默认 200，最大 1000）",
                },
                "min_level": {
                    "type": "integer",
                    "description": "事件最低级别：10=DEBUG 20=INFO 30=WARN 40=ERROR 50=FATAL（默认 30）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _tool(args: Dict):
        return analyze_rosout(args)

    # ===================================================================
    # analyze_diagnostics — /diagnostics 健康聚合
    # ===================================================================
    @server.tool(
        name="analyze_diagnostics",
        description=(
            "【严谨模式】分析 /diagnostics — hardware_id 节点清单、status 名称、"
            "级别直方图、ERROR/WARN 聚合（按 status 去重计数与起止时间）。"
            "file_path 必须与用户提供的一致，不可编造。"
        ),
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
                "max_status_details": {
                    "type": "integer",
                    "description": "最多返回的非 OK status 明细条数（默认 100，最大 500）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _tool(args: Dict):
        return analyze_diagnostics(args)

    # ===================================================================
    # (Legacy) list_topics — kept for backward compat, reads from local file
    # ===================================================================
    @server.tool(
        name="list_topics",
        description="【严谨模式】列出 MCAP 文件中的所有话题。file_path 必须与用户提供的一致，不可编造。",
        input_schema={
            "type": "object",
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "MCAP 文件路径（必须使用用户提供的真实路径，不可编造）",
                },
            },
            "required": ["file_path"],
        },
    )
    def _tool(args: Dict):
        return list_topics(args)

    return server


def main():
    server = create_server()
    server.run()


if __name__ == "__main__":
    main()
