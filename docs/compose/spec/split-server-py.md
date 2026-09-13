---
feature: split-server-py
status: delivered
updated: 2026-09-13
branch: main
commits: 0866b27..uncommitted
---

# 拆分 server.py 为 autoware_diag 包

## Report

**What was built** — 将约 4773 行的单体 `server.py` 拆为 `autoware_diag/` 包：`server.py` 仅保留 MCP 工具注册（487 行）；业务逻辑按职责落在 `core` / `noise` / `parsers` / `overview` / `logs` / `modules` / `events` / `health` / `incident` / `schema` / `mcp`，并补充 `topics` / `mcap_support` / `log`。16 个工具名、输入 schema、JSON 输出契约不变；`python3 server.py` 入口不变。can1 噪音过滤保留在 `noise.py`，由 `analyze_rosout` 与 `investigate_incident` 共用。

实现过程中发现拆分初稿存在多处断裂 import（`_scan_mcap`、`defaultdict`、`json`、`Ros2DecoderFactory`、`TOPIC_*`、`CUSTOM_MESSAGE_CATALOG` 等），已补齐并重新验证。

**Verification** —
- `python3 -m py_compile server.py autoware_diag/*.py` → PASS
- `create_server()` 注册 16 个工具 → PASS
- `python3 server.py` 子进程存活（stdio 启动）→ PASS
- 样本 `failcase_data_20260913132945_456.mcap` 上 16/16 工具调用 → PASS
- `analyze_rosout`：`total_messages=43460`，`noise_filtered=15`，events 中 can1=0
- `analyze_planning_control`：`/planning/scenario_planning/trajectory` 缺失异常仍在
- `investigate_incident`（15:36:30 ±2s）：无 can1 告警
- 独立 review：Spec compliance / Correctness / Consistency 均 PASS，无 critical

**Journey log** —
1. 工作区在动手前已有一版未提交拆分与 spec；本次是验证 + 修断裂 import + 收尾，而非从零搬移。
2. AST 扫描（private name used-not-defined）比 `py_compile` 更早暴露 import 缺口；`py_compile` 不查名字。
3. 动态探测必须带真实 MCAP：缺文件时只走到 path check，掩盖了 `NameError`。
4. T5 行数目标 `<400` 未达成（487），属纯行数指标，不影响契约；可接受。
5. 变更仍在 `main` 未提交；提交/PR 由用户决定收尾方式。

## [S1] Problem

`server.py` 约 4773 行，把 MCAP 读写、消息解析、噪音规则、各模块分析、MCP 协议实现、16 个工具 handler 全塞在一个文件里。后续加过滤规则或改诊断逻辑时定位成本高，也难以做局部回归。需要在**不改变对外工具名与 JSON 输出契约**的前提下拆成可维护的包结构。

当前 main 工作区有未提交改动（can1 噪音过滤 + README），作为本次拆分的基线继续工作；用户明确选择不建 worktree。

## [S2] Design

### 目标结构（中等粒度）

```text
Pix-Autoware/
  server.py                 # 仅入口：main() + create_server() 注册
  autoware_diag/
    __init__.py             # 版本与公共 re-export（可选、保持精简）
    core.py                 # MCAP 打开/扫描/时间/截断/频率/_val
    noise.py                # 已知噪音规则（can1 等）
    parsers.py              # Pix/AEB/scenario/control 字段解析
    overview.py             # bag 元数据、命名空间分组、推断节点、服务话题
    logs.py                 # rosout + diagnostics 流式分析
    modules.py              # perception / chassis / localization / planning_control
    events.py               # MRM + AEB/FCW
    health.py               # analyze_vehicle_health
    incident.py             # investigate_incident
    schema.py               # describe_message_schema + list_custom_messages
    mcp.py                  # McpServer / McpError / 协议常量
    topics.py               # 共享话题列表（实现时补充）
    mcap_support.py         # 可选 mcap/ros2 依赖探测（实现时补充）
    log.py                  # logging 封装（实现时补充）
  docs/compose/spec/split-server-py.md
```

### 模块职责与迁移映射

| 目标文件 | 迁入内容（现 server.py） |
|---|---|
| `core.py` | `HAVE_MCAP/HAVE_ROS2`、`Ros2DecoderFactory`、`_open_mcap`、`_require_file_path`、`_decoding_status`、`_get_summary`、`_time_str`、`_dictify`、`_get_per_topic_messages`、`_scan_mcap`、`_format_stats`、`_build_anomalies`、`_collect_topic_data*`、`_freq_analysis`、`_cap_*`、`_truncate_text`、`_read_topic_timestamps`、`_BoundedList`、`MAX_RESPONSE_CHARS` 等 |
| `noise.py` | `_ROSOUT_NOISE_PATTERNS`、`_is_rosout_noise` |
| `parsers.py` | `_val`、`_parse_aeb_*`、`_extract_engage`、`_extract_control_summary`、`_labelize`、`_parse_pix_*`、`_extract_scenario_summary`、`_extract_gate_mode`、`_reduce_diagnostics`、`_reduce_trajectory` |
| `overview.py` | `_read_bag_metadata`、`_classify_topics_by_namespace`、`_service_like_topics`、`_infer_nodes`、`_NS_MODULE_HINTS` |
| `logs.py` | `_analyze_rosout_stream`、`_analyze_diagnostics_stream`、`_ROSOUT_LEVEL_LABELS`、`_DIAG_LEVEL_LABELS` |
| `modules.py` | 各模块 handler 的纯函数体：`analyze_perception`、`analyze_chassis`、`analyze_localization`、`analyze_planning_control` |
| `events.py` | `detect_mrm_events`、`detect_aeb_events` 函数体 |
| `health.py` | `analyze_vehicle_health` 函数体（含 `_group_has_data`） |
| `incident.py` | `investigate_incident` 函数体 |
| `schema.py` | `describe_message_schema`、`list_custom_messages` 函数体 + 相关 catalog 常量 |
| `mcp.py` | `McpError`、`McpServer`、JSON-RPC 编解码、stdio 循环 |
| `server.py` | `create_server()` 里只保留 `@server.tool` 注册（name/description/schema 不变），handler 一行转发到上述函数；`main()` |

实现时的合理偏差（review 确认可接受）：`MAX_RESPONSE_CHARS`/`_truncate_text` 落在 `mcp.py`；`detect_anomalies_local` 在 `core.py`；`CUSTOM_MESSAGE_CATALOG` 在 `parsers.py`。

### 契约（必须保持）

1. **MCP 工具名不变**（16 个）：`analyze_mcap_file`、`detect_anomalies_local`、`analyze_perception`、`analyze_chassis`、`analyze_localization`、`analyze_planning_control`、`detect_mrm_events`、`detect_aeb_events`、`analyze_vehicle_health`、`investigate_incident`、`describe_message_schema`、`list_custom_messages`、`analyze_bag_overview`、`analyze_rosout`、`analyze_diagnostics`、`list_topics`。
2. **输入 schema / description / instructions 文本保持等价**（允许把 can1 噪音说明留在 analyze_rosout 描述中）。
3. **返回 JSON 结构不变**；can1 噪音过滤行为保留（`noise_filtered` 字段与 incident 的 `rosout_noise_filtered` 摘要）。
4. **入口不变**：`python3 server.py` 仍可启动；`mimocode.jsonc` 中 `autoware-diag` command 无需修改。
5. **无新第三方依赖**；仍用 `mcap` / `mcap-ros2-support` / `python-dateutil`。

### 实现约束

- 拆分以**搬移 + 改 import** 为主，不顺手改业务逻辑。
- 允许将原先嵌在 `create_server()` 闭包里的 handler 抽成模块级函数；抽函数时参数为已解析的 `args: Dict`，返回值为原 tool 内容 dict。
- 循环依赖：`core` 不 import 其它业务模块；`parsers`/`noise` 不依赖 `modules*`；`modules/events/health/incident/schema` 可依赖 `core/parsers/noise/overview/logs`。
- 工作区已有 can1/README 未提交改动，作为基线保留，不单独回滚。

### 验证边界

- 语法：`python3 -m py_compile` 覆盖 `server.py` 与 `autoware_diag/*.py`。
- 启动：`python3 server.py` 能起来（stdio 等待输入，用超时/管道探测而非交互）。
- 行为：对同一只读 MCAP（如 `failcase_data_20260913132945_456.mcap`）抽样对比拆分前后关键工具的顶层键与关键字段（`analyze_rosout` 的 noise 过滤、`analyze_planning_control` 的 trajectory 缺失、`analyze_mcap_file` 的 total_messages）。
- 无现成单测框架；不强制新建 pytest 套件，除非实现时发现某 helper 值得最小单测。

## [S3] Out of Scope

- 不改诊断算法、阈值、话题列表或噪音规则语义。
- 不引入打包发布（setup.py/pyproject）以外的重构；若加 `pyproject.toml` 仅便于本地 import，非必须。
- 不拆分/重写 MCP 协议层。
- 不处理 `mimocode.jsonc` 配置变更。
- 不为本次拆分单独提交/回滚 can1 相关既有改动（随基线保留）。

## Tasks

- [x] T1: 创建 `autoware_diag/` 包并迁入 `core.py` + `noise.py` + `parsers.py` — acceptance: `server.py` 可 import 这些模块；`py_compile` 通过；can1 噪音 API 仍在 `noise.py` 可被调用 (covers: S2)
- [x] T2: 迁入 `mcp.py`（McpServer/McpError/协议）— acceptance: `from autoware_diag.mcp import McpServer, McpError` 可用；`server.py` 不再内嵌协议类 (covers: S2; depends: T1)
- [x] T3: 迁入 `overview.py` + `logs.py` — acceptance: bag 元数据/命名空间/rosout/diagnostics 分析函数从包内可导入，且仍使用 `noise._is_rosout_noise` (covers: S2; depends: T1)
- [x] T4: 迁入 `modules.py` + `events.py` + `health.py` + `incident.py` + `schema.py` — acceptance: 16 个工具对应纯函数均在包内，无对旧闭包变量的依赖 (covers: S2; depends: T1)
- [x] T5: 精简 `server.py` 为薄注册层 — acceptance: 文件明显变短（目标 <400 行）；`python3 server.py` 可启动；工具名与 schema 不变 (covers: S2; depends: T2) — 备注：落地 487 行，契约满足，行数目标未严格达到
- [x] T6: 回归验证 — acceptance: `py_compile` 全通过；对样本 MCAP 对比 `analyze_rosout`（含 noise_filtered）、`analyze_mcap_file`、`analyze_planning_control` 关键字段一致 (covers: S2; depends: T5)
