# Autoware MCAP Diagnostics

用于分析 Pix Robobus / Autoware 的本地 MCAP 录包，覆盖感知、红绿灯、规划控制、底盘、定位、AEB 和 MRM。

项目采用本地 Python CLI，不需要 MCP server、外部服务或网络连接。

## 安装

建议在仓库根目录使用独立虚拟环境，避免依赖安装到错误的 Python：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

后续命令均在已激活的环境中运行；也可以直接用 `.venv/bin/python` 替代示例中的 `python3`。

依赖：

- `mcap`
- `mcap-ros2-support`
- `python-dateutil`

## 快速开始

所有命令从仓库根目录执行，结果以结构化 JSON 输出到 stdout：

```bash
python3 scripts/autoware_diag.py triage /path/to/recording.mcap
```

查看完整命令列表：

```bash
python3 scripts/autoware_diag.py --help
```

## 命令概览

| 命令 | 用途 |
|---|---|
| `inspect` | 仅读 Summary：录制时间、消息数、话题与 Schema 概要，无 Summary 时不扫描 |
| `triage` | 单次迭代分诊：覆盖和频率、WARN/ERROR、诊断状态与 AEB/MRM |
| `cache info` / `cache clear` | 查看 / 清理本工具派生缓存，不接收录包路径 |
| `mcap` | 话题、频率、消息数和包统计 |
| `overview` | 包时间范围、命名空间、节点、元数据和零消息话题 |
| `topics` | 列出录包中的所有话题 |
| `rosout` | 分析 `/rosout` 的 WARN/ERROR/FATAL 日志 |
| `diagnostics` | 聚合 `/diagnostics` 的 ERROR/WARN 状态 |
| `anomalies` | 检测消息间隙和低频话题 |
| `perception` | BEVFusion、目标跟踪、预测和 LiDAR |
| `traffic-light` | 红绿灯识别链路、灯态变化和接管上下文 |
| `planning-control` | 规划、控制指令、场景、GateMode 和控制诊断 |
| `chassis` | Pix Robobus 控制指令、底盘反馈和轮速 |
| `localization` | GNSS、IMU、里程计和姿态 |
| `aeb` | AEB/FCW 请求、状态和障碍物信息 |
| `mrm` | MRM、Emergency、Engage 和 GateMode 事件 |
| `health` | 感知、规划控制、MRM、AEB 和底盘综合健康检查 |
| `incident` | 指定时间点或时间段的上下文调查 |
| `schema` | 查询录包内嵌消息 schema 和字段语义 |
| `custom-messages` | 列出 Pix/AEB/压缩点云等私有消息 |

除 `cache` 管理命令外，分析命令都要求使用真实的 MCAP 路径，不会猜测或补全文件路径。

## MCAP 默认查找约定

项目约定：需要查找录包时，默认先在以下缓存根目录中查找：

```text
~/.pix-mcap-cache
```

缓存目录按车辆组织；当前常见结构为：

```text
~/.pix-mcap-cache/pix-failcase-data/<车辆>/<日期>/Mcap/
```

以后你会告诉我具体查看哪辆车的哪个 bag。我应当按以下规则执行：

1. 只在用户指定的车辆及日期范围内匹配实际文件。已知日期时直达该目录，不递归遍历整个缓存根目录。
2. 只使用实际存在的 MCAP 文件路径；不能根据车辆名、日期或文件名猜测不存在的路径。
3. 如果同一车辆下存在多个候选 bag，列出候选路径并让用户确认，不要自行选择。
4. 如果缓存中找不到指定 bag，明确说明未找到，并请求补充车辆、日期或 bag 名称；不要静默改查其他位置。
5. 分析报告中保留实际使用的完整来源路径，避免跨车辆或跨 bag 混淆。

这条约定同时写入 `.agents/skills/autoware-diag/SKILL.md`，供 Codex 在新会话中继续使用。

## 推荐诊断流程

- **只说「看这个 bag」**：运行一次 `triage RECORDING.mcap`。默认 JSON ≤16 KiB、每类最多 10 条代表明细；计数来自扫描而非明细数量。`--details` 展开有界明细，不自动执行其他模块。日志摘要不保存原始正文，要查原文再定向运行 `rosout` 或 `incident`。
- **已经给出时间**：直接运行 `incident`，不用先运行 `inspect` 或整包分析。仅当需要在多个文件中定位时间时，先用 `inspect` 逐个确认 Summary 时间范围。
- **已知症状但没有时间**：只调用能回答首个问题的模块，例如意外制动先查 `aeb` 或 `mrm`，绿灯不走先查 `traffic-light`。仅在证据有缺口时补查。红绿灯结论仍需核对最终灯态、规划/控制证据和 `control_mode`。
- **复用已取得的结果**：同一轮不重复调用相同命令，不并行扫描同一包，也不默认运行所有模块。

### 指定时间调查

单点模式会分析目标时间前后窗口。用户时间必须原样传入，不要自行偏移：

```bash
python3 scripts/autoware_diag.py incident RECORDING.mcap \
  --timestamp-iso '2026-10-11T12:39:18+08:00' \
  --window-seconds 2
```

时间段模式：

```bash
python3 scripts/autoware_diag.py incident RECORDING.mcap \
  --start-iso '2026-10-11T12:39:16+08:00' \
  --end-iso '2026-10-11T12:39:20+08:00'
```

执行后检查 `time_range` 是否与请求一致。窗口使用闭区间 `log_time`，可精确到纳秒；有 chunk 时间索引时 `scope.mode` 为 `indexed_window`，无索引时为 `linear_fallback`，只流式读取一次。窗口第一条可用状态已经激活时，标注“窗口开始时已激活”，不要当作窗口内首次触发。

### 派生缓存

默认目录 `~/.pix-mcap-cache/.autoware-diag-cache/`。只保存 `inspect` 元数据与完整 `triage` 的有界统计和证据摘要，不复制 MCAP，不存原始消息正文或全量时间戳。每包最多 1 MiB，总计最多 256 MiB / 500 个 bag，超限按最近最少使用淘汰。缓存命中会重新打开源文件并校验文件身份和分析版本；缓存不可用时正常分析并在 JSON 中提示。

```bash
python3 scripts/autoware_diag.py cache info
python3 scripts/autoware_diag.py cache clear
python3 scripts/autoware_diag.py triage RECORDING.mcap --no-cache
```

`--no-cache` 同样适用于 `inspect`，完全跳过派生缓存访问。缓存命中只验证当时的源文件身份，不代表源文件可以一直读取。

## 结果解释原则

- 只有话题清单或消息计数证实不存在时，才写“未录制”。
- 话题存在但窗口内没有消息时，写“当前窗口未观察到”。
- 解码失败时，不能据此排除事件。
- 返回结果被截断或过滤时，必须说明证据不完整。
- 单个异常只能作为候选原因，必须结合上下游和同级话题交叉验证。
- `log_time` 用于组织时间线；记录顺序不单独证明因果关系。
- AEB 判断优先使用 `/pix/AebInfoShadow` 的滤波后请求字段 `*_req_filter`。
- Pix 转向反馈字段使用 degree，不要按 radian 解读。

## 多个录包

当前 CLI 每次处理一个 MCAP 文件，不提供内建的跨文件联合窗口或统一跨包时间线。

处理多个文件时：

1. 分别运行 `inspect` 获取候选文件的 Summary 时间范围。索引不可用时标明覆盖未知，不自动全包扫描。
2. 按记录时间确定目标时刻所在的文件。
3. 对每个相关文件分别执行定域分析。
4. 在报告中保留来源文件，并标记文件间的缺口或重叠。
5. 不要根据文件名编号猜测时间顺序，也不要跨缺口推断状态连续。

## Codex Skill

诊断方法和证据约束位于 `.agents/skills/autoware-diag/SKILL.md`。Skill 负责选择诊断路线和解释结果；CLI 负责读取 MCAP 并生成数据。

Codex 通过项目内的 `.agents/skills/` 发现 Skill。本项目已按 Codex 标准格式直接放置实体 Skill 目录，不使用软链接：

```text
Pix-Autoware/
└── .agents/skills/autoware-diag/
    ├── SKILL.md
    ├── agents/openai.yaml
    └── locales/
```

`agents/openai.yaml` 提供 Codex 的展示名称、简短说明和默认提示词。保留默认的按任务自动匹配，也可以显式使用 `$autoware-diag`。

### 在 Codex 中使用

1. 在 Codex 中打开本仓库目录，或从该目录启动 Codex CLI。
2. 在任务中输入，例如：

   ```text
   使用 $autoware-diag 分析 /实际路径/recording.mcap，调查指定时间段的突然刹车原因。
   ```

3. Skill 会通过其真实文件位置定位仓库，再调用本地 Python CLI；不依赖当前任务的工作目录，不需要 MCP 配置。

如果技能列表未更新，重启 Codex 后再检查。

Skill 的唯一项目入口是仓库内的 `.agents/skills/autoware-diag/`；请在 Codex 中打开 `Pix-Autoware/` 仓库目录。外层 `Pix-autoware/` 不再配置 Skill 入口，原 `.mimocode/skills/` 也不作为 Codex 的 Skill 接入方式。

Skill 入口可被发现不代表诊断 CLI 已可运行：仍需安装上述依赖，并保证 `autoware_diag/` 下的 Python 源文件完整。

## 开发验证

```bash
python3 -m py_compile scripts/autoware_diag.py autoware_diag/*.py
python3 scripts/autoware_diag.py --help
python3 -m unittest discover -s tests -v
```

回归测试会在临时目录生成 ROS2/CDR MCAP，覆盖原有 17 个分析命令和新增分诊、索引窗口与派生缓存的启动和行为，以及消息统计、录制时间间隙、ROS2 解码、日志/诊断、AEB 滤波后请求、紧急命令/退出自动驾驶、时间窗口和错误返回。不需要额外测试依赖或真实录包；测试录包不能替代真实车辆数据验收。

CLI 错误会输出到 stderr 并返回非零退出码；正常分析结果输出到 stdout，便于保存或继续使用 JSON 工具处理。
