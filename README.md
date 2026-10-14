# Autoware MCAP Diagnostics

用于分析 Pix Robobus / Autoware 的本地 MCAP 录包，覆盖感知、红绿灯、规划控制、底盘、定位、AEB 和 MRM。

项目采用本地 Python CLI，不需要 MCP server、外部服务或网络连接。

## 安装

```bash
python3 -m pip install -r requirements.txt
```

依赖：

- `mcap`
- `mcap-ros2-support`
- `python-dateutil`

## 快速开始

所有命令从仓库根目录执行，结果以结构化 JSON 输出到 stdout：

```bash
python3 scripts/autoware_diag.py overview /path/to/recording.mcap
python3 scripts/autoware_diag.py rosout /path/to/recording.mcap
python3 scripts/autoware_diag.py diagnostics /path/to/recording.mcap
```

查看完整命令列表：

```bash
python3 scripts/autoware_diag.py --help
```

## 命令概览

| 命令 | 用途 |
|---|---|
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

每个命令都要求使用真实的 MCAP 路径，不会猜测或补全文件路径。

## 推荐诊断流程

没有明确故障现象时：

```bash
python3 scripts/autoware_diag.py overview RECORDING.mcap
python3 scripts/autoware_diag.py rosout RECORDING.mcap
python3 scripts/autoware_diag.py diagnostics RECORDING.mcap
python3 scripts/autoware_diag.py health RECORDING.mcap
python3 scripts/autoware_diag.py anomalies RECORDING.mcap
```

然后根据结果进入具体模块，并用 `incident` 对关键时间点做定向核查。

### 突然刹车

```bash
python3 scripts/autoware_diag.py aeb RECORDING.mcap
python3 scripts/autoware_diag.py mrm RECORDING.mcap
python3 scripts/autoware_diag.py planning-control RECORDING.mcap
python3 scripts/autoware_diag.py chassis RECORDING.mcap
```

### 绿灯后不走

```bash
python3 scripts/autoware_diag.py traffic-light RECORDING.mcap
python3 scripts/autoware_diag.py planning-control RECORDING.mcap
python3 scripts/autoware_diag.py rosout RECORDING.mcap
```

红绿灯结论必须同时核对最终灯态、规划/控制证据和 `control_mode`。BEV 健康或目标数量正常，不代表红绿灯识别正常。

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

执行后检查返回的 `time_range` 是否与请求一致。

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

1. 分别运行 `overview` 获取每个文件的真实时间范围。
2. 按记录时间确定目标时刻所在的文件。
3. 对每个相关文件分别执行定域分析。
4. 在报告中保留来源文件，并标记文件间的缺口或重叠。
5. 不要根据文件名编号猜测时间顺序，也不要跨缺口推断状态连续。

## Skill

诊断方法和证据约束位于 `skills/autoware-diag/SKILL.md`。Skill 负责选择诊断路线和解释结果；CLI 负责读取 MCAP 并生成数据。

## 开发验证

```bash
python3 -m py_compile scripts/autoware_diag.py autoware_diag/*.py
python3 scripts/autoware_diag.py --help
```

CLI 错误会输出到 stderr 并返回非零退出码；正常分析结果输出到 stdout，便于保存或继续使用 JSON 工具处理。
