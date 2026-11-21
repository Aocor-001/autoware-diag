# Autoware MCAP Diagnostics

面向 Autoware / Pix Robobus 的本地 MCAP 录包诊断工具。支持整包快速分诊、按时间窗口排查，以及感知、规划控制、底盘、定位、AEB 和 MRM 等专项分析。分析不需要联网，也不会修改原始录包。

## 安装与快速开始

在仓库根目录运行：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python scripts/autoware_diag.py --help
```

下面的 `BAG` 是示例路径，运行前替换为实际录包路径：

```bash
BAG="$HOME/path/to/recording.mcap"
.venv/bin/python scripts/autoware_diag.py triage "$BAG"
```

分析命令每次处理一个 MCAP 文件，结果以 JSON 输出到 stdout；命令执行失败时，错误输出到 stderr，退出码为非零。

## 如何选择命令

| 需求 | 命令 | 行为 |
|---|---|---|
| 先看一个包有什么问题 | `triage BAG` | 一次消息迭代，汇总关键话题覆盖与频率、WARN/ERROR、诊断状态及 AEB/MRM 事件。默认输出不超过 16 KiB，每类最多 10 条代表明细；`--details` 展开更多明细。 |
| 确认包的录制时间或话题 | `inspect BAG` | 仅读 MCAP Summary，返回可用的录制时间、消息数、话题和 Schema。无 Summary 时不自动扫描。 |
| 已知故障时间 | `incident BAG` | 直接查询指定窗口，不必先跑整包分诊。有时间索引时跳过无关 chunk；无索引时标记 `linear_fallback`。 |
| 已知故障领域 | 对应专项命令 | 只查当前症状所需的证据，不必逐个运行全部命令。 |

指定时刻（以下示例查前后各 2 秒）：

```bash
.venv/bin/python scripts/autoware_diag.py incident "$BAG" \
  --timestamp-iso '2025-01-01T12:00:00+08:00' --window-seconds 2
```

也可用 `--start-iso` 和 `--end-iso` 指定起止时间。窗口按 `log_time` 取闭区间，不自动移动或扩大；第一条可用状态已经激活时，不能据此判定事件在窗口内首次触发。多个候选包先用 `inspect` 核对各自时间范围；缺少 Summary 时，覆盖情况未知，不应仅凭文件名排除。

<details>
<summary>专项命令</summary>

| 关注内容 | 命令 |
|---|---|
| 内容概览、话题、消息定义 | `overview`、`mcap`、`topics`、`schema`、`custom-messages` |
| 日志、诊断状态、健康与频率 | `rosout`、`diagnostics`、`health`、`anomalies` |
| 感知、红绿灯、规划控制 | `perception`、`traffic-light`、`planning-control` |
| 底盘、定位和安全事件 | `chassis`、`localization`、`aeb`、`mrm` |

</details>

## Codex Skill 与录包查找

仓库内置 `$autoware-diag` Skill，位于 `.agents/skills/autoware-diag/`。在 Codex 中打开仓库根目录后，可输入「使用 `$autoware-diag` 分析指定车辆和日期的录包」。Skill 选择诊断流程，CLI 负责读取录包。

未提供文件路径时，Skill 默认在**当前用户**的 `$HOME/.pix-mcap-cache/` 下查找。按车辆和日期组织录包时，可采用以下目录结构（占位符按实际情况替换）：

```text
~/.pix-mcap-cache/
└── pix-failcase-data/
    └── <车辆>/<日期>/Mcap/
```

Skill 只在指定车辆、日期范围内查找；候选不唯一时先确认。CLI 不负责查找录包，分析命令仍需传入实际文件路径。

## 派生缓存与结果分享

`inspect` 和 `triage` 的派生缓存默认保存在 `~/.pix-mcap-cache/.autoware-diag-cache/`，可用 `AUTOWARE_DIAG_CACHE_DIR` 指定其他目录。缓存只保留元数据和已完整分析的有界摘要，不复制录包或保存原始消息正文。上限为每包 1 MiB、总计 256 MiB / 500 个包，超出时优先淘汰最久未使用的条目。命中前会核验源文件身份；缓存不可用时继续分析。

```bash
.venv/bin/python scripts/autoware_diag.py cache info
.venv/bin/python scripts/autoware_diag.py cache clear
.venv/bin/python scripts/autoware_diag.py triage "$BAG" --no-cache
```

`cache clear` 只清理本工具的派生缓存；`inspect` 和 `triage` 均支持 `--no-cache`。分析 JSON 可能含来源绝对路径、车辆或文件标识及诊断内容。**公开分享前请先脱敏**，不要把真实录包、缓存或原始诊断输出提交到仓库。

## 解读与验证

- 话题未录制、窗口内无消息、解码失败和输出省略是不同情况；缺少证据不等于没有事件。分诊的总计数与展示明细数量不同。
- 用 `log_time` 对齐上下游证据；记录先后不等于因果。AEB 判定优先看 `/pix/AebInfoShadow` 的滤波后请求字段。
- 跨包排查需分别保留来源及时间覆盖，不能跨录制缺口推断状态连续。

测试使用程序生成的 MCAP，不依赖真实录包：

```bash
.venv/bin/python -m unittest discover -s tests -v
```
