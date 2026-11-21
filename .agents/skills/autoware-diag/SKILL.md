---
name: autoware-diag
description: "Use when the user explicitly asks to analyze Autoware or Pix Robobus MCAP recordings, topics, or incident time windows for perception, traffic lights, localization, planning/control, chassis, AEB, FCW, or MRM. Do not use for ordinary audio transcription, general software troubleshooting, or source-code discussion without MCAP evidence."
---

# Autoware Diagnostics (MCAP)

Diagnose Pix Robobus / Autoware MCAP recordings with the repository's local CLI. Build evidence chains; do not infer a root cause from an isolated anomaly.

## Hard rules

1. **`file_path` must be real.** Resolve it from the cache convention below; ask only if missing or ambiguous.
2. **Preserve the user's time.** Copy the user's timestamp, start, and end verbatim. Do not shift a supplied time. If an ISO time lacks an offset and timezone affects which recording or window is selected, ask which timezone it represents. After `investigate_incident`, verify returned `time_range`.
3. **Check evidence availability before diagnosis.** Read `ros2_decoding`, topic/message counts, filters, and truncation fields such as `events_suppressed` before interpreting missing results. A decoder being enabled does not prove every message decoded.
4. **State missing evidence precisely.** Use only the applicable wording:
   - Topic inventory or message count proves absence: `not recorded` / `no messages on this topic`.
   - Topic exists but the selected window has no message: `not observed in this window`.
   - Message content could not be decoded: `content unavailable; event cannot be excluded`.
   - Output was truncated or filtered: `the returned output is incomplete`.
   - Decoding, coverage, and detection conditions are sufficient: `not detected in the inspected scope`.
5. **Evidence only.** Every conclusion needs returned MCAP data with a topic and timestamp (`publish_time` and/or `log_time`). State uncertainty and the missing evidence when data is insufficient.
6. **Cross-validate.** Check upstream, downstream, and sibling topics. An isolated anomaly is a candidate cause, not a root cause.
7. **Treat time ordering as evidence, not proof of causality.** Use `log_time` to organize the recording timeline. Account for message timestamps and known transport/recording delay before assigning cause and effect. If alignment is insufficient, state that causality cannot be confirmed.
8. **No pre-window blame.** Errors before a target range are independent unless their state is visibly continuous into that range; cite the continuous interval.
9. **Bound incident windows.** Keep `window_seconds` at or below 30 seconds unless the user requests more.
10. **On error, read the message.** Do not retry identical parameters.
11. **Use the local CLI.** Resolve `REPO` as described below, then run `python3 "REPO/scripts/autoware_diag.py" <command> "FILE"` with the actual paths substituted. Do not invent a path or silently replace the project CLI with ad-hoc parsing.

## MCAP discovery convention

When the user asks to inspect a bag without providing an absolute path, search `~/.pix-mcap-cache` first. This is the project's persistent local MCAP cache, organized by vehicle. The currently observed layout is `~/.pix-mcap-cache/pix-failcase-data/<vehicle>/<date>/Mcap/`. Use the actual path, but do not walk all vehicles or dates to verify this layout.

The user will normally specify the vehicle and bag. Resolve only candidates under that cache root:

1. Open the requested vehicle directory directly; if the date is known, open that date directory directly.
2. Match the requested bag/name inside that scope (`Mcap/` first). Reuse any paths found in this task. Never recursively scan the cache root.
3. If there is exactly one matching file, use its absolute path.
4. If there are multiple matches, show the full candidate paths and ask which bag to inspect.
5. If no match exists, report that the requested bag was not found in the cache and ask for a more precise vehicle/date/bag identifier. Do not invent a path, silently select another vehicle, or silently search another root.

Always include the selected absolute MCAP path in the evidence/report. The cache lookup convention is project-specific and takes precedence over generic recording-path examples below.

## CLI command map

Codex discovers this skill directly at `.agents/skills/autoware-diag/SKILL.md`. Resolve the loaded `SKILL.md` path; `REPO` is the fourth parent of that file (`.agents/skills/autoware-diag/SKILL.md` → repository root). Confirm that `REPO/scripts/autoware_diag.py` exists. Do not assume the current working directory is the repository root: the project may be opened at its containing directory.

If `REPO/.venv/bin/python` exists, use it instead of `python3` in all commands below so the CLI uses the project dependencies even when no environment is activated. Otherwise use `python3`. A missing dependency is a setup failure, not evidence about a recording; follow the README installation instructions rather than silently changing the analysis method.

In the examples below, replace `REPO` and `FILE` with the actual repository and recording paths. Use the absolute CLI path so commands work from either directory; quote paths containing spaces. Each command writes structured JSON to stdout and errors to stderr with a nonzero exit code.

| Intent | Command |
|---|---|
| Summary-only metadata / candidate time ranges | `inspect` (does not scan when Summary is unavailable) |
| Default bag review | `triage` (use `--details` only when more examples would help) |
| Detailed package structure | `python3 "REPO/scripts/autoware_diag.py" overview "FILE"` |
| Node logs | `python3 "REPO/scripts/autoware_diag.py" rosout "FILE"` |
| Diagnostics | `python3 "REPO/scripts/autoware_diag.py" diagnostics "FILE"` |
| Vehicle health | `python3 "REPO/scripts/autoware_diag.py" health "FILE"` |
| Gaps and frequency | `python3 "REPO/scripts/autoware_diag.py" anomalies "FILE"` |
| Traffic lights | `python3 "REPO/scripts/autoware_diag.py" traffic-light "FILE"` |
| Planning/control | `python3 "REPO/scripts/autoware_diag.py" planning-control "FILE"` |
| Perception/chassis/localization | `perception`, `chassis`, `localization` |
| AEB/MRM | `aeb`, `mrm` |
| Incident window | `python3 "REPO/scripts/autoware_diag.py" incident "FILE" --timestamp-iso 'USER_TIME' --window-seconds 2` |
| Topics and schemas | `topics`, `schema`, `custom-messages` |

The CLI is local and does not require an MCP server or external service. Keep the JSON result as the evidence source and report command failures instead of treating them as missing recording data.

## Evidence availability gate

Before any absence, health, or root-cause conclusion:

1. Reuse topic/window counts already returned by `triage` or `incident`. Use `inspect` or `topics` only if topic inventory is still needed.
2. Check `ros2_decoding`. If it is disabled or reports an error, content-based event absence cannot be concluded.
3. Check tool-specific scope, `decode_failures`, omitted counts, and truncation. Cache hits describe the analyzed source identity; they do not prove ongoing access to the recording. For example, `analyze_rosout` may report `events_suppressed`; an incident response may reduce timeline or snapshot entries to fit its response budget.
4. If evidence is incomplete, report the limitation first, then make only observations supported by the available result.

## Select a route by symptom

Use the user's symptom to minimize unrelated scans. Every next tool call must state which candidate explanation it distinguishes. Stop when evidence is insufficient or the candidate has been confirmed or ruled out; do not run every module for completeness.

| Symptom | Start with | Cross-check next |
|---|---|---|
| Hard brake / unexpected deceleration | `aeb` or `mrm`, depending on the symptom | Determine whether AEB/MRM, planned control, or manual takeover commanded it; then inspect its trigger evidence. |
| Green but vehicle does not move | `traffic-light` | Match the same signal group and time; verify final traffic light state, stop/planning evidence, and whether control remained autonomous. |
| Cannot enter autonomous operation | engage, operation mode, gate mode, `analyze_diagnostics` | Identify the blocking condition and its associated diagnostic or module state. |
| Steering abnormal | steering command and feedback via `chassis` | Compare command versus feedback, then assess trajectory/curvature, localization, and communication status. |
| Localization jump / instability | `analyze_localization`, relevant diagnostics | Check source consistency, timing, and downstream control/planning effects. |

## Default workflow

- **Given a time and a single bag:** run `incident` directly. Do not run metadata or full-bag commands first. Pass the exact user time/range; never shift or enlarge it automatically. An initially active state is not a new trigger in the window.
- **“Look at this bag,” no symptom/time:** run `triage` once. Summarize coverage, observed WARN/ERROR and diagnostic statuses, AEB/MRM, and the most useful next check. Zero observations with missing/undecoded evidence cannot exclude an event.
- **Given a symptom without a time:** select a relevant module from the symptom table. Open another module only if it supplies missing discriminating evidence.
- **Several candidate files for a time:** run `inspect` on those candidates and investigate only files whose `log_time` range covers the request. An unavailable Summary is unknown coverage, not an exclusion; do not launch silent full scans. Ask when candidates remain ambiguous.
- Reuse JSON already returned in this task. Do not repeat identical calls, start parallel overlapping scans on one bag, or chain `overview → rosout → diagnostics → health → anomalies → aeb → mrm` by default.

## Traffic light / green-not-go route

If a timestamp is given, start with `incident` for window context. Otherwise start with `traffic-light` to check the recognition chain and final `traffic_signals`. Use `inspect` only to locate time across multiple candidate files. Add `planning-control` for missing stop/release evidence or `rosout` for missing fine-detector/takeover evidence; reuse what the earlier result already provides and align all evidence by timestamp and source.

`investigate_incident` currently supplies surrounding planning/control context; it does **not** independently establish traffic-light state. Attribute signal-state evidence to `analyze_traffic_light`.

**Fine-detector evidence:** `TL_FINE_DETECTOR_NO_MATCH` or `TL_FINE_DETECTOR_EMPTY_OUTPUT` is a candidate explanation for UNKNOWN output, not proof that it made the vehicle stop. To establish that chain, verify the same relevant signal group and matching period in classification and final output, obtain planning stop evidence, and verify control mode. State any unavailable link.

**Takeover evidence:** `control_mode` 1 -> 4, `Brake/Teleop Takeover event started`, or `is_autoware_control_enabled=false` indicates takeover evidence. Vehicle motion after takeover is not evidence that autonomous driving released the vehicle. Time ordering alone does not identify a specific human action.

## Cross-check guide

- Planning absent or abnormal: `/planning/scenario_planning/scenario` upstream plus `/control/trajectory_follower/control_cmd` downstream.
- Control abnormal: lateral diagnostics versus longitudinal diagnostics, and command versus chassis feedback.
- Perception abnormal: LiDAR cloud, tracking, predicted objects, and downstream planning effect.
- BEV healthy but vehicle does not move after green: use the traffic-light chain, final `traffic_signals`, planning evidence, and takeover state. BEV message rate or object count does not demonstrate traffic-light recognition.
- Chassis feedback absent: compare `av_*_ctrl` with `va_*_fb`; inspect `*_life` and `*_communication_lost` only when decoded evidence is available.
- AEB/FCW: prefer `/pix/AebInfoShadow` fields `*_req_filter`; `*_req_no_filter=1` and `*_req_filter=0` means `REQUEST_FILTERED`. Check the legacy `/pix_robobus/av_aeb_ctrl` only when applicable.

## Custom field constraints

- `pix_robobus_driver_msgs/*`: steering angle is degrees, not radians. `*_life` stuck or `*_communication_lost` requires decoded supporting data.
- `aeb_interfaces/AebInfoShadow`: prefer `*_req_filter`; the unfiltered request alone is not an active brake request.
- `aeb_interfaces/AebObstacleDebugData`: closest object distance and velocity may be unavailable without a target.
- `autoware_control_msgs/Control`: uses `lateral.steering_tire_angle` in radians and `longitudinal.velocity`, `acceleration`, and `jerk`; it has no top-level `steering`.
- `point_cloud_interfaces/CompressedPointCloud2`: compressed point cloud, not direct XYZ data.
- `tier4_control_msgs/GateMode`: field `data`; 0 is AUTO and 1 is EXTERNAL.
- `autoware_internal_planning_msgs/Scenario`: field `current_scenario`.
- `autoware_planning_validator/PlanningValidatorStatus`: `is_valid_*=false` is a trajectory-validation anomaly; pair it with timing and control evidence.
- Traffic-light message families must not be mixed: classifier `tier4_perception_msgs/TrafficLightElement` has RED=1, AMBER=2, GREEN=3; internal/judged/final `autoware_perception_msgs/TrafficLightElement` has UNKNOWN=0, RED=1, GREEN=3, and SOLID_ON=2. Planning consumes `/perception/traffic_light_recognition/traffic_signals`.

## Incident investigation

`investigate_incident` accepts:

- Point mode: `timestamp` or `timestamp_iso`, optionally `window_seconds` (default 2).
- Range mode: paired `start` and `end`, or paired `start_iso` and `end_iso`.

Pass user-provided values unchanged. After the call, compare its returned `time_range` with the requested interval. Use its `log_time` to align events, but do not convert observed order directly into causality.

## Capability boundaries

The current local CLI analyzes a supplied MCAP file per call. It does not currently provide a single multi-file incident-window query, a unified cross-file event timeline, or a normal-versus-abnormal comparison tool. For multiple files, map each file's actual time range separately, identify which file covers the user's time, analyze each required file explicitly, and label every source file. Do not claim continuous state across an uncovered gap or present multi-file correlation as a built-in result.

## Candidate-cause review

When there is more than one plausible explanation, organize the report as a small table:

| Candidate | Supporting evidence | Counter-evidence | Missing evidence | Next discriminating check | Status |
|---|---|---|---|---|---|
| Candidate from the returned data | topic, timestamp, tool | observed contrary data | unavailable topic, decode, or time range | one check that separates candidates | supported / not supported / unresolved |

Do not invent confidence scores. Missing output is not sufficient counter-evidence.

## Report format

```text
[Phenomenon] user description and requested time/range
[Evidence availability] decoding state, topic/window coverage, filters, truncation
[Evidence chain] topic + timestamp + source tool for each observation
[Timeline] observed log_time ordering; state any timing limitations
[Candidate causes] supported, ruled out, and unresolved candidates with evidence gaps
[Conclusion] direct observation, root cause only if supported, and excluded items with reasons
[Follow-up] short-term action, required additional capture, and retest conditions
```

Put CRITICAL events first: loss of control, MRM, or AEB fault. Then HIGH events: module failure, no pose, or no chassis feedback.

## Retest guidance

When the evidence is insufficient or a change is proposed, state:

- Which topic, log, or schema must be captured next and why.
- The required window before and after the symptom, plus relevant operational conditions.
- What observable condition would demonstrate the intended behavior after a fix.
- Which conclusions current recordings can verify and which require another test.

Do not invent pass thresholds. Use a documented requirement, configuration, or validated baseline; otherwise mark the threshold as to be determined.

## Answering the user

- Use only tool-returned facts; distinguish not recorded, not observed in window, undecodable, incomplete output, and not detected in inspected scope.
- Cite topic, timestamp, tool, and source file for AEB/MRM events, gaps, frequency drops, and causal claims.
- For files over 500 MB, warn that full scans may be slower and prefer targeted tools when the symptom is known.
- Reply in Chinese when the user writes Chinese; retain topic and tool names in English.
