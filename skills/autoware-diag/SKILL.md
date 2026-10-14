---
name: autoware-diag
description: "Use when the user explicitly asks to analyze Autoware or Pix Robobus MCAP recordings, topics, or incident time windows for perception, traffic lights, localization, planning/control, chassis, AEB, FCW, or MRM. Do not use for ordinary audio transcription, general software troubleshooting, or source-code discussion without MCAP evidence."
---

# Autoware Diagnostics (MCAP)

Diagnose Pix Robobus / Autoware MCAP recordings with the repository's local CLI. Build evidence chains; do not infer a root cause from an isolated anomaly.

## Hard rules

1. **`file_path` must be real.** Never invent a path; ask for it when missing.
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
11. **Use the local CLI.** From the repository root, run `cd REPO/scripts && python3 autoware_diag.py <command> <file_path>`. Do not invent a path or silently replace the project CLI with ad-hoc parsing.

## CLI command map

Run commands from the repository root. Each command writes structured JSON to stdout and errors to stderr with a nonzero exit code.

| Intent | Command |
|---|---|
| Package structure | `cd REPO/scripts && python3 autoware_diag.py overview FILE` |
| Node logs | `cd REPO/scripts && python3 autoware_diag.py rosout FILE` |
| Diagnostics | `cd REPO/scripts && python3 autoware_diag.py diagnostics FILE` |
| Vehicle health | `cd REPO/scripts && python3 autoware_diag.py health FILE` |
| Gaps and frequency | `cd REPO/scripts && python3 autoware_diag.py anomalies FILE` |
| Traffic lights | `cd REPO/scripts && python3 autoware_diag.py traffic-light FILE` |
| Planning/control | `cd REPO/scripts && python3 autoware_diag.py planning-control FILE` |
| Perception/chassis/localization | `perception`, `chassis`, `localization` |
| AEB/MRM | `aeb`, `mrm` |
| Incident window | `cd REPO/scripts && python3 autoware_diag.py incident FILE --timestamp-iso 'USER_TIME' --window-seconds 2` |
| Topics and schemas | `topics`, `schema`, `custom-messages` |

The CLI is local and does not require an MCP server or external service. Keep the JSON result as the evidence source and report command failures instead of treating them as missing recording data.

## Evidence availability gate

Before any absence, health, or root-cause conclusion:

1. Confirm the recording and relevant topic are present with `analyze_bag_overview` or `list_topics`.
2. Check `ros2_decoding`. If it is disabled or reports an error, content-based event absence cannot be concluded.
3. Check tool-specific scope and truncation. For example, `analyze_rosout` may report `events_suppressed`; an incident response may reduce timeline or snapshot entries to fit its response budget.
4. If evidence is incomplete, report the limitation first, then make only observations supported by the available result.

## Select a route by symptom

Use the user's symptom to minimize unrelated scans. Every next tool call must state which candidate explanation it distinguishes. Stop when evidence is insufficient or the candidate has been confirmed or ruled out; do not run every module for completeness.

| Symptom | Start with | Cross-check next |
|---|---|---|
| Hard brake / unexpected deceleration | `detect_aeb_events`, `detect_mrm_events`, `analyze_planning_control`, chassis feedback | Determine whether AEB/MRM, planned control, or manual takeover commanded it; then inspect its trigger evidence. |
| Green but vehicle does not move | `analyze_traffic_light`, takeover/control state, `analyze_planning_control` | Match the same signal group and time; verify final traffic light state, stop/planning evidence, and whether control remained autonomous. |
| Cannot enter autonomous operation | engage, operation mode, gate mode, `analyze_diagnostics` | Identify the blocking condition and its associated diagnostic or module state. |
| Steering abnormal | steering command and feedback via `analyze_chassis`, `analyze_planning_control` | Compare command versus feedback, then assess trajectory/curvature, localization, and communication status. |
| Localization jump / instability | `analyze_localization`, relevant diagnostics | Check source consistency, timing, and downstream control/planning effects. |

## Standard workflow

Use this path when the symptom does not choose a narrower route:

```text
1. analyze_bag_overview     - map time range, namespaces, nodes, zero-message topics
2. analyze_rosout           - node WARN/ERROR timeline and output completeness
3. analyze_diagnostics      - hardware/status aggregation
4. list_custom_messages + describe_message_schema - when Pix, AEB, or custom fields matter
5. Deep-dive by domain      - perception / traffic_light / planning_control / chassis / localization
6. detect_mrm_events / detect_aeb_events - when safety events are relevant
7. investigate_incident     - planning, control, chassis, localization, AEB/MRM context at an exact time
```

**Fast path:** when the returned evidence already identifies a concrete gap, filtered AEB request, or planning-validator invalid flag, use `investigate_incident` with a 1-2 second window to collect context. Do not claim the window tool covers topics it does not return.

## Traffic light / green-not-go route

```text
1. analyze_bag_overview
   - map each supplied recording's actual time range; do not infer order from filenames.
2. analyze_traffic_light
   - obtain traffic-light chain evidence and final traffic_signals state.
3. analyze_planning_control
   - obtain planning/control evidence for stopping or releasing.
4. analyze_rosout
   - inspect TL fine-detector and takeover logs, including truncation.
5. investigate_incident
   - collect planning, control, chassis, localization, AEB/MRM, rosout, and diagnostics context at the user's exact time.
6. Align results by timestamp and source tool.
```

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
