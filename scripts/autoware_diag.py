#!/usr/bin/env python3
"""Local CLI for Autoware MCAP diagnostics.

Examples:
  python3 scripts/autoware_diag.py overview recording.mcap
  python3 scripts/autoware_diag.py rosout recording.mcap --max-events 100
  python3 scripts/autoware_diag.py incident recording.mcap --timestamp-iso '2026-10-11T12:39:18+08:00'
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Callable, Dict

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from autoware_diag.core import detect_anomalies_local
from autoware_diag.errors import AnalysisError
from autoware_diag.events import detect_aeb_events, detect_mrm_events
from autoware_diag.health import analyze_vehicle_health
from autoware_diag.incident import investigate_incident
from autoware_diag.logs import analyze_diagnostics, analyze_rosout
from autoware_diag.modules import (
    analyze_chassis,
    analyze_localization,
    analyze_mcap_file,
    analyze_perception,
    analyze_planning_control,
    analyze_traffic_light,
)
from autoware_diag.overview import analyze_bag_overview, list_topics
from autoware_diag.schema import describe_message_schema, list_custom_messages


COMMANDS: Dict[str, Callable[[Dict[str, Any]], Any]] = {
    "mcap": analyze_mcap_file,
    "anomalies": detect_anomalies_local,
    "perception": analyze_perception,
    "traffic-light": analyze_traffic_light,
    "chassis": analyze_chassis,
    "localization": analyze_localization,
    "planning-control": analyze_planning_control,
    "mrm": detect_mrm_events,
    "aeb": detect_aeb_events,
    "health": analyze_vehicle_health,
    "incident": investigate_incident,
    "schema": describe_message_schema,
    "custom-messages": list_custom_messages,
    "overview": analyze_bag_overview,
    "rosout": analyze_rosout,
    "diagnostics": analyze_diagnostics,
    "topics": list_topics,
}


def _add_file(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("file_path", type=Path, help="真实 MCAP 文件路径")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="本地 Autoware / Pix Robobus MCAP 诊断")
    sub = parser.add_subparsers(dest="command", required=True)

    for name in COMMANDS:
        command = sub.add_parser(name, help=f"运行 {name} 分析")
        _add_file(command)

    anomalies = sub.choices["anomalies"]
    anomalies.add_argument("--gap-threshold-s", type=float)
    anomalies.add_argument("--min-frequency-hz", type=float)

    rosout = sub.choices["rosout"]
    rosout.add_argument("--max-events", type=int)
    rosout.add_argument("--min-level", type=int)

    diagnostics = sub.choices["diagnostics"]
    diagnostics.add_argument("--max-status-details", type=int)

    schema = sub.choices["schema"]
    schema.add_argument("--topic")
    schema.add_argument("--schema-name")
    schema.add_argument("--include-schema-text", action=argparse.BooleanOptionalAction, default=None)

    incident = sub.choices["incident"]
    incident.add_argument("--timestamp", type=float)
    incident.add_argument("--timestamp-iso")
    incident.add_argument("--window-seconds", type=float)
    incident.add_argument("--start", type=float)
    incident.add_argument("--end", type=float)
    incident.add_argument("--start-iso")
    incident.add_argument("--end-iso")
    return parser


def _args_to_payload(namespace: argparse.Namespace) -> Dict[str, Any]:
    payload = vars(namespace).copy()
    payload["file_path"] = str(payload.pop("file_path"))
    payload.pop("command", None)
    return {key: value for key, value in payload.items() if value is not None}


def _unwrap_result(result: Any) -> Any:
    if not isinstance(result, dict) or "content" not in result:
        return result
    content = result.get("content") or []
    if len(content) != 1 or content[0].get("type") != "text":
        return result
    text = content[0].get("text", "")
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return {"text": text}


def main() -> int:
    parser = _build_parser()
    namespace = parser.parse_args()
    payload = _args_to_payload(namespace)
    try:
        result = COMMANDS[namespace.command](payload)
        print(json.dumps(_unwrap_result(result), ensure_ascii=False, indent=2, default=str))
        return 0
    except AnalysisError as exc:
        print(json.dumps({"error": exc.message, "code": exc.code, "data": exc.data}, ensure_ascii=False, indent=2), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
