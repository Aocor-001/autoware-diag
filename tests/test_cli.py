"""Offline CLI regression tests using a generated ROS2/CDR MCAP recording."""
from __future__ import annotations

import datetime as dt
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from mcap_ros2.writer import Writer

REPO_ROOT = Path(__file__).resolve().parents[1]
CLI = REPO_ROOT / "scripts" / "autoware_diag.py"
BASE_SECONDS = 1_800_000_000
BASE_NS = BASE_SECONDS * 1_000_000_000
COMMANDS = (
    "mcap", "anomalies", "perception", "traffic-light", "chassis",
    "localization", "planning-control", "mrm", "aeb", "health", "incident",
    "schema", "custom-messages", "overview", "rosout", "diagnostics", "topics",
)
TIME_DEFINITION = """
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""
LOG_DEFINITION = """builtin_interfaces/Time stamp
uint8 level
string name
string msg
string file
string function
uint32 line
""" + TIME_DEFINITION
DIAGNOSTICS_DEFINITION = """std_msgs/Header header
diagnostic_msgs/DiagnosticStatus[] status
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: diagnostic_msgs/DiagnosticStatus
byte level
string name
string message
string hardware_id
diagnostic_msgs/KeyValue[] values
================================================================================
MSG: diagnostic_msgs/KeyValue
string key
string value
""" + TIME_DEFINITION
AEB_DEFINITION = """uint8 aeb_status
uint8 fcw_status
uint8 aeb_req_filter
uint8 fcw_req_filter
uint8 aeb_req_no_filter
uint8 fcw_req_no_filter
"""


def write_fixture(path: Path) -> None:
    with path.open("wb") as output:
        writer = Writer(output)
        log = writer.register_msgdef("rcl_interfaces/msg/Log", LOG_DEFINITION)
        diagnostics = writer.register_msgdef(
            "diagnostic_msgs/msg/DiagnosticArray", DIAGNOSTICS_DEFINITION,
        )
        aeb = writer.register_msgdef("aeb_interfaces/msg/AebInfoShadow", AEB_DEFINITION)
        emergency = writer.register_msgdef(
            "tier4_vehicle_msgs/msg/VehicleEmergencyStamped", "bool emergency\n",
        )
        engage = writer.register_msgdef("autoware_vehicle_msgs/msg/Engage", "bool engage\n")
        mode = writer.register_msgdef(
            "autoware_vehicle_msgs/msg/ControlModeReport", "uint8 mode\n",
        )
        for index, offset in enumerate((0, 1, 4)):
            stamp = {"sec": BASE_SECONDS + offset, "nanosec": 0}
            messages = (
                ("/rosout", log, {
                    "stamp": stamp, "level": (20, 30, 40)[index],
                    "name": "fixture_node", "msg": f"fixture event {index}",
                    "file": "fixture.cpp", "function": "tick", "line": index + 1,
                }),
                ("/diagnostics", diagnostics, {
                    "header": {"stamp": stamp, "frame_id": "base_link"},
                    "status": [{
                        "level": (0, 1, 2)[index], "name": "fixture_sensor",
                        "message": ("OK", "late", "failed")[index],
                        "hardware_id": "fixture_hardware", "values": [],
                    }],
                }),
                ("/pix/AebInfoShadow", aeb, {
                    "aeb_status": 1, "fcw_status": 1,
                    "aeb_req_filter": int(index == 2), "fcw_req_filter": 0,
                    "aeb_req_no_filter": int(index >= 1), "fcw_req_no_filter": 0,
                }),
                ("/control/command/emergency_cmd", emergency, {"emergency": index == 2}),
                ("/api/autoware/get/engage", engage, {"engage": index != 2}),
                ("/vehicle/status/control_mode", mode, {"mode": 1}),
            )
            for topic, schema, message in messages:
                writer.write_message(
                    topic, schema, message,
                    log_time=BASE_NS + offset * 1_000_000_000,
                    # Deliberately different: gap/window analysis must use log_time.
                    publish_time=BASE_NS + index * 100_000_000,
                )
        writer.finish()


class CliRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.temp = tempfile.TemporaryDirectory(prefix="autoware cli test ")
        cls.directory = Path(cls.temp.name)
        cls.recording = cls.directory / "fixture recording.mcap"
        write_fixture(cls.recording)

    @classmethod
    def tearDownClass(cls) -> None:
        cls.temp.cleanup()

    def invoke(self, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [sys.executable, str(CLI), *map(str, args)],
            cwd=self.directory, capture_output=True, text=True, timeout=30,
            env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
        )

    def analyze(self, command: str, *args: str) -> dict:
        result = self.invoke(command, str(self.recording), *args)
        self.assertEqual(result.returncode, 0, result.stderr)
        payload = json.loads(result.stdout)
        self.assertIsInstance(payload, dict)
        self.assertNotIn("content", payload, "CLI must unwrap the legacy result envelope")
        return payload

    def test_all_command_help(self) -> None:
        result = self.invoke("--help")
        self.assertEqual(result.returncode, 0, result.stderr)
        for command in COMMANDS:
            with self.subTest(command=command):
                self.assertIn(command, result.stdout)
                result_sub = self.invoke(command, "--help")
                self.assertEqual(result_sub.returncode, 0, result_sub.stderr)

    def test_all_commands_emit_json(self) -> None:
        for command in COMMANDS:
            with self.subTest(command=command):
                args = {
                    "incident": ("--timestamp", str(BASE_SECONDS + 1)),
                    "schema": ("--topic", "/pix/AebInfoShadow"),
                }.get(command, ())
                self.analyze(command, *args)

    def test_overview_counts_and_ros2_decoding(self) -> None:
        payload = self.analyze("overview")
        self.assertEqual(payload["ros2_decoding"], "enabled")
        self.assertEqual(payload["basic_info"]["total_messages"], 18)
        self.assertEqual(payload["basic_info"]["unique_topics_in_scan"], 6)
        self.assertEqual(payload["basic_info"]["duration_seconds"], 4)

    def test_topic_inventory(self) -> None:
        payload = self.analyze("topics")
        self.assertEqual(payload["total"], 6)
        self.assertIn("/pix/AebInfoShadow", {row["topic"] for row in payload["topics"]})

    def test_anomalies_use_log_time(self) -> None:
        payload = self.analyze("anomalies", "--gap-threshold-s", "2")
        gaps = [row for row in payload["anomalies"] if row["type"] == "message_gap"]
        self.assertEqual(len(gaps), 6)
        self.assertTrue(all(row["total_gaps"] == 1 for row in gaps))
        self.assertEqual(payload["statistics"]["time_range_ns"]["start"], BASE_NS)
        self.assertEqual(payload["statistics"]["time_range_ns"]["end"], BASE_NS + 4_000_000_000)

    def test_rosout_decodes_levels_and_messages(self) -> None:
        payload = self.analyze("rosout")
        self.assertEqual(payload["ros2_decoding"], "enabled")
        self.assertEqual(payload["total_messages"], 3)
        self.assertEqual([event["level"] for event in payload["events"]], ["WARN", "ERROR"])
        self.assertIn("fixture event 2", json.dumps(payload))

    def test_diagnostics_decodes_nested_status(self) -> None:
        payload = self.analyze("diagnostics")
        self.assertEqual(payload["message_count"], 3)
        self.assertEqual(payload["status_count"], 3)
        self.assertEqual(payload["non_ok_total"], 2)
        self.assertEqual(payload["non_ok_statuses"][0]["level"], 2)

    def test_aeb_activation_uses_filtered_request(self) -> None:
        payload = self.analyze("aeb")
        activations = [row for row in payload["aeb_events"] if row["event"] == "AEB_ACTIVATED"]
        self.assertEqual(len(activations), 1)
        observed = dt.datetime.fromisoformat(activations[0]["time"]).timestamp()
        self.assertEqual(observed, BASE_SECONDS + 4)

    def test_mrm_detects_disengage_and_emergency(self) -> None:
        payload = self.analyze("mrm")
        self.assertEqual(payload["summary"]["disengage_events"], 1)
        self.assertEqual(payload["summary"]["emergency_cmd_activations"], 1)

    def test_incident_range_preserves_requested_instants(self) -> None:
        start = dt.datetime.fromtimestamp(BASE_SECONDS + 1, dt.timezone.utc).isoformat()
        end = dt.datetime.fromtimestamp(BASE_SECONDS + 4, dt.timezone.utc).isoformat()
        payload = self.analyze("incident", "--start-iso", start, "--end-iso", end)
        time_range = payload["time_range"]
        self.assertEqual(dt.datetime.fromisoformat(time_range["start_iso"]).timestamp(), BASE_SECONDS + 1)
        self.assertEqual(dt.datetime.fromisoformat(time_range["end_iso"]).timestamp(), BASE_SECONDS + 4)

    def test_schema_lookup(self) -> None:
        payload = self.analyze("schema", "--topic", "/pix/AebInfoShadow", "--include-schema-text")
        self.assertEqual(len(payload["schemas"]), 1)
        self.assertEqual(payload["schemas"][0]["schema_name"], "aeb_interfaces/msg/AebInfoShadow")
        self.assertIn("aeb_req_filter", json.dumps(payload))

    def test_missing_file_is_structured_error_for_all_commands(self) -> None:
        for command in COMMANDS:
            with self.subTest(command=command):
                result = self.invoke(command, str(self.directory / "missing.mcap"))
                self.assertEqual(result.returncode, 2)
                self.assertEqual(result.stdout, "")
                self.assertIn("File not found", json.loads(result.stderr)["error"])

    def test_empty_file_is_structured_error(self) -> None:
        empty = self.directory / "empty.mcap"
        empty.touch()
        result = self.invoke("overview", str(empty))
        self.assertEqual(result.returncode, 2)
        self.assertIn("empty", json.loads(result.stderr)["error"])

    def test_schema_requires_a_query(self) -> None:
        result = self.invoke("schema", str(self.recording))
        self.assertEqual(result.returncode, 2)
        self.assertEqual(result.stdout, "")
        payload = json.loads(result.stderr)
        self.assertEqual(payload["code"], -32602)
        self.assertIn("topic or schema_name", payload["error"])

    def test_incident_requires_a_timestamp(self) -> None:
        result = self.invoke("incident", str(self.recording))
        self.assertEqual(result.returncode, 2)
        self.assertIn("error", json.loads(result.stderr))


if __name__ == "__main__":
    unittest.main()
