#!/usr/bin/env python3
"""Tests for ProcessApi.get_subsystem_status (F22 T01)."""
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from dome_control.commands.config_manager import ConfigManager
from dome_control.ros2_api.process_api import ProcessApi

CONFIG_FILE = str(Path.home() / ".dome" / "config" / "control.yaml")

PS_HEADER = "USER  PID %CPU %MEM VSZ RSS TTY STAT START TIME COMMAND"


def ps_line(pid: str, command: str) -> str:
    return f"pito {pid} 0.1 0.2 1000 2000 pts/0 S 00:00 0:00 {command}"


class TestGetSubsystemStatus(unittest.TestCase):
    def setUp(self):
        config_manager = ConfigManager.create(CONFIG_FILE)
        self.process_api = ProcessApi(config_manager)

    def run_with_ps_output(self, lines: list[str]) -> dict:
        stdout = "\n".join([PS_HEADER, *lines])
        with patch(
            "dome_control.ros2_api.process_api.subprocess.run",
            return_value=Mock(returncode=0, stdout=stdout),
        ):
            return self.process_api.get_subsystem_status()

    def test_all_subsystems_present_when_nothing_running(self):
        status = self.run_with_ps_output([])

        self.assertEqual(
            set(status.keys()),
            {"gendrv", "nav", "semantic", "control", "mission", "vision"},
        )
        for name, entry in status.items():
            self.assertFalse(entry["running"], name)
            self.assertEqual(entry["processes"], [])

    def test_matches_some_subsystems_and_not_others(self):
        lines = [
            ps_line("100", "/usr/bin/python3 micro_ros_agent serial"),
            ps_line("200", "/usr/bin/python3 behavior_manager --ros-args"),
            ps_line("300", "unrelated_process --flag"),
        ]
        status = self.run_with_ps_output(lines)

        self.assertTrue(status["gendrv"]["running"])
        self.assertEqual(status["gendrv"]["processes"][0]["pid"], "100")
        self.assertTrue(status["control"]["running"])
        self.assertEqual(status["control"]["processes"][0]["pid"], "200")

        for name in ("nav", "semantic", "mission", "vision"):
            self.assertFalse(status[name]["running"], name)
            self.assertEqual(status[name]["processes"], [])

    def test_one_process_can_match_multiple_subsystem_keywords_independently(self):
        lines = [
            ps_line("400", "/usr/bin/python3 mission_node --ros-args"),
            ps_line("500", "/usr/bin/python3 dome_vision_ros_node --ros-args"),
        ]
        status = self.run_with_ps_output(lines)

        self.assertTrue(status["mission"]["running"])
        self.assertTrue(status["vision"]["running"])
        self.assertFalse(status["gendrv"]["running"])

    def test_returns_all_not_running_on_ps_failure(self):
        with patch(
            "dome_control.ros2_api.process_api.subprocess.run",
            return_value=Mock(returncode=1, stdout=""),
        ):
            status = self.process_api.get_subsystem_status()

        for entry in status.values():
            self.assertFalse(entry["running"])


class TestGetMissionState(unittest.TestCase):
    def setUp(self):
        config_manager = ConfigManager.create(CONFIG_FILE)
        self.process_api = ProcessApi(config_manager)

    def test_returns_unknown_when_no_message_arrives(self):
        with patch("dome_control.ros2_api.process_api.rclpy.spin_once"):
            state = self.process_api.get_mission_state(timeout_s=0.01)

        self.assertEqual(state, "unknown")

    def test_returns_published_state(self):
        captured = {}

        def fake_create_subscription(msg_type, topic, callback, qos):
            captured["callback"] = callback
            return Mock()

        def fake_spin_once(node, timeout_sec):
            if "callback" in captured and "fired" not in captured:
                captured["fired"] = True
                captured["callback"](Mock(data="EXPLORING"))

        self.process_api.create_subscription = fake_create_subscription
        self.process_api.destroy_subscription = Mock()

        with patch(
            "dome_control.ros2_api.process_api.rclpy.spin_once",
            side_effect=fake_spin_once,
        ):
            state = self.process_api.get_mission_state(timeout_s=1.0)

        self.assertEqual(state, "EXPLORING")


if __name__ == "__main__":
    unittest.main()
