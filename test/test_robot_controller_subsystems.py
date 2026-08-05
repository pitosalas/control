#!/usr/bin/env python3
"""Tests for RobotController.get_subsystems_status (F22 T03)."""
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from dome_control.commands.config_manager import ConfigManager
from dome_control.commands.robot_controller import RobotController

CONFIG_FILE = str(Path.home() / ".dome" / "config" / "control.yaml")

FAKE_SUBSYSTEMS = {
    "gendrv": {"running": False, "processes": []},
    "nav": {"running": False, "processes": []},
    "semantic": {"running": False, "processes": []},
    "control": {"running": True, "processes": [{"pid": "1", "cpu": "0", "mem": "0", "command": "behavior_manager"}]},
    "mission": {"running": True, "processes": [{"pid": "2", "cpu": "0", "mem": "0", "command": "mission_node"}]},
    "vision": {"running": False, "processes": []},
}


class TestGetSubsystemsStatus(unittest.TestCase):
    def setUp(self):
        config_manager = ConfigManager.create(CONFIG_FILE)
        with patch("dome_control.commands.robot_controller.ProcessApi"), \
             patch("dome_control.commands.robot_controller.MovementApi"), \
             patch("dome_control.commands.robot_controller.CalibrationApi"):
            self.controller = RobotController(config_manager)
        self.controller.process_node = MagicMock()

    def test_reports_all_six_subsystems_with_mission_state(self):
        self.controller.process_node.get_subsystem_status.return_value = {
            name: dict(entry) for name, entry in FAKE_SUBSYSTEMS.items()
        }
        self.controller.process_node.get_mission_state.return_value = "EXPLORING"

        response = self.controller.get_subsystems_status()

        self.assertTrue(response.success)
        subsystems = response.data["subsystems"]
        self.assertEqual(
            set(subsystems.keys()),
            {"gendrv", "nav", "semantic", "control", "mission", "vision"},
        )
        self.assertEqual(subsystems["mission"]["state"], "EXPLORING")
        self.assertTrue(subsystems["control"]["running"])
        self.assertFalse(subsystems["gendrv"]["running"])

    def test_mission_state_unknown_when_nothing_published(self):
        self.controller.process_node.get_subsystem_status.return_value = {
            name: dict(entry) for name, entry in FAKE_SUBSYSTEMS.items()
        }
        self.controller.process_node.get_mission_state.return_value = "unknown"

        response = self.controller.get_subsystems_status()

        self.assertEqual(response.data["subsystems"]["mission"]["state"], "unknown")


if __name__ == "__main__":
    unittest.main()
