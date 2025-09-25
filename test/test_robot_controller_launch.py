#!/usr/bin/env python3
import unittest
from unittest.mock import MagicMock

from control.commands.config_manager import ConfigManager
from control.commands.robot_controller import RobotController


class TestRobotControllerLaunch(unittest.TestCase):
    """Test cases for RobotController launch methods"""

    def setUp(self):
        """Set up test fixtures"""
        self.config_manager = ConfigManager()
        self.controller = RobotController(self.config_manager)

        # Mock the ProcessApi to avoid actual process launching
        self.controller.process = MagicMock()

    def test_launch_list(self):
        """Test launch list returns available launch types"""
        # Mock ProcessApi methods
        self.controller.process.get_available_launch_types.return_value = [
            "nav",
            "slam",
            "map_server",
        ]
        mock_configs = {
            "nav": MagicMock(description="Navigation stack"),
            "slam": MagicMock(description="SLAM toolbox"),
            "map_server": MagicMock(description="Map server"),
        }
        self.controller.process.get_launch_config.side_effect = lambda t: mock_configs[
            t
        ]

        # Mock launch status
        self.controller._is_launch_running = MagicMock(return_value=False)

        response = self.controller.launch_list()

        self.assertTrue(response.success)
        # New format returns formatted table in message, no data
        self.assertIn("TYPE", response.message)
        self.assertIn("STATUS", response.message)
        self.assertIn("DESCRIPTION", response.message)
        self.assertIn("nav", response.message)
        self.assertIn("slam", response.message)
        self.assertIn("map_server", response.message)

    def test_launch_start_success(self):
        """Test successful launch start"""
        self.controller.process.launch_by_type.return_value = "test-process-id"
        self.controller._is_launch_running = MagicMock(return_value=False)

        response = self.controller.launch_start("nav", use_sim_time=True)

        self.assertTrue(response.success)
        self.assertEqual(response.message, "Started nav")
        self.assertEqual(response.data["process_id"], "test-process-id")
        self.assertEqual(self.controller.launch_process_ids["nav"], "test-process-id")

    def test_launch_start_conflict(self):
        """Test launch start with conflict"""
        self.controller._is_launch_running = MagicMock(return_value=True)

        response = self.controller.launch_start("nav")

        self.assertFalse(response.success)
        self.assertIn("already running", response.message)

    def test_launch_start_invalid_type(self):
        """Test launch start with invalid type"""
        self.controller.process.launch_by_type.side_effect = ValueError(
            "Unknown launch type: invalid"
        )
        self.controller._is_launch_running = MagicMock(return_value=False)

        response = self.controller.launch_start("invalid")

        self.assertFalse(response.success)
        self.assertIn("Unknown launch type", response.message)

    def test_launch_kill_success(self):
        """Test successful launch kill"""
        # Set up running process
        self.controller.launch_process_ids["nav"] = "test-process-id"
        self.controller.process.is_process_running.return_value = True
        self.controller.process.kill_by_type.return_value = True

        response = self.controller.launch_kill("nav")

        self.assertTrue(response.success)
        self.assertEqual(response.message, "nav stopped")
        self.assertIsNone(self.controller.launch_process_ids["nav"])

    def test_launch_kill_not_running(self):
        """Test launch kill when nothing is running"""
        self.controller.launch_process_ids["nav"] = None

        response = self.controller.launch_kill("nav")

        self.assertFalse(response.success)
        self.assertIn("No nav running", response.message)

    def test_launch_status_specific_running(self):
        """Test launch status for specific running launch type"""
        # Set up running process
        self.controller.launch_process_ids["nav"] = "test-process-id"
        self.controller.process.is_process_running.return_value = True

        mock_process_info = MagicMock()
        mock_process_info.pid = 12345
        self.controller.process.processes = {"test-process-id": mock_process_info}

        response = self.controller.launch_status("nav")

        self.assertTrue(response.success)
        self.assertEqual(response.data["launch_type"], "nav")
        self.assertTrue(response.data["running"])
        self.assertEqual(response.data["process_id"], "test-process-id")
        self.assertEqual(response.data["pid"], 12345)

    def test_launch_status_specific_not_running(self):
        """Test launch status for specific non-running launch type"""
        self.controller.launch_process_ids["nav"] = None

        response = self.controller.launch_status("nav")

        self.assertTrue(response.success)
        self.assertEqual(response.data["launch_type"], "nav")
        self.assertFalse(response.data["running"])
        self.assertIsNone(response.data["process_id"])
        self.assertIsNone(response.data["pid"])

    def test_launch_status_all(self):
        """Test launch status for all launch types"""
        self.controller._get_launch_status = MagicMock(
            return_value={
                "nav": {"running": True, "process_id": "nav-id", "pid": 123},
                "slam": {"running": False, "process_id": None, "pid": None},
                "map_server": {"running": False, "process_id": None, "pid": None},
            }
        )

        response = self.controller.launch_status()

        self.assertTrue(response.success)
        self.assertEqual(response.message, "All launch status")
        self.assertIn("launches", response.data)


if __name__ == "__main__":
    unittest.main()
