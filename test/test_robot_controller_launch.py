#!/usr/bin/env python3
import unittest
from unittest.mock import MagicMock, patch

from control.commands.config_manager import ConfigManager
from control.commands.robot_controller import RobotController

CONFIG_FILE = "/home/pitosalas/.control/config.yaml"


class TestRobotControllerLaunch(unittest.TestCase):
    """Test cases for RobotController launch methods"""

    @patch("control.commands.robot_controller.ProcessApi")
    @patch("control.commands.robot_controller.MovementApi")
    @patch("control.commands.robot_controller.CalibrationApi")
    def setUp(self, mock_calib, mock_move, mock_proc):
        """Set up test fixtures"""
        self.config_manager = ConfigManager(CONFIG_FILE)
        self.controller = RobotController(self.config_manager)

        # Mock the ProcessApi to avoid actual process launching
        self.controller.process = MagicMock()

    def test_launch_list(self):
        """Test launch list returns available launch types"""
        self.controller.process.get_available_launch_types.return_value = [
            "nav",
            "slam",
            "map",
        ]
        mock_configs = {
            "nav": MagicMock(launch_type="nav", description="Navigation stack"),
            "slam": MagicMock(launch_type="slam", description="SLAM toolbox"),
            "map": MagicMock(launch_type="map", description="Map server"),
        }
        self.controller.process.get_launch_config.side_effect = lambda t: mock_configs[t]
        self.controller._is_launch_running = MagicMock(return_value=False)

        response = self.controller.launch_list()

        self.assertTrue(response.success)
        self.assertIn("NAME", response.message)
        self.assertIn("DESCRIPTION", response.message)

    def test_launch_start_success(self):
        """Test successful launch start"""
        self.controller.process.launch_by_type.return_value = "test-process-id"
        self.controller._is_launch_running = MagicMock(return_value=False)

        response = self.controller.launch_start("nav", use_sim_time=True)

        self.assertTrue(response.success)
        self.assertEqual(response.message, "Started nav")
        self.assertEqual(response.data["process_id"], "test-process-id")

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

    def test_launch_stop_success(self):
        """Test successful launch stop"""
        self.controller.launch_process_ids["nav"] = "test-process-id"
        self.controller.process.is_process_running.return_value = True
        self.controller.process.kill_by_type.return_value = True

        response = self.controller.launch_stop("nav")

        self.assertTrue(response.success)
        self.assertEqual(response.message, "nav stopped")
        self.assertIsNone(self.controller.launch_process_ids["nav"])

    def test_launch_stop_not_running(self):
        """Test launch stop when nothing is running"""
        self.controller.launch_process_ids["nav"] = None

        response = self.controller.launch_stop("nav")

        self.assertFalse(response.success)
        self.assertIn("No nav running", response.message)


if __name__ == "__main__":
    unittest.main()
