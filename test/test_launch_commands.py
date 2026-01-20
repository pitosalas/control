#!/usr/bin/env python3
import unittest
from unittest.mock import MagicMock, patch
from control.commands.command_dispatcher import CommandDispatcher
from control.commands.robot_controller import RobotController, CommandResponse
from control.commands.config_manager import ConfigManager

CONFIG_FILE = "/home/pitosalas/.control/config.yaml"


class TestLaunchCommands(unittest.TestCase):
    """Test cases for launch command definitions"""

    @patch("control.commands.robot_controller.ProcessApi")
    @patch("control.commands.robot_controller.MovementApi")
    @patch("control.commands.robot_controller.CalibrationApi")
    def setUp(self, mock_calib, mock_move, mock_proc):
        """Set up test fixtures"""
        self.config_manager = ConfigManager(CONFIG_FILE)
        self.controller = RobotController(self.config_manager)
        self.dispatcher = CommandDispatcher(self.controller)

        # Mock the controller methods to avoid actual execution
        self.controller.launch_list = MagicMock(return_value=CommandResponse(True, "Mock list"))
        self.controller.launch_start = MagicMock(return_value=CommandResponse(True, "Mock start"))
        self.controller.launch_stop = MagicMock(return_value=CommandResponse(True, "Mock stop"))
        self.controller.launch_info = MagicMock(return_value=CommandResponse(True, "Mock info"))

    def test_launch_commands_registered(self):
        """Test that all launch commands are properly registered"""
        expected_commands = [
            "launch.list", "launch.start", "launch.stop", "launch.info"
        ]

        for cmd in expected_commands:
            with self.subTest(command=cmd):
                self.assertIn(cmd, self.dispatcher.commands,
                            f"Launch command {cmd} should be registered")

    def test_launch_list_command(self):
        """Test launch.list command definition and execution"""
        cmd_def = self.dispatcher.commands["launch.list"]

        self.assertEqual(cmd_def.method_name, "launch_list")
        self.assertEqual(cmd_def.group, "launch")
        self.assertEqual(len(cmd_def.parameters), 0)

        response = self.dispatcher.execute("launch.list", {})
        self.assertTrue(response.success)
        self.controller.launch_list.assert_called_once_with()

    def test_launch_start_command(self):
        """Test launch.start command definition and execution"""
        cmd_def = self.dispatcher.commands["launch.start"]

        self.assertEqual(cmd_def.method_name, "launch_start")
        self.assertEqual(cmd_def.group, "launch")

        param_names = [p.name for p in cmd_def.parameters]
        self.assertIn("launch_type", param_names)

        response = self.dispatcher.execute("launch.start", {"launch_type": "nav"})
        self.assertTrue(response.success)

    def test_launch_stop_command(self):
        """Test launch.stop command definition and execution"""
        cmd_def = self.dispatcher.commands["launch.stop"]

        self.assertEqual(cmd_def.method_name, "launch_stop")
        self.assertEqual(cmd_def.group, "launch")
        self.assertEqual(len(cmd_def.parameters), 1)

        param = cmd_def.parameters[0]
        self.assertEqual(param.name, "launch_type")
        self.assertEqual(param.param_type, str)
        self.assertTrue(param.required)

        response = self.dispatcher.execute("launch.stop", {"launch_type": "nav"})
        self.assertTrue(response.success)
        self.controller.launch_stop.assert_called_once_with(launch_type="nav")

    def test_launch_info_command(self):
        """Test launch.info command definition and execution"""
        cmd_def = self.dispatcher.commands["launch.info"]

        self.assertEqual(cmd_def.method_name, "launch_info")
        self.assertEqual(cmd_def.group, "launch")

        response = self.dispatcher.execute("launch.info", {"launch_type": "nav"})
        self.assertTrue(response.success)

    def test_launch_commands_group(self):
        """Test that all launch commands are in the launch group"""
        launch_commands = [cmd for cmd in self.dispatcher.commands.keys()
                          if cmd.startswith("launch.")]

        for cmd in launch_commands:
            with self.subTest(command=cmd):
                cmd_def = self.dispatcher.commands[cmd]
                self.assertEqual(cmd_def.group, "launch",
                               f"Command {cmd} should be in launch group")


if __name__ == "__main__":
    unittest.main()
