#!/usr/bin/env python3
import unittest
from unittest.mock import MagicMock
from control.commands.command_dispatcher import CommandDispatcher
from control.commands.robot_controller import RobotController, CommandResponse
from control.commands.config_manager import ConfigManager


class TestLaunchCommands(unittest.TestCase):
    """Test cases for new launch command definitions"""

    def setUp(self):
        """Set up test fixtures"""
        self.config_manager = ConfigManager()
        self.controller = RobotController(self.config_manager)
        self.dispatcher = CommandDispatcher(self.controller)

        # Mock the controller methods to avoid actual execution
        self.controller.launch_list = MagicMock(return_value=CommandResponse(True, "Mock list"))
        self.controller.launch_start = MagicMock(return_value=CommandResponse(True, "Mock start"))
        self.controller.launch_kill = MagicMock(return_value=CommandResponse(True, "Mock kill"))
        self.controller.launch_status = MagicMock(return_value=CommandResponse(True, "Mock status"))

    def test_launch_commands_registered(self):
        """Test that all launch commands are properly registered"""
        expected_commands = [
            "launch.list", "launch.start", "launch.kill", "launch.status"
        ]

        for cmd in expected_commands:
            with self.subTest(command=cmd):
                self.assertIn(cmd, self.dispatcher.commands,
                            f"Launch command {cmd} should be registered")

    def test_launch_list_command(self):
        """Test launch.list command definition and execution"""
        cmd_def = self.dispatcher.commands["launch.list"]

        # Test command definition
        self.assertEqual(cmd_def.method_name, "launch_list")
        self.assertEqual(cmd_def.description, "List all available launch types with status")
        self.assertEqual(cmd_def.group, "launch")
        self.assertEqual(len(cmd_def.parameters), 0)

        # Test execution
        response = self.dispatcher.execute("launch.list", {})
        self.assertTrue(response.success)
        self.controller.launch_list.assert_called_once_with()

    def test_launch_start_command(self):
        """Test launch.start command definition and execution"""
        cmd_def = self.dispatcher.commands["launch.start"]

        # Test command definition
        self.assertEqual(cmd_def.method_name, "launch_start")
        self.assertEqual(cmd_def.description, "Start a launch process by type")
        self.assertEqual(cmd_def.group, "launch")
        self.assertEqual(len(cmd_def.parameters), 2)

        # Test parameter definitions
        param_names = [p.name for p in cmd_def.parameters]
        self.assertIn("launch_type", param_names)
        self.assertIn("use_sim_time", param_names)

        # Test execution with required parameter
        response = self.dispatcher.execute("launch.start", {"launch_type": "nav"})
        self.assertTrue(response.success)
        self.controller.launch_start.assert_called_with(launch_type="nav", use_sim_time=False)

        # Test execution with optional parameter
        self.controller.launch_start.reset_mock()
        response = self.dispatcher.execute("launch.start", {
            "launch_type": "slam",
            "use_sim_time": True
        })
        self.assertTrue(response.success)
        self.controller.launch_start.assert_called_with(launch_type="slam", use_sim_time=True)

    def test_launch_kill_command(self):
        """Test launch.kill command definition and execution"""
        cmd_def = self.dispatcher.commands["launch.kill"]

        # Test command definition
        self.assertEqual(cmd_def.method_name, "launch_kill")
        self.assertEqual(cmd_def.description, "Stop a launch process by type")
        self.assertEqual(cmd_def.group, "launch")
        self.assertEqual(len(cmd_def.parameters), 1)

        # Test parameter definition
        param = cmd_def.parameters[0]
        self.assertEqual(param.name, "launch_type")
        self.assertEqual(param.param_type, str)
        self.assertTrue(param.required)

        # Test execution
        response = self.dispatcher.execute("launch.kill", {"launch_type": "nav"})
        self.assertTrue(response.success)
        self.controller.launch_kill.assert_called_once_with(launch_type="nav")

    def test_launch_status_command(self):
        """Test launch.status command definition and execution"""
        cmd_def = self.dispatcher.commands["launch.status"]

        # Test command definition
        self.assertEqual(cmd_def.method_name, "launch_status")
        self.assertEqual(cmd_def.description, "Show status of launch processes")
        self.assertEqual(cmd_def.group, "launch")
        self.assertEqual(len(cmd_def.parameters), 1)

        # Test parameter definition
        param = cmd_def.parameters[0]
        self.assertEqual(param.name, "launch_type")
        self.assertEqual(param.param_type, str)
        self.assertFalse(param.required)  # Optional parameter

        # Test execution without parameter (all status)
        response = self.dispatcher.execute("launch.status", {})
        self.assertTrue(response.success)
        self.controller.launch_status.assert_called_with()

        # Test execution with specific launch type
        self.controller.launch_status.reset_mock()
        response = self.dispatcher.execute("launch.status", {"launch_type": "slam"})
        self.assertTrue(response.success)
        self.controller.launch_status.assert_called_with(launch_type="slam")

    def test_launch_commands_group(self):
        """Test that all launch commands are in the launch group"""
        launch_commands = [cmd for cmd in self.dispatcher.commands.keys()
                          if cmd.startswith("launch.")]

        for cmd in launch_commands:
            with self.subTest(command=cmd):
                cmd_def = self.dispatcher.commands[cmd]
                self.assertEqual(cmd_def.group, "launch",
                               f"Command {cmd} should be in launch group")

    def test_command_count_increased(self):
        """Test that total command count increased with new launch commands"""
        total_commands = len(self.dispatcher.commands)

        # Should have added 4 launch commands
        # Current count should be 21
        self.assertEqual(total_commands, 21,
                        f"Expected 21 commands after adding launch commands, got {total_commands}")


if __name__ == "__main__":
    unittest.main()