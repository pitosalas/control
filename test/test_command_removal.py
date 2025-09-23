#!/usr/bin/env python3
import unittest
from control.commands.command_dispatcher import CommandDispatcher
from control.commands.robot_controller import RobotController
from control.commands.config_manager import ConfigManager


class TestCommandRemoval(unittest.TestCase):
    """Test cases verifying old nav/slam commands have been removed"""

    def setUp(self):
        """Set up test fixtures"""
        self.config_manager = ConfigManager()
        self.controller = RobotController(self.config_manager)
        self.dispatcher = CommandDispatcher(self.controller)

    def test_nav_commands_removed(self):
        """Test that nav.start and nav.stop commands have been removed"""
        removed_nav_commands = ["nav.start", "nav.stop"]

        for cmd in removed_nav_commands:
            with self.subTest(command=cmd):
                self.assertNotIn(cmd, self.dispatcher.commands,
                               f"Command {cmd} should have been removed")

    def test_slam_commands_removed(self):
        """Test that slam.start and slam.stop commands have been removed"""
        removed_slam_commands = ["slam.start", "slam.stop"]

        for cmd in removed_slam_commands:
            with self.subTest(command=cmd):
                self.assertNotIn(cmd, self.dispatcher.commands,
                               f"Command {cmd} should have been removed")

    def test_map_commands_remain(self):
        """Test that map commands are still available"""
        expected_map_commands = [
            "map.save", "map.load", "map.list",
            "map.stop_save", "map.stop_load"
        ]

        for cmd in expected_map_commands:
            with self.subTest(command=cmd):
                self.assertIn(cmd, self.dispatcher.commands,
                            f"Map command {cmd} should still exist")

    def test_other_commands_unaffected(self):
        """Test that other command groups remain unaffected"""
        # Sample commands from other groups that should still exist
        expected_commands = [
            "move.distance", "move.time",  # movement commands
            "turn.time", "turn.radians", "turn.degrees",  # turning commands
            "robot.stop", "robot.status",  # control commands
            "config.set", "config.get", "system.topics"  # system commands
        ]

        for cmd in expected_commands:
            with self.subTest(command=cmd):
                self.assertIn(cmd, self.dispatcher.commands,
                            f"Command {cmd} should still exist")

    def test_command_count_reduced(self):
        """Test that total command count has been reduced appropriately"""
        # Should have removed 4 commands (nav.start, nav.stop, slam.start, slam.stop)
        # Current count should be around 19 (was 23 before removal)
        total_commands = len(self.dispatcher.commands)

        # Verify we have reasonable number of commands (should be 19)
        self.assertEqual(total_commands, 19,
                        f"Expected 19 commands after removal, got {total_commands}")

    def test_navigation_commands_function_only_has_map_commands(self):
        """Test that build_navigation_commands only returns map commands"""
        from control.commands.navigation_commands import build_navigation_commands

        nav_commands = build_navigation_commands()

        # All commands should be map commands
        for cmd_name in nav_commands.keys():
            self.assertTrue(cmd_name.startswith("map."),
                          f"Navigation commands should only contain map commands, found {cmd_name}")

    def test_removed_commands_not_executable(self):
        """Test that attempting to execute removed commands fails gracefully"""
        removed_commands = ["nav.start", "nav.stop", "slam.start", "slam.stop"]

        for cmd in removed_commands:
            with self.subTest(command=cmd):
                response = self.dispatcher.execute(cmd, {})
                self.assertFalse(response.success)
                self.assertIn("unknown command", response.message.lower())


if __name__ == "__main__":
    unittest.main()