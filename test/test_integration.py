#!/usr/bin/env python3
import pytest
import time
from unittest.mock import patch, MagicMock

class TestNavigationWorkflow:
    """Integration tests for complete navigation stack lifecycle."""

    def test_complete_navigation_workflow(self, command_processor):
        """Test full navigation stack lifecycle with shared state."""
        cp = command_processor

        # Start navigation stack
        result = cp.process_command("nav start stack")
        assert result.success, f"Failed to start nav stack: {result.message}"
        assert result.data is not None, "No data returned from nav start"
        assert 'process_id' in result.data, "No process ID in response"

        process_id = result.data['process_id']
        assert process_id is not None, "Process ID is None"

        # Verify RobotController tracks the process
        assert cp.robot_controller.nav_stack_process_id == process_id

        # Check that process is running
        running_result = cp.robot_controller.is_process_running(process_id)
        assert running_result.success
        assert running_result.data['running'] is True

        # Kill navigation stack
        result = cp.process_command("nav kill stack")
        assert result.success, f"Failed to kill nav stack: {result.message}"
        assert "Navigation stack stopped" in result.message

        # Verify state cleanup in RobotController
        assert cp.robot_controller.nav_stack_process_id is None

        # Try to kill again - should fail gracefully
        result = cp.process_command("nav kill stack")
        assert not result.success, "Should fail when no nav stack running"
        assert "No navigation stack running" in result.message

    def test_navigation_singleton_behavior(self, command_processor):
        """Test that starting nav stack twice kills the first instance."""
        cp = command_processor

        # Start first navigation stack
        result1 = cp.process_command("nav start stack")
        assert result1.success
        first_process_id = result1.data['process_id']

        # Start second navigation stack - should kill first
        result2 = cp.process_command("nav start stack")
        assert result2.success
        second_process_id = result2.data['process_id']

        # Should be different process IDs
        assert first_process_id != second_process_id

        # RobotController should track only the latest
        assert cp.robot_controller.nav_stack_process_id == second_process_id

    def test_navigation_with_sim_time_flag(self, command_processor):
        """Test navigation stack with use_sim_time flag."""
        cp = command_processor

        # Start with sim time flag
        result = cp.process_command("nav start stack --use-sim-time")
        assert result.success
        assert result.data['process_id'] is not None


class TestSharedConfiguration:
    """Integration tests for shared configuration across components."""

    def test_shared_configuration_affects_robot(self, command_processor):
        """Test that CLI variable changes immediately affect robot operations."""
        cp = command_processor

        # Set robot speeds via CLI
        result = cp.process_command("set linear_speed 0.1")
        assert result.success, f"Failed to set linear_speed: {result.message}"

        result = cp.process_command("set angular_speed 0.2")
        assert result.success, f"Failed to set angular_speed: {result.message}"

        # Verify ConfigManager has the new values
        assert cp.config_manager.get_variable('linear_speed') == 0.1
        assert cp.config_manager.get_variable('angular_speed') == 0.2

        # Verify robot controller uses new speeds
        status = cp.robot_controller.get_robot_status()
        assert status.success
        assert status.data['status']['linear'] == 0.1
        assert status.data['status']['angular'] == 0.2

    def test_variable_type_conversion(self, command_processor):
        """Test that variables are properly typed across the system."""
        cp = command_processor

        # Set different types of variables
        test_cases = [
            ("int_var", "42", int, 42),
            ("float_var", "3.14", float, 3.14),
            ("bool_var", "true", bool, True),
            ("str_var", "hello", str, "hello")
        ]

        for var_name, var_value, expected_type, expected_value in test_cases:
            # Set via CLI
            result = cp.process_command(f"set {var_name} {var_value}")
            assert result.success

            # Check type conversion worked
            actual_value = cp.config_manager.get_variable(var_name)
            assert isinstance(actual_value, expected_type)
            assert actual_value == expected_value

            # Verify via show command
            result = cp.process_command(f"show {var_name}")
            assert result.success
            assert expected_type.__name__ in result.message

    def test_config_persistence_across_instances(self, test_config_manager):
        """Test that configuration persists across CommandProcessor instances."""
        # Create first instance and set variables
        with patch('control.robot_controller.ConfigManager', return_value=test_config_manager):
            from control.command_processor import CommandProcessor
            cp1 = CommandProcessor()
            cp1.config_manager = test_config_manager

            result = cp1.process_command("set test_persistence 999")
            assert result.success
            cp1.save_config()

        # Create second instance - should have same config
        with patch('control.robot_controller.ConfigManager', return_value=test_config_manager):
            cp2 = CommandProcessor()
            cp2.config_manager = test_config_manager

            result = cp2.process_command("show test_persistence")
            assert result.success
            assert "999" in result.message


class TestErrorPropagation:
    """Integration tests for error handling across architectural layers."""

    def test_navigation_error_propagation(self, command_processor):
        """Test error handling from API → RobotController → CommandProcessor → CLI."""
        cp = command_processor

        # Try to kill non-existent navigation stack
        result = cp.process_command("nav kill stack")
        assert not result.success, "Should fail when no nav stack running"
        assert "No navigation stack running" in result.message

        # Error should propagate cleanly without exceptions
        assert isinstance(result.message, str)
        assert len(result.message) > 0

    def test_invalid_command_handling(self, command_processor):
        """Test handling of invalid commands."""
        cp = command_processor

        # Invalid command should be handled gracefully
        result = cp.process_command("invalid command here")
        assert not result.success
        assert "Command error" in result.message or "No such command" in result.message

    def test_variable_error_handling(self, command_processor):
        """Test error handling in variable operations."""
        cp = command_processor

        # Show non-existent variable should handle gracefully
        result = cp.process_command("show nonexistent_var")
        # Should either succeed with default or fail gracefully
        assert isinstance(result.success, bool)
        assert isinstance(result.message, str)

    def test_process_management_error_handling(self, command_processor):
        """Test error handling in process management."""
        cp = command_processor

        # Try to kill non-existent process
        fake_process_id = "nonexistent-process-id"
        result = cp.robot_controller.kill_process(fake_process_id)
        assert not result.success
        assert "Failed to kill process" in result.message

    @patch('subprocess.Popen')
    def test_subprocess_failure_handling(self, mock_popen, command_processor):
        """Test handling when subprocess fails to start."""
        cp = command_processor

        # Mock subprocess to raise exception
        mock_popen.side_effect = OSError("Command not found")

        # Should handle subprocess failure gracefully
        with pytest.raises((OSError, Exception)):
            cp.process_command("nav start stack")


class TestEndToEndCommands:
    """Integration tests for complete command workflows."""

    def test_movement_command_flow(self, command_processor):
        """Test movement commands from CLI to API layer."""
        cp = command_processor

        # Test basic movement commands
        movement_commands = [
            "move dist 1.0",
            "move time 2.0",
            "turn radians 1.57",
            "turn degrees 90",
            "turn time 1.0",
            "stop"
        ]

        for cmd in movement_commands:
            result = cp.process_command(cmd)
            assert result.success, f"Command '{cmd}' failed: {result.message}"
            assert isinstance(result.message, str)
            assert len(result.message) > 0

    def test_calibration_command_flow(self, command_processor):
        """Test calibration commands through full stack."""
        cp = command_processor

        result = cp.process_command("calibrate square 1.0")
        assert result.success, f"Calibration failed: {result.message}"
        assert "square calibration" in result.message

    def test_help_system_integration(self, command_processor):
        """Test help system works across all command groups."""
        cp = command_processor

        # Test general help
        result = cp.process_command("help")
        assert result.success
        assert len(result.message) > 0

        # Test specific command help
        help_commands = [
            "help move",
            "help turn",
            "help nav",
            "help calibrate"
        ]

        for cmd in help_commands:
            result = cp.process_command(cmd)
            assert result.success, f"Help command '{cmd}' failed"

    def test_topic_listing_integration(self, command_processor):
        """Test ROS topic listing through command system."""
        cp = command_processor

        result = cp.process_command("show topics")
        assert result.success, f"Topic listing failed: {result.message}"
        assert "Active ROS topics" in result.message