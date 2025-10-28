#!/usr/bin/env python3
# test_script_commands.py - Test script commands including square and stress_test
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import pytest
from unittest.mock import Mock, MagicMock, patch
from control.commands.command_dispatcher import CommandDispatcher
from control.commands.robot_controller import CommandResponse, RobotController
from control.commands.config_manager import ConfigManager


class TestScriptCommands:
    """Unit tests for script commands."""

    @pytest.fixture
    def mock_robot_controller(self):
        """Create a mock robot controller."""
        mock = Mock()
        mock.script_square.return_value = CommandResponse(True, "Completed square pattern with 1.0m sides")
        mock.script_stress_test.return_value = CommandResponse(True, "Stress test completed")
        return mock

    @pytest.fixture
    def dispatcher(self, mock_robot_controller):
        """Create a command dispatcher with mocked robot controller."""
        return CommandDispatcher(mock_robot_controller)

    def test_script_commands_in_registry(self, dispatcher):
        """Test that script commands are registered."""
        commands = dispatcher.list_commands()

        assert "script.square" in commands
        assert "script.stress_test" in commands

    def test_script_group_exists(self, dispatcher):
        """Test that script group exists."""
        groups = dispatcher.get_groups()
        assert "script" in groups

    def test_script_square_command_info(self, dispatcher):
        """Test script.square command definition."""
        info = dispatcher.get_command_info("script.square")

        assert info is not None
        assert info.method_name == "script_square"
        assert info.group == "script"
        assert len(info.parameters) == 1
        assert info.parameters[0].name == "meters"
        assert info.parameters[0].param_type == float

    def test_script_stress_test_command_info(self, dispatcher):
        """Test script.stress_test command definition."""
        info = dispatcher.get_command_info("script.stress_test")

        assert info is not None
        assert info.method_name == "script_stress_test"
        assert info.group == "script"
        assert len(info.parameters) == 0

    def test_execute_script_square(self, dispatcher, mock_robot_controller):
        """Test executing script.square command."""
        result = dispatcher.execute("script.square", {"meters": 2.0})

        assert result.success is True
        assert "square pattern" in result.message.lower()
        mock_robot_controller.script_square.assert_called_once_with(meters=2.0)

    def test_execute_script_square_with_negative_meters(self, dispatcher, mock_robot_controller):
        """Test executing script.square with negative value."""
        result = dispatcher.execute("script.square", {"meters": -1.0})

        assert result.success is True
        mock_robot_controller.script_square.assert_called_once_with(meters=-1.0)

    def test_execute_script_stress_test(self, dispatcher, mock_robot_controller):
        """Test executing script.stress_test command."""
        result = dispatcher.execute("script.stress_test", {})

        assert result.success is True
        mock_robot_controller.script_stress_test.assert_called_once()

    def test_script_square_missing_parameter(self, dispatcher):
        """Test script.square with missing required parameter."""
        result = dispatcher.execute("script.square", {})

        assert result.success is False
        assert "Missing required parameter: meters" in result.message

    def test_script_square_type_conversion(self, dispatcher, mock_robot_controller):
        """Test script.square with string parameter gets converted to float."""
        result = dispatcher.execute("script.square", {"meters": "1.5"})

        assert result.success is True
        mock_robot_controller.script_square.assert_called_once_with(meters=1.5)

    def test_script_commands_help(self, dispatcher):
        """Test getting help for script commands."""
        help_text = dispatcher.get_help("script.square")

        assert "script.square:" in help_text
        assert "meters" in help_text
        assert "float" in help_text

    @patch('control.commands.robot_controller.CalibrationApi')
    @patch('control.commands.robot_controller.MovementApi')
    @patch('control.commands.robot_controller.ProcessApi')
    def test_robot_controller_script_square(self, mock_process, mock_movement, mock_calibration):
        """Test RobotController.script_square method."""
        config = ConfigManager()
        controller = RobotController(config)

        mock_calibration_instance = controller.calibration
        mock_calibration_instance.run_square_pattern = Mock()

        result = controller.script_square(2.5)

        assert result.success is True
        assert "2.5m sides" in result.message
        mock_calibration_instance.run_square_pattern.assert_called_once_with(2.5)

    @patch('control.commands.robot_controller.CalibrationApi')
    @patch('control.commands.robot_controller.MovementApi')
    @patch('control.commands.robot_controller.ProcessApi')
    def test_robot_controller_script_stress_test(self, mock_process, mock_movement, mock_calibration):
        """Test RobotController.script_stress_test method."""
        config = ConfigManager()
        controller = RobotController(config)

        mock_calibration_instance = controller.calibration
        mock_calibration_instance.run_stress_test = Mock()

        result = controller.script_stress_test()

        assert result.success is True
        assert "Stress test completed" in result.message
        mock_calibration_instance.run_stress_test.assert_called_once()
