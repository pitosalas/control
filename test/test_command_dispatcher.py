#!/usr/bin/env python3
import pytest
from unittest.mock import Mock, MagicMock
from control.commands.command_dispatcher import CommandDispatcher
from control.commands.parameter_def import ParameterDef
from control.commands.command_def import CommandDef
from control.commands.robot_controller import CommandResponse


class TestCommandDispatcher:
    """Unit tests for the command dispatcher."""

    @pytest.fixture
    def mock_robot_controller(self):
        """Create a mock robot controller."""
        mock = Mock()
        mock.move_distance.return_value = CommandResponse(True, "Moved 1.0 meters")
        mock.get_robot_status.return_value = CommandResponse(True, "Status retrieved", {"status": {"linear": 0.3}})
        return mock

    @pytest.fixture
    def dispatcher(self, mock_robot_controller):
        """Create a command dispatcher with mocked robot controller."""
        return CommandDispatcher(mock_robot_controller)

    def test_command_registry_creation(self, dispatcher):
        """Test that command registry is properly created."""
        commands = dispatcher.list_commands()

        assert len(commands) > 0
        assert "move.distance" in commands
        assert "launch.start" in commands

    def test_execute_simple_command(self, dispatcher, mock_robot_controller):
        """Test executing a command with parameters."""
        result = dispatcher.execute("move.distance", {"distance": 1.5})

        assert result.success is True
        mock_robot_controller.move_distance.assert_called_once_with(distance=1.5)

    def test_execute_command_with_optional_params(self, dispatcher, mock_robot_controller):
        """Test executing command with optional parameters."""
        mock_robot_controller.launch_start.return_value = CommandResponse(True, "Nav started")

        result = dispatcher.execute("launch.start", {"launch_type": "nav"})

        assert result.success is True
        mock_robot_controller.launch_start.assert_called_once_with(launch_type="nav", use_sim_time=False)

    def test_missing_required_parameter(self, dispatcher):
        """Test error handling for missing required parameters."""
        result = dispatcher.execute("move.distance", {})

        assert result.success is False
        assert "Missing required parameter: distance" in result.message

    def test_invalid_command_name(self, dispatcher):
        """Test error handling for invalid command names."""
        result = dispatcher.execute("invalid.command", {})

        assert result.success is False
        assert "Unknown command: invalid.command" in result.message

    def test_parameter_type_conversion(self, dispatcher, mock_robot_controller):
        """Test automatic parameter type conversion."""
        result = dispatcher.execute("move.distance", {"distance": "1.5"})

        assert result.success is True
        mock_robot_controller.move_distance.assert_called_once_with(distance=1.5)

    def test_boolean_parameter_conversion(self, dispatcher, mock_robot_controller):
        """Test boolean parameter conversion from strings."""
        mock_robot_controller.launch_start.return_value = CommandResponse(True, "Nav started")

        result = dispatcher.execute("launch.start", {"launch_type": "nav", "use_sim_time": "true"})

        assert result.success is True
        mock_robot_controller.launch_start.assert_called_once_with(launch_type="nav", use_sim_time=True)

    def test_list_commands_by_group(self, dispatcher):
        """Test listing commands filtered by group."""
        movement_commands = dispatcher.list_commands(group="movement")

        assert "move.distance" in movement_commands
        assert "turn.time" in movement_commands
        assert "nav.start" not in movement_commands

    def test_get_command_info(self, dispatcher):
        """Test retrieving command information."""
        info = dispatcher.get_command_info("move.distance")

        assert info is not None
        assert info.method_name == "move_distance"
        assert len(info.parameters) == 1
        assert info.parameters[0].name == "distance"
        assert info.parameters[0].param_type == float

    def test_get_groups(self, dispatcher):
        """Test getting list of command groups."""
        groups = dispatcher.get_groups()

        assert "movement" in groups
        assert "launch" in groups
        assert "control" in groups

    def test_get_help_for_specific_command(self, dispatcher):
        """Test getting help for a specific command."""
        help_text = dispatcher.get_help("move.distance")

        assert "move.distance:" in help_text
        assert "distance" in help_text
        assert "float" in help_text

    def test_get_general_help(self, dispatcher):
        """Test getting general help for all commands."""
        help_text = dispatcher.get_help()

        assert "Available Commands:" in help_text
        assert "MOVEMENT COMMANDS:" in help_text
        assert "move.distance" in help_text