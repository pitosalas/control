#!/usr/bin/env python3
import pytest
from unittest.mock import Mock, MagicMock
from dome_control.commands.command_dispatcher import CommandDispatcher
from dome_control.commands.parameter_def import ParameterDef
from dome_control.commands.command_def import CommandDef
from dome_control.commands.robot_controller import CommandResponse


class TestCommandDispatcher:
    """Unit tests for the command dispatcher."""

    @pytest.fixture
    def mock_robot_controller(self):
        """Create a mock robot controller."""
        mock = Mock()
        mock.move_for_time.return_value = CommandResponse(True, "Moved for 1.0 seconds")
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
        assert "move.time" in commands
        assert "launch.start" in commands

    def test_execute_simple_command(self, dispatcher, mock_robot_controller):
        """Test executing a command with parameters."""
        result = dispatcher.execute("move.time", {"seconds": 1.5})

        assert result.success is True
        mock_robot_controller.move_for_time.assert_called_once_with(seconds=1.5)

    def test_execute_command_with_optional_params(self, dispatcher, mock_robot_controller):
        """Test executing command with optional parameters."""
        mock_robot_controller.launch_start.return_value = CommandResponse(True, "Nav started")

        result = dispatcher.execute("launch.start", {"launch_type": "nav"})

        assert result.success is True
        # Optional params with None default are not passed if not provided
        mock_robot_controller.launch_start.assert_called_once_with(launch_type="nav")

    def test_missing_required_parameter(self, dispatcher):
        """Test error handling for missing required parameters."""
        result = dispatcher.execute("move.time", {})

        assert result.success is False
        assert "Missing required parameter: seconds" in result.message

    def test_invalid_command_name(self, dispatcher):
        """Test error handling for invalid command names."""
        result = dispatcher.execute("invalid.command", {})

        assert result.success is False
        assert "Unknown command: invalid.command" in result.message

    def test_parameter_type_conversion(self, dispatcher, mock_robot_controller):
        """Test automatic parameter type conversion."""
        result = dispatcher.execute("move.time", {"seconds": "1.5"})

        assert result.success is True
        mock_robot_controller.move_for_time.assert_called_once_with(seconds=1.5)

    def test_boolean_parameter_conversion(self, dispatcher, mock_robot_controller):
        """Test boolean parameter conversion from strings."""
        mock_robot_controller.launch_start.return_value = CommandResponse(True, "Nav started")

        result = dispatcher.execute("launch.start", {"launch_type": "nav", "use_sim_time": "true"})

        assert result.success is True
        mock_robot_controller.launch_start.assert_called_once_with(launch_type="nav", use_sim_time=True)

    def test_list_commands_by_group(self, dispatcher):
        """Test listing commands filtered by group."""
        movement_commands = dispatcher.list_commands(group="movement")

        assert "move.time" in movement_commands
        assert "turn.time" in movement_commands
        assert "nav.start" not in movement_commands

    def test_get_command_info(self, dispatcher):
        """Test retrieving command information."""
        info = dispatcher.get_command_info("move.time")

        assert info is not None
        assert info.method_name == "move_for_time"
        assert len(info.parameters) == 1
        assert info.parameters[0].name == "seconds"
        assert info.parameters[0].param_type == float

    def test_move_distance_and_turn_degrees_removed(self, dispatcher):
        """move.distance/turn.degrees were dropped (F21 D1) — the directional
        pairs (move.forward/backward, turn.clockwise/counterclockwise) are
        the sole way to express movement/turning by amount now."""
        assert dispatcher.execute("move.distance", {"distance": 1.0}).success is False
        assert dispatcher.execute("turn.degrees", {"degrees": 90.0}).success is False
        commands = dispatcher.list_commands()
        assert "move.distance" not in commands
        assert "turn.degrees" not in commands

    def test_directional_and_radians_commands_still_work(self, dispatcher, mock_robot_controller):
        """Confirms F21 D1 kept the directional pairs and turn.radians."""
        mock_robot_controller.move_forward.return_value = CommandResponse(True, "ok")
        mock_robot_controller.move_backward.return_value = CommandResponse(True, "ok")
        mock_robot_controller.turn_clockwise.return_value = CommandResponse(True, "ok")
        mock_robot_controller.turn_counterclockwise.return_value = CommandResponse(True, "ok")
        mock_robot_controller.turn_radians.return_value = CommandResponse(True, "ok")

        assert dispatcher.execute("move.forward", {"meters": 1.0}).success is True
        assert dispatcher.execute("move.backward", {"meters": 1.0}).success is True
        assert dispatcher.execute("turn.clockwise", {"degrees": 90.0}).success is True
        assert dispatcher.execute("turn.counterclockwise", {"degrees": 90.0}).success is True
        assert dispatcher.execute("turn.radians", {"radians": 1.0}).success is True

    def test_get_groups(self, dispatcher):
        """Test getting list of command groups."""
        groups = dispatcher.get_groups()

        assert "movement" in groups
        assert "launch" in groups
        assert "control" in groups

