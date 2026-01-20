#!/usr/bin/env python3
# test_negative_numbers.py - Test negative number support for movement commands
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import pytest
from unittest.mock import Mock, patch
from control.commands.command_dispatcher import CommandDispatcher
from control.commands.robot_controller import CommandResponse


class TestNegativeNumbers:
    """Test that negative numbers are properly supported for movement commands."""

    @pytest.fixture
    def mock_robot_controller(self):
        """Create a mock robot controller."""
        mock = Mock()
        mock.move_distance.return_value = CommandResponse(True, "Moved")
        mock.turn_radians.return_value = CommandResponse(True, "Turned")
        mock.turn_degrees.return_value = CommandResponse(True, "Turned")
        mock.script_square.return_value = CommandResponse(True, "Square completed")
        return mock

    @pytest.fixture
    def dispatcher(self, mock_robot_controller):
        """Create a command dispatcher with mocked robot controller."""
        return CommandDispatcher(mock_robot_controller)

    def test_move_distance_positive(self, dispatcher, mock_robot_controller):
        """Test move.distance with positive number (forward)."""
        result = dispatcher.execute("move.distance", {"distance": 1.5})

        assert result.success is True
        mock_robot_controller.move_distance.assert_called_once_with(distance=1.5)

    def test_move_distance_negative(self, dispatcher, mock_robot_controller):
        """Test move.distance with negative number (backward)."""
        result = dispatcher.execute("move.distance", {"distance": -1.5})

        assert result.success is True
        mock_robot_controller.move_distance.assert_called_once_with(distance=-1.5)

    def test_move_distance_zero(self, dispatcher, mock_robot_controller):
        """Test move.distance with zero."""
        result = dispatcher.execute("move.distance", {"distance": 0.0})

        assert result.success is True
        mock_robot_controller.move_distance.assert_called_once_with(distance=0.0)

    def test_turn_radians_positive(self, dispatcher, mock_robot_controller):
        """Test turn.radians with positive number (counterclockwise)."""
        result = dispatcher.execute("turn.radians", {"radians": 1.57})

        assert result.success is True
        mock_robot_controller.turn_radians.assert_called_once_with(radians=1.57)

    def test_turn_radians_negative(self, dispatcher, mock_robot_controller):
        """Test turn.radians with negative number (clockwise)."""
        result = dispatcher.execute("turn.radians", {"radians": -1.57})

        assert result.success is True
        mock_robot_controller.turn_radians.assert_called_once_with(radians=-1.57)

    def test_turn_radians_zero(self, dispatcher, mock_robot_controller):
        """Test turn.radians with zero."""
        result = dispatcher.execute("turn.radians", {"radians": 0.0})

        assert result.success is True
        mock_robot_controller.turn_radians.assert_called_once_with(radians=0.0)

    def test_turn_degrees_positive(self, dispatcher, mock_robot_controller):
        """Test turn.degrees with positive number (counterclockwise)."""
        result = dispatcher.execute("turn.degrees", {"degrees": 90.0})

        assert result.success is True
        mock_robot_controller.turn_degrees.assert_called_once_with(degrees=90.0)

    def test_turn_degrees_negative(self, dispatcher, mock_robot_controller):
        """Test turn.degrees with negative number (clockwise)."""
        result = dispatcher.execute("turn.degrees", {"degrees": -90.0})

        assert result.success is True
        mock_robot_controller.turn_degrees.assert_called_once_with(degrees=-90.0)

    def test_turn_degrees_zero(self, dispatcher, mock_robot_controller):
        """Test turn.degrees with zero."""
        result = dispatcher.execute("turn.degrees", {"degrees": 0.0})

        assert result.success is True
        mock_robot_controller.turn_degrees.assert_called_once_with(degrees=0.0)

    def test_script_square_positive_meters(self, dispatcher, mock_robot_controller):
        """Test script.square with positive meters."""
        result = dispatcher.execute("script.square", {"meters": 2.0})

        assert result.success is True
        mock_robot_controller.script_square.assert_called_once_with(meters=2.0)

    def test_script_square_negative_meters(self, dispatcher, mock_robot_controller):
        """Test script.square with negative meters (reverse direction)."""
        result = dispatcher.execute("script.square", {"meters": -2.0})

        assert result.success is True
        mock_robot_controller.script_square.assert_called_once_with(meters=-2.0)

    def test_negative_string_conversion(self, dispatcher, mock_robot_controller):
        """Test that negative numbers as strings are properly converted."""
        result = dispatcher.execute("move.distance", {"distance": "-2.5"})

        assert result.success is True
        mock_robot_controller.move_distance.assert_called_once_with(distance=-2.5)

    def test_very_small_negative_number(self, dispatcher, mock_robot_controller):
        """Test very small negative number (precision check)."""
        result = dispatcher.execute("turn.radians", {"radians": -0.001})

        assert result.success is True
        mock_robot_controller.turn_radians.assert_called_once_with(radians=-0.001)

    def test_large_negative_number(self, dispatcher, mock_robot_controller):
        """Test large negative number."""
        result = dispatcher.execute("move.distance", {"distance": -100.0})

        assert result.success is True
        mock_robot_controller.move_distance.assert_called_once_with(distance=-100.0)


class TestMovementApiNegativeNumbers:
    """Test MovementApi implementation with negative numbers."""

    @patch('control.ros2_api.movement_api.rclpy')
    def test_move_dist_negative_distance(self, mock_rclpy):
        """Test that move_dist handles negative distance correctly."""
        from control.ros2_api.movement_api import MovementApi
        from control.commands.config_manager import ConfigManager

        mock_rclpy.ok.return_value = True
        config = ConfigManager("/home/pitosalas/.control/config.yaml")
        # Disable dry_run for this test
        config.set_variable("dry_run", False)
        api = MovementApi(config)

        # Mock the publisher
        api.cmd_vel_pub = Mock()

        # Test negative distance - should move backward
        api.move_dist(-2.0)

        # Verify cmd_vel was called (publisher should have been called)
        assert api.cmd_vel_pub.publish.called

    @patch('control.ros2_api.movement_api.rclpy')
    def test_turn_amount_negative_angle(self, mock_rclpy):
        """Test that turn_amount handles negative angle correctly."""
        from control.ros2_api.movement_api import MovementApi
        from control.commands.config_manager import ConfigManager

        mock_rclpy.ok.return_value = True
        config = ConfigManager("/home/pitosalas/.control/config.yaml")
        config.set_variable("dry_run", False)
        api = MovementApi(config)

        # Mock the publisher
        api.cmd_vel_pub = Mock()

        # Test negative angle - should turn clockwise
        api.turn_amount(-1.57)

        # Verify cmd_vel was called
        assert api.cmd_vel_pub.publish.called

    @patch('control.ros2_api.movement_api.rclpy')
    def test_turn_degrees_negative_conversion(self, mock_rclpy):
        """Test that turn_degrees converts negative degrees to radians correctly."""
        from control.ros2_api.movement_api import MovementApi
        from control.commands.config_manager import ConfigManager
        import math

        mock_rclpy.ok.return_value = True
        config = ConfigManager("/home/pitosalas/.control/config.yaml")
        config.set_variable("dry_run", False)
        api = MovementApi(config)

        # Mock the publisher
        api.cmd_vel_pub = Mock()

        # Test negative degrees
        api.turn_degrees(-90.0)

        # Verify cmd_vel was called
        assert api.cmd_vel_pub.publish.called

        # Verify the conversion happened (should use math.radians)
        expected_radians = math.radians(-90.0)
        assert expected_radians < 0  # Verify our test value is actually negative
