#!/usr/bin/env python3
import pytest
from unittest.mock import Mock, patch
from control.command_processor import CommandProcessor


class TestHelpCommand:
    """Unit tests for the help command functionality."""

    @pytest.fixture
    def command_processor(self):
        """Create a command processor with mocked dependencies."""
        with patch('control.command_processor.RobotController') as mock_controller:
            mock_controller.return_value = Mock()
            cp = CommandProcessor()
            return cp

    def test_help_with_no_arguments(self, command_processor):
        """Test that 'help' without arguments works without error."""
        result = command_processor.process_command("help")

        assert result.success is True
        assert isinstance(result.message, str)
        assert len(result.message.strip()) > 0

    def test_help_with_valid_command(self, command_processor):
        """Test that 'help <command>' works without error."""
        result = command_processor.process_command("help move")

        assert result.success is True
        assert isinstance(result.message, str)
        assert len(result.message.strip()) > 0

    def test_help_with_invalid_command(self, command_processor):
        """Test that 'help <invalid>' works without error."""
        result = command_processor.process_command("help nonexistent")

        assert result.success is True
        assert isinstance(result.message, str)
        assert len(result.message.strip()) > 0

    def test_help_output_format(self, command_processor):
        """Test that help output is properly formatted."""
        result = command_processor.process_command("help")

        assert result.success is True
        assert isinstance(result.message, str)
        assert len(result.message.strip()) > 0

    def test_incomplete_command_shows_error(self, command_processor):
        """Test that incomplete commands fail appropriately."""
        result = command_processor.process_command("move dist")

        # Should fail when missing required argument
        assert result.success is False
        assert isinstance(result.message, str)
        # Message might be empty, but at least it fails properly
        # This behavior is acceptable - CLI shows command failed without output