#!/usr/bin/env python3
import pytest
import tempfile
import os
from unittest.mock import patch, MagicMock
from control.config_manager import ConfigManager
from control.command_processor import CommandProcessor

@pytest.fixture
def temp_config_dir():
    """Create temporary directory for config files during tests."""
    with tempfile.TemporaryDirectory() as temp_dir:
        yield temp_dir

@pytest.fixture
def mock_rclpy():
    """Mock rclpy to avoid ROS2 initialization in tests."""
    with patch('rclpy.init'), \
         patch('rclpy.ok', return_value=True), \
         patch('rclpy.spin_once'), \
         patch('rclpy.shutdown'):
        yield

@pytest.fixture
def mock_subprocess():
    """Mock subprocess calls to avoid actual process execution."""
    mock_process = MagicMock()
    mock_process.pid = 12345
    mock_process.poll.return_value = None
    mock_process.wait.return_value = 0
    mock_process.stdout.readline.return_value = ""

    with patch('subprocess.Popen', return_value=mock_process):
        yield mock_process

@pytest.fixture
def test_config_manager(temp_config_dir):
    """Create ConfigManager with temporary config file."""
    config_file = os.path.join(temp_config_dir, "test_config.json")
    with patch.object(ConfigManager, '_get_config_file_path', return_value=config_file):
        config_manager = ConfigManager()
        # Set default values for testing
        config_manager.set_variable('linear_speed', 0.3)
        config_manager.set_variable('angular_speed', 0.4)
        yield config_manager

@pytest.fixture
def command_processor(mock_rclpy, mock_subprocess, test_config_manager):
    """Create CommandProcessor with mocked dependencies."""
    with patch('control.robot_controller.ConfigManager', return_value=test_config_manager):
        cp = CommandProcessor()
        cp.config_manager = test_config_manager
        yield cp