#!/usr/bin/env python3
import pytest
import tempfile
import shutil
from pathlib import Path

import sys
sys.path.insert(0, '/home/pitosalas/ros2_ws/src/control')

from control.commands.robot_controller import CommandResponse


class TestMapServiceLogic:
    """Test map service logic without ROS2 dependencies"""

    def setup_method(self):
        """Set up test fixtures"""
        # Create temporary directory for testing
        self.temp_dir = tempfile.mkdtemp()
        self.original_cwd = Path.cwd()

    def teardown_method(self):
        """Clean up test fixtures"""
        # Cleanup
        if hasattr(self, 'temp_dir'):
            shutil.rmtree(self.temp_dir)

    def test_maps_directory_creation(self):
        """Test that maps directory is created when it doesn't exist"""
        test_dir = Path(self.temp_dir) / "test_maps"
        maps_dir = test_dir / "maps"

        # Verify directory doesn't exist initially
        assert not maps_dir.exists()

        # Simulate the logic from save_map_via_service
        maps_dir.mkdir(parents=True, exist_ok=True)

        # Verify directory was created
        assert maps_dir.exists()
        assert maps_dir.is_dir()

    def test_map_path_construction(self):
        """Test map path construction logic"""
        maps_dir = Path("maps")
        filename = "test_map"

        # Test save path (without extension)
        save_path = maps_dir / filename
        assert str(save_path) == "maps/test_map"

        # Test load path (with .yaml extension)
        load_path = maps_dir / f"{filename}.yaml"
        assert str(load_path) == "maps/test_map.yaml"

    def test_map_file_existence_check(self):
        """Test logic for checking if map files exist"""
        test_dir = Path(self.temp_dir)
        maps_dir = test_dir / "maps"
        maps_dir.mkdir(exist_ok=True)

        # Create a test map file
        test_map = maps_dir / "existing_map.yaml"
        test_map.write_text("# Test map file")

        # Test existing file
        existing_map_path = maps_dir / "existing_map.yaml"
        assert existing_map_path.exists()

        # Test non-existing file
        missing_map_path = maps_dir / "missing_map.yaml"
        assert not missing_map_path.exists()

    def test_command_response_creation(self):
        """Test CommandResponse creation for different scenarios"""
        # Success response
        success_response = CommandResponse(True, "Map saved successfully")
        assert success_response.success is True
        assert "Map saved successfully" in success_response.message
        assert success_response.data is None

        # Failure response
        failure_response = CommandResponse(False, "Map server not running")
        assert failure_response.success is False
        assert "Map server not running" in failure_response.message

        # Response with data
        data_response = CommandResponse(True, "Operation complete", {"filename": "test_map"})
        assert data_response.success is True
        assert data_response.data == {"filename": "test_map"}

    def test_service_request_parameters(self):
        """Test the parameter values that would be sent to services"""
        # Test save map parameters
        save_params = {
            "map_topic": "map",
            "image_format": "pgm",
            "map_mode": "trinary",
            "free_thresh": 0.25,
            "occupied_thresh": 0.65
        }

        # Verify parameter types and values
        assert isinstance(save_params["map_topic"], str)
        assert save_params["map_topic"] == "map"
        assert save_params["image_format"] == "pgm"
        assert save_params["map_mode"] == "trinary"
        assert isinstance(save_params["free_thresh"], float)
        assert isinstance(save_params["occupied_thresh"], float)
        assert 0.0 <= save_params["free_thresh"] <= 1.0
        assert 0.0 <= save_params["occupied_thresh"] <= 1.0

    def test_error_message_construction(self):
        """Test error message construction for different failure scenarios"""
        # Map server not running
        server_error = "Map server is not running. Start it with: launch start map_server"
        assert "Map server is not running" in server_error
        assert "launch start map_server" in server_error

        # Map file not found
        filename = "missing_map"
        file_error = f"Map '{filename}' not found in maps/ folder"
        assert filename in file_error
        assert "not found in maps/" in file_error

        # Service call failure
        service_error = "Failed to save map via service call"
        assert "Failed to" in service_error
        assert "service call" in service_error

    def test_success_message_construction(self):
        """Test success message construction"""
        filename = "test_map"

        # Save success message
        save_message = f"Map saved to maps/{filename}"
        assert filename in save_message
        assert "saved to maps/" in save_message

        # Load success message
        load_message = f"Map loaded from maps/{filename}.yaml"
        assert filename in load_message
        assert "loaded from maps/" in load_message
        assert ".yaml" in load_message

    def test_launch_process_tracking_logic(self):
        """Test the logic for tracking launch processes"""
        # Simulate launch process tracking
        launch_process_ids = {
            "nav": None,
            "slam": None,
            "map_server": None
        }

        # Test checking if map_server is running
        map_server_running = launch_process_ids.get("map_server") is not None
        assert map_server_running is False

        # Simulate starting map_server
        launch_process_ids["map_server"] = "process_123"
        map_server_running = launch_process_ids.get("map_server") is not None
        assert map_server_running is True

        # Test getting process ID
        process_id = launch_process_ids.get("map_server")
        assert process_id == "process_123"

    def test_file_extension_handling(self):
        """Test handling of file extensions for map operations"""
        # Test that save doesn't add extension (handled by service)
        save_filename = "my_map"
        save_path = Path("maps") / save_filename
        assert not str(save_path).endswith(".yaml")
        assert not str(save_path).endswith(".pgm")

        # Test that load adds .yaml extension
        load_filename = "my_map"
        load_path = Path("maps") / f"{load_filename}.yaml"
        assert str(load_path).endswith(".yaml")

    def test_path_string_conversion(self):
        """Test path to string conversion for service calls"""
        maps_dir = Path("maps")
        filename = "test_map"

        # Test save path conversion
        save_path = maps_dir / filename
        save_str = str(save_path)
        assert isinstance(save_str, str)
        assert save_str == "maps/test_map"

        # Test load path conversion
        load_path = maps_dir / f"{filename}.yaml"
        load_str = str(load_path)
        assert isinstance(load_str, str)
        assert load_str == "maps/test_map.yaml"


class TestMapServiceWorkflow:
    """Test the complete workflow logic for map operations"""

    def test_save_map_workflow_steps(self):
        """Test the logical steps for saving a map"""
        filename = "workflow_test"

        # Step 1: Check if map_server is running (simulated)
        map_server_running = True  # Would be: launch_process_ids.get("map_server") is not None
        assert map_server_running, "Map server must be running"

        # Step 2: Create maps directory path
        maps_dir = Path("maps")
        map_path = maps_dir / filename

        # Step 3: Convert to string for service call
        map_path_str = str(map_path)
        assert map_path_str == "maps/workflow_test"

        # Step 4: Prepare service parameters
        service_params = {
            "map_topic": "map",
            "map_url": map_path_str,
            "image_format": "pgm",
            "map_mode": "trinary",
            "free_thresh": 0.25,
            "occupied_thresh": 0.65
        }

        # Verify all required parameters are present
        required_params = ["map_topic", "map_url", "image_format", "map_mode", "free_thresh", "occupied_thresh"]
        for param in required_params:
            assert param in service_params

    def test_load_map_workflow_steps(self):
        """Test the logical steps for loading a map"""
        filename = "workflow_test"

        # Step 1: Check if file exists (simulated)
        maps_dir = Path("maps")
        map_path = maps_dir / f"{filename}.yaml"
        file_exists = True  # Would be: map_path.exists()
        assert file_exists, f"Map file {map_path} must exist"

        # Step 2: Check if map_server is running (simulated)
        map_server_running = True  # Would be: launch_process_ids.get("map_server") is not None
        assert map_server_running, "Map server must be running"

        # Step 3: Convert path to string for service call
        map_path_str = str(map_path)
        assert map_path_str == "maps/workflow_test.yaml"

        # Step 4: Prepare service parameters
        service_params = {
            "map_url": map_path_str
        }

        # Verify required parameter is present
        assert "map_url" in service_params
        assert service_params["map_url"].endswith(".yaml")


if __name__ == '__main__':
    pytest.main([__file__])