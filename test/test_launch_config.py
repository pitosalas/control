#!/usr/bin/env python3
import unittest
from control.ros2_api.process_api import LAUNCH_CONFIGS, LaunchConfig, ProcessApi
from control.commands.config_manager import ConfigManager


class TestLaunchConfig(unittest.TestCase):
    """Test cases for static launch configuration table"""

    def setUp(self):
        """Set up test fixtures"""
        self.config_manager = ConfigManager()
        self.process_api = ProcessApi(self.config_manager)

    def test_launch_configs_exist(self):
        """Test that all expected launch configurations are defined"""
        expected_types = ["nav", "slam", "map_server"]

        for launch_type in expected_types:
            with self.subTest(launch_type=launch_type):
                self.assertIn(launch_type, LAUNCH_CONFIGS)
                config = LAUNCH_CONFIGS[launch_type]
                self.assertIsInstance(config, LaunchConfig)

    def test_launch_config_structure(self):
        """Test that each launch config has required fields"""
        for launch_type, config in LAUNCH_CONFIGS.items():
            with self.subTest(launch_type=launch_type):
                self.assertEqual(config.launch_type, launch_type)
                self.assertIsInstance(config.command_template, str)
                self.assertIsInstance(config.description, str)
                self.assertIsInstance(config.default_params, dict)

                # Command template should contain {params} placeholder
                self.assertIn("{params}", config.command_template)

                # Description should not be empty
                self.assertGreater(len(config.description), 0)

    def test_nav_launch_config(self):
        """Test navigation launch configuration"""
        config = LAUNCH_CONFIGS["nav"]

        self.assertEqual(config.launch_type, "nav")
        self.assertIn("nav2_bringup", config.command_template)
        self.assertIn("navigation_launch.py", config.command_template)
        self.assertIn("navigation", config.description.lower())
        self.assertIn("use_sim_time", config.default_params)
        self.assertEqual(config.default_params["use_sim_time"], "false")

    def test_slam_launch_config(self):
        """Test SLAM launch configuration"""
        config = LAUNCH_CONFIGS["slam"]

        self.assertEqual(config.launch_type, "slam")
        self.assertIn("slam_toolbox", config.command_template)
        self.assertIn("online_async_launch.py", config.command_template)
        self.assertIn("slam", config.description.lower())
        self.assertIn("use_sim_time", config.default_params)
        self.assertEqual(config.default_params["use_sim_time"], "false")

    def test_map_server_launch_config(self):
        """Test map server launch configuration"""
        config = LAUNCH_CONFIGS["map_server"]

        self.assertEqual(config.launch_type, "map_server")
        self.assertIn("nav2_map_server", config.command_template)
        self.assertIn("map_server", config.command_template)
        self.assertIn("map", config.description.lower())
        self.assertEqual(len(config.default_params), 1)  # Has use_sim_time default param
        self.assertEqual(config.default_params["use_sim_time"], "false")

    def test_get_available_launch_types(self):
        """Test getting list of available launch types"""
        types = self.process_api.get_available_launch_types()

        self.assertIsInstance(types, list)
        self.assertEqual(set(types), {"nav", "slam", "map_server"})

    def test_get_launch_config_valid(self):
        """Test getting launch config for valid types"""
        for launch_type in ["nav", "slam", "map_server"]:
            with self.subTest(launch_type=launch_type):
                config = self.process_api.get_launch_config(launch_type)

                self.assertIsNotNone(config)
                self.assertIsInstance(config, LaunchConfig)
                self.assertEqual(config.launch_type, launch_type)

    def test_get_launch_config_invalid(self):
        """Test getting launch config for invalid type"""
        config = self.process_api.get_launch_config("invalid_type")
        self.assertIsNone(config)

    def test_format_launch_params_empty(self):
        """Test parameter formatting with empty params"""
        result = self.process_api._format_launch_params({})
        self.assertEqual(result, "")

    def test_format_launch_params_single(self):
        """Test parameter formatting with single parameter"""
        params = {"use_sim_time": "true"}
        result = self.process_api._format_launch_params(params)
        self.assertEqual(result, "use_sim_time:=true")

    def test_format_launch_params_multiple(self):
        """Test parameter formatting with multiple parameters"""
        params = {"use_sim_time": "true", "map": "/path/to/map.yaml"}
        result = self.process_api._format_launch_params(params)

        # Order may vary, so check both possibilities
        expected1 = "use_sim_time:=true map:=/path/to/map.yaml"
        expected2 = "map:=/path/to/map.yaml use_sim_time:=true"

        self.assertIn(result, [expected1, expected2])

    def test_command_template_formatting(self):
        """Test that command templates format correctly with parameters"""
        test_cases = [
            ("nav", {"use_sim_time": "true"}, "ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true"),
            ("slam", {}, "ros2 launch slam_toolbox online_async_launch.py "),
            ("map_server", {"yaml_filename": "/path/map.yaml"}, "ros2 launch nav2_map_server map_server.launch.py yaml_filename:=/path/map.yaml")
        ]

        for launch_type, params, expected_command in test_cases:
            with self.subTest(launch_type=launch_type):
                config = self.process_api.get_launch_config(launch_type)
                params_str = self.process_api._format_launch_params(params)
                actual_command = config.command_template.format(params=params_str)

                self.assertEqual(actual_command.strip(), expected_command.strip())


class TestLaunchTypeMethods(unittest.TestCase):
    """Test cases for launch-type-based methods"""

    def setUp(self):
        """Set up test fixtures"""
        self.config_manager = ConfigManager()
        self.process_api = ProcessApi(self.config_manager)

    def test_launch_by_type_valid_nav(self):
        """Test launching navigation with custom parameters"""
        # Mock the launch_command method to avoid actually launching
        original_launch_command = self.process_api.launch_command
        self.process_api.launch_command = lambda cmd: "test-process-id"

        try:
            process_id = self.process_api.launch_by_type("nav", use_sim_time=True)
            self.assertEqual(process_id, "test-process-id")
        finally:
            self.process_api.launch_command = original_launch_command

    def test_launch_by_type_invalid_type(self):
        """Test launching with invalid launch type"""
        with self.assertRaises(ValueError) as context:
            self.process_api.launch_by_type("invalid_type")

        self.assertIn("Unknown launch type: invalid_type", str(context.exception))

    def test_launch_by_type_param_merging(self):
        """Test that default and custom parameters are merged correctly"""
        # Mock launch_command to capture the command that would be executed
        captured_command = None

        def mock_launch_command(command):
            nonlocal captured_command
            captured_command = command
            return "test-id"

        original_launch_command = self.process_api.launch_command
        self.process_api.launch_command = mock_launch_command

        try:
            self.process_api.launch_by_type("nav", use_sim_time=True, custom_param="value")

            # Check that both default and custom params are in the command
            self.assertIn("use_sim_time:=True", captured_command)
            self.assertIn("custom_param:=value", captured_command)

        finally:
            self.process_api.launch_command = original_launch_command

    def test_kill_by_type_with_process_id(self):
        """Test killing process by type with valid process ID"""
        # Mock kill_process method
        self.process_api.kill_process = lambda pid: True

        result = self.process_api.kill_by_type("nav", "test-process-id")
        self.assertTrue(result)

    def test_kill_by_type_no_process_id(self):
        """Test killing process by type with no process ID"""
        result = self.process_api.kill_by_type("nav", "")
        self.assertFalse(result)

        result = self.process_api.kill_by_type("nav", None)
        self.assertFalse(result)

    def test_get_launch_status_all(self):
        """Test getting status of all launch processes"""
        # Mock get_running_processes
        mock_processes = {"proc1": {"command": "test", "running": True}}
        self.process_api.get_running_processes = lambda: mock_processes

        status = self.process_api.get_launch_status()
        self.assertEqual(status, mock_processes)

    def test_get_launch_status_specific(self):
        """Test getting status of specific launch type"""
        status = self.process_api.get_launch_status("nav")

        self.assertIsInstance(status, dict)
        self.assertEqual(status["launch_type"], "nav")
        self.assertIn("status", status)


if __name__ == "__main__":
    unittest.main()