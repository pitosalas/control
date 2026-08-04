#!/usr/bin/env python3
"""Tests verifying F02 cleanup changes."""
import inspect


class TestT03NoRclpyInitInBaseApi:
    """T03: BaseApi.__init__ must not call rclpy.init()."""

    def test_base_api_init_does_not_call_rclpy_init(self):
        import dome_control.ros2_api.base_api as base_api_module
        src = inspect.getsource(base_api_module.BaseApi.__init__)
        assert "rclpy.init" not in src

    def test_base_api_source_no_rclpy_init(self):
        src = inspect.getsource(__import__("dome_control.ros2_api.base_api", fromlist=["BaseApi"]).BaseApi)
        assert "rclpy.init" not in src
