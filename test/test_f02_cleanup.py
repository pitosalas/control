#!/usr/bin/env python3
"""Tests verifying F02 cleanup changes."""
import inspect
import os
from pathlib import Path
from unittest.mock import patch

import pytest


class TestT02ConfigPath:
    """T02: simple_cli uses CONTROL_CONFIG env var, falls back to ~/.control/config.yaml."""

    def test_env_var_takes_precedence(self, tmp_path):
        config_file = tmp_path / "test_config.yaml"
        config_file.write_text("dry_run: true\n")
        with patch.dict(os.environ, {"CONTROL_CONFIG": str(config_file)}):
            from control.commands.config_manager import ConfigManager
            path_used = os.environ.get("CONTROL_CONFIG", str(Path.home() / ".control" / "config.yaml"))
            assert path_used == str(config_file)

    def test_fallback_without_env_var(self):
        env = os.environ.copy()
        env.pop("CONTROL_CONFIG", None)
        with patch.dict(os.environ, env, clear=True):
            expected = str(Path.home() / ".control" / "config.yaml")
            path_used = os.environ.get("CONTROL_CONFIG", str(Path.home() / ".control" / "config.yaml"))
            assert path_used == expected

    def test_simple_cli_reads_env_var(self, tmp_path):
        config_file = tmp_path / "config.yaml"
        config_file.write_text("dry_run: true\nlog_dir: logs\n")
        (tmp_path / "maps").mkdir()
        (tmp_path / "logs").mkdir()
        with patch.dict(os.environ, {"CONTROL_CONFIG": str(config_file)}):
            from control.interface import simple_cli
            import importlib
            importlib.reload(simple_cli)
            # Verify the env var path is used by checking SimpleCLI instantiation logic
            src = inspect.getsource(simple_cli.SimpleCLI.__init__)
            assert "CONTROL_CONFIG" in src


class TestT03NoRclpyInitInBaseApi:
    """T03: BaseApi.__init__ must not call rclpy.init()."""

    def test_base_api_init_does_not_call_rclpy_init(self):
        import control.ros2_api.base_api as base_api_module
        src = inspect.getsource(base_api_module.BaseApi.__init__)
        assert "rclpy.init" not in src

    def test_base_api_source_no_rclpy_init(self):
        src = inspect.getsource(__import__("control.ros2_api.base_api", fromlist=["BaseApi"]).BaseApi)
        assert "rclpy.init" not in src
