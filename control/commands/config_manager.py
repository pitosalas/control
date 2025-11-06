#!/usr/bin/env python3
"""
ConfigManager - Manages configuration for robot control system
Author: Pito Salas and Claude Code
Open Source Under MIT license
"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml


class ConfigManager:
    def __init__(self, config_file: str):
        self.config_file = Path(config_file)
        self.control_dir = self.config_file.parent
        self.variables: dict[str, object] = {}
        self.load_config()
        self.ensure_subdirs()
        self._detect_test_environment()

    def _init_control_dir(self, config_file: str) -> Path:
        return Path(config_file).parent

    def _init_config_file(self, config_file: str) -> Path:
        return Path(config_file)

    def load_config(self):
        with self.config_file.open() as f:
            self.variables = yaml.safe_load(f) or {}

    def save_config(self):
        with self.config_file.open("w") as f:
            yaml.dump(self.variables, f, default_flow_style=False, sort_keys=False)

    def set_variable(self, name: str, value: object):
        # If value is already a native type (not string), use it directly
        if not isinstance(value, str):
            self.variables[name] = value
        # Try to convert string to appropriate type
        elif value.lower() in ["true", "false"]:
            self.variables[name] = value.lower() == "true"
        elif self._is_number(value):
            self.variables[name] = self._convert_number(value)
        else:
            # Keep as string
            self.variables[name] = value

        # Save configuration after setting variable
        self.save_config()

    def _convert_number(self, value: str) -> int | float:
        if "." in value:
            return float(value)
        return int(value)

    def _is_number(self, value: str) -> bool:
        try:
            float(value)
            return True
        except ValueError:
            return False

    def get_variable(self, name: str):
        return self.variables.get(name)

    def get_all_variables(self) -> dict[str, object]:
        return self.variables.copy()

    def delete_variable(self, name: str):
        del self.variables[name]

    def variable_exists(self, name: str) -> bool:
        return name in self.variables

    def get_control_dir(self) -> Path:
        return self.control_dir

    def ensure_subdirs(self):
        maps_dir = self.get_maps_dir()
        maps_dir.mkdir(parents=True, exist_ok=True)

        log_dir = self.resolve_path(self.get_variable("log_dir") or "logs")
        log_dir.mkdir(parents=True, exist_ok=True)

    def get_maps_dir(self) -> Path:
        return self.control_dir / "maps"

    def get_launch_templates(self) -> dict:
        """Get launch templates from config file"""
        return self.variables.get("launch_templates", {})

    def resolve_path(self, path_str: str) -> Path:
        """
        Resolve a path relative to ~/.control/ directory.
        If path is absolute or starts with ~, use as-is.
        Otherwise, resolve relative to ~/.control/ directory.
        """
        path = Path(path_str).expanduser()
        if path.is_absolute():
            return path
        # Resolve relative to ~/.control/ directory
        return (self.control_dir / path).resolve()

    def _detect_test_environment(self):
        if "pytest" in sys.modules or "unittest" in sys.modules:
            self.variables["dry_run"] = True

    def is_dry_run(self) -> bool:
        return self.variables.get("dry_run", False)
