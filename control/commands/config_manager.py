#!/usr/bin/env python3
"""
ConfigManager - Manages configuration for robot control system
Author: Pito Salas and Claude Code
Open Source Under MIT license
"""

import sys
from pathlib import Path

import yaml


class ConfigManager:
    def __init__(self, config_file: str):
        self.config_file = Path(config_file)
        self.control_dir = self.config_file.parent
        self.variables: dict[str, object] = {}

    @classmethod
    def create(cls, config_file: str) -> "ConfigManager":
        manager = cls(config_file)
        manager.load_config()
        manager.ensure_subdirs()
        manager.detect_test_environment()
        return manager

    def load_config(self):
        with self.config_file.open() as f:
            self.variables = yaml.safe_load(f) or {}

    def save_config(self):
        with self.config_file.open("w") as f:
            yaml.dump(self.variables, f, default_flow_style=False, sort_keys=False)

    def set_variable(self, name: str, value: object):
        if not isinstance(value, str):
            self.variables[name] = value
        elif value.lower() in ["true", "false"]:
            self.variables[name] = value.lower() == "true"
        elif self.is_number(value):
            self.variables[name] = self.convert_number(value)
        else:
            self.variables[name] = value
        self.save_config()

    def convert_number(self, value: str) -> int | float:
        if "." in value:
            return float(value)
        return int(value)

    def is_number(self, value: str) -> bool:
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
        return self.variables.get("launch_templates", {})

    def resolve_path(self, path_str: str) -> Path:
        path = Path(path_str).expanduser()
        if path.is_absolute():
            return path
        return (self.control_dir / path).resolve()

    def detect_test_environment(self):
        if "pytest" in sys.modules or "unittest" in sys.modules:
            self.variables["dry_run"] = True

    def is_dry_run(self) -> bool:
        return self.variables.get("dry_run", False)
