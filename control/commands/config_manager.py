#!/usr/bin/env python3

import yaml
import os
from pathlib import Path
from typing import Dict, Any

class ConfigManager:
    DEFAULT_CONFIG = {
        "linear_speed": 0.3,
        "angular_speed": 0.4,
        "stress_test_rotation_speed": 0.2,
        "log_dir": "logs",
        "maps_dir": "maps",
        "dry_run": False
    }

    def __init__(self, config_file: str = None):
        if config_file is None:
            self.control_dir = Path.home() / ".control"
            self.control_dir.mkdir(parents=True, exist_ok=True)
            self.config_file = self.control_dir / "config.yaml"
        else:
            self.config_file = Path(config_file)
            self.control_dir = self.config_file.parent

        self.variables: Dict[str, Any] = {}
        self.load_config()
        self.ensure_subdirs()
        self._detect_test_environment()

    def load_config(self):
        try:
            with open(self.config_file, 'r') as f:
                loaded_vars = yaml.safe_load(f) or {}
                # Start with defaults, then update with loaded values
                self.variables = self.DEFAULT_CONFIG.copy()
                self.variables.update(loaded_vars)
                # Save to update config file with any new default values
                self.save_config()
        except FileNotFoundError:
            self.variables = self.DEFAULT_CONFIG.copy()
            self.save_config()

    def save_config(self):
        with open(self.config_file, 'w') as f:
            yaml.dump(self.variables, f, default_flow_style=False, sort_keys=False)

    def set_variable(self, name: str, value):
        """Set a variable value, attempting type conversion"""
        # If value is already a native type (not string), use it directly
        if not isinstance(value, str):
            self.variables[name] = value
        else:
            # Try to convert string to appropriate type
            if value.lower() in ['true', 'false']:
                self.variables[name] = value.lower() == 'true'
            elif self._is_number(value):
                # Try float first, then int
                if '.' in value:
                    self.variables[name] = float(value)
                else:
                    self.variables[name] = int(value)
            else:
                # Keep as string
                self.variables[name] = value

        # Save configuration after setting variable
        self.save_config()

    def _is_number(self, value: str) -> bool:
        """Check if string represents a valid number (int or float)"""
        try:
            float(value)
            return True
        except ValueError:
            return False

    def get_variable(self, name: str) -> Any:
        """Get a variable value"""
        return self.variables.get(name)

    def get_all_variables(self) -> Dict[str, Any]:
        """Get all variables"""
        return self.variables.copy()

    def delete_variable(self, name: str):
        """Delete a variable"""
        del self.variables[name]

    def variable_exists(self, name: str) -> bool:
        """Check if variable exists"""
        return name in self.variables

    def get_control_dir(self) -> Path:
        """Get the ~/.control directory for user data"""
        return self.control_dir

    def ensure_subdirs(self):
        maps_dir = self.get_maps_dir()
        maps_dir.mkdir(parents=True, exist_ok=True)

        log_dir = self.resolve_path(self.get_variable("log_dir") or "logs")
        log_dir.mkdir(parents=True, exist_ok=True)

    def get_maps_dir(self) -> Path:
        return self.resolve_path(self.get_variable("maps_dir") or "maps")

    def resolve_path(self, path_str: str) -> Path:
        """
        Resolve a path relative to ~/.control/ directory.
        If path is absolute or starts with ~, use as-is.
        Otherwise, resolve relative to ~/.control/ directory.
        """
        path = Path(path_str).expanduser()
        if path.is_absolute():
            return path
        else:
            # Resolve relative to ~/.control/ directory
            return (self.control_dir / path).resolve()

    def _detect_test_environment(self):
        """Automatically enable dry_run mode when running in test environment."""
        import sys
        if 'pytest' in sys.modules or 'unittest' in sys.modules:
            self.variables['dry_run'] = True

    def is_dry_run(self) -> bool:
        """Check if dry_run mode is enabled."""
        return self.variables.get('dry_run', False)