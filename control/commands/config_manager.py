#!/usr/bin/env python3

import json
import os
from pathlib import Path
from typing import Dict, Any

class ConfigManager:
    DEFAULT_CONFIG = {
        "linear_speed": 0.3,
        "angular_speed": 0.4,
        "log_dir": "logs"
    }

    def __init__(self, config_file: str = None):
        if config_file is None:
            # Use ~/.control/ for all user data (ROS2 best practice for user-writable data)
            self.control_dir = Path.home() / ".control"
            self.control_dir.mkdir(parents=True, exist_ok=True)
            self.config_file = self.control_dir / "control_config.json"
        else:
            self.config_file = Path(config_file)
            self.control_dir = self.config_file.parent

        self.variables: Dict[str, Any] = {}
        self.load_config()

    def load_config(self):
        """Load configuration from file"""
        try:
            with open(self.config_file, 'r') as f:
                self.variables = json.load(f)
        except FileNotFoundError:
            self.variables = self.DEFAULT_CONFIG.copy()
            self.save_config()

    def save_config(self):
        """Save configuration to file"""
        with open(self.config_file, 'w') as f:
            json.dump(self.variables, f, indent=2)

    def set_variable(self, name: str, value: str):
        """Set a variable value, attempting type conversion"""
        # Try to convert to appropriate type
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