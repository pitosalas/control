#!/usr/bin/env python3

import json
import os
from pathlib import Path
from typing import Dict, Any

class ConfigManager:
    DEFAULT_CONFIG = {
        "linear_speed": 0.3,
        "angular_speed": 0.4
    }

    def __init__(self, config_file: str = None):
        if config_file is None:
            # Use user's home directory for writable config file
            self.config_file = Path.home() / ".config" / "control_config.json"
            # Create .config directory if it doesn't exist
            self.config_file.parent.mkdir(parents=True, exist_ok=True)
        else:
            self.config_file = Path(config_file)

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