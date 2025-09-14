#!/usr/bin/env python3

import json
import os
from pathlib import Path
from typing import Dict, Any

class ConfigManager:
    def __init__(self, config_file: str = None):
        if config_file is None:
            home = Path.home()
            self.config_file = home / ".control_config.json"
        else:
            self.config_file = Path(config_file)

        self.variables: Dict[str, Any] = {}
        self.load_config()

    def load_config(self):
        """Load configuration from file"""
        try:
            if self.config_file.exists():
                with open(self.config_file, 'r') as f:
                    self.variables = json.load(f)
        except Exception as e:
            print(f"Warning: Could not load config file: {e}")
            self.variables = {}

    def save_config(self):
        """Save configuration to file"""
        try:
            os.makedirs(self.config_file.parent, exist_ok=True)
            with open(self.config_file, 'w') as f:
                json.dump(self.variables, f, indent=2)
        except Exception as e:
            print(f"Warning: Could not save config file: {e}")

    def set_variable(self, name: str, value: str) -> bool:
        """Set a variable value, attempting type conversion"""
        try:
            # Try to convert to appropriate type
            if value.lower() in ['true', 'false']:
                self.variables[name] = value.lower() == 'true'
            elif value.replace('-', '').replace('.', '').isdigit():
                # Try float first, then int
                if '.' in value:
                    self.variables[name] = float(value)
                else:
                    self.variables[name] = int(value)
            else:
                # Keep as string
                self.variables[name] = value
            return True
        except Exception:
            return False

    def get_variable(self, name: str) -> Any:
        """Get a variable value"""
        return self.variables.get(name)

    def get_all_variables(self) -> Dict[str, Any]:
        """Get all variables"""
        return self.variables.copy()

    def delete_variable(self, name: str) -> bool:
        """Delete a variable"""
        if name in self.variables:
            del self.variables[name]
            return True
        return False

    def variable_exists(self, name: str) -> bool:
        """Check if variable exists"""
        return name in self.variables