#!/usr/bin/env python3

from typing import Dict, Callable, Any
from dataclasses import dataclass
from .teleopapi import TeleopApi

@dataclass
class CommandResult:
    success: bool
    message: str
    data: Any = None

class CommandProcessor:
    def __init__(self):
        self.commands: Dict[str, Callable] = {}
        self.teleop_api = None
        self._register_commands()

    def _get_teleop_api(self):
        if self.teleop_api is None:
            self.teleop_api = TeleopApi()
        return self.teleop_api

    def _register_commands(self):
        self.commands['move'] = self._handle_move

    def process_command(self, command_line: str) -> CommandResult:
        parts = command_line.strip().split()
        if not parts:
            return CommandResult(False, "Empty command")

        command = self._expand_abbreviation(parts[0])
        if command not in self.commands:
            return CommandResult(False, f"Unknown command: {parts[0]}")

        try:
            return self.commands[command](parts[1:])
        except Exception as e:
            return CommandResult(False, f"Command error: {str(e)}")

    def _expand_abbreviation(self, abbrev: str) -> str:
        abbrev = abbrev.lower()
        for cmd in self.commands.keys():
            if cmd.startswith(abbrev[:4]):
                return cmd
        return abbrev

    def _handle_move(self, args: list) -> CommandResult:
        if len(args) < 2:
            return CommandResult(False, "move requires: <subcommand> <value>")

        subcommand = args[0].lower()
        if subcommand == "dist":
            try:
                distance = float(args[1])
                return self._execute_move_distance(distance)
            except ValueError:
                return CommandResult(False, "Distance must be a number")
        else:
            return CommandResult(False, f"Unknown move subcommand: {subcommand}")

    def _execute_move_distance(self, distance: float) -> CommandResult:
        try:
            api = self._get_teleop_api()
            api.move_dist(distance)
            return CommandResult(True, f"Moved {distance} meters", {"distance": distance})
        except Exception as e:
            return CommandResult(False, f"Movement failed: {str(e)}")