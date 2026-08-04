#!/usr/bin/env python3
"""
Command Dispatcher - Central command registry and execution dispatcher
Author: Pito Salas and Claude Code
Open Source Under MIT license
"""
import shlex
from typing import Any

import dome_control.commands.command_def as cd
import dome_control.commands.control_commands as ctrl_cmd
import dome_control.commands.launch_commands as lch_cmd
import dome_control.commands.movement_commands as mov_cmd
import dome_control.commands.navigation_commands as nav_cmd
import dome_control.commands.parameter_def as paramdef_mod
import dome_control.commands.robot_controller as rc
import dome_control.commands.survey_commands as surv_cmd
import dome_control.commands.system_commands as sys_cmd


ABBREV_TO_FULL = {
    "m": "move", "t": "turn", "r": "robot", "lch": "launch",
    "c": "config", "sys": "system", "scr": "script",
    "fwd": "forward", "bak": "backward", "dis": "distance", "tim": "time",
    "clk": "clockwise", "ccw": "counterclockwise", "deg": "degrees", "rad": "radians",
    "stp": "stop", "sts": "status",
    "lst": "list", "inf": "info", "sta": "start", "kil": "kill",
    "set": "set", "get": "get",
    "top": "topics", "ps": "ps", "lau": "launches",
    "sqr": "square", "rot": "rotate_stress", "cir": "circle_stress",
    "sav": "save", "ser": "serialize",
    "hlp": "help", "q": "exit", "x": "exit",
}
FULL_NAMES = set(ABBREV_TO_FULL.values())


def resolve_keyword(word: str) -> str:
    if word in FULL_NAMES:
        return word
    return ABBREV_TO_FULL.get(word, word)


def parse_value(value_str: str) -> Any:
    if value_str.lower() in ("true", "yes", "1"):
        return True
    if value_str.lower() in ("false", "no", "0"):
        return False
    if "." not in value_str:
        try:
            return int(value_str)
        except ValueError:
            pass
    try:
        return float(value_str)
    except ValueError:
        pass
    return value_str


BEHAVIOR_COMMANDS: dict[str, str] = {
    "intent.stop": "stop",
    "intent.describe_scene": "describe_scene",
    "intent.count_objects": "count_objects",
    "intent.list_objects": "list_objects",
    "scene.describe": "describe_scene",
    "scene.count": "count_objects",
    "scene.objects": "list_objects",
}

BEHAVIOR_COMMAND_DESCRIPTIONS: dict[str, tuple[str, str]] = {
    "intent.stop":           ("Stop the robot", "intent"),
    "intent.describe_scene": ("Describe the current scene", "intent"),
    "intent.count_objects":  ("Count objects in scene", "intent"),
    "intent.list_objects":   ("List currently detected objects", "intent"),
    "scene.describe":        ("Describe the current scene", "scene"),
    "scene.count":           ("Count objects in scene", "scene"),
    "scene.objects":         ("List currently detected objects", "scene"),
}


class CommandDispatcher:
    """
    Unified command dispatcher for CLI, TUI, and REST API interfaces.
    Provides standardized parameter handling and response format.
    """

    def __init__(self, robot_controller, intent_publisher=None):
        self.robot_controller = robot_controller
        self.intent_publisher = intent_publisher
        self.commands = self._build_command_registry()

    def _build_command_registry(self) -> dict[str, cd.CommandDef]:
        commands = {}
        commands.update(mov_cmd.build_movement_commands())
        commands.update(ctrl_cmd.build_control_commands())
        commands.update(nav_cmd.build_navigation_commands())
        commands.update(lch_cmd.build_launch_commands())
        commands.update(sys_cmd.build_system_commands())
        commands.update(surv_cmd.build_survey_commands())
        for name, (description, group) in BEHAVIOR_COMMAND_DESCRIPTIONS.items():
            commands[name] = cd.CommandDef("", [], description, group)
        return commands

    def execute(
        self, command_name: str, params: dict[str, object]
    ) -> rc.CommandResponse:
        intent_name = BEHAVIOR_COMMANDS.get(command_name)
        if intent_name is not None:
            reply = self.publish_intent(intent_name, params)
            msg = reply if reply is not None else f"Intent published: {intent_name}"
            return rc.CommandResponse(True, msg)

        if command_name not in self.commands:
            return rc.CommandResponse(
                success=False, message=f"Unknown command: {command_name}"
            )

        command_def = self.commands[command_name]

        try:
            validated_params = self._validate_parameters(command_def, params)
        except ValueError as e:
            return rc.CommandResponse(
                success=False, message=f"Parameter error: {e!s}"
            )

        try:
            method = getattr(self.robot_controller, command_def.method_name)
        except AttributeError:
            return rc.CommandResponse(
                success=False, message=f"Method {command_def.method_name} not found"
            )

        try:
            if validated_params:
                result = method(**validated_params)
            else:
                result = method()

            if isinstance(result, rc.CommandResponse):
                return result
            return rc.CommandResponse(
                success=True,
                message=str(result) if result is not None else "Command completed",
            )

        except Exception as e:
            return rc.CommandResponse(
                success=False, message=f"Command execution error: {e!s}"
            )

    def _validate_parameters(
        self, command_def: cd.CommandDef, params: dict[str, object]
    ) -> dict[str, object]:
        validated = {}

        for param_def in command_def.parameters:
            param_name = param_def.name

            if param_def.required and param_name not in params:
                raise ValueError(f"Missing required parameter: {param_name}")

            if param_name not in params:
                if param_def.default is not None:
                    validated[param_name] = param_def.default
                continue

            value = params[param_name]
            try:
                validated[param_name] = self._convert_parameter_value(param_def, value)
            except (ValueError, TypeError):
                expected = param_def.param_type.__name__
                got = type(value).__name__
                raise ValueError(
                    f"Invalid type for {param_name}: expected {expected}, got {got}"
                )

        return validated

    def _convert_parameter_value(
        self, param_def: paramdef_mod.ParameterDef, value: object
    ) -> object:
        if param_def.param_type == bool:
            if isinstance(value, str):
                return value.lower() in ("true", "1", "yes", "on")
            return bool(value)
        if param_def.param_type == str:
            return str(value)
        return param_def.param_type(value)

    def dispatch_text(self, text: str) -> rc.CommandResponse:
        try:
            tokens = shlex.split(text.strip())
        except ValueError:
            tokens = text.strip().split()
        if not tokens:
            return rc.CommandResponse(success=False, message="Empty command")

        command = resolve_keyword(tokens[0])

        if len(tokens) == 1:
            command_name = command
            args = []
        else:
            second = resolve_keyword(tokens[1])
            candidate2 = f"{command}.{second}"

            # Try 3-token candidate first so "nav explore stop" hits nav.explore.stop
            if len(tokens) >= 3:
                third = resolve_keyword(tokens[2])
                candidate3 = f"{command}.{second}.{third}"
                if candidate3 in self.commands or candidate3 in BEHAVIOR_COMMANDS:
                    command_name = candidate3
                    args = [parse_value(t) for t in tokens[3:]]
                else:
                    in_registry2 = candidate2 in self.commands or candidate2 in BEHAVIOR_COMMANDS
                    if second in FULL_NAMES or second != tokens[1] or in_registry2:
                        command_name = candidate2
                        args = [parse_value(t) for t in tokens[2:]]
                    else:
                        command_name = command
                        args = [parse_value(t) for t in tokens[1:]]
            else:
                in_registry2 = candidate2 in self.commands or candidate2 in BEHAVIOR_COMMANDS
                if second in FULL_NAMES or second != tokens[1] or in_registry2:
                    command_name = candidate2
                    args = [parse_value(t) for t in tokens[2:]]
                else:
                    command_name = command
                    args = [parse_value(t) for t in tokens[1:]]

        intent_name = BEHAVIOR_COMMANDS.get(command_name)
        if intent_name is not None:
            slots = {}
            cmd_def = self.get_command_info(command_name)
            if cmd_def and args:
                slots = {
                    cmd_def.parameters[i].name: args[i]
                    for i in range(min(len(args), len(cmd_def.parameters)))
                }
            reply = self.publish_intent(intent_name, slots)
            msg = reply if reply is not None else f"Intent published: {intent_name}"
            return rc.CommandResponse(True, msg)

        cmd_def = self.get_command_info(command_name)
        if not cmd_def:
            return self.execute(command_name, {})

        # If last param is str and there are extra tokens, join them into one string
        if (cmd_def.parameters and
                cmd_def.parameters[-1].param_type == str and
                len(args) > len(cmd_def.parameters)):
            last_idx = len(cmd_def.parameters) - 1
            args = list(args[:last_idx]) + [" ".join(str(a) for a in args[last_idx:])]

        params = {
            cmd_def.parameters[i].name: args[i]
            for i in range(min(len(args), len(cmd_def.parameters)))
        }
        return self.execute(command_name, params)

    def publish_intent(self, name: str, slots: dict) -> str | None:
        if self.intent_publisher is not None:
            return self.intent_publisher.publish(name, "cli", slots)
        from dome_control.commands.intent_publisher import IntentPublisher
        return IntentPublisher().publish(name, "cli", slots)

    def list_commands(self, group: str | None = None) -> list[str]:
        if group:
            return [
                name
                for name, cmd_def in self.commands.items()
                if cmd_def.group == group
            ]
        return list(self.commands.keys())

    def get_command_info(self, command_name: str) -> cd.CommandDef | None:
        return self.commands.get(command_name)

    def get_groups(self) -> list[str]:
        groups = {cmd_def.group for cmd_def in self.commands.values()}
        return sorted(groups)
