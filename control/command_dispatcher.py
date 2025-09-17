#!/usr/bin/env python3
from typing import Dict, List, Any, Optional
from .robot_controller import CommandResponse
from .command_def import CommandDef
from .parameter_def import ParameterDef
from .movement_commands import build_movement_commands
from .control_commands import build_control_commands
from .navigation_commands import build_navigation_commands
from .system_commands import build_system_commands


class CommandDispatcher:
    """
    Unified command dispatcher for CLI, TUI, and REST API interfaces.
    Provides standardized parameter handling and response format.
    """

    def __init__(self, robot_controller):
        self.robot_controller = robot_controller
        self.commands = self._build_command_registry()

    def _build_command_registry(self) -> Dict[str, CommandDef]:
        """Build the registry of all available commands."""
        commands = {}
        commands.update(build_movement_commands())
        commands.update(build_control_commands())
        commands.update(build_navigation_commands())
        commands.update(build_system_commands())
        return commands

    def execute(self, command_name: str, params: Dict[str, Any]) -> CommandResponse:
        """Execute a command with given parameters."""
        if command_name not in self.commands:
            return CommandResponse(
                success=False,
                message=f"Unknown command: {command_name}"
            )

        command_def = self.commands[command_name]

        try:
            validated_params = self._validate_parameters(command_def, params)
        except ValueError as e:
            return CommandResponse(
                success=False,
                message=f"Parameter error: {str(e)}"
            )

        try:
            method = getattr(self.robot_controller, command_def.method_name)
        except AttributeError:
            return CommandResponse(
                success=False,
                message=f"Method {command_def.method_name} not found"
            )

        try:
            if validated_params:
                result = method(**validated_params)
            else:
                result = method()

            if isinstance(result, CommandResponse):
                return result
            else:
                return CommandResponse(
                    success=True,
                    message=str(result) if result is not None else "Command completed"
                )

        except Exception as e:
            return CommandResponse(
                success=False,
                message=f"Command execution error: {str(e)}"
            )

    def _validate_parameters(self, command_def: CommandDef, params: Dict[str, Any]) -> Dict[str, Any]:
        """Validate and convert parameters according to command definition."""
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
            except (ValueError, TypeError) as e:
                raise ValueError(f"Invalid type for {param_name}: expected {param_def.param_type.__name__}, got {type(value).__name__}")

        return validated

    def _convert_parameter_value(self, param_def: ParameterDef, value: Any) -> Any:
        """Convert a parameter value to the expected type."""
        if param_def.param_type == bool:
            if isinstance(value, str):
                return value.lower() in ('true', '1', 'yes', 'on')
            else:
                return bool(value)
        elif param_def.param_type == str:
            return str(value)
        else:
            return param_def.param_type(value)

    def list_commands(self, group: Optional[str] = None) -> List[str]:
        """List available commands, optionally filtered by group."""
        if group:
            return [name for name, cmd_def in self.commands.items()
                   if cmd_def.group == group]
        return list(self.commands.keys())

    def get_command_info(self, command_name: str) -> Optional[CommandDef]:
        """Get detailed information about a command."""
        return self.commands.get(command_name)

    def get_groups(self) -> List[str]:
        """Get list of all command groups."""
        groups = set(cmd_def.group for cmd_def in self.commands.values())
        return sorted(groups)

    def get_help(self, command_name: Optional[str] = None) -> str:
        """Generate help text for commands."""
        if command_name:
            return self._get_command_help(command_name)
        else:
            return self._get_general_help()

    def _get_command_help(self, command_name: str) -> str:
        """Get help for a specific command."""
        if command_name not in self.commands:
            return f"Unknown command: {command_name}"

        cmd_def = self.commands[command_name]
        help_text = f"{command_name}: {cmd_def.description}\n"

        if cmd_def.parameters:
            help_text += "Parameters:\n"
            for param in cmd_def.parameters:
                required_str = "required" if param.required else f"optional (default: {param.default})"
                help_text += f"  {param.name} ({param.param_type.__name__}, {required_str}): {param.description}\n"
        else:
            help_text += "No parameters required.\n"

        return help_text

    def _get_general_help(self) -> str:
        """Get general help for all commands."""
        help_text = "Available Commands:\n\n"

        for group in self.get_groups():
            help_text += f"{group.upper()} COMMANDS:\n"
            group_commands = [name for name, cmd_def in self.commands.items()
                            if cmd_def.group == group]

            for cmd_name in sorted(group_commands):
                cmd_def = self.commands[cmd_name]
                help_text += f"  {cmd_name:<20} - {cmd_def.description}\n"
            help_text += "\n"

        help_text += "Use get_help('<command_name>') for detailed parameter information.\n"
        return help_text