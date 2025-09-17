#!/usr/bin/env python3
"""
CLI adapter that uses the CommandDispatcher.
This shows how to migrate from the current Click-based CLI to the unified dispatcher.
"""

from typing import Dict, Any
from .command_dispatcher import CommandDispatcher
from .robot_controller import RobotController, CommandResponse
from .config_manager import ConfigManager


class DispatcherCLIAdapter:
    """
    CLI adapter that translates command-line style inputs to dispatcher calls.
    This bridges the gap between CLI parsing and the unified dispatcher.
    """

    def __init__(self, robot_controller: RobotController):
        self.dispatcher = CommandDispatcher(robot_controller)

    def process_command_line(self, command_line: str) -> CommandResponse:
        """
        Process a command line string using the dispatcher.

        Examples:
            "move distance 1.5" -> dispatcher.execute("move.distance", {"distance": 1.5})
            "robot speeds 0.5 0.3" -> dispatcher.execute("robot.speeds", {"linear": 0.5, "angular": 0.3})
            "nav start --sim-time" -> dispatcher.execute("nav.start", {"use_sim_time": True})
        """
        if not command_line.strip():
            return CommandResponse(False, "Empty command")

        # Parse the command line
        parts = command_line.strip().split()

        # Handle special cases
        if parts[0] == "help":
            if len(parts) == 1:
                return CommandResponse(True, self.dispatcher.get_help())
            else:
                # Convert "help move distance" to "move.distance"
                if len(parts) >= 3:
                    cmd_name = f"{parts[1]}.{parts[2]}"
                    return CommandResponse(True, self.dispatcher.get_help(cmd_name))
                else:
                    return CommandResponse(False, "Invalid help command")

        if parts[0] == "list":
            if len(parts) == 1:
                commands = self.dispatcher.list_commands()
                return CommandResponse(True, f"Available commands: {', '.join(commands)}")
            elif parts[1] == "groups":
                groups = self.dispatcher.get_groups()
                return CommandResponse(True, f"Command groups: {', '.join(groups)}")

        # Map CLI format to dispatcher format
        try:
            command_name, params = self._parse_command_line(parts)
            return self.dispatcher.execute(command_name, params)
        except ValueError as e:
            return CommandResponse(False, f"Parse error: {str(e)}")

    def _parse_command_line(self, parts: list) -> tuple[str, Dict[str, Any]]:
        """
        Parse command line parts into dispatcher format.

        Maps:
            ["move", "distance", "1.5"] -> ("move.distance", {"distance": 1.5})
            ["robot", "speeds", "0.5", "0.3"] -> ("robot.speeds", {"linear": 0.5, "angular": 0.3})
            ["nav", "start", "--sim-time"] -> ("nav.start", {"use_sim_time": True})
        """
        if len(parts) < 2:
            raise ValueError("Command requires at least 2 parts (group and subcommand)")

        group = parts[0]
        subcommand = parts[1]
        command_name = f"{group}.{subcommand}"

        # Get command definition to understand expected parameters
        cmd_info = self.dispatcher.get_command_info(command_name)
        if not cmd_info:
            raise ValueError(f"Unknown command: {command_name}")

        # Parse remaining arguments based on command definition
        args = parts[2:]
        params = {}

        # Handle flags first (like --sim-time)
        flag_args = []
        positional_args = []

        for arg in args:
            if arg.startswith('--'):
                flag_args.append(arg)
            else:
                positional_args.append(arg)

        # Process flags
        for flag in flag_args:
            if flag == "--sim-time":
                params["use_sim_time"] = True
            elif flag == "--brief":
                params["brief"] = True
            # Add more flags as needed

        # Process positional arguments
        param_defs = [p for p in cmd_info.parameters if not p.name in params]

        if len(positional_args) > len(param_defs):
            raise ValueError(f"Too many arguments for {command_name}")

        for i, arg in enumerate(positional_args):
            if i < len(param_defs):
                param_def = param_defs[i]
                try:
                    # Convert argument to expected type
                    if param_def.param_type == float:
                        params[param_def.name] = float(arg)
                    elif param_def.param_type == int:
                        params[param_def.name] = int(arg)
                    elif param_def.param_type == bool:
                        params[param_def.name] = arg.lower() in ('true', '1', 'yes', 'on')
                    else:
                        params[param_def.name] = str(arg)
                except ValueError:
                    raise ValueError(f"Invalid {param_def.param_type.__name__} value: {arg}")

        return command_name, params

    def get_available_commands(self) -> list[str]:
        """Get list of available commands for tab completion."""
        return self.dispatcher.list_commands()

    def get_command_groups(self) -> list[str]:
        """Get list of command groups."""
        return self.dispatcher.get_groups()

    def get_completions(self, partial_command: str) -> list[str]:
        """
        Get command completions for partial input.
        This would be used for tab completion in an interactive CLI.
        """
        if not partial_command.strip():
            return self.get_command_groups()

        parts = partial_command.strip().split()

        if len(parts) == 1:
            # Complete group names
            groups = self.get_command_groups()
            return [g for g in groups if g.startswith(parts[0])]

        elif len(parts) == 2:
            # Complete subcommand names for the group
            group = parts[0]
            prefix = parts[1]
            commands = self.dispatcher.list_commands(group=group)
            subcommands = [cmd.split('.')[1] for cmd in commands if cmd.startswith(f"{group}.")]
            return [sc for sc in subcommands if sc.startswith(prefix)]

        return []


# Example usage function
def demo_dispatcher_cli():
    """Demonstrate the dispatcher CLI adapter."""
    print("=== Dispatcher CLI Adapter Demo ===")

    # Setup
    config = ConfigManager()
    robot = RobotController(config)
    cli = DispatcherCLIAdapter(robot)

    # Test commands
    test_commands = [
        "move distance 1.5",
        "robot speeds 0.5 0.3",
        "nav start --sim-time",
        "help",
        "help move distance",
        "list",
        "list groups",
        "invalid command",
        "move distance",  # Missing parameter
    ]

    for cmd in test_commands:
        print(f"\n> {cmd}")
        result = cli.process_command_line(cmd)
        status = "✓" if result.success else "✗"
        print(f"{status} {result.message}")


if __name__ == "__main__":
    demo_dispatcher_cli()