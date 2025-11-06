#!/usr/bin/env python3
"""
Simple CLI using SimpleCommandParser instead of Click
Author: Pito Salas and Claude Code
Open Source Under MIT license
"""

import sys
from datetime import datetime
from pathlib import Path

from prompt_toolkit import prompt
from prompt_toolkit.history import FileHistory

import control.commands.command_dispatcher as cd
import control.commands.config_manager as cm
import control.commands.robot_controller as rc
from control.interface.simple_parser import ParsedCommand, SimpleCommandParser


class SimpleCLI:
    """
    Command-line interface using SimpleCommandParser.

    Provides interactive REPL for robot control commands.
    Simpler alternative to Click-based CLI.
    """

    def __init__(self):
        self.parser = SimpleCommandParser()
        self.config_manager = cm.ConfigManager("/home/pitosalas/.control/config.yaml")
        self.robot_controller = rc.RobotController(self.config_manager)
        self.dispatcher = cd.CommandDispatcher(self.robot_controller)
        self.running = True

        # Set up command history with timestamps in control directory
        control_dir = self.config_manager.get_control_dir()
        self.command_history_file = control_dir / "command_history.txt"
        history_file = control_dir / "prompt_history.txt"
        self.prompt_history = FileHistory(str(history_file))

    def _map_to_dispatcher_format(self, parsed: ParsedCommand):
        """
        Map ParsedCommand to CommandDispatcher format.

        Args:
            parsed: ParsedCommand from parser

        Returns:
            (command_name, params_dict) for dispatcher

        Examples:
            ParsedCommand("move", "forward", [1.5])
                -> ("move.forward", {"meters": 1.5})
            ParsedCommand("config", "set", ["linear_speed", 0.5])
                -> ("config.set", {"name": "linear_speed", "value": 0.5})

        """
        # Build command name
        if parsed.subcommand:
            command_name = f"{parsed.command}.{parsed.subcommand}"
        else:
            command_name = parsed.command

        # Get command definition to map arguments to parameter names
        cmd_def = self.dispatcher.get_command_info(command_name)

        if not cmd_def:
            return command_name, {}

        # Map positional arguments to named parameters
        params = {}
        for i, arg in enumerate(parsed.arguments):
            if i < len(cmd_def.parameters):
                param_name = cmd_def.parameters[i].name
                params[param_name] = arg

        return command_name, params

    def _load_help_file(self, filename: str):
        """Load help text from docs directory."""
        # Resolve symlinks to get actual source directory
        module_path = Path(__file__).resolve()
        docs_dir = module_path.parent.parent.parent / "docs"
        help_file = docs_dir / filename

        if help_file.exists():
            return help_file.read_text()
        return None

    def _handle_help(self, parsed: ParsedCommand):
        # Parser treats "help move forward" as:
        #   command="help", subcommand="move", arguments=["forward"]
        # So we need to reconstruct the command name from subcommand + arguments

        # Special case: "help commands" - list all commands alphabetically
        if parsed.arguments and len(parsed.arguments) == 1 and parsed.arguments[0] == "commands":
            print("All Available Commands (alphabetical):")
            print("=" * 70)
            for cmd_name in sorted(self.dispatcher.commands.keys()):
                cmd_def = self.dispatcher.commands[cmd_name]
                # Format: "command.subcommand" becomes "command subcommand"
                display_name = cmd_name.replace(".", " ")
                print(f"  {display_name:<30} - {cmd_def.description}")
            print("\nFor detailed help on any command: help <command> <subcommand>")
            return

        if parsed.subcommand:
            # "help move forward" case
            if parsed.arguments:
                # Two-part command: "move.forward"
                filename = f"{parsed.subcommand}.{parsed.arguments[0]}.txt"
            else:
                # Single-part command: "move"
                filename = f"{parsed.subcommand}.txt"

            help_text = self._load_help_file(filename)
            if help_text:
                print(help_text)
            else:
                # If no help file found, try showing available subcommands
                command_name = parsed.subcommand
                suggestions = [
                    cmd for cmd in self.dispatcher.commands.keys()
                    if cmd.startswith(f"{command_name}.")
                ]
                if suggestions:
                    print(f"Available '{command_name}' commands:")
                    for suggestion in sorted(suggestions):
                        cmd_def = self.dispatcher.commands[suggestion]
                        subcommand = suggestion.split(".", 1)[1]
                        print(f"  {command_name} {subcommand:<20} - {cmd_def.description}")
                    print(f"\nFor detailed help, use: help {command_name} <subcommand>")
                else:
                    command_str = (
                        f"{parsed.subcommand} {parsed.arguments[0]}"
                        if parsed.arguments
                        else parsed.subcommand
                    )
                    print(f"No help available for: {command_str}")
        else:
            # General help - load index file
            help_text = self._load_help_file("_index.txt")
            if help_text:
                print(help_text)
            else:
                print("Help documentation not found.")

    def _handle_exit(self):
        print("Goodbye!")
        self.running = False

    def execute_command(self, input_text: str):
        if not input_text.strip():
            return

        parsed, error = self.parser.parse(input_text)

        if error:
            print(f"Error: {error.message}")
            return

        # Handle special commands
        if parsed.command == "help":
            self._handle_help(parsed)
            return

        if parsed.command == "exit":
            self._handle_exit()
            return

        # Map to dispatcher format and execute
        command_name, params = self._map_to_dispatcher_format(parsed)
        result = self.dispatcher.execute(command_name, params)

        # Display result
        if result.success:
            print(f"✓ {result.message}")
            if result.data:
                self._print_data(result.data)
        else:
            print(f"✗ {result.message}")
            # If command not found, suggest available subcommands
            if "Unknown command" in result.message and "." not in command_name:
                suggestions = [
                    cmd for cmd in self.dispatcher.commands.keys()
                    if cmd.startswith(f"{command_name}.")
                ]
                if suggestions:
                    print(f"Did you mean one of these?")
                    for suggestion in sorted(suggestions):
                        cmd_def = self.dispatcher.commands[suggestion]
                        subcommand = suggestion.split(".", 1)[1]
                        print(f"  {command_name} {subcommand} - {cmd_def.description}")

    def _print_data(self, data):
        if isinstance(data, dict):
            for key, value in data.items():
                if isinstance(value, dict):
                    print(f"{key}:")
                    for k, v in value.items():
                        print(f"  {k}: {v}")
                elif isinstance(value, list):
                    print(f"{key}: {', '.join(str(v) for v in value)}")
                else:
                    print(f"{key}: {value}")
        else:
            print(data)

    def _log_command(self, command: str):
        """Log command with timestamp to command_history.txt."""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        with open(self.command_history_file, "a") as f:
            f.write(f"\n# {timestamp}\n")
            f.write(f"+{command}\n")

    def repl(self):
        """
        Run interactive Read-Eval-Print Loop.

        Reads commands from user, executes them, and displays results.
        Exits on 'exit' command or Ctrl+D.
        """
        print("Robot Control CLI (Simple Parser)")
        print("Type 'help' for available commands, 'exit' to quit")
        print()

        while self.running:
            try:
                # Read command with history support
                input_text = prompt("robot> ", history=self.prompt_history)

                # Skip empty commands
                if not input_text.strip():
                    continue

                # Log command to file
                self._log_command(input_text.strip())

                # Execute command
                self.execute_command(input_text)

            except EOFError:
                # Ctrl+D
                print()
                self._handle_exit()
            except KeyboardInterrupt:
                # Ctrl+C
                print()
                print("Use 'exit' to quit")
            except Exception as e:
                print(f"Error: {e}")

    def run_command(self, command: str):
        self.execute_command(command)


def main():
    """
    Entry point for robot control CLI.

    Runs in interactive REPL mode if no args provided,
    or executes single command from command-line args.
    """
    cli = SimpleCLI()

    if len(sys.argv) > 1:
        # Non-interactive: execute command from args
        command = " ".join(sys.argv[1:])
        cli.run_command(command)
    else:
        # Interactive REPL
        cli.repl()


if __name__ == "__main__":
    main()
