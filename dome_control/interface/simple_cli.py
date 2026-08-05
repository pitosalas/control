#!/usr/bin/env python3
"""
Simple CLI using CommandDispatcher
Author: Pito Salas and Claude Code
Open Source Under MIT license
"""

import sys
from datetime import datetime
from pathlib import Path

from prompt_toolkit import prompt
from prompt_toolkit.history import FileHistory

import dome_control.commands.command_dispatcher as cd
import dome_control.commands.config_manager as cm
import dome_control.commands.robot_controller as rc
from dome_control.commands.command_dispatcher import resolve_keyword
from dome_control.commands.intent_publisher import IntentPublisher


def format_table(rows: list[dict], columns: list[tuple[str, str]]) -> str:
    """Format a list of dicts as a simple fixed-width table."""
    if not rows:
        return ""

    widths = {}
    for key, heading in columns:
        values = [_format_cell(row.get(key)) for row in rows]
        widths[key] = max(len(heading), *(len(value) for value in values))

    header = _format_table_row(
        {key: heading for key, heading in columns}, columns, widths
    )
    separator = _format_table_row(
        {key: "-" * widths[key] for key, _ in columns}, columns, widths
    )
    body = [
        _format_table_row(row, columns, widths)
        for row in rows
    ]
    return "\n".join([header, separator, *body])


def _format_table_row(
    row: dict, columns: list[tuple[str, str]], widths: dict[str, int]
) -> str:
    cells = []
    last_index = len(columns) - 1
    for index, (key, _) in enumerate(columns):
        cell = _format_cell(row.get(key))
        cells.append(cell if index == last_index else cell.ljust(widths[key]))
    return "  ".join(cells)


def _format_cell(value) -> str:
    if value is None:
        return "-"
    if isinstance(value, bool):
        return "yes" if value else "no"
    return str(value)


class SimpleCLI:

    def __init__(self):
        config_path = str(Path.home() / ".dome" / "config" / "control.yaml")
        self.config_manager = cm.ConfigManager.create(config_path)
        self.robot_controller = rc.RobotController(self.config_manager)
        self.intent_publisher = IntentPublisher()
        self.intent_publisher.get_api()  # warm up ROS2 node + DDS discovery at startup
        self.dispatcher = cd.CommandDispatcher(self.robot_controller, self.intent_publisher)
        self.running = True

        control_dir = self.config_manager.get_control_dir()
        self.command_history_file = control_dir / "command_history.txt"
        self.prompt_history = FileHistory(str(control_dir / "prompt_history.txt"))

    def load_help_file(self, filename: str) -> str | None:
        docs_dir = Path(__file__).resolve().parent.parent.parent / "docs"
        help_file = docs_dir / filename
        if help_file.exists():
            return help_file.read_text()
        return None

    def show_subcommand_suggestions(
        self, command_name: str, header: str, footer: str | None = None
    ) -> bool:
        suggestions = [
            cmd for cmd in self.dispatcher.commands.keys()
            if cmd.startswith(f"{command_name}.")
        ]
        if not suggestions:
            return False

        print(header)
        for suggestion in sorted(suggestions):
            cmd_def = self.dispatcher.commands[suggestion]
            subcommand = suggestion.split(".", 1)[1]
            print(f"  {command_name} {subcommand:<20} - {cmd_def.description}")
        if footer:
            print(footer)
        return True

    def show_common_commands(self) -> None:
        print("Common Commands:")
        print()
        print("Movement:")
        print("  move forward <meters>")
        print("  move backward <meters>")
        print("  turn clockwise <degrees>")
        print("  turn counterclockwise <degrees>")
        print()
        print("Control:")
        print("  robot stop")
        print("  robot status")
        print()
        print("Configuration:")
        print("  config get <variable>")
        print("  config set <variable> <value>")
        print()
        print("Launch:")
        print("  launch start <type>")
        print("  launch stop <type>")
        print()
        print("Behaviors (publish to /intent):")
        print("  scene describe")
        print("  scene count")
        print("  intent stop")
        print()
        print("Other:")
        print("  help <command>        - Show help for specific command")
        print("  help commands         - Show all available commands")
        print("  exit                  - Exit the program")

    def show_all_commands(self) -> None:
        print("All Available Commands (alphabetical):")
        print("=" * 70)
        for cmd_name in sorted(self.dispatcher.commands.keys()):
            cmd_def = self.dispatcher.commands[cmd_name]
            display_name = cmd_name.replace(".", " ")
            print(f"  {display_name:<30} - {cmd_def.description}")
        print("\nFor detailed help on any command: help <command> <subcommand>")

    def show_specific_help(
        self, subcommand: str | None, arguments: list
    ) -> None:
        if arguments:
            filename = f"{subcommand}.{arguments[0]}.txt"
        else:
            filename = f"{subcommand}.txt"

        help_text = self.load_help_file(filename)
        if help_text:
            print(help_text)
            return

        found = self.show_subcommand_suggestions(
            subcommand,
            f"Available '{subcommand}' commands:",
            f"\nFor detailed help, use: help {subcommand} <subcommand>",
        )
        if not found:
            command_str = (
                f"{subcommand} {arguments[0]}" if arguments else subcommand
            )
            print(f"No help available for: {command_str}")

    def handle_help(self, subcommand: str | None, arguments: list) -> None:
        if subcommand == "commands":
            self.show_all_commands()
            return
        if subcommand:
            self.show_specific_help(subcommand, arguments)
        else:
            self.show_common_commands()

    def execute_command(self, input_text: str) -> None:
        if not input_text.strip():
            return

        tokens = input_text.strip().split()
        first = tokens[0].lower()

        if first in ("help", "hlp"):
            subcommand = resolve_keyword(tokens[1]) if len(tokens) > 1 else None
            arguments = [resolve_keyword(t) for t in tokens[2:]]
            self.handle_help(subcommand, arguments)
            return

        if first in ("exit", "quit", "q", "x"):
            print("Goodbye!")
            self.running = False
            return

        result = self.dispatcher.dispatch_text(input_text)

        if result.success:
            print(f"✓ {result.message}")
            if result.data:
                self.print_data(result.data)
        else:
            print(f"✗ {result.message}")
            if "Unknown command" in result.message and "." not in tokens[0]:
                self.show_subcommand_suggestions(
                    tokens[0], "Did you mean one of these?"
                )

    def print_data(self, data) -> None:
        if isinstance(data, dict):
            if "variables" in data and isinstance(data["variables"], dict):
                self.print_variables(data["variables"])
                return
            if "status" in data and isinstance(data["status"], dict):
                self.print_status(data["status"])
                return
            if "subsystems" in data and isinstance(data["subsystems"], dict):
                self.print_subsystems(data["subsystems"])
                return

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

    def print_variables(self, variables: dict) -> None:
        rows = [
            {"name": name, "value": value}
            for name, value in sorted(variables.items())
        ]
        table = format_table(rows, [("name", "NAME"), ("value", "VALUE")])
        if table:
            print(table)

    def print_status(self, status: dict) -> None:
        speeds = status.get("speeds", {})
        if speeds:
            print("configured speeds:")
            rows = [
                {"name": name, "value": value}
                for name, value in sorted(speeds.items())
            ]
            print(format_table(rows, [("name", "NAME"), ("value", "VALUE")]))

        processes = status.get("processes", {})
        if processes:
            if speeds:
                print()
            print("launch processes:")
            rows = [
                {
                    "name": name,
                    "running": details.get("running"),
                    "pid": details.get("pid"),
                    "process_id": details.get("process_id"),
                }
                for name, details in sorted(processes.items())
            ]
            print(format_table(
                rows,
                [
                    ("name", "NAME"),
                    ("running", "RUNNING"),
                    ("pid", "PID"),
                    ("process_id", "PROCESS ID"),
                ],
            ))

        nodes = status.get("nodes", {})
        if nodes:
            if speeds or processes:
                print()
            print("api nodes:")
            rows = [
                {"name": name, "state": state}
                for name, state in sorted(nodes.items())
            ]
            print(format_table(rows, [("name", "NAME"), ("state", "STATE")]))

    def print_subsystems(self, subsystems: dict) -> None:
        rows = [
            {
                "subsystem": name,
                "running": entry.get("running"),
                "detail": entry["state"] if name == "mission"
                          else str(len(entry.get("processes", []))),
            }
            for name, entry in sorted(subsystems.items())
        ]
        print(format_table(
            rows,
            [
                ("subsystem", "SUBSYSTEM"),
                ("running", "RUNNING"),
                ("detail", "DETAIL"),
            ],
        ))

    def log_command(self, command: str) -> None:
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        with open(self.command_history_file, "a") as f:
            f.write(f"\n# {timestamp}\n")
            f.write(f"+{command}\n")

    def repl(self) -> None:
        print("Robot Control CLI")
        print("Type 'help' for available commands, 'exit' to quit")
        print()

        while self.running:
            try:
                input_text = prompt("robot> ", history=self.prompt_history)
                if not input_text.strip():
                    continue
                self.log_command(input_text.strip())
                self.execute_command(input_text)
            except EOFError:
                print()
                print("Goodbye!")
                self.running = False
            except KeyboardInterrupt:
                print()
                print("Use 'exit' to quit")


def main() -> None:
    cli = SimpleCLI()

    if len(sys.argv) > 1:
        cli.execute_command(" ".join(sys.argv[1:]))
    else:
        cli.repl()


if __name__ == "__main__":
    main()
