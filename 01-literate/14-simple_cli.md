---
version: "2.1"
generated: "2026-05-12"
---

# SimpleCLI

`simple_cli.py` is the top-level entry point for the robot control CLI. It assembles the full stack — config, controller, dispatcher — and provides both an interactive REPL and a non-interactive single-command mode.

## Assembly at Startup

```python
class SimpleCLI:
    def __init__(self):
        config_path = str(Path.home() / ".dome" / "config" / "control.yaml")
        self.config_manager = cm.ConfigManager.create(config_path)
        self.robot_controller = rc.RobotController(self.config_manager)
        self.intent_publisher = IntentPublisher()
        self.intent_publisher.get_api()  # warm up ROS2 node + DDS discovery at startup
        self.dispatcher = cd.CommandDispatcher(self.robot_controller, self.intent_publisher)
```

Construction order: `ConfigManager` → `RobotController` → `IntentPublisher` (warmed up) → `CommandDispatcher`. Config path is fixed at `~/.dome/config/control.yaml`.

`IntentPublisher` is constructed and its `IntentApi` ROS2 node is created eagerly at startup (the `.get_api()` call). This ensures DDS publisher-subscriber discovery happens before the first intent is published, avoiding the race condition where early commands are dropped.

`SimpleCommandParser` was removed in F15/T05. All text parsing now goes through `CommandDispatcher.dispatch_text`.

## Two Modes

```python
def main():
    cli = SimpleCLI()
    if len(sys.argv) > 1:
        cli.execute_command(" ".join(sys.argv[1:]))
    else:
        cli.repl()
```

Non-interactive: `ros2 run dome_control run move forward 1.0` — executes one command and exits. Interactive REPL: `prompt_toolkit` with persistent history in `~/.dome/config/prompt_history.txt`. Commands are also timestamped to `~/.dome/config/command_history.txt`.

## execute_command

```python
def execute_command(self, input_text: str) -> None:
    tokens = input_text.strip().split()
    first = tokens[0].lower()

    if first in ("help", "hlp"):
        subcommand = resolve_keyword(tokens[1]) if len(tokens) > 1 else None
        arguments = [resolve_keyword(t) for t in tokens[2:]]
        self.handle_help(subcommand, arguments)
        return

    if first in ("exit", "quit", "q", "x"):
        ...

    result = self.dispatcher.dispatch_text(input_text)
```

Three paths: help handling (in-CLI), exit handling (in-CLI), everything else goes to `dispatch_text`. Help and exit are intercepted before dispatch because they require CLI-level state (`self.running`, `self.dispatcher.commands`).

## Help System

```
help                      → show_common_commands()
help commands             → show_all_commands()
help <group>              → show_subcommand_suggestions()
help <group> <subcommand> → load_help_file() → docs/<group>.<sub>.txt
```

`show_all_commands` dumps the full `dispatcher.commands` registry alphabetically, including behavior commands (intent/scene group). Detailed help files load from `docs/` relative to the package source root.

## Error Feedback

When an unknown command is entered:

```python
if "Unknown command" in result.message and "." not in tokens[0]:
    self.show_subcommand_suggestions(tokens[0], "Did you mean one of these?")
```

Lists all `<group>.*` commands if the token looks like a group name.

## Observations

- `log_command` writes to file on every command with no buffering or rotation — history file grows indefinitely.
- `load_help_file` traverses three levels up from `__file__` to find `docs/`. Brittle to layout changes.
- Empty input is silently ignored in both REPL and non-interactive mode.
