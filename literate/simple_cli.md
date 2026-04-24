# SimpleCLI — Interactive Robot Control Shell

## Overview

`SimpleCLI` is the human interface to the robot. It runs as an interactive REPL (Read-Eval-Print Loop) or accepts a single command from the shell's argument vector. Either way, it parses free-text input, maps it to dispatcher commands, and prints results.

The class sits at the top of the dependency stack: it owns a `ConfigManager`, a `RobotController`, and a `CommandDispatcher`, all wired together at startup.

## Startup and Configuration

Construction resolves the config path from an environment variable before anything else — this is the only place the `CONTROL_CONFIG` env var is read:

```python
def __init__(self):
    self.parser = SimpleCommandParser()
    config_path = os.environ.get("CONTROL_CONFIG", str(Path.home() / ".control" / "config.yaml"))
    self.config_manager = cm.ConfigManager.create(config_path)
    self.robot_controller = rc.RobotController(self.config_manager)
    self.dispatcher = cd.CommandDispatcher(self.robot_controller)
    self.running = True
```

Command history is stored in two files inside the control directory: `prompt_history.txt` feeds `prompt_toolkit` for up-arrow recall, and `command_history.txt` is a timestamped audit log.

## Input → Dispatcher: The Mapping Problem

The parser returns a `ParsedCommand` with positional `arguments`. The dispatcher expects named keyword parameters. `_map_to_dispatcher_format` bridges them by looking up the command's `ParameterDef` list and zipping positional args to parameter names in order:

```python
def _map_to_dispatcher_format(self, parsed: ParsedCommand):
    if parsed.subcommand:
        command_name = f"{parsed.command}.{parsed.subcommand}"
    else:
        command_name = parsed.command

    cmd_def = self.dispatcher.get_command_info(command_name)
    if not cmd_def:
        return command_name, {}

    params = {}
    for i, arg in enumerate(parsed.arguments):
        if i < len(cmd_def.parameters):
            param_name = cmd_def.parameters[i].name
            params[param_name] = arg

    return command_name, params
```

Extra positional args beyond what the command defines are silently dropped. Named flags from the parser (e.g. `--map basement`) are handled by the parser itself before this mapping runs.

## Command Execution Flow

```
user types "launch start nav --map basement"
         │
         ▼
   SimpleCommandParser.parse()
         │  ParsedCommand(command="launch", subcommand="start",
         │               arguments=["nav"], flags={"map": "basement"})
         ▼
   _map_to_dispatcher_format()
         │  ("launch.start", {"launch_type": "nav", "map": "basement"})
         ▼
   CommandDispatcher.execute()
         │  CommandResponse(success=True, message="Started nav", data={...})
         ▼
   print "✓ Started nav"
```

## Help System

Help is layered: generic `help` shows common commands; `help <group>` lists subcommands in that group; `help <group> <subcommand>` loads a `.txt` file from `docs/`. The file-based help is optional — missing files fall back to subcommand listings.

```python
def _load_help_file(self, filename: str):
    module_path = Path(__file__).resolve()
    docs_dir = module_path.parent.parent.parent / "docs"
    help_file = docs_dir / filename
    if help_file.exists():
        return help_file.read_text()
    return None
```

The path resolves symlinks first (`resolve()`) so the docs directory is found correctly whether the package is installed or run from source.

## Error Handling and Suggestions

When the dispatcher returns an unknown-command error and the input had no dot (i.e., it was a bare group name like `launch` without a subcommand), `execute_command` offers suggestions:

```python
if "Unknown command" in result.message and "." not in command_name:
    self._show_subcommand_suggestions(command_name, "Did you mean one of these?")
```

This recovers gracefully from the common case of a user typing a command group without a subcommand.

## Entry Points

`main()` decides between interactive and non-interactive mode based on `sys.argv`:

```python
def main():
    cli = SimpleCLI()
    if len(sys.argv) > 1:
        command = " ".join(sys.argv[1:])
        cli.execute_command(command)
    else:
        cli.repl()
```

This allows scripting: `control move forward 2` works from a shell script, while `control` alone drops into the REPL.

## Possible Improvements

- **`_show_common_commands` is hardcoded.** Any new command group requires editing this method. It should be generated from the dispatcher's command registry, filtered to a "common" subset via a flag on `CommandDef`.
- **`_print_data` has implicit structure assumptions.** It handles dict-of-dict and dict-of-list but silently treats anything else as a string. A structured result type (or a `__str__` on `CommandResponse.data`) would be more robust.
- **`__init__` mixes logic with assignment.** History file paths are computed and `FileHistory` is constructed inline. Extracting this to a `setup_history()` method would make the constructor intent clearer.
- **Audit log uses plain `open()` in `_log_command`.** If the control directory doesn't exist (race condition between startup and first command), this raises. The `ensure_subdirs` call in `ConfigManager.create` should prevent it, but the dependency is implicit.
- **Single-command mode ignores the return code.** If `execute_command` fails, `main()` exits with 0. Scripts that call `control <cmd>` cannot detect failures.
