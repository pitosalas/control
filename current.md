# Current State - Control Package

### How to use Current.md

* read claude.md and obey it fully
* read the rest of this document and make it part of your context
* adopt the todo list as your own
* when asked to create a new current.md, create a new file, include your current context, loose ends, todo list, and anything else.
* Also include this section called "How To Use Current.md"

## Status: COMPLETE - Major Refactoring Complete

The package has been successfully renamed from `dome_control` to `control` and significantly enhanced with a new click-based command system.

### Completed Major Changes:
1. ✅ **Complete package rename** - All files, directories, and imports updated
2. ✅ **Replaced typer with click** - Better command parsing and help generation
3. ✅ **Added variable management system** - Persistent storage with set/show commands
4. ✅ **Simplified main.py** - Direct CLI entry point (removed unused ROS2 node mode)
5. ✅ **Enhanced command structure** - Hierarchical commands with auto-generated help
6. ✅ **Added move time command** - Duration-based movement
7. ✅ **Configuration persistence** - JSON-based config saved to ~/.control_config.json

## Todo List:
1. [pending] Add more movement commands (turn, stop, etc.)
2. [pending] Implement ROS2 topic subscription interface
3. [pending] Implement TUI interface with textual (future)
4. [pending] Add error handling and logging improvements
5. [pending] Create unit tests for command processor

## Current Architecture:
- **Main Entry Point**: `control/main.py` - Direct CLI entry point
- **Command Processor**: `control/command_processor.py` - Click-based command parsing
- **CLI Interface**: `control/cli_interface.py` - Interactive "> " prompt interface
- **Robot API**: `control/teleopapi.py` - TeleopApi for robot movement
- **Config Manager**: `control/config_manager.py` - Variable storage and persistence

## Implemented Features:
- ✅ **CLI interface** with "> " prompt
- ✅ **Click-based command parsing** with auto-generated help
- ✅ **Variable management** - `set <var> <value>` and `show [var]` commands
- ✅ **Movement commands**:
  - `move dist <distance>` - Move specific distance in meters
  - `move time <seconds>` - Move for specified duration
- ✅ **Help system** - `help` command with support for `help <command>`
- ✅ **Configuration persistence** - Auto-save on exit, auto-load on startup
- ✅ **Type conversion** - Automatic detection of int, float, bool, string types
- ✅ **ROS2 package structure** with colcon build support

## Available Commands:
- `help [command]` - Show help for all commands or specific command
- `exit` - Exit the program cleanly
- `move dist <value>` - Move robot distance in meters (+ forward, - backward)
- `move time <seconds>` - Move robot for duration at default speed
- `set <varname> <value>` - Set persistent variable (auto-typed)
- `show [varname]` - Show variable value(s) with type info

## Dependencies:
- `click` - Command line interface framework
- `rclpy` - ROS2 Python client library
- Standard library: `json`, `pathlib`, `typing`, `dataclasses`

## Configuration:
- **Config file**: `~/.control_config.json`
- **Variables**: Stored as JSON with automatic type conversion
- **Persistence**: Saved on exit, loaded on startup

## Current Issues/Blockers:
- None - package is functional and ready for additional features

## Files in Package:
- `package.xml` - ROS2 package definition
- `setup.py` - Python package setup with click dependency
- `control/main.py` - Entry point
- `control/cli_interface.py` - Interactive CLI loop
- `control/command_processor.py` - Click command definitions
- `control/config_manager.py` - Variable storage management
- `control/teleopapi.py` - Robot movement API (existing)
- `resource/control` - ROS2 resource marker file

## Next Priority Features:
1. **Movement commands**: Add turn, stop, and other movement primitives
2. **Error handling**: Improve error messages and recovery
3. **Unit tests**: Test command processor and config manager
4. **ROS2 integration**: Topic subscription for external control