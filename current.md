# Current State - Control Package

### How to use Current.md

* read claude.md and obey it fully
* read the rest of this document and make it part of your context
* adopt the todo list as your own
* when asked to create a new current.md, create a new file, include your current context, loose ends, todo list, and anything else.
* Also include this section called "How To Use Current.md"

## Status: FEATURE COMPLETE - Enhanced Robot Control System

The package has been significantly enhanced with comprehensive movement commands, configurable speeds, command history, and ROS topic integration.

### Latest Major Changes (Dec 2024):
1. ✅ **Added comprehensive turn commands** - turn time, turn radians, turn degrees
2. ✅ **Added stop command** - Immediate robot stopping
3. ✅ **Added calibration commands** - calibrate square for movement testing
4. ✅ **Configurable robot speeds** - linear_speed and angular_speed variables
5. ✅ **Command history with arrow keys** - prompt_toolkit integration
6. ✅ **Persistent command history** - Saved across sessions
7. ✅ **ROS topic listing** - show topics command
8. ✅ **Improved number parsing** - Handles negative numbers correctly
9. ✅ **Configuration moved to app directory** - control_config.json in package
10. ✅ **Comprehensive README** - Installation and usage instructions

### Previous Major Changes:
1. ✅ **Complete package rename** - All files, directories, and imports updated
2. ✅ **Replaced typer with click** - Better command parsing and help generation
3. ✅ **Added variable management system** - Persistent storage with set/show commands
4. ✅ **Simplified main.py** - Direct CLI entry point (removed unused ROS2 node mode)
5. ✅ **Enhanced command structure** - Hierarchical commands with auto-generated help
6. ✅ **Added move time command** - Duration-based movement
7. ✅ **Configuration persistence** - JSON-based config saved to control_config.json

## Todo List:
1. [pending] Implement ROS2 topic subscription interface
2. [pending] Implement TUI interface with textual (future)
3. [pending] Add error handling and logging improvements
4. [pending] Create unit tests for command processor
5. [pending] Add more calibration patterns (circle, figure-8)

## Current Architecture:
- **Main Entry Point**: `control/main.py` - Direct CLI entry point
- **Command Processor**: `control/command_processor.py` - Click-based command parsing
- **CLI Interface**: `control/cli_interface.py` - Interactive "> " prompt interface
- **Robot API**: `control/teleopapi.py` - TeleopApi for robot movement
- **Config Manager**: `control/config_manager.py` - Variable storage and persistence

## Implemented Features:
- ✅ **CLI interface** with "> " prompt and command history
- ✅ **Click-based command parsing** with auto-generated help
- ✅ **Variable management** - `set <var> <value>` and `show [var]` commands
- ✅ **Movement commands**:
  - `move dist <distance>` - Move specific distance in meters (handles negative values)
  - `move time <seconds>` - Move for specified duration
- ✅ **Turning commands**:
  - `turn time <seconds>` - Turn for specified duration
  - `turn radians <angle>` - Turn by angle in radians
  - `turn degrees <angle>` - Turn by angle in degrees
- ✅ **Control commands**:
  - `stop` - Stop robot immediately
- ✅ **Calibration commands**:
  - `calibrate square <meters>` - Move in square pattern
- ✅ **System commands**:
  - `show topics` - List active ROS topics
- ✅ **Help system** - `help` command with support for `help <command>`
- ✅ **Configuration persistence** - Auto-save on exit, auto-load on startup
- ✅ **Type conversion** - Automatic detection of int, float, bool, string types
- ✅ **Command history** - Up/down arrow keys with persistent history
- ✅ **ROS2 package structure** with colcon build support

## Available Commands:
- `help [command]` - Show help for all commands or specific command
- `exit` - Exit the program cleanly
- `move dist <value>` - Move robot distance in meters (+ forward, - backward)
- `move time <seconds>` - Move robot for duration at default speed
- `turn time <seconds>` - Turn robot for duration at default speed
- `turn radians <angle>` - Turn robot by angle in radians
- `turn degrees <angle>` - Turn robot by angle in degrees
- `stop` - Stop robot immediately
- `calibrate square <meters>` - Move robot in square pattern
- `set <varname> <value>` - Set persistent variable (auto-typed)
- `show` - Show all variables
- `show <varname>` - Show specific variable value
- `show *` - Show all variables
- `show topics` - Show active ROS topics

## Dependencies:
- `click` - Command line interface framework
- `prompt_toolkit` - Command history and input enhancement
- `rclpy` - ROS2 Python client library
- Standard library: `json`, `pathlib`, `typing`, `dataclasses`

## Configuration:
- **Config file**: `control_config.json` (in package directory)
- **Variables**: Stored as JSON with automatic type conversion
- **Persistence**: Saved on exit, loaded on startup
- **Command history**: Stored in `command_history.txt` (in package directory)

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