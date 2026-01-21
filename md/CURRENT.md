# Current Status - ROS2 Control Package

## Overview
ROS2 control package with CLI interface for robot movement, navigation, mapping, and system management.

## Architecture

### CLI & Command System
- **Active CLI**: `control/interface/simple_cli.py` - SimpleCommandParser-based (REPL + non-interactive)
- **Parser**: `control/interface/simple_parser.py` - Custom command parser
- **Dispatcher**: `control/commands/command_dispatcher.py` - Framework-agnostic command registry

### Command Organization
Commands defined in separate files by category:
- `movement_commands.py` - Robot movement (move, turn)
- `navigation_commands.py` - Map operations (map.save, map.list, map.serialize)
- `launch_commands.py` - Process launching (launch.start, launch.stop, launch.list, launch.info)
- `system_commands.py` - System utilities (config, scripts, process management)
- `control_commands.py` - Robot control (status, stop)

All commands route through `RobotController` (646 lines after cleanup).

### Launch System
- Launch types loaded from `~/.control/config.yaml` under `launch_templates`
- Currently configured: `nav`, `slam`, `map`
- Uses `ProcessApi` for unified process handling
- Creates process groups for clean shutdown (SIGTERM → SIGKILL)

### Map Management
- `map.save` - Uses `map_saver_cli` (one-shot command, 15s timeout)
- `map.list` - Lists available maps in `~/.control/maps/`
- `map.serialize` - SLAM Toolbox serialized format

### Configuration
- Config file: `~/.control/config.yaml`
- Variables: linear_speed, angular_speed, log_dir, map_name, etc.
- Commands: `config.get`, `config.set`, `config.list`

## Current State

### Working Features
- CLI with SimpleCommandParser (REPL and non-interactive modes)
- Dynamic launch system from config templates
- Map save using CLI tool (map_saver_cli)
- Movement commands (move, turn, stop)
- Script commands (square, rotate_stress, circle_stress)
- System commands (topics, processes, kill)
- **67 passing tests** (updated Jan 2026)

### Recent Changes (Jan 21)
- Process termination now uses SIGINT (Ctrl+C equivalent) instead of SIGTERM→SIGKILL
- Output capture is now continuous until process ends (was limited to first 20 lines)
- Added `CommandConfig` dataclass for `run_command_sync`
- Added debug logging throughout launch system
- Added `stdin=subprocess.DEVNULL` to detach launched processes from terminal

### Known Issues

**Code Quality:**
1. Bare `except Exception` in simple_cli.py line 220
2. `robot_controller.py` still 646 lines (over 300 line RULES.md limit)
3. Debug print statements in robot_controller.py and process_api.py (should be removed or converted to proper logging)

### Test Suite
- `test_command_dispatcher.py` - 12 tests
- `test_launch_commands.py` - 6 tests
- `test_map_service_logic.py` - 12 tests
- `test_negative_numbers.py` - 18 tests
- `test_robot_controller_launch.py` - 6 tests
- `test_script_commands.py` - 13 tests

Run with: `python3 -m pytest test/ -v --ignore=test/__init__.py`

## File Locations

### Source Code
- `control/interface/simple_cli.py` - Active CLI
- `control/commands/robot_controller.py` - Main controller (646 lines)
- `control/commands/*_commands.py` - Command definitions
- `control/ros2_api/process_api.py` - Process management (483 lines)
- `control/ros2_api/movement_api.py` - Robot movement (160 lines)
- `control/ros2_api/calibration_api.py` - Calibration/scripts (118 lines)

### User Data
- `~/.control/config.yaml` - Configuration
- `~/.control/maps/*.{yaml,pgm}` - Map files
- `~/.control/logs/*.log` - Process logs

## What to Work On Next

### High Priority
1. Split robot_controller.py (646 lines, limit is 300)
2. Fix bare except in simple_cli.py

### Medium Priority
1. Remove click_cli.py (deprecated)
2. Add more specific exception handling

### Low Priority
1. Validate map file existence before operations
2. Add health checks for launched processes
