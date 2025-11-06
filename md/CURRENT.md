# Current Status - ROS2 Control Package

## Overview
ROS2 control package with CLI interface for robot movement, navigation, mapping, and system management.

## Recent Session (Latest)
**Map Save Refactoring - Complete**
- ✅ Fixed `map_save()` attribute error (self.process_api → self.process)
- ✅ Added `run_command_sync()` method to ProcessApi for synchronous command execution with logging
- ✅ Refactored `map_save()` to use ProcessApi.run_command_sync() instead of direct subprocess
- ✅ Map save operations now create log files: `~/.control/logs/map_save_<timestamp>.log`
- ✅ Removed orphaned `save_current_map()` method (old service-based approach)
- ✅ Removed `save_map_via_service()` from ProcessApi (no longer used)
- ✅ Removed SaveMap service client import from ProcessApi

**Current Implementation:**
- `map.save` command uses `ros2 run nav2_map_server map_saver_cli` (one-shot CLI approach)
- No longer requires persistent map_server to be running
- Automatic logging to `~/.control/logs/`
- 15 second timeout with proper error handling

## Architecture

### CLI & Command System
- **Active CLI**: `control/interface/simple_cli.py` - SimpleCommandParser-based (REPL + non-interactive)
- **Parser**: `control/interface/simple_parser.py` - Custom command parser
- **Dispatcher**: `control/commands/command_dispatcher.py` - Framework-agnostic command registry
- **Deprecated**: `control/interface/click_cli.py` - Old Click-based CLI (ready for removal)

### Command Organization
Commands are defined in separate files by category:
- `movement_commands.py` - Robot movement (move, turn)
- `navigation_commands.py` - Map operations (map.save, map.list)
- `launch_commands.py` - Process launching (launch.start, launch.stop, launch.list)
- `system_commands.py` - System utilities (config, scripts, process management)
- `control_commands.py` - Robot control (status, stop)

All commands route through `RobotController` in `robot_controller.py`

### Launch System
**Dynamic Configuration:**
- Launch types loaded from `~/.control/config.yaml` under `launch_templates`
- Currently configured: `nav`, `slam` (map launch removed from templates)
- Each template specifies: name, command, description, default_params
- Singleton process tracking per launch type

**Process Management:**
- Uses `ProcessApi` for unified process handling
- Creates process groups for clean shutdown (SIGTERM → SIGKILL)
- Automatic log file generation: `<launch_type>_<timestamp>.log`
- Tracks PIDs and provides status information

### Map Management
**Current Approach (CLI-based):**
- `map.save [map_name]` - Uses `map_saver_cli` (one-shot command)
  - Doesn't require running map_server
  - Saves to `~/.control/maps/<name>.yaml` and `.pgm`
  - Creates log file for debugging
  - 15 second timeout

- `map.list` - Lists available maps in `~/.control/maps/`

**Map Loading (for navigation):**
- Use `launch.start` with map parameter (e.g., `launch start nav --map basement`)
- Map server can be launched with specific map: `launch start map --map-name <name>`
- Maps resolve to `~/.control/maps/<name>.yaml`

### Path Resolution
**Standard Locations:**
- `~/.control/` - All user-writable data (ROS2 best practice)
- `~/.control/config.yaml` - Persistent configuration
- `~/.control/maps/` - Map files (.yaml, .pgm)
- `~/.control/logs/` - Process and command logs (configurable)
- `~/.control/command_history.txt` - Timestamped command log
- `~/.control/prompt_history.txt` - Prompt toolkit history

**Path Behavior:**
- Relative paths → resolve to `~/.control/` (e.g., `logs` → `~/.control/logs/`)
- Absolute paths → used as-is (e.g., `/var/log/ros2`)
- Home paths → expanded (e.g., `~/ros2_logs`)
- Portable across machines, survives package rebuilds
- Works from any directory

### Configuration Management
- Config file: `~/.control/config.yaml`
- Variables: linear_speed, angular_speed, log_dir, map_name, control_dir, etc.
- Launch templates dynamically loaded from config
- Commands: `config.get`, `config.set`, `config.list`

## Current State

### Working Features
✅ CLI with SimpleCommandParser (REPL and non-interactive modes)
✅ Dynamic launch system from config templates
✅ Map save using CLI tool (map_saver_cli)
✅ Automatic process logging to files
✅ Centralized config in `~/.control/`
✅ Command history with timestamps
✅ Movement commands (move, turn, stop)
✅ Script commands (square, rotate_stress, circle_stress)
✅ System commands (topics, processes, kill)

### Known Issues

**Code Quality:**
1. Bare `except Exception as e:` in simple_cli.py line 220 (violates RULES.md)
2. Exception handling in process_api.py could be more specific

**Architecture:**
1. robot_controller.py has some unused helper methods (see below)
2. Launch types hard-coded in some places vs dynamic from config

### Unused/Helper Methods in robot_controller.py
These methods exist but aren't registered as commands. They may be useful for future features or REST API:
- `start_slam(use_sim_time, **kwargs)` - Wrapper for launch_start with slam
- `launch_file(package, launch_file, **kwargs)` - Generic launch file runner
- `run_ros_node(package, executable, **kwargs)` - Generic node runner
- `launch_command(command)` - Generic command launcher
- `get_active_processes()` - Get running processes
- `get_process_output(process_id, lines)` - Get output from process
- `move_continuous(linear, angular)` - Continuous movement
- `get_current_position()` - Get robot position
- `turn_by_radians(radians)`, `turn_by_degrees(degrees)` - Alternative turn methods
- `get_topics()` - Get topics (alternative to list_topics)

These are kept as internal utilities but could be exposed as commands or removed if truly unused.

## What to Work On Next

### High Priority
1. **Test map.save command** - Verify it works with new ProcessApi infrastructure
2. **Consider robot_controller.py organization** - File is 670 lines, mixes concerns (movement, launch, config, maps, system). Consider splitting into focused controllers or at minimum document the sections.

### Medium Priority
1. **Click removal completion** - Remove click_cli.py and Click dependency entirely
2. **Exception handling** - Fix bare except statements for better error messages
3. **Map loading workflow** - Document/test map parameter with nav launch

### Low Priority / Future
1. Validate map file existence before operations
2. Better error messages for missing maps
3. Consider tracking actual node PIDs vs shell wrapper PIDs
4. Add health checks for launched processes

## Usage Examples

### Map Operations
```bash
# Save current map (uses config map_name or defaults to 'basement')
map save

# Save with specific name
map save mymap

# List available maps
map list

# Check log if there are issues
cat ~/.control/logs/map_save_<timestamp>.log
```

### Launch Operations
```bash
# List available launch types
launch list

# Show details about a launch type
launch info nav

# Start navigation (with optional map)
launch start nav --map basement

# Start SLAM
launch start slam

# Stop a launch
launch stop nav
```

### Configuration
```bash
# View all config
config list

# Get specific variable
config get map_name

# Set variable
config set map_name mymap

# Change log directory
config set log_dir ~/ros2_logs
```

### Robot Movement
```bash
# Move distance (meters)
move forward 1.0
move backward 0.5

# Turn (degrees)
turn clockwise 90
turn counterclockwise 45

# Stop
stop
```

### Scripts
```bash
# List available scripts
script list

# Run square pattern
script square 1.0

# Run stress tests
script rotate_stress
script circle_stress 2.0
```

### System Commands
```bash
# List ROS topics
system topics

# List ROS processes
system ps

# List launch processes
system launches

# Kill process by PID
system kill <pid>
```

## File Locations

### Source Code (control package)
- `control/interface/simple_cli.py` - Active CLI
- `control/interface/simple_parser.py` - Command parser
- `control/commands/command_dispatcher.py` - Command registry
- `control/commands/robot_controller.py` - Main controller (670 lines)
- `control/commands/*_commands.py` - Command definitions
- `control/ros2_api/process_api.py` - Process management
- `control/ros2_api/movement_api.py` - Robot movement
- `control/ros2_api/calibration_api.py` - Calibration and scripts
- `control/commands/config_manager.py` - Configuration

### User Data (Runtime)
- `~/.control/config.yaml` - Configuration
- `~/.control/maps/*.{yaml,pgm}` - Map files
- `~/.control/logs/*.log` - Process logs
- `~/.control/command_history.txt` - Command history
- `~/.control/prompt_history.txt` - Prompt history

## Notes for Future Development

### Considered but Deferred
- Moving map operations to separate MapApi class - Deferred for simplicity
- Splitting robot_controller.py - Large but functional, split if it becomes problematic
- Removing unused helper methods - Kept for potential REST API or future commands

### Recently Completed
- Map save now uses CLI tool instead of service calls
- Removed service-based map loading (now done at launch time)
- Centralized all user data in ~/.control/
- Dynamic launch system from config
