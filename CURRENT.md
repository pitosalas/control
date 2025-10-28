# Current Status - ROS2 Control Package

## Recent Changes Completed
- ✅ Removed `map.load` command from navigation commands
- ✅ Added `map_name` parameter to `launch.start` command
- ✅ Updated map_server launch config to use yaml_filename parameter
- ✅ Enhanced ProcessApi to handle map file loading at launch time
- ✅ Cleaned up unused map load service client and methods
- ✅ Fixed command syntax documentation (map_server → map, map_name=x → --map-name x)
- ✅ Added automatic log file generation for all launch processes (stored in logs/ directory)
- ✅ Updated launch status to display log file locations

## Current Todo List
- [ ] Fix bare except statements in process_api.py for proper error handling
- [ ] Test new map_name parameter functionality with: launch start map --map-name <mapname>

## Known Issues & Bugs

### High Priority
1. **Bare Exception Handling**: process_api.py has several bare `except:` statements that should specify exception types
2. **Map Server Lifecycle**: The map server starts in "unconfigured" state and doesn't automatically activate, causing service calls to fail

### Medium Priority
1. **Process Tree Tracking**: Control package tracks shell wrapper PID (58320) while other tools show actual node PID (58325)
2. **Service Timeout**: Map service calls use 2-second timeout which may be too short

## Architecture Notes

### Path Resolution (ROS2 Standard)
- **User Data Directory**: `~/.control/` - All user-writable data (follows ROS2 best practices)
- **Maps Directory**: Configurable via `maps_dir` config (default: `~/.control/maps/`)
- **Logs Directory**: Configurable via `log_dir` config (default: `~/.control/logs/`)
- **Config File**: `~/.control/config.yaml` - persistent configuration
- **Behavior**:
  - Relative paths (e.g., `logs`) → resolve to `~/.control/` directory (`~/.control/logs/`)
  - Absolute paths (e.g., `/var/log/ros2`) → used as-is
  - Home paths (e.g., `~/ros2_logs`) → expanded and used as-is
- **Benefits**:
  - Works from any directory (can run `ros2 run control run` from anywhere)
  - No hardcoded user paths, portable across machines
  - Follows ROS2 conventions for user data
  - Survives package rebuilds

### Launch System
- Uses unified launch command system with singleton process tracking
- Three launch types: `nav`, `slam`, `map_server`
- Process management uses shell=True for parameter flexibility
- Creates process hierarchy: shell → ros2 command → actual node

### Map Management
- **OLD**: Used service calls to load maps after map_server was running
- **NEW**: Launch map_server with specific map file using yaml_filename parameter
- Maps stored in `~/.control/maps/` directory with .yaml and .pgm files
- Map save still uses service calls (working)
- All map operations use `~/.control/` paths (portable, no hardcoded user paths)

### Process Logging
- All launch processes (nav, slam, map) automatically log output to files
- Log files stored in configurable directory (default: `~/.control/logs/`) with format: `<launch_type>_<timestamp>.log`
- Includes header with command, PID, and start time
- Real-time output capture to both memory and log file
- Log file path shown in `launch status` command output
- Useful for debugging issues like map loading errors or navigation problems
- **Configuration**: Use `config set log_dir <path>` to change log directory (supports ~ for home directory)
  - Default: `logs` (relative to `~/.control/` directory)
  - Relative paths resolve to `~/.control/` (e.g., `logs` → `~/.control/logs/`)
  - Absolute paths and `~/` paths work as expected
  - Example: `config set log_dir ~/ros2_logs` or `config set log_dir /var/log/ros2`
  - Settings saved to `~/.control/config.yaml` and persist across sessions

## What to Work On Next

### Immediate (High Priority)
1. Fix the bare except statements in process_api.py for better error handling
2. Test the new `launch start map_server map_name=mapX` functionality
3. Verify map server starts with correct map loaded

### Short Term
1. Consider using proper launch files instead of ros2 run for better lifecycle management
2. Add validation for map file existence before launching
3. Improve error messages for missing map files

### Medium Term
1. Investigate if we should track actual node PIDs instead of shell wrapper PIDs
2. Add health checks for launched processes
3. Consider adding map server lifecycle management if needed

## Usage Examples

### New Map Loading (Current Approach)
```bash
# Start map server with specific map
launch start map --map-name map5

# Check status (shows PID and log file location)
launch status map

# View log file (shown in status output)
cat logs/map_<timestamp>.log

# Stop map server
launch kill map
```

### Other Launch Commands
```bash
launch list                    # Show available launch types
launch start slam              # Start SLAM
launch start nav               # Start navigation
launch status                  # Show all launch status
```

### Configuration Examples
```bash
# View current log directory setting
config get log_dir

# Change log directory to home directory
config set log_dir ~/ros2_logs

# Change to absolute path
config set log_dir /var/log/ros2

# View all configuration
config list
```

## File Locations

### Source Code
- Launch commands: `control/commands/launch_commands.py`
- Navigation commands: `control/commands/navigation_commands.py`
- Process API: `control/ros2_api/process_api.py`
- Robot controller: `control/commands/robot_controller.py`
- CLI interface: `control/interface/click_cli.py`

### User Data (Runtime)
- **Configuration**: `~/.control/config.yaml` - persistent config settings
- **Maps**: `~/.control/maps/` - saved map files (.yaml, .pgm) (configurable)
- **Logs**: `~/.control/logs/` - launch process logs (configurable)


## New features and bug fixes

- check that i can enter negative numbers for turn and move
- allow all commands to be abbreviated to three letters (maybe just make them three letters)
- change the way launch works to have a series of "canned" or parameterized launch commands that come directly from the config
- change the way the map command works to simply include commands like map save to save both kinds of maps
- show me how to try the eew parser to see if I like it
- no need to have "default defaults" ... if its not in the config than give an error
- create a new script "circle_stress" that takes one param which is the diameter of the circle
- rename old one to "rotate_stress"
- remove the attempt to print out voltage
- when the command first starts capture the time and print elapsed time so far every 10
- create a new command "do" that allows me to choose from a preset list of
 ros2 commands. Each command will have a template which might have a variable in it, and it will have a name. So for example:

 nav -> "ros2 launch linorobot2_navigation slam rviz:={rviz}"

 with the command

 do nav rviz true

 