# Current Status - ROS2 Control Package

## Recent Changes Completed
- ✅ Removed `map.load` command from navigation commands
- ✅ Added `map_name` parameter to `launch.start` command
- ✅ Updated map_server launch config to use yaml_filename parameter
- ✅ Enhanced ProcessApi to handle map file loading at launch time
- ✅ Cleaned up unused map load service client and methods

## Current Todo List
- [ ] Fix bare except statements in process_api.py (lines 305, 333, 361, 406) for proper error handling
- [ ] Test new map_name parameter functionality with launch start map_server

## Known Issues & Bugs

### High Priority
1. **Bare Exception Handling**: process_api.py has several bare `except:` statements that should specify exception types
2. **Map Server Lifecycle**: The map server starts in "unconfigured" state and doesn't automatically activate, causing service calls to fail

### Medium Priority
1. **Process Tree Tracking**: Control package tracks shell wrapper PID (58320) while other tools show actual node PID (58325)
2. **Service Timeout**: Map service calls use 2-second timeout which may be too short

## Architecture Notes

### Launch System
- Uses unified launch command system with singleton process tracking
- Three launch types: `nav`, `slam`, `map_server`
- Process management uses shell=True for parameter flexibility
- Creates process hierarchy: shell → ros2 command → actual node

### Map Management
- **OLD**: Used service calls to load maps after map_server was running
- **NEW**: Launch map_server with specific map file using yaml_filename parameter
- Maps stored in `maps/` directory with .yaml and .pgm files
- Map save still uses service calls (working)

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
launch start map_server map_name=map5

# Check status
launch status map_server

# Stop map server
launch kill map_server
```

### Other Launch Commands
```bash
launch list                    # Show available launch types
launch start slam              # Start SLAM
launch start nav               # Start navigation
launch status                  # Show all launch status
```

## File Locations
- Launch commands: `control/commands/launch_commands.py`
- Navigation commands: `control/commands/navigation_commands.py`
- Process API: `control/ros2_api/process_api.py`
- Robot controller: `control/commands/robot_controller.py`
- CLI interface: `control/interface/click_cli.py`