# Complete Process Management Refactor Design

## Overview
Refactor the process management system from individual command-based operations to a unified launch file management system with separate map operations.

## New Command Structure

### **Launch Commands**
- `launch list` - Show all available launch types and their descriptions
- `launch start <type>` - Start a specific launch file type
- `launch kill <type>` - Stop a specific launch file type
- `launch status` - Show status of all launch types (running/stopped, PIDs, uptime)
- `launch status <type>` - Show detailed status of a specific launch type

### **Map Commands (Separate)**
- `map save <filename>` - Command running map_server to save current map
- `map load <filename>` - Command running map_server to load/publish specific map
- `map list` - List available saved maps

## Static Launch Configuration Table

### **Three Launch Types:**
1. **`nav`**
   - Command: `ros2 launch nav2_bringup navigation_launch.py`
   - Description: "Navigation stack with path planning and obstacle avoidance"

2. **`slam`**
   - Command: `ros2 launch slam_toolbox online_async_launch.py`
   - Description: "SLAM mapping and localization"

3. **`map_server`**
   - Command: `ros2 run nav2_map_server map_server`
   - Description: "Map server for loading and saving maps"

## Architecture Changes

### **ProcessApi Refactor**
- Replace individual methods (`start_navigation`, `start_slam`, etc.) with:
  - `launch_by_type(launch_type, **params)` - Start launch by type name
  - `kill_by_type(launch_type)` - Stop launch by type name
  - `get_launch_status(launch_type=None)` - Get status of one or all launches
  - `get_available_types()` - Return list of available launch types

### **RobotController Updates**
- Replace process-specific methods with:
  - `launch_start(launch_type, **kwargs)`
  - `launch_kill(launch_type)`
  - `launch_status(launch_type=None)`
  - `launch_list()`
- Keep existing map operations but modify them to communicate with running map_server

### **Command Definitions**
- Remove: `nav.*`, `slam.*` commands
- Add: `launch.*` command group
- Keep: `map.*` commands but modify behavior

### **CLI Interface**
- Add new `launch` command group with subcommands
- Remove individual `nav` and `slam` groups
- Update `map` commands to work with persistent map_server

## Map Server Workflow

### **Persistent Service Model**
1. `launch start map_server` - Starts empty map server (ready to receive commands)
2. `map load my_map` - Map server starts publishing the specified map
3. `map save new_map` - Map server saves current map to file (doesn't kill server)
4. `launch kill map_server` - Stops the map server entirely

### **Map Command Behavior Changes**
- `map save` - Sends save command to running map_server (no separate process)
- `map load` - Sends load command to running map_server (no separate process)
- Both require map_server to be running via `launch start map_server`

## Process Tracking Changes

### **Unified Tracking**
- Track only the 3 launch types: nav, slam, map_server
- Remove separate tracking for map_save/map_load operations
- Each launch type is singleton (only one instance can run)

### **Status Display**
- `launch status` shows all launch types with running status, PIDs, uptime
- `launch status <type>` shows detailed info for specific type
- `robot status` continues to show process summary but uses new launch tracking

## Benefits

### **Consistency**
- All launch operations use same interface pattern
- Clear separation between launch management and map operations
- Predictable command structure

### **Extensibility**
- Easy to add new launch types to static table
- Consistent parameter handling across all launch types
- Single place to modify launch commands

### **Clarity**
- Map server is clearly a persistent service
- Map operations are clearly commands sent to the service
- Launch operations are clearly about process lifecycle

## Implementation Todo List

1. **Create static launch configuration table with 3 launch types**
2. **Refactor ProcessApi to use launch-type-based methods**
3. **Update RobotController launch methods to use new ProcessApi**
4. **Remove old nav/slam command definitions**
5. **Create new launch command definitions (list, start, kill, status)**
6. **Update CLI interface to add launch command group**
7. **Remove old nav/slam CLI command groups**
8. **Modify map commands to work with persistent map_server**
9. **Update process tracking to use 3 launch types only**
10. **Update robot status to use new launch tracking**
11. **Test complete refactored system**
12. **Update help documentation for new command structure**

## Example Usage

### **Current Commands (Old)**
```bash
nav start --sim-time
slam start
map save my_map
nav stop
slam stop
```

### **New Commands (After Refactor)**
```bash
launch list                    # Show available launch types
launch start map_server        # Start map server
launch start nav --sim-time    # Start navigation
launch status                  # Show all launch status
map save my_map               # Save map via running map_server
map load my_map               # Load map via running map_server
launch kill nav               # Stop navigation
launch kill map_server        # Stop map server
```

## Key Design Principles

1. **Single Responsibility**: Launch commands manage processes, map commands manage map data
2. **Persistent Services**: Map server runs continuously, responds to map commands
3. **Static Configuration**: Launch types defined in code table, easy to extend
4. **Consistent Interface**: All launch operations follow same pattern
5. **Clear Status**: Always know what's running and its state
6. **No Auto-killing**: Commands fail with clear messages if conflicts exist