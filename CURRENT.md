# Current State - Control Package

### How to use Current.md

* read claude.md and obey it fully
* read the rest of this document and make it part of your context
* adopt the todo list as your own
* when asked to create a new current.md, create a new file, include your current context, loose ends, todo list, and anything else.
* Also include this section called "How To Use Current.md"

## Status: MAJOR REFACTORING COMPLETE + Map Management System Added

The package has undergone a complete architectural refactoring from a monolithic structure to a clean layered architecture with proper separation of concerns, dependency injection, and unified response handling. Recent work focused on implementing a comprehensive map management system with save/load/list functionality.

### Latest Changes (September 2025 - Map Management Implementation):

1. ✅ **Added map command group** - Complete map management with `map save`, `map list`, and `map load`
2. ✅ **Implemented maps/ folder organization** - All maps automatically saved to and loaded from `maps/` directory
3. ✅ **Enhanced map save functionality** - Auto-creates maps directory, saves with full path display
4. ✅ **Added map list command** - Lists available maps by scanning for .yaml files in maps/ folder
5. ✅ **Added map load command** - Loads maps from maps/ folder with validation
6. ✅ **Updated help system** - Map commands appear in alphabetical order with other commands

### Previous Changes (September 2025 - Help System & Command Cleanup):

1. ✅ **Fixed help command formatting** - Removed inconsistent indentation, commands now start at column 0
2. ✅ **Implemented alphabetical ordering** - Help commands now display in alphabetical order by group
3. ✅ **Removed robot speeds command** - Deleted `robot speeds` command from frontend, backend, command definitions, and tests
4. ✅ **Improved help consistency** - Both general help and specific command group help use consistent 32-character column formatting
5. ✅ **Simplified help generation** - Replaced manual group ordering with automatic alphabetical sorting

### Previous Changes (September 2025 - Configuration and Testing Cleanup):

1. ✅ **Fixed configuration file paths** - Config files now stored in user-writable ~/.config/ directory
2. ✅ **Eliminated duplicate code** - Created DEFAULT_CONFIG constant in ConfigManager to centralize default values
3. ✅ **Removed problematic tests** - Deleted overly complex integration tests and linting tests that enforced unwanted style rules
4. ✅ **Improved help documentation** - Enhanced nav command help text to show actual command syntax
5. ✅ **Added config file auto-creation** - ConfigManager creates default config if file doesn't exist
6. ✅ **Updated CLAUDE.md** - Added preference for double quotes over single quotes

### Major Architectural Changes (January 2025 - Major Refactoring):

1. ✅ **Created BaseApi class** - Shared ROS2 node functionality and utilities
2. ✅ **Split TeleopApi into domain APIs** - MovementApi, CalibrationApi, ProcessApi
3. ✅ **Created RobotController** - Business logic orchestration layer
4. ✅ **Refactored CommandProcessor** - Pure CLI adapter using RobotController
5. ✅ **Implemented CommandResponse** - Unified response format (success/message/data)
6. ✅ **Added ProcessApi** - Subprocess management for launch files and commands
7. ✅ **Added navigation commands** - `nav start stack`, `nav kill stack` with singleton management
8. ✅ **Fixed dependency injection** - Shared ConfigManager across all components
9. ✅ **Code style cleanup** - Removed unnecessary docstrings per coding standards

### Earlier Major Changes:
1. ✅ **Complete package rename** - All files, directories, and imports updated
2. ✅ **Replaced typer with click** - Better command parsing and help generation
3. ✅ **Added variable management system** - Persistent storage with set/show commands
4. ✅ **Simplified main.py** - Direct CLI entry point (removed unused ROS2 node mode)
5. ✅ **Enhanced command structure** - Hierarchical commands with auto-generated help
6. ✅ **Added move time command** - Duration-based movement
7. ✅ **Configuration persistence** - JSON-based config saved to user directory

## Current Architecture (POST-REFACTORING):

### Layered Architecture:
```
CLI Interface (cli_interface.py)
       ↓
ClickCLI (click_cli.py) - Click command parsing and help generation
       ↓
CommandDispatcher (command_dispatcher.py) - Command routing and execution
       ↓
RobotController (robot_controller.py) - Business logic orchestration
       ↓
Domain APIs: MovementApi + CalibrationApi + ProcessApi
       ↓
BaseApi (base_api.py) - Shared ROS2 functionality
       ↓
ROS2 Layer (publishers, subscribers, nodes)
```

### Key Components:

**Entry Points:**
- `control/main.py` - CLI entry point using CliInterface
- `control/__main__.py` - Python module entry point

**CLI Layer:**
- `control/interface/cli_interface.py` - Interactive "> " prompt with command history
- `control/interface/click_cli.py` - Click-based command parsing with help generation

**Command System:**
- `control/commands/command_dispatcher.py` - Routes commands to RobotController methods
- `control/commands/command_def.py` - Command definition structure
- `control/commands/parameter_def.py` - Parameter definition structure
- `control/commands/movement_commands.py` - Movement command definitions
- `control/commands/control_commands.py` - Control command definitions (robot stop, status)
- `control/commands/navigation_commands.py` - Navigation, SLAM, and map command definitions
- `control/commands/system_commands.py` - System command definitions

**Business Logic Layer:**
- `control/commands/robot_controller.py` - Orchestrates all APIs, handles business logic
- `control/commands/config_manager.py` - Configuration management and persistence

**Domain API Layer:**
- `control/ros2_api/base_api.py` - Shared ROS2 node functionality and utilities
- `control/ros2_api/movement_api.py` - Robot movement and velocity control
- `control/ros2_api/calibration_api.py` - Movement patterns and calibration
- `control/ros2_api/process_api.py` - Subprocess management for launch files

**Legacy Layer:**
- `control/teleopapi.py` - Now a facade over new APIs for backward compatibility

**Testing:**
- `test/__init__.py` - Minimal test structure (complex tests removed)
- `test/test_command_dispatcher.py` - Unit tests for command dispatcher

## Implemented Features:

### CLI Commands (Alphabetical Order in Help):
- ✅ **Calibration**: `calibrate square <meters>`
- ✅ **Configuration**: `config set <name> <value>`, `config get <name>`, `config list`
- ✅ **Map Management**: `map save <filename>`, `map load <filename>`, `map list`
- ✅ **Movement**: `move dist <distance>`, `move time <seconds>`
- ✅ **Navigation**: `nav start [--sim-time]`, `nav stop`
- ✅ **Other**: `set <var> <value>`, `get [var]`, `help [command]`, `exit`
- ✅ **Robot**: `robot stop`, `robot status`
- ✅ **SLAM**: `slam start [--sim-time]`, `slam stop`
- ✅ **System**: `system topics`
- ✅ **Turning**: `turn time <seconds>`, `turn radians <angle>`, `turn degrees <angle>`

### Map Management System:
- ✅ **Automatic organization** - All maps stored in `maps/` directory
- ✅ **Map saving** - `map save <filename>` saves to `maps/<filename>.yaml/.pgm`
- ✅ **Map listing** - `map list` shows available maps by scanning for .yaml files
- ✅ **Map loading** - `map load <filename>` loads from `maps/` with validation
- ✅ **Directory auto-creation** - Creates `maps/` folder if it doesn't exist
- ✅ **Path validation** - Checks if requested map exists before loading

### Help System:
- ✅ **Consistent formatting** - All commands use 32-character column width, no indentation
- ✅ **Alphabetical ordering** - Commands grouped and sorted alphabetically by group name
- ✅ **General help** - `help` shows all commands in logical order
- ✅ **Specific help** - `help <group>` shows commands for specific group (e.g., `help map`)
- ✅ **Parameter display** - Shows required arguments and optional flags

### Process Management:
- ✅ **Subprocess launching** - Shell commands, ROS2 nodes, launch files
- ✅ **Process tracking** - UUID-based process management
- ✅ **Output capture** - Real-time background thread collection
- ✅ **Process killing** - SIGTERM → SIGKILL fallback with process groups
- ✅ **Singleton management** - Navigation stack and SLAM auto-kill previous instances
- ✅ **Map server integration** - Launches map_server for loading maps

### Configuration System:
- ✅ **Type conversion** - Automatic int, float, bool, string detection
- ✅ **Persistence** - Config files stored in ~/.config/ directory
- ✅ **Auto-creation** - Default config created if file doesn't exist
- ✅ **Shared state** - Single ConfigManager instance across all components
- ✅ **DRY compliance** - Centralized default values in DEFAULT_CONFIG

### ROS2 Integration:
- ✅ **Movement control** - /cmd_vel publishing with safety limits
- ✅ **Odometry** - /odom subscription for position tracking
- ✅ **Topic listing** - Live ROS2 graph querying
- ✅ **Node lifecycle** - Proper initialization and cleanup
- ✅ **Navigation integration** - nav2_bringup navigation_launch.py
- ✅ **SLAM integration** - slam_toolbox online_async_launch.py
- ✅ **Map server integration** - nav2_map_server for save/load operations

## Current File Locations:

### Configuration Files:
- **Config file**: `~/.config/control_config.json` (user-writable location)
- **Command history**: `~/.config/control_command_history.txt` (user-writable location)
- **Default speeds**: linear_speed=0.3, angular_speed=0.4 (defined in ConfigManager.DEFAULT_CONFIG)

### Map Storage:
- **Maps directory**: `./maps/` (auto-created in current working directory)
- **Map files**: `maps/<name>.yaml` (metadata) and `maps/<name>.pgm` (image data)

### Source Code Organization:
```
control/
├── __init__.py
├── __main__.py              # Python module entry point
├── main.py                  # Main CLI entry point
├── interface/               # CLI interface layer
│   ├── __init__.py
│   ├── cli_interface.py     # Interactive prompt with history
│   └── click_cli.py         # Click command definitions and help
├── commands/                # Command system and business logic
│   ├── __init__.py
│   ├── command_def.py       # Command definition structure
│   ├── parameter_def.py     # Parameter definition structure
│   ├── command_dispatcher.py # Command routing and execution
│   ├── robot_controller.py  # Business logic orchestration
│   ├── config_manager.py    # Configuration management
│   ├── movement_commands.py # Movement command definitions
│   ├── control_commands.py  # Control command definitions
│   ├── navigation_commands.py # Navigation, SLAM, and map commands
│   └── system_commands.py   # System command definitions
└── ros2_api/               # ROS2 integration layer
    ├── __init__.py
    ├── base_api.py         # Shared ROS2 functionality
    ├── movement_api.py     # Movement control
    ├── calibration_api.py  # Calibration patterns
    └── process_api.py      # Subprocess management
```

## Key Architectural Decisions:

### 1. Dependency Injection Pattern:
```python
# Shared ConfigManager prevents state separation
config_manager = ConfigManager()
robot_controller = RobotController(config_manager)  # Required injection
command_dispatcher = CommandDispatcher(robot_controller)
click_cli = ClickCLI()  # Creates and shares config through dispatcher
```

### 2. Unified Response Format:
```python
@dataclass
class CommandResponse:
    success: bool
    message: str
    data: Any = None
```

### 3. Process Singleton Management:
```python
# RobotController tracks navigation stack and SLAM process IDs
self.nav_stack_process_id = None  # Only one nav stack can run
self.slam_process_id = None       # Only one SLAM can run
```

### 4. Configuration Management:
```python
# Centralized default values prevent duplication
DEFAULT_CONFIG = {
    "linear_speed": 0.3,
    "angular_speed": 0.4
}
```

### 5. Help System Design:
```python
# Alphabetical ordering with consistent formatting
for group_name in sorted(groups.keys()):
    for cmd, desc in groups[group_name]:
        help_text += f"{cmd:<32} - {desc}\n"
```

### 6. Map Management Pattern:
```python
# Automatic maps/ directory organization
maps_dir = Path("maps")
maps_dir.mkdir(exist_ok=True)
map_path = maps_dir / filename
```

## Current Issues/Blockers:

### 1. ROS2 Node Architecture Issue:
**Problem**: Each API creates its own ROS2 node, causing potential conflicts:
- Multiple `rclpy.init()` calls
- Multiple nodes publishing to same topics
- Spinning conflicts

**Solution Needed**: Implement shared node pattern where RobotController creates single node and injects it into APIs.

### 2. Missing NavigationApi:
**Status**: Planned but not implemented
**Purpose**: Handle navigation-specific ROS2 operations (goal setting, path planning, costmaps)

### 3. Limited Process Commands:
**Current**: Only `nav start/stop`, `slam start/stop`, and map operations
**Missing**: Generic process management (`list processes`, `kill <process_id>`, `show output <process_id>`)

### 4. Testing Infrastructure:
**Status**: Minimal tests remain after cleanup
**Needed**: Simple unit tests for individual components
**Approach**: Use fakes/stubs instead of complex mocking

## Todo List (Priority Order):

1. **[pending]** Add exit cleanup to detect and optionally kill running processes
2. **[pending]** Update setup.py with any new dependencies
3. **[pending]** Implement shared node pattern in RobotController
4. **[pending]** Add process management commands (list, kill by ID)
5. **[pending]** Create NavigationApi class for navigation operations
6. **[pending]** Add more nav commands (save map, start slam)
7. **[pending]** Create simple unit tests for core components

## How to Run the Control Tool:

### Direct execution:
```bash
cd /home/pitosalas/ros2_ws/src/control
python3 -m control.main
```

### As ROS2 package:
```bash
cd /home/pitosalas/ros2_ws
colcon build --packages-select control
source install/setup.bash
ros2 run control control
```

### Command Examples:
- `help` - Show available commands in alphabetical order
- `help map` - Show map management commands
- `move dist 1.0` - Move robot 1 meter
- `nav start --sim-time` - Start navigation stack with simulation time
- `slam start` - Start SLAM
- `map save my_map` - Save current map to maps/my_map
- `map list` - List available maps
- `map load my_map` - Load maps/my_map into map server
- `config set linear_speed 0.5` - Change movement speed
- `system topics` - List ROS topics
- `robot status` - Show robot and process status
- `exit` - Quit program

## Dependencies:
- `click` - Command line interface framework
- `prompt_toolkit` - Command history and input enhancement
- `rclpy` - ROS2 Python client library
- Standard library: `json`, `pathlib`, `typing`, `dataclasses`, `subprocess`, `threading`

## Testing Strategy (Future):
- **Unit tests** - Test individual components with simple fakes
- **Component tests** - Test CLI command parsing without ROS2
- **Acceptance tests** - Test actual CLI output for correctness
- **Avoid complex mocking** - Use test doubles for external dependencies

## For Future REST API Integration:

The refactored architecture enables easy REST API addition:

```python
# FastAPI server would use same RobotController
from fastapi import FastAPI
app = FastAPI()
controller = RobotController(config_manager)

@app.post("/api/map/save")
async def save_map(request: SaveMapRequest):
    return controller.save_current_map(request.filename)

@app.get("/api/maps")
async def list_maps():
    return controller.list_maps()
```

**Benefits of Current Architecture:**
- **Shared business logic** - CLI and REST use same RobotController
- **Unified responses** - CommandResponse format works for both
- **Proper separation** - No business logic in CLI layer
- **Testable** - Mock RobotController for API testing

## Recent Implementation Work:

### Map Management System:
- Implemented complete map workflow: save → list → load
- Added automatic maps/ directory organization with auto-creation
- Enhanced map save to display full path and create directory structure
- Added map list functionality that scans for .yaml files and returns sorted names
- Added map load with validation and proper ROS2 map_server integration
- Updated help system to show all map commands in alphabetical order

### Process Management Enhancement:
- Added SLAM process tracking with singleton management (auto-kill previous SLAM)
- Enhanced kill_process method to clear both nav_stack_process_id and slam_process_id
- Updated robot status to include both navigation and SLAM running status
- Added map server process launching for map loading operations

### Command Structure Improvements:
- All map commands grouped under single `map` command group
- Consistent help text showing "maps/ folder" organization
- Parameter validation for map loading (checks file existence)
- Enhanced command definitions with proper descriptions and grouping

## Coding Standards Applied:
- Functions/methods max 50 lines
- Files max ~300 lines
- Classes in separate files named after the class
- Avoid nested if/else > 1 level
- DRY principle applied (eliminated config duplication)
- No unnecessary docstrings (only when function name insufficient)
- Required dependency injection (no optional defaults that create hidden instances)
- Prefer async/await over threading
- **Double quotes preferred** (consistently applied)
- Incremental development (program remains functional after each step)

## Git Branch Status:
- **Current branch**: `refactor`
- **Status**: Major refactoring complete, map management system implemented, ready for next features
- **Main changes**: Architecture separation, configuration management, help improvements, command cleanup, map management

## Context for AI Assistant:
- This is a ROS2 control package for robot teleoperation
- Major refactoring completed: monolithic → layered architecture
- Recent session focused on implementing comprehensive map management system
- Focus on clean separation of concerns and testability
- Follow coding standards in CLAUDE.md strictly (including double quote preference)
- Current work is on `refactor` branch
- Ready for next phase: exit cleanup, shared node pattern, simple testing, and extended process management
- Configuration files now properly located in user-writable ~/.config/ directory
- Help system now displays commands in consistent alphabetical order with proper formatting
- Map management system provides complete save/load/list functionality with automatic organization
- Process tracking includes navigation stack, SLAM, and map server operations