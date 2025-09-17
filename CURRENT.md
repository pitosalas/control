# Current State - Control Package

### How to use Current.md

* read claude.md and obey it fully
* read the rest of this document and make it part of your context
* adopt the todo list as your own
* when asked to create a new current.md, create a new file, include your current context, loose ends, todo list, and anything else.
* Also include this section called "How To Use Current.md"

## Status: MAJOR REFACTORING COMPLETE + Configuration and Testing Cleanup

The package has undergone a complete architectural refactoring from a monolithic structure to a clean layered architecture with proper separation of concerns, dependency injection, and unified response handling. Recent work focused on configuration file management, test suite cleanup, and code quality improvements.

### Latest Changes (September 2025 - Configuration and Testing Cleanup):

1. ✅ **Fixed configuration file paths** - Config files now stored in user-writable ~/.config/ directory
2. ✅ **Eliminated duplicate code** - Created DEFAULT_CONFIG constant in ConfigManager to centralize default values
3. ✅ **Removed problematic tests** - Deleted overly complex integration tests and linting tests that enforced unwanted style rules
4. ✅ **Improved help documentation** - Enhanced nav command help text to show actual command syntax
5. ✅ **Added config file auto-creation** - ConfigManager creates default config if file doesn't exist
6. ✅ **Updated CLAUDE.md** - Added preference for double quotes over single quotes

### Previous Major Changes (January 2025 - Major Refactoring):

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
CommandProcessor (command_processor.py) - Click command parsing
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

**CLI Layer:**
- `control/cli_interface.py` - Interactive "> " prompt with command history
- `control/command_processor.py` - Click-based command parsing (thin adapter)

**Business Logic Layer:**
- `control/robot_controller.py` - Orchestrates all APIs, handles business logic
- `control/config_manager.py` - Configuration management and persistence

**Domain API Layer:**
- `control/base_api.py` - Shared ROS2 node functionality and utilities
- `control/movement_api.py` - Robot movement and velocity control
- `control/calibration_api.py` - Movement patterns and calibration
- `control/process_api.py` - Subprocess management for launch files

**Legacy Layer:**
- `control/teleopapi.py` - Now a facade over new APIs for backward compatibility

**Testing:**
- `test/__init__.py` - Minimal test structure (complex tests removed)

## Implemented Features:

### CLI Commands:
- ✅ **Movement**: `move dist <distance>`, `move time <seconds>`
- ✅ **Turning**: `turn time <seconds>`, `turn radians <angle>`, `turn degrees <angle>`
- ✅ **Control**: `stop`
- ✅ **Calibration**: `calibrate square <meters>`
- ✅ **Navigation**: `nav start stack [--use-sim-time]`, `nav kill stack`
- ✅ **Variables**: `set <var> <value>`, `show [var]`, `show *`
- ✅ **System**: `show topics`, `help [command]`, `exit`

### Process Management:
- ✅ **Subprocess launching** - Shell commands, ROS2 nodes, launch files
- ✅ **Process tracking** - UUID-based process management
- ✅ **Output capture** - Real-time background thread collection
- ✅ **Process killing** - SIGTERM → SIGKILL fallback with process groups
- ✅ **Singleton management** - Navigation stack auto-kill previous instance

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

## Current File Locations:

### Configuration Files:
- **Config file**: `~/.config/control_config.json` (user-writable location)
- **Command history**: `~/.config/control_command_history.txt` (user-writable location)
- **Default speeds**: linear_speed=0.3, angular_speed=0.4 (defined in ConfigManager.DEFAULT_CONFIG)

### Source Code Organization:
- **Package root**: `/home/pitosalas/ros2_ws/src/control/`
- **Main package**: `control/` directory with all Python modules
- **Legacy config**: `control_config.json` in package root (no longer used)

## Key Architectural Decisions:

### 1. Dependency Injection Pattern:
```python
# Shared ConfigManager prevents state separation
config_manager = ConfigManager()
robot_controller = RobotController(config_manager)  # Required injection
command_processor = CommandProcessor()  # Creates and shares config
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
# RobotController tracks navigation stack process ID
self.nav_stack_process_id = None  # Only one nav stack can run
```

### 4. Configuration Management:
```python
# Centralized default values prevent duplication
DEFAULT_CONFIG = {
    "linear_speed": 0.3,
    "angular_speed": 0.4
}
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
**Current**: Only `nav start/kill stack`
**Missing**: Generic process management (`list processes`, `kill <process_id>`, `show output <process_id>`)

### 4. Testing Infrastructure:
**Status**: All tests removed due to complexity
**Needed**: Simple unit tests for individual components
**Approach**: Use fakes/stubs instead of complex mocking

## Todo List (Priority Order):

1. **[pending]** Update setup.py with any new dependencies
2. **[pending]** Implement shared node pattern in RobotController
3. **[pending]** Add process management commands (list, kill by ID)
4. **[pending]** Create NavigationApi class for navigation operations
5. **[pending]** Add more nav commands (save map, start slam)
6. **[pending]** Create simple unit tests for core components

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
- `help` - Show available commands
- `move dist 1.0` - Move robot 1 meter
- `nav start stack` - Start navigation stack
- `nav start stack --use-sim-time` - Start with simulation time
- `nav kill stack` - Stop navigation stack
- `set linear_speed 0.5` - Change movement speed
- `show topics` - List ROS topics
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

@app.post("/api/move/dist")
async def move_distance(request: MoveDistRequest):
    return controller.move_distance(request.distance)
```

**Benefits of Current Architecture:**
- **Shared business logic** - CLI and REST use same RobotController
- **Unified responses** - CommandResponse format works for both
- **Proper separation** - No business logic in CLI layer
- **Testable** - Mock RobotController for API testing

## Recent Cleanup Work:

### Configuration Management:
- Fixed config file paths to use ~/.config/ directory (user-writable)
- Added auto-creation of default config file
- Eliminated duplicate default values using DEFAULT_CONFIG constant
- Both config and history files now in consistent location

### Testing Cleanup:
- Removed complex integration tests with brittle ROS2/subprocess mocking
- Removed ament-flake8 and ament-pep257 tests that enforced unwanted style rules
- Cleaned up test directory to minimal structure
- Simplified testing approach for future implementation

### Code Quality:
- Improved help documentation for nav commands
- Applied DRY principle to eliminate duplicate default values
- Enhanced docstrings to show actual command syntax
- Followed coding standards from CLAUDE.md consistently

## Coding Standards Applied:
- Functions/methods max 50 lines
- Files max ~300 lines
- Classes in separate files named after the class
- Avoid nested if/else > 1 level
- DRY principle applied (eliminated config duplication)
- No unnecessary docstrings (only when function name insufficient)
- Required dependency injection (no optional defaults that create hidden instances)
- Prefer async/await over threading
- **Double quotes preferred** (added to CLAUDE.md)
- Incremental development (program remains functional after each step)

## Git Branch Status:
- **Current branch**: `refactor`
- **Status**: Major refactoring complete, config cleanup complete, ready for next features
- **Main changes**: Architecture separation, configuration management, test cleanup

## Context for AI Assistant:
- This is a ROS2 control package for robot teleoperation
- Major refactoring completed: monolithic → layered architecture
- Recent session focused on configuration management and testing cleanup
- Focus on clean separation of concerns and testability
- Follow coding standards in CLAUDE.md strictly (including double quote preference)
- Current work is on `refactor` branch
- Ready for next phase: shared node pattern, simple testing, and extended process management
- Configuration files now properly located in user-writable ~/.config/ directory
- All problematic tests removed - future tests should be simple unit tests with fakes/stubs