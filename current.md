# Current State - Control Package

### How to use Current.md

* read claude.md and obey it fully
* read the rest of this document and make it part of your context
* adopt the todo list as your own
* when asked to create a new current.md, create a new file, include your current context, loose ends, todo list, and anything else.
* Also include this section called "How To Use Current.md"

## Status: MAJOR REFACTORING COMPLETE - Layered Architecture Implemented

The package has undergone a complete architectural refactoring from a monolithic structure to a clean layered architecture with proper separation of concerns, dependency injection, and unified response handling. The original CLI functionality is preserved while enabling future REST API integration.

### Latest Major Changes (January 2025 - Major Refactoring):

1. ✅ **Created BaseApi class** - Shared ROS2 node functionality and utilities
2. ✅ **Split TeleopApi into domain APIs** - MovementApi, CalibrationApi, ProcessApi
3. ✅ **Created RobotController** - Business logic orchestration layer
4. ✅ **Refactored CommandProcessor** - Pure CLI adapter using RobotController
5. ✅ **Implemented CommandResponse** - Unified response format (success/message/data)
6. ✅ **Added ProcessApi** - Subprocess management for launch files and commands
7. ✅ **Added navigation commands** - `nav start stack`, `nav kill stack` with singleton management
8. ✅ **Fixed dependency injection** - Shared ConfigManager across all components
9. ✅ **Comprehensive integration tests** - End-to-end workflow validation
10. ✅ **Code style cleanup** - Removed unnecessary docstrings per coding standards

### Previous Major Changes:
1. ✅ **Complete package rename** - All files, directories, and imports updated
2. ✅ **Replaced typer with click** - Better command parsing and help generation
3. ✅ **Added variable management system** - Persistent storage with set/show commands
4. ✅ **Simplified main.py** - Direct CLI entry point (removed unused ROS2 node mode)
5. ✅ **Enhanced command structure** - Hierarchical commands with auto-generated help
6. ✅ **Added move time command** - Duration-based movement
7. ✅ **Configuration persistence** - JSON-based config saved to control_config.json

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
- `control/movement_api.py` - Robot movement, velocity control, odometry
- `control/calibration_api.py` - Movement patterns (square, circle, figure-8)
- `control/process_api.py` - Subprocess management for launch files

**Legacy Layer:**
- `control/teleopapi.py` - Now a facade over new APIs for backward compatibility

**Testing:**
- `test/conftest.py` - Pytest fixtures with mocked dependencies
- `test/test_integration.py` - Comprehensive end-to-end integration tests

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
- ✅ **Persistence** - JSON config file with auto-save/load
- ✅ **Shared state** - Single ConfigManager instance across all components

### ROS2 Integration:
- ✅ **Movement control** - /cmd_vel publishing with safety limits
- ✅ **Odometry** - /odom subscription for position tracking
- ✅ **Topic listing** - Live ROS2 graph querying
- ✅ **Node lifecycle** - Proper initialization and cleanup

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

### 4. API Separation by Domain:
- **MovementApi** - ROS2 pub/sub for robot control
- **ProcessApi** - System subprocess management (no ROS2)
- **CalibrationApi** - Uses MovementApi for patterns

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

## Todo List (Priority Order):

1. **[pending]** Update setup.py with any new dependencies
2. **[pending]** Implement shared node pattern in RobotController
3. **[pending]** Add process management commands (list, kill by ID)
4. **[pending]** Create NavigationApi class for navigation operations
5. **[pending]** Add more nav commands (save map, start slam)
6. **[pending]** Remove unnecessary docstrings from remaining files

## Files in Package:

### Core Package Files:
- `package.xml` - ROS2 package definition
- `setup.py` - Python package setup with Click dependency
- `CLAUDE.md` - Development guidelines and coding standards

### Source Code:
- `control/main.py` - Entry point (calls CliInterface)
- `control/cli_interface.py` - Interactive CLI loop with command history
- `control/command_processor.py` - Click command definitions (thin adapter)
- `control/robot_controller.py` - Business logic orchestration
- `control/config_manager.py` - Configuration storage and persistence

### API Layer:
- `control/base_api.py` - Shared ROS2 node functionality
- `control/movement_api.py` - Robot movement and velocity control
- `control/calibration_api.py` - Movement patterns and calibration
- `control/process_api.py` - Subprocess management for launch files
- `control/teleopapi.py` - Legacy facade (now delegates to new APIs)

### Testing:
- `test/conftest.py` - Pytest fixtures with mocked dependencies
- `test/test_integration.py` - End-to-end integration tests
- `test/test_flake8.py`, `test_copyright.py`, `test_pep257.py` - ROS2 standard tests

### Resources:
- `resource/control` - ROS2 resource marker file
- `launch/localization_launch.py` - ROS2 launch file

## Configuration:
- **Config file**: `control_config.json` (in package directory)
- **Variables**: Stored as JSON with automatic type conversion
- **Command history**: `command_history.txt` (in package directory)
- **Default speeds**: linear_speed=0.3, angular_speed=0.4

## Dependencies:
- `click` - Command line interface framework
- `prompt_toolkit` - Command history and input enhancement
- `rclpy` - ROS2 Python client library
- Standard library: `json`, `pathlib`, `typing`, `dataclasses`, `subprocess`, `threading`

## Testing Strategy:
- **Integration tests** - End-to-end workflows with mocked ROS2/subprocess
- **Shared fixtures** - ConfigManager with temporary files
- **Error propagation** - Validate error handling across all layers
- **Dependency injection** - Test shared state management

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

## Coding Standards Applied:
- Functions/methods max 50 lines
- Files max ~300 lines
- Classes in separate files named after the class
- Avoid nested if/else > 1 level
- DRY principle applied
- No unnecessary docstrings (only when function name insufficient)
- Required dependency injection (no optional defaults that create hidden instances)
- Prefer async/await over threading
- Incremental development (program remains functional after each step)

## Recent Technical Decisions:

### 1. Removed Optional Dependencies:
**Old**: `def __init__(self, config_manager: ConfigManager = None)`
**New**: `def __init__(self, config_manager: ConfigManager)`
**Reason**: Prevents hidden separate ConfigManager instances

### 2. Singleton Process Management:
**Implementation**: RobotController tracks `nav_stack_process_id`
**Behavior**: Starting new nav stack automatically kills previous
**Reason**: Only one navigation stack should run at a time

### 3. CommandResponse Format:
**Structure**: success (bool), message (str), data (optional dict)
**Usage**: Consistent across all RobotController methods
**Benefits**: Easy to convert to JSON for REST API

### 4. Process Group Management:
**Implementation**: `preexec_fn=os.setpgrp` in subprocess.Popen
**Purpose**: Kill launch files and all spawned child processes
**Fallback**: SIGTERM → wait → SIGKILL sequence

## Git Branch Status:
- **Current branch**: `refactor`
- **Status**: Major refactoring complete, ready for testing/integration
- **Main changes**: Architectural separation, dependency injection, unified responses

## Next Session Priorities:
1. Test the refactored system end-to-end with real ROS2
2. Implement shared node pattern to resolve ROS2 conflicts
3. Add comprehensive process management commands
4. Consider creating REST API server as proof of architecture
5. Update package documentation and setup.py dependencies

## Context for AI Assistant:
- This is a ROS2 control package for robot teleoperation
- Major refactoring just completed: monolithic → layered architecture
- Focus on clean separation of concerns and testability
- Follow coding standards in CLAUDE.md strictly
- Current work is on `refactor` branch
- Integration tests validate the new architecture works
- Ready for next phase: shared node pattern and extended process management