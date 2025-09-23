# Current State - Control Package

### How to use Current.md

* read claude.md and obey it fully
* read the rest of this document and make it part of your context
* adopt the todo list as your own
* when asked to create a new current.md, create a new file, include your current context, loose ends, todo list, and anything else.
* Also include this section called "How To Use Current.md"

## Status: MAJOR LAUNCH MANAGEMENT REFACTOR IN PROGRESS

The package has undergone a complete architectural refactoring and map management implementation. Currently in the middle of a major launch management system refactor to replace individual nav/slam commands with a unified `launch` command structure.

### Current Session (September 2025 - Launch Management Refactor):

**MAJOR REFACTOR PROGRESS - 5 of 12 Steps Complete:**

1. ✅ **Created static launch configuration table** - Added LaunchConfig dataclass and LAUNCH_CONFIGS with 3 launch types (nav, slam, map_server)
2. ✅ **Refactored ProcessApi** - Added launch-type-based methods (`launch_by_type`, `kill_by_type`, `get_launch_status`) with comprehensive tests
3. ✅ **Updated RobotController** - New launch methods (`launch_list`, `launch_start`, `launch_kill`, `launch_status`) with unified process tracking
4. ✅ **Removed old nav/slam command definitions** - Cleaned navigation_commands.py, removed 4 old commands, verified via tests
5. ✅ **Created new launch command definitions** - Added launch_commands.py with 4 new commands, integrated with command dispatcher

**REMAINING STEPS:**
6. 🔄 **Update CLI interface to add launch command group** - Add @cli.group() for launch commands in click_cli.py
7. **Remove old nav/slam CLI command groups** - Clean up click_cli.py nav/slam groups
8. **Modify map commands to work with persistent map_server** - Change map save/load to use running map_server
9. **Update process tracking to use 3 launch types only** - Remove map_save/map_load tracking
10. **Update robot status to use new launch tracking** - Display launch status instead of old process status
11. **Test complete refactored system** - End-to-end testing of new launch commands
12. **Update help documentation for new command structure** - Update help text for new command structure

### New Launch Command Design (Target):

**New Commands:**
- `launch list` - Show all available launch types and their status
- `launch start <type>` - Start a specific launch file type (nav, slam, map_server)
- `launch kill <type>` - Stop a specific launch file type
- `launch status [type]` - Show status of launch processes

**New Map Workflow:**
1. `launch start map_server` - Start persistent map server
2. `map save <filename>` - Command running map_server to save current map
3. `map load <filename>` - Command running map_server to load/publish specific map
4. `launch kill map_server` - Stop the map server entirely

### Latest Technical Implementation:

**New Files Created:**
- `control/commands/launch_commands.py` - Launch command definitions
- `test/test_launch_config.py` - Static launch configuration tests (19 test cases)
- `test/test_robot_controller_launch.py` - RobotController launch method tests (10 test cases)
- `test/test_command_removal.py` - Verification of old command removal (7 test cases)
- `test/test_launch_commands.py` - New launch command definition tests (7 test cases)

**Modified Files:**
- `control/ros2_api/process_api.py` - Added LaunchConfig, LAUNCH_CONFIGS, launch-type methods
- `control/commands/robot_controller.py` - Updated to use launch_process_ids, new launch methods
- `control/commands/navigation_commands.py` - Removed old nav/slam commands, kept map commands
- `control/commands/command_dispatcher.py` - Added launch_commands import and registration

**Key Technical Changes:**
- **Static Launch Configuration**: `LAUNCH_CONFIGS` dict with LaunchConfig dataclass for nav/slam/map_server
- **Unified Process Tracking**: `launch_process_ids` with 3 launch types instead of mixed tracking
- **Conflict Prevention**: Launch commands return "X is already running, stop it first" instead of auto-killing
- **Parameter Merging**: Default params merged with custom params in ProcessApi.launch_by_type()
- **Comprehensive Testing**: 43 total test cases covering all new functionality

## Current Architecture (POST-LAUNCH-REFACTOR):

### New Launch Management Layer:
```
CLI Interface → ClickCLI → CommandDispatcher → RobotController → ProcessApi → ROS2 Launch Files
```

### Static Launch Configuration:
```python
LAUNCH_CONFIGS = {
    "nav": LaunchConfig(
        launch_type="nav",
        command_template="ros2 launch nav2_bringup navigation_launch.py {params}",
        description="Navigation stack with path planning and obstacle avoidance",
        default_params={"use_sim_time": "false"}
    ),
    "slam": LaunchConfig(...),
    "map_server": LaunchConfig(...)
}
```

### Updated Process Tracking:
```python
# Old (mixed tracking)
self.process_ids = {
    "navigation": None, "slam": None, "map_save": None, "map_load": None
}

# New (launch-only tracking)
self.launch_process_ids = {
    "nav": None, "slam": None, "map_server": None
}
```

## Current Source Code Organization:

```
control/
├── commands/
│   ├── launch_commands.py       # NEW: Launch command definitions
│   ├── navigation_commands.py   # MODIFIED: Only map commands remain
│   ├── command_dispatcher.py    # MODIFIED: Added launch commands
│   └── robot_controller.py      # MODIFIED: Launch management methods
├── ros2_api/
│   └── process_api.py           # MODIFIED: Launch configuration + type-based methods
└── test/
    ├── test_launch_config.py         # NEW: Launch configuration tests
    ├── test_robot_controller_launch.py # NEW: Launch method tests
    ├── test_command_removal.py       # NEW: Command removal verification
    └── test_launch_commands.py       # NEW: Launch command definition tests
```

## Current Test Coverage:

**Total Test Files**: 5 new/modified test files
**Total Test Cases**: 43 test cases covering launch functionality

1. **test_launch_config.py** (19 tests):
   - Static configuration validation
   - Parameter formatting
   - Launch type methods
   - Command template formatting

2. **test_robot_controller_launch.py** (10 tests):
   - Launch method functionality
   - Conflict detection
   - Status reporting
   - Backward compatibility

3. **test_command_removal.py** (7 tests):
   - Verification old nav/slam commands removed
   - Command count validation
   - Other commands unaffected

4. **test_launch_commands.py** (7 tests):
   - Command definition validation
   - Parameter structure verification
   - Command dispatcher integration

## Current Issues/Open Work:

### 1. CLI Interface Update Needed (Step 6):
**Status**: In progress
**Needs**: Add `@cli.group()` for launch commands in click_cli.py
**Impact**: Users can't access new launch commands until CLI updated

### 2. Old CLI Command Groups (Step 7):
**Status**: Pending
**Needs**: Remove old `@cli.group() def nav()` and `@cli.group() def slam()` from click_cli.py
**Impact**: Old commands still accessible through CLI despite backend removal

### 3. Map Command Behavior Change (Step 8):
**Status**: Pending
**Needs**: Modify map save/load to communicate with persistent map_server instead of launching separate processes
**Impact**: Current map commands still launch separate processes

### 4. Process Status Display (Step 10):
**Status**: Pending
**Needs**: Update robot status to show launch tracking instead of old mixed tracking
**Impact**: Status display doesn't reflect new launch management

## Todo List (Active Refactor):

### Immediate (Current Session):
6. 🔄 **[in_progress]** Update CLI interface to add launch command group
7. **[pending]** Remove old nav/slam CLI command groups
8. **[pending]** Modify map commands to work with persistent map_server
9. **[pending]** Update process tracking to use 3 launch types only
10. **[pending]** Update robot status to use new launch tracking
11. **[pending]** Test complete refactored system
12. **[pending]** Update help documentation for new command structure

### Future (Post-Refactor):
- Add exit cleanup to detect and optionally kill running processes
- Update setup.py with any new dependencies
- Implement shared node pattern in RobotController
- Add generic process management commands (list, kill by ID)
- Create NavigationApi class for navigation operations
- Create simple unit tests for core components

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

### Current Command Examples (Mixed Old/New):
**Available Now (Backend Complete):**
- Command dispatcher recognizes: `launch.list`, `launch.start nav`, `launch.kill slam`, `launch.status`
- Old commands removed from dispatcher: `nav.start`, `nav.stop`, `slam.start`, `slam.stop`

**CLI Access (Pending Step 6):**
- CLI still has old nav/slam groups until click_cli.py updated
- New launch commands not accessible through CLI until interface updated

**Testing New Commands:**
```python
# Via command dispatcher (works now)
from control.commands.command_dispatcher import CommandDispatcher
response = dispatcher.execute("launch.list", {})
```

## Dependencies:
- `click` - Command line interface framework
- `prompt_toolkit` - Command history and input enhancement
- `rclpy` - ROS2 Python client library
- Standard library: `json`, `pathlib`, `typing`, `dataclasses`, `subprocess`, `threading`

## Design Files:
- **Full refactor design**: `/home/pitosalas/ros2_ws/src/control/new_launch.md`
- **Implementation plan**: 12-step todo list with complete architecture changes

## Coding Standards Applied:
- Functions/methods max 50 lines
- Files max ~300 lines
- Classes in separate files named after the class
- Avoid nested if/else > 1 level
- DRY principle applied
- No unnecessary docstrings (only when function name insufficient)
- Required dependency injection
- Prefer async/await over threading
- **Double quotes preferred** (consistently applied)
- Incremental development (program remains functional after each step)

## Git Branch Status:
- **Current branch**: `refactor`
- **Status**: Major launch management refactor 5/12 steps complete
- **Main changes**: Static launch config, ProcessApi refactor, RobotController launch methods, command definition cleanup

## Context for AI Assistant:
- This is a ROS2 control package for robot teleoperation
- Currently in middle of major launch management refactor (step 6 of 12)
- Backend refactor substantially complete, CLI interface update needed next
- Focus on clean separation of concerns and testability
- Follow coding standards in CLAUDE.md strictly (including double quote preference)
- Current work is on `refactor` branch
- All new functionality has comprehensive test coverage (43 test cases)
- Old nav/slam individual commands being replaced with unified launch management
- Map server becoming persistent service instead of one-time processes