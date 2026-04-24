# Tasks for Feature F02 — Code Cleanup

## T01 — Remove from __future__ import annotations
**Status**: not done
**Description**: Remove the banned import from `command_dispatcher.py`,
`robot_controller.py`, `movement_commands.py`, `navigation_commands.py`,
`system_commands.py`. Replace any forward references with string literals
or `X | None` syntax. Run tests after each file to catch regressions.

## T02 — Fix hardcoded config path in SimpleCLI
**Status**: not done
**Description**: In `simple_cli.py`, replace
`ConfigManager("/home/pitosalas/.control/config.yaml")` with a path resolved
from env var `CONTROL_CONFIG`, falling back to `~/.control/config.yaml`.
Ensures the CLI runs on any machine, not just the Pi.

## T03 — Remove rclpy.init() from BaseApi
**Status**: not done
**Description**: In `base_api.py`, remove the `if not rclpy.ok(): rclpy.init()`
guard from `__init__`. Update all callers that construct a `BaseApi` subclass
to call `rclpy.init()` first. This follows standard ROS2 convention.

## T04 — Isolate blocking loop in cmd_vel_helper
**Status**: not done
**Description**: In `movement_api.py`, add a guard that raises `RuntimeError`
if `cmd_vel_helper` is called from a context where blocking is unsafe (i.e.,
from a lifecycle node). Add a docstring comment explaining the blocking
constraint. Do not refactor to async — that is out of scope here.

## T05 — Defer eager API instantiation in RobotController
**Status**: not done
**Description**: In `robot_controller.py`, make `MovementApi`, `CalibrationApi`,
and `ProcessApi` instantiate lazily on first access via properties. Existing call
sites unchanged. Allows constructing `RobotController` without spinning up all
three ROS2 nodes.

## T06 — Tests confirming all fixes
**Status**: not done
**Description**: Verify existing test suite still passes after each change.
Add one new test each for T02 (config path reads env var) and T03 (no rclpy.init
in BaseApi constructor). No new tests needed for T01/T04/T05 beyond passing
the existing suite.
