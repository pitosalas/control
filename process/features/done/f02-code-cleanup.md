# Feature description for feature F02
## F02 — Fix structural problems in codebase
**Priority**: Medium
**Done:** yes
**Tests Written:** yes
**Test Passing:** yes
**Description**: Correct five violations of coding standards and architectural
constraints found during review. No new functionality; all existing tests must
continue to pass. Violations: banned `from __future__ import annotations` imports,
eager API instantiation in RobotController, blocking spin loop in cmd_vel_helper,
hardcoded Pi-only config path in SimpleCLI, and rclpy.init() called inside BaseApi
instead of by the caller.

## How to Demo
**Setup**: no ROS2 required; tests are self-contained.

**Steps**:
1. `cd /home/pitosalas/ros2_ws/src/control`
2. `python3 -m pytest test/ -v --ignore=test/__init__.py`

**Expected output**: all 67 tests pass, zero failures.
