# Feature description for feature F02
## F02 — Fix structural problems in codebase
**Priority**: Medium
**Done:** no
**Tests Written:** no
**Test Passing:** no
**Description**: Correct five violations of coding standards and architectural
constraints found during review. No new functionality; all existing tests must
continue to pass. Violations: banned `from __future__ import annotations` imports,
eager API instantiation in RobotController, blocking spin loop in cmd_vel_helper,
hardcoded Pi-only config path in SimpleCLI, and rclpy.init() called inside BaseApi
instead of by the caller.
