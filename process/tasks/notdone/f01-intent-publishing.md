# Tasks for Feature F01 — Intent Publishing

## T01 — Implement IntentApi
**Status**: not done
**Description**: Create `control/ros2_api/intent_api.py`. Class `IntentApi(BaseApi)`
with one method: `publish(name, source, slots)`. Serializes to JSON string, publishes
on `/intent` as `std_msgs/String`. Node name: `intent_api`.

## T02 — Implement intent_commands module
**Status**: not done
**Description**: Create `control/commands/intent_commands.py`. Command defs for
`intent.stop`, `intent.explore`, `intent.describe_scene`, `intent.count_objects`
(last one has required `object_type` string param). All map to `publish_intent`
method on `RobotController`. Group: `intent`.

## T03 — Wire IntentApi into RobotController
**Status**: not done
**Description**: In `robot_controller.py`, add `self.intent = IntentApi(self.config)`
and add method `publish_intent(name, slots)` that calls `self.intent.publish(name, "cli", slots)`.

## T04 — Register intent_commands in CommandDispatcher
**Status**: not done
**Description**: In `command_dispatcher.py`, import `intent_commands` and add
`commands.update(intent_cmd.build_intent_commands())` in `_build_command_registry`.

## T05 — Tests for IntentApi and intent commands
**Status**: not done
**Description**: In `test/test_intent_commands.py`, verify:
- `IntentApi.publish` serializes correct JSON and calls publisher
- Each intent command dispatches with correct name and slots
- `intent.count_objects` with missing `object_type` returns parameter error
Use mock publisher; no live ROS2 required.

## T06 — Smoke test instructions
**Status**: not done
**Description**: Add a short note to `docs/_index.txt` (or a new `docs/intent.txt`)
showing how to verify: run REPL, type `intent stop`, observe output on
`ros2 topic echo /intent` in a second terminal.
