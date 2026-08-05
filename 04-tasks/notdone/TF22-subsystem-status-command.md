# Tasks for Feature F22

## T01 — Subsystem keyword map + process detection
**Status**: not done
**Description**: In `dome_control/ros2_api/process_api.py`, add a module-level
keyword map (mirrors the `ros_keywords` list already used in
`list_ros_processes()`, `process_api.py:461`):

```python
SUBSYSTEM_KEYWORDS = {
    "gendrv": ["micro_ros_agent"],
    "nav": ["slam_manager_node", "explorer_manager_node", "slam_toolbox", "bt_navigator", "amcl"],
    "semantic": ["semantic_map_node"],
    "control": ["behavior_manager"],
    "mission": ["mission_node"],
    "vision": ["dome_vision_ros_node"],
}
```

Add `ProcessApi.get_subsystem_status() -> dict[str, dict]`: runs `ps aux`
once, and for each subsystem in `SUBSYSTEM_KEYWORDS`, reports
`{"running": bool, "processes": [{"pid", "cpu", "mem", "command"}, ...]}` by
matching any of that subsystem's keywords per line (same parsing as
`list_ros_processes`). Extract the shared `ps aux` line-parsing into a small
helper so it isn't duplicated between this and `list_ros_processes`.
Test: mock `subprocess.run` output with lines matching some keywords and not
others; assert each subsystem's `running` flag and process list are correct,
including the case where zero subsystems match.

## T02 — `/mission/state` publisher in `dome_mission`
**Status**: not done
**Description**: Touches `~/ros2_ws/src/dome_mission`, not `dome_control`.
In `dome_mission/dome_mission/mission_node.py`, add a `std_msgs/String`
publisher on `/mission/state`, published once at startup and again on every
FSM transition (alongside the existing
`self.get_logger().info(f"intent ... -> state {self.fsm.state.name}")` call
in the same place) — the message data is `self.fsm.state.name`
(`IDLE`/`EXPLORING`/`LOCATING`/`GOING_TO_TARGET`). Latched/transient-local
QoS so a late subscriber (like the CLI's brief read in T03) still gets the
current state without waiting for the next transition.
Test (in `dome_mission`'s own test suite): drive the FSM through a
transition and assert the publisher's last message matches the new state
name. Record here once done, since this repo's task file is the record of
scope, but the actual test lives in `dome_mission`.

## T03 — `RobotController.get_subsystems_status()`
**Status**: not done
**Description**: Add a method to `RobotController`
(`dome_control/commands/robot_controller.py`) that calls
`self.process.get_subsystem_status()` (T01), then enriches the `mission` row
with a live FSM state read from `/mission/state`: subscribe transiently and
`rclpy.spin_once` in a deadline loop up to ~1s (same pattern as
`IntentApi.publish`'s reply-wait in `ros2_api/intent_api.py:31-44` — reuse
that shape, don't reinvent it), defaulting to `"unknown"` if no message
arrives (e.g. mission not running, or running but pre-T02). Register a new
`CommandDef` `"robot.subsystems"` in `control_commands.py`
(`method_name="get_subsystems_status"`, no parameters).
Test: unit test with a mocked `ProcessApi` and a fake `/mission/state`
publish (or mocked subscription callback) — assert the returned status dict
has all six subsystems and mission's state field reflects the mocked value,
and reflects `"unknown"` when nothing is published within the deadline.

## T04 — CLI table formatting
**Status**: not done
**Description**: In `simple_cli.py`, format `robot subsystems`' response
using the existing `format_table` helper (`simple_cli.py:23`) — columns:
`SUBSYSTEM`, `RUNNING`, `DETAIL` (process count for five subsystems, FSM
state for `mission`).
Test: extend `test_simple_cli_formatting.py` with a case asserting the table
renders expected columns/values for a representative status dict (mix of
running/not-running, including mission with a known state).

## T05 — Docs
**Status**: not done
**Description**: Add `robot subsystems` to `02-doc/cli-reference.md` and add
a line to `02-doc/current.md`'s "What Is Built" documenting the command and
the `/mission/state` topic it depends on for mission detail.

## T06 — Live Pi verification
**Status**: not done
**Description**: On the robot: run `robot subsystems` with only
`behavior_manager` (+ `mission_node`, post-T02 build) up, confirm
`control`/`mission` show running (mission showing `IDLE`) and the other four
show not-running. Publish `nav explore`, confirm mission's row updates to
`EXPLORING` on the next call. Launch `gendrv`/`nav`/`semantic`/`vision` (as
available) one at a time and confirm each flips to running. Hardware/runtime
only — no plain-suite test; record manual test notes (command, setup,
expected vs. actual) directly in this task file once run.

## T07 — Test-writing rollup
**Status**: not done
**Description**: Confirm T01/T03/T04's tests (and T02's, in `dome_mission`)
together cover: correct classification per subsystem, the "nothing running"
case, mission's live-state enrichment (both present and `"unknown"`
fallback), and the CLI table rendering. Fill any gap found; record T06's
manual notes here once available.
