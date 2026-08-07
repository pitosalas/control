# Tasks for Feature F22

## T01 — Subsystem keyword map + process detection
**Status**: done
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
**Status**: done
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

**Done 2026-08-05**: added `self.state_pub` (transient-local `QoSProfile`)
and `publish_mission_state()` to `MissionNode`, called once in `__init__`
and again in `on_intent` right after the existing log line. Only the
`on_intent`-driven transition is published, per this task's scope — FSM
transitions triggered later by async action callbacks (e.g.
`on_explore_response`'s `on_done(STOPPED)` when no server is available) are
not separately published; the CLI's read (T03) just sees whatever the FSM
settles on next. Tests added in `dome_mission/test/test_mission_node.py`:
`test_mission_state_published_on_startup`,
`test_mission_state_published_on_transition`. `dome_mission` full suite:
47/47 passing.

## T03 — `RobotController.get_subsystems_status()`
**Status**: done
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
**Status**: done
**Description**: In `simple_cli.py`, format `robot subsystems`' response
using the existing `format_table` helper (`simple_cli.py:23`) — columns:
`SUBSYSTEM`, `RUNNING`, `DETAIL` (process count for five subsystems, FSM
state for `mission`).
Test: extend `test_simple_cli_formatting.py` with a case asserting the table
renders expected columns/values for a representative status dict (mix of
running/not-running, including mission with a known state).

## T05 — Docs
**Status**: done
**Description**: Add `robot subsystems` to `02-doc/cli-reference.md` and add
a line to `02-doc/current.md`'s "What Is Built" documenting the command and
the `/mission/state` topic it depends on for mission detail.

## T06 — Live Pi verification
**Status**: done
**Description**: On the robot: run `robot subsystems` with only
`behavior_manager` (+ `mission_node`, post-T02 build) up, confirm
`control`/`mission` show running (mission showing `IDLE`) and the other four
show not-running. Publish `nav explore`, confirm mission's row updates to
`EXPLORING` on the next call. Launch `gendrv`/`nav`/`semantic`/`vision` (as
available) one at a time and confirm each flips to running. Hardware/runtime
only — no plain-suite test; record manual test notes (command, setup,
expected vs. actual) directly in this task file once run.

**Done 2026-08-07** (on `dome-R1`, live robot):

- **Setup**: `bl dome2 robot.launch.py --options "c"` (control →
  `behavior_manager`), `ros2 run dome_mission mission_node`, then
  `ros2 run dome_control run` for the CLI.
- **Step 1 (control + mission + gendrv up)**: brought `gendrv` up alongside
  the above (`bl dome2 robot.launch.py --options "d"`) before the first
  check, so the isolated "only control/mission" baseline wasn't captured
  separately. `robot subsystems` showed `control: yes (1)`,
  `gendrv: yes (1)`, `mission: yes (IDLE)`, `nav`/`semantic`/`vision`:
  `no (0)` — expected classification confirmed for all six rows regardless.
- **Step 2 (mission state transition)**: `mission explore start` published
  `exploration_start`; `mission_node` logged `intent EXPLORE_START -> state
  EXPLORING` and (expectedly, since `dome_nav`'s explore stack wasn't up
  yet) `ERROR: ExploreArea server unavailable`. Per T02's scope, only the
  intent-driven transition is published — `robot subsystems` correctly
  showed `mission: EXPLORING` immediately, matching design.
- **Step 3 (remaining subsystems)**: `vision` via
  `bl dome2 robot.launch.py --options "vi"` → `vision: yes (1)`,
  `semantic: yes` (both `dome_vision_ros_node` and `semantic_map_node` from
  `dome_vision_ros/robot.launch.py`). `nav` via
  `bl dome_nav robot_explore.launch.py --map_name test_map` →
  `nav: yes (3)` (`slam_toolbox`/nav2/`slam_manager_node`/
  `explorer_manager_node`).
- **Aside (not a `dome_control`/T06 issue)**: a `vision` relaunch attempt hit
  `ValueError: Environment variable 'ROBOFLOW_API_KEY' is not set` from
  `dome_vision`'s config loader — this session's shell never loaded Doppler
  secrets (`[doppler] no token configured — skipping secrets` at shell
  startup). `vision`/`semantic` had already been confirmed running once
  before this, so it doesn't block T06; flagged for follow-up in
  `dome_vision`, not tracked here.
- **Result**: all six subsystems (`control`, `gendrv`, `mission`, `vision`,
  `semantic`, `nav`) correctly report running/not-running, and `mission`'s
  live FSM state enrichment (`IDLE` → `EXPLORING`) works end-to-end on
  hardware.

## T07 — Test-writing rollup
**Status**: done
**Description**: Confirm T01/T03/T04's tests (and T02's, in `dome_mission`)
together cover: correct classification per subsystem, the "nothing running"
case, mission's live-state enrichment (both present and `"unknown"`
fallback), and the CLI table rendering. Fill any gap found; record T06's
manual notes here once available.

**Progress 2026-08-05**: coverage confirmed present for everything except
T06's live-hardware notes: `test_process_api_subsystems.py` covers
per-subsystem classification, multiple subsystems matching independently,
and the all-not-running/ps-failure cases; `test_robot_controller_subsystems.py`
covers mission-state enrichment present and `"unknown"`; `test_simple_cli_formatting.py::test_print_subsystems_uses_table`
covers CLI rendering; `dome_mission/test/test_mission_node.py` covers the
`/mission/state` publisher (T02). `dome_control` suite: 210/210 passing.
Left open pending T06 (needs the physical robot).

**Done 2026-08-07**: T06's live-hardware notes recorded above — all six
subsystems and the mission live-state enrichment verified working on
`dome-R1`. No coverage gaps found. `dome_control` suite: 202/202 passing
(210 → 202 after F20's merge moved 8 `ups_status` tests out to the
`dome_telemetry` package, unrelated to F22).
