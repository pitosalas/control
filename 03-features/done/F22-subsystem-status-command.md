# Feature description for feature F22
## F22 — subsystem status command

**Priority**: Medium
**Done:** yes
**Tasks File Created:** yes
**Tests Written:** yes
**Test Passing:** yes
**Description**: New CLI command `robot subsystems` reports, in one place,
whether each of the robot's six key subsystems is running and (where
available) what it's currently doing: `gendrv` (linorobot2 driver bringup +
micro_ros_agent), `nav` (`dome_nav` — SLAM/Nav2), `semantic`
(`dome_semantic`), `control` (`dome_control` itself), `mission`
(`dome_mission`), `vision` (`dome_vision_ros`).

Detection is uniform across all six: extend the existing
`ProcessApi.list_ros_processes()` `ps aux`-grep pattern
(`dome_control/dome_control/ros2_api/process_api.py:461`) with a
per-subsystem keyword map, rather than mixing in ROS2 lifecycle `GetState`
calls, service calls, or topic reads for some subsystems and not others.
That richer per-subsystem introspection (Nav2/vision lifecycle state,
`/telemetry`/`/telemetry/oak` snapshots) is deliberately out of scope for
this iteration — flagged as a follow-up, not solved here.

One exception: `mission` is the only subsystem with literally zero
externally visible state today (its FSM state in `dome_mission/dome_mission/
mission_fsm.py` is logged, never published). This feature adds a
`/mission/state` publisher to `dome_mission`'s `mission_node.py` (published
on every FSM transition) so `robot subsystems` can show mission's live state
(`IDLE`/`EXPLORING`/etc.), not just running/not-running. This is the one
task in this feature that touches a second package
(`~/ros2_ws/src/dome_mission`).

**Keyword map** (established by reading each package's `setup.py`
`console_scripts` — see F22 investigation in session history):

| Subsystem | grep keyword(s) |
|---|---|
| gendrv | `micro_ros_agent` |
| nav | `slam_manager_node`, `explorer_manager_node`, `slam_toolbox`, `bt_navigator`, `amcl` |
| semantic | `semantic_map_node` |
| control | `behavior_manager` |
| mission | `mission_node` |
| vision | `dome_vision_ros_node` |

## How to Demo

**Setup**: on the robot, with some subsystems up and some down (e.g. only
`behavior_manager` + `mission_node` running, nav/vision/semantic not
launched).

**Steps**:
1. `ros2 run dome_control run robot subsystems`
2. Confirm `control` and `mission` report running (with mission's live FSM
   state shown), and `gendrv`/`nav`/`semantic`/`vision` report not running.
3. Launch `mission_node`, publish an `exploration_start` intent, re-run
   `robot subsystems` mid-explore — confirm mission's state now reads
   `EXPLORING`.

**Expected output**: one table, one row per subsystem, accurate
running/not-running plus process detail for all six, and live FSM state for
`mission` specifically.
