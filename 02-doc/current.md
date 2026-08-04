# control — Current Session Handoff

Use this file as the first context document at the start of a new session. It is
not intended to replace `README.md`; keep full usage details there. This file
should answer: what is built, what likely comes next, and what loose issues need
attention.

## Snapshot

**Branch:** `main`  
**Last checkpoint:** 2026-08-04 — F19 closed out (dome_mission `/intent`
integration: dead `intent.explore` removed, split ownership documented, live
Pi verification done — see `03-features/done/F19-dome-mission-intent-integration.md`).
Control config moved from `~/.control/config.yaml` to
`~/.dome/config/control.yaml`, with the `CONTROL_CONFIG` env var override
dropped (path is now fixed). F21 (new) D3a adds the `nav`→`mission` CLI
domain rename that follows from F19's T02; F21 T01 (drop
`move.distance`/`turn.degrees`) implemented and tested.

This repo is the ROS2 control package and CLI for robot movement, launch/process
management, map operations, configuration, scripts, and intent publishing.

## What Is Built

- CLI with REPL and non-interactive modes (`SimpleCLI` → `CommandDispatcher.dispatch_text`).
- Command dispatcher with two routing paths:
  - **Behavior path**: behavior commands (`scene describe`, `scene objects`, `intent stop`, etc.)
    publish JSON to `/intent` via `IntentPublisher`. Query intents wait up to 5s for reply on
    `/announcement` and print the result in the CLI.
  - **Direct path**: all other commands call `RobotController` methods synchronously.
- `RobotController` orchestration layer.
- ROS2 API wrappers: movement, calibration, process, intent publishing, survey.
- `SpinSurvey` + `SpinSurveyNode`: 360° spin behavior moved from `dome_vision`. `SpinSurveyNode` is a regular ROS2 node triggered by `/survey/start` Trigger service.
- `survey start` CLI command via `SurveyApi` → `/survey/start` → `SpinSurveyNode`.
- Launch management through configured templates.
- Map save/list/serialize commands.
- `BehaviorManagerNode`: subscribes to `/intent` and `/oak/detections`, routes to domain handlers.
  Caches latest `Detection2DArray` in `latest_detections` for `PerceptionBehavior`.
- `MotionBehavior`: handles `stop`, `drive_square`, `turn_right`, `turn_left`, `get_status`.
- `PerceptionBehavior`: handles `describe_scene` (async `/describe_scene` Trigger service call to
  `SemanticMapNode`), `list_objects` (cached detections from `/oak/detections`).
  If service not ready, publishes user-facing "Oak Camera not connected" message.
- `IntentParser`: pure Python JSON→Intent parser.
- `describe_scene_stub` node for smoke testing without vision hardware.
- Voice pipeline in sibling package `dome_voice`. `voice_input_node.py` lives there.
- Voice intent mapping: 8 single-word commands — stop, right, left, explore, describe, objects, status, help.
- Wake word: "alexa". Threshold 0.7, wake_hits 3, cooldown 1.5s.
- `audio_feedback.py` (in dome_voice): beep via aplay, 20% amplitude.
- Launch files — each package owns its own robot/remote launches:
  - `dome_control/launch/robot.launch.py` — speech_output + behavior_manager (no args)
  - `dome_control/launch/remote.launch.py` — optional behavior_manager
  - `dome_voice/launch/robot.launch.py` — voice_input (Seeed board assumed)
  - `dome_vision_ros/launch/robot.launch.py` — oak camera + semantic_map + optional spin_survey
- `TelemetryNode` (F17): publishes combined `/telemetry` (`dome_control/msg/Telemetry`,
  flat typed fields) at `publish_rate_hz` (config/telemetry.yaml, default 1). Reads
  UPS (INA219) directly; gets OAK by subscribing to `/telemetry/oak`
  (`dome_telemetry_msgs/OakStats`) so it runs alongside the vision stack (no USB
  conflict) and gains fps + perf timings. Host stats (`pi_*` incl. `pi_uptime_s`)
  from `/proc`+`/sys`. Fail-soft per source. Foxglove plots `/telemetry.<field>` with
  no JSON. Also logs each row to `~/.dome/telemetry/telemetryDDMMYY.csv` every
  `log_interval_s` (default 10), keeping `max_log_files` (default 25) daily files.
  Included in `robot.launch.py`.
- `SocEstimator` (in `ups_status.py`): replaced linear `battery_percent`. Tracks
  session-peak voltage as `v_full` proxy (batteries rise for ~1–2 h after charge).
  `v_empty=9.0V`, `v_full_initial=12.18V`, `r_int=0.0Ω` (wired, disabled). INA219
  current convention: positive=discharging, so sag correction is `voltage + I*R`.
- **`/intent` ownership is split between two packages** (F19), each name
  handled by exactly one, never both:
  - `dome_control` (`behavior_manager` → `MotionBehavior`/`PerceptionBehavior`):
    `stop`, `drive_square`, `turn_right`, `turn_left`, `get_status`,
    `describe_scene`, `count_objects`.
  - `dome_mission` (`mission_node`, driving Nav2 + dome_nav's `ExploreArea`):
    `exploration_start`, `exploration_stop`, `navigation_go`,
    `navigation_cancel`.
  No coordinated stop: `stop` is a pure `dome_control` motor halt with no
  knowledge of `dome_mission`'s FSM state; cancelling an active mission goal
  is the separate, explicit `nav cancel`/`nav explore.stop` (renamed to
  `mission.*` under F21).
- Feature files:
  - `F01`–`F03`, `F13`–`F15`, `F17`, `F19`: done

## Known Issues / Pending

- **Empty STT turns**: voice turns returning empty. Debug log in `voice_input_node` shows
  `floor`/`cutoff`/`command_started`/`raw_text`. Next: observe on hardware.
- **Wake re-trigger**: cooldown fix unconfirmed on hardware.
- **Config location changed (2026-08-04)**: control config moved from
  `~/.control/config.yaml` to `~/.dome/config/control.yaml`; `CONTROL_CONFIG`
  env var no longer works (path is now fixed). If a session or script still
  points at the old path/env var, it will silently pick up nothing — check
  `~/.dome/config/control.yaml` exists.
- **F21 (new, 2026-08-04)**: CLI syntax orthogonality cleanup —
  `03-features/notdone/F21-cli-syntax-orthogonality.md`. D3a renames the
  `nav` domain to `mission` (it talks to `dome_mission`, not `dome_control`
  navigation logic), following from F19's T02 decision. T01 done (dropped
  `move.distance`/`turn.degrees`, kept the directional pairs +
  `turn.radians`). Remaining: T02/T03/T03a (the `nav`→`mission` rename),
  T04 (positional-param doc note), T05 (split `navigation_commands.py`),
  T06 (update `02-doc/cli-reference.md`).

## Likely Next Steps

1. Implement F21's `nav`→`mission` rename (T02/T03/T03a/T05/T06).
2. Test `scene describe` and `scene objects` with real oak hardware on robot.
3. Observe empty-turn voice debug log on hardware.
4. Split `RobotController` into smaller modules.

## Quick Commands

```bash
# Tests
python3 -m pytest test/ -v

# CLI (dev machine)
ros2 run dome_control run

# On robot — three separate terminals:
bl dome_control robot.launch.py
bl dome_voice robot.launch.py
bl dome_vision_ros robot.launch.py

# Stub mode (no oak hardware):
bl dome_control robot.launch.py
# manually: ros2 run dome_control describe_scene_stub
```
