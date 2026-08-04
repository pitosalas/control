# control — Current Session Handoff

Use this file as the first context document at the start of a new session. It is
not intended to replace `README.md`; keep full usage details there. This file
should answer: what is built, what likely comes next, and what loose issues need
attention.

## Snapshot

**Branch:** `main`  
**Last checkpoint:** 2026-08-04 — F19 T02 resolved (no coordinated stop); F21
(new) D3a adds the `nav`→`mission` CLI domain rename that follows from it;
F21 T01 (drop `move.distance`/`turn.degrees`) implemented and tested.

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
- `MotionBehavior`: handles `stop`, `explore`, `drive_square`, `turn_right`, `turn_left`, `get_status`.
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
- Feature files:
  - `F01`–`F03`, `F13`–`F15`, `F17`: done

## Known Issues / Pending

- **Empty STT turns**: voice turns returning empty. Debug log in `voice_input_node` shows
  `floor`/`cutoff`/`command_started`/`raw_text`. Next: observe on hardware.
- **Wake re-trigger**: cooldown fix unconfirmed on hardware.
- **F19**: sibling package `dome_mission` now also owns `/intent`
  (`exploration_start`/`exploration_stop`/`navigation_go`/`navigation_cancel`,
  driving Nav2 + dome_nav's `ExploreArea`). `behavior_manager` was never
  updated for that arrival: CLI `intent.explore` is still dead (publishes
  `"explore"`, which `MotionBehavior` stubs as `pass  # not yet
  implemented` — the live path is `nav.explore`/F18, soon `mission
  explore`/F21). **T02 resolved (2026-08-04): no coordinated stop.** `stop`
  stays a pure `dome_control` motor halt; cancelling an active
  `dome_mission` goal remains a separate, explicit command (today
  `nav cancel`/`nav explore.stop`, renamed to `mission.*` under F21). T03
  is superseded (no wiring needed). Remaining: T01 (retire dead
  `intent.explore`), T04 (docs), T05 (live Pi verification), T06 (tests).
  See `03-features/notdone/F19-dome-mission-intent-integration.md`.
- **F21 (new, 2026-08-04)**: CLI syntax orthogonality cleanup —
  `03-features/notdone/F21-cli-syntax-orthogonality.md`. D3a renames the
  `nav` domain to `mission` (it talks to `dome_mission`, not `dome_control`
  navigation logic), following from F19's T02 decision. T01 done (dropped
  `move.distance`/`turn.degrees`, kept the directional pairs +
  `turn.radians`). Remaining: T02/T03/T03a (the `nav`→`mission` rename),
  T04 (positional-param doc note), T05 (split `navigation_commands.py`),
  T06 (update `02-doc/cli-reference.md`).

## Likely Next Steps

1. Implement remaining F19 tasks (T01 dead-intent removal, T04 docs, T05
   live Pi verification, T06 tests) — this is currently blocking `dome2`'s
   F02 full-stack smoke test, which needs `behavior_manager` and
   `mission_node` running together.
2. Implement F21's `nav`→`mission` rename (T02/T03/T03a/T05/T06).
3. Test `scene describe` and `scene objects` with real oak hardware on robot.
4. Observe empty-turn voice debug log on hardware.
5. Split `RobotController` into smaller modules.

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
