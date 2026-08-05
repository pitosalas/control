# control — Current Session Handoff

Use this file as the first context document at the start of a new session. It is
not intended to replace `README.md`; keep full usage details there. This file
should answer: what is built, what likely comes next, and what loose issues need
attention.

## Snapshot

**Branch:** `main`  
**Last checkpoint:** 2026-08-05 — **F21 closed** (all tasks T01–T06 done):
`nav` domain renamed to `mission` throughout the CLI (`mission.go.start`,
`mission.go.stop`, `mission.explore.start`, `mission.explore.stop`,
`mission.explore.status`), `navigation_commands.py` split into
`map_commands.py` + `mission_commands.py` (`command_dispatcher.py` updated
to import/register both), `02-doc/cli-reference.md` and `docs/launch.start.txt`
updated for the rename and the positional-optional-parameter note.
**F22 (`robot subsystems` status command) nearly done**: T01/T02/T03/T04/T05/T07
all done — `ProcessApi.get_subsystem_status()` + `get_mission_state()`,
`RobotController.get_subsystems_status()`, `robot.subsystems` CLI command
with table output, `dome_mission`'s `/mission/state` publisher. Only T06
(live Pi hardware verification) remains. 210 tests passing.
Control config moved from `~/.control/config.yaml` to
`~/.dome/config/control.yaml`, with the `CONTROL_CONFIG` env var override
dropped (path is now fixed).

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
  is the separate, explicit `mission.go.stop`/`mission.explore.stop` (F21
  renamed the `nav` domain to `mission`).
- `dome_mission`'s `mission_node.py` publishes its FSM state (`IDLE`/
  `EXPLORING`/etc.) on `/mission/state` (`std_msgs/String`, transient-local
  QoS) on startup and every `/intent`-driven transition (F22 T02) — consumed
  by `robot subsystems`' `mission` row.
- `robot subsystems` (F22): reports running/not-running for `gendrv`/`nav`/
  `semantic`/`control`/`mission`/`vision` via a per-subsystem `ps aux`
  keyword grep (`ProcessApi.get_subsystem_status()`), plus `mission`'s live
  FSM state read from `/mission/state` (`ProcessApi.get_mission_state()`,
  ~1s deadline, `"unknown"` on timeout).
- Feature files:
  - `F01`–`F03`, `F13`–`F15`, `F17`, `F19`, `F21`: done
  - `F22`: T01–T05, T07 done; T06 (live Pi verification) remains

## Known Issues / Pending

- **Empty STT turns**: voice turns returning empty. Debug log in `voice_input_node` shows
  `floor`/`cutoff`/`command_started`/`raw_text`. Next: observe on hardware.
- **Wake re-trigger**: cooldown fix unconfirmed on hardware.
- **Config location changed (2026-08-04)**: control config moved from
  `~/.control/config.yaml` to `~/.dome/config/control.yaml`; `CONTROL_CONFIG`
  env var no longer works (path is now fixed). If a session or script still
  points at the old path/env var, it will silently pick up nothing — check
  `~/.dome/config/control.yaml` exists.
- **F22 (2026-08-05, nearly done)**: cross-package `robot subsystems` status
  command — `03-features/notdone/F22-subsystem-status-command.md` /
  `04-tasks/notdone/TF22-subsystem-status-command.md`. Reports
  running/not-running for `gendrv`/`nav`/`semantic`/`control`/`mission`/
  `vision` via `ProcessApi.get_subsystem_status()`, plus live FSM state for
  `mission` via `/mission/state` (published from `dome_mission`'s
  `mission_node.py`, T02). CLI table output via `SimpleCLI.print_subsystems`.
  T01–T05 and T07 done, 210/210 tests passing. Only **T06** (live Pi
  verification — needs the physical robot) remains before the feature can
  close.

## Likely Next Steps

1. Run F22 T06 (live Pi verification of `robot subsystems`), then close F22.
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
