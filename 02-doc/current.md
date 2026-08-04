# control — Current Session Handoff

Use this file as the first context document at the start of a new session. It is
not intended to replace `README.md`; keep full usage details there. This file
should answer: what is built, what likely comes next, and what loose issues need
attention.

## Snapshot

**Branch:** `main`  
**Last checkpoint:** 2026-08-04 — F20: battery telemetry split into standalone `dome_telemetry` package

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
  flat typed fields) at `publish_rate_hz` (config/telemetry.yaml, default 1). Owns
  no hardware: gets UPS by subscribing to `/telemetry/battery`
  (`dome_telemetry_msgs/BatteryStats`, from the sibling `dome_telemetry` package,
  F20) and OAK by subscribing to `/telemetry/oak` (`dome_telemetry_msgs/OakStats`)
  so it runs alongside the vision/hardware stacks with no device conflicts. Host
  stats (`pi_*` incl. `pi_uptime_s`) from `/proc`+`/sys`. Fail-soft per source.
  Foxglove plots `/telemetry.<field>` with no JSON. Also logs each row to
  `~/.dome/telemetry/telemetryDDMMYY.csv` every `log_interval_s` (default 10),
  keeping `max_log_files` (default 25) daily files. Included in `robot.launch.py`.
- **F20 (new, 2026-08-04)**: battery telemetry moved out of `dome_control` into a
  new standalone sibling package `dome_telemetry` (own repo at
  `src/dome_telemetry`, `.claude/` kit copied from `~/j3` — see feedback memory
  on new-package scaffolding). `BatteryTelemetryNode` opens the INA219 (fail-soft)
  on a single 60s timer and publishes `dome_telemetry_msgs/BatteryStats` on
  `/telemetry/battery`; launched via `dome_telemetry/launch/robot.launch.py`,
  included from `dome2`'s `gendrv.launch.py` so it starts with hardware driver
  bringup, independent of `dome_control`/`dome_vision`/`dome_voice`. `ups_status.py`
  (INA219 driver, `SocEstimator`, `read_ups_stats`) moved there too; `dome_control`
  no longer imports `smbus` or opens I2C. All 7 tasks done, unit tests pass
  (10/10 in `dome_telemetry`, 5/5 telemetry-node tests in `dome_control`) — **not
  yet verified on real hardware**; the feature's "How to Demo" steps
  (`bl dome2 gendrv.launch.py`, echo `/telemetry/battery` and `/telemetry`) are
  still outstanding.
- `SocEstimator` (now in `dome_telemetry/ups_status.py`): replaced linear
  `battery_percent`. Tracks session-peak voltage as `v_full` proxy (batteries rise
  for ~1–2 h after charge). `v_empty=9.0V`, `v_full_initial=12.18V`, `r_int=0.0Ω`
  (wired, disabled). INA219 current convention: positive=discharging, so sag
  correction is `voltage + I*R`.
- Feature files:
  - `F01`–`F03`, `F13`–`F15`, `F17`, `F20`: done

## Known Issues / Pending

- **Empty STT turns**: voice turns returning empty. Debug log in `voice_input_node` shows
  `floor`/`cutoff`/`command_started`/`raw_text`. Next: observe on hardware.
- **Wake re-trigger**: cooldown fix unconfirmed on hardware.
- **F20 hardware verification (new, 2026-08-04)**: implementation + unit tests are
  done, but the demo steps in `03-features/done/F20-dome-telemetry-package.md`
  (launch `gendrv.launch.py`, confirm `/telemetry/battery` publishes every ~60s
  with plausible values, confirm `dome_control`'s `/telemetry` still carries
  `ups_*` fields when both are running) have not been run on the robot yet.
- **F19 (new, 2026-08-04)**: sibling package `dome_mission` now also owns
  `/intent` (`exploration_start`/`exploration_stop`/`navigation_go`/
  `navigation_cancel`, driving Nav2 + dome_nav's `ExploreArea`).
  `behavior_manager` was never updated for that arrival: CLI `intent.explore`
  is dead (publishes `"explore"`, which `MotionBehavior` stubs as
  `pass  # not yet implemented` — the live path is `nav.explore`/F18, which
  publishes `exploration_start`). `stop` only halts cmd_vel; it does not
  cancel an outstanding `dome_mission` goal. No live double-dispatch today
  (the two packages' intent-name sets happen to be disjoint), but the gap is
  real. See `03-features/notdone/F19-dome-mission-intent-integration.md` —
  T02 (coordinated-stop design decision) needs to be resolved before T01/T03
  can proceed.

## Likely Next Steps

1. Resolve F19 T02 (coordinated-stop design decision), then implement F19
   T01/T03–T06 — this is currently blocking `dome2`'s F02 full-stack smoke
   test, which needs `behavior_manager` and `mission_node` running together.
2. Run F20's on-hardware demo steps (see above) to confirm `dome_telemetry`
   actually publishes `/telemetry/battery` and `dome_control` picks it up.
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

# Battery telemetry (F20) — starts with hardware driver bringup:
bl dome2 gendrv.launch.py
ros2 topic echo /telemetry/battery

# Stub mode (no oak hardware):
bl dome_control robot.launch.py
# manually: ros2 run dome_control describe_scene_stub
```
