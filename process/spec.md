# control — Spec

ROS2 robot control package with interactive CLI interface for a Raspberry Pi 5
differential-drive dome robot running ROS2 Jazzy.

## Purpose

Provides a CLI REPL and programmatic API for controlling the robot: movement,
navigation, map management, process/launch management, and system queries.
Publishes normalized `Intent` messages on `/intent` so the behavior manager
(brain) can dispatch to behaviors and services.

## Architecture

Three layers:
- `interface/` — CLI REPL (`SimpleCLI`) and command parser (`SimpleCommandParser`)
- `commands/` — command registry (`CommandDef`), dispatcher (`CommandDispatcher`),
  per-group command modules, `RobotController` orchestration
- `ros2_api/` — thin ROS2 node wrappers (`BaseApi` subclasses) for movement,
  calibration, process management, and intent publishing

## Key constraints

- Runs on Raspberry Pi 5, ROS2 Jazzy, Python 3.12
- **No hardware dependency** — must run on any computer with ROS2 installed.
  Hardware (motors, sensors, OAK-D) is on the other end of ROS2 topics;
  this package only publishes/subscribes. Never import hardware SDKs directly.
- All movement APIs use `std_msgs/String` JSON on `/intent` for high-level commands;
  direct `/cmd_vel` for low-level movement
- Fully offline — no cloud services
- Package installable via `pip3 install -e . --break-system-packages` and buildable
  with `colcon build`
