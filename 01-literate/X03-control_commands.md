---
version: "1.1"
generated: "2026-08-05"
---

# Control Commands (Appendix)

`control_commands.py` defines the `robot.*` group: basic robot control commands that don't
fit any other domain.

```python
"robot.stop":       CommandDef("stop_robot",            [], "Stop robot movement", group="control")
"robot.status":     CommandDef("get_robot_status",       [], "Get current robot status", group="control")
"robot.speak":      CommandDef("speak_text",             [text: str], "Speak text aloud via speech output", group="control")
"robot.subsystems": CommandDef("get_subsystems_status",  [], "Report running/not-running status for each subsystem", group="control")
```

`robot.stop` publishes a zero-velocity Twist via `MovementApi`. `robot.status` returns a
structured status dict including speed config, tracked launch process PIDs, and which API
nodes are initialised. `robot.speak` takes the remaining tokens as free text and routes them
to `SpeechApi`.

`robot.subsystems` (F22) is the newest addition: it takes no parameters and reports
running/not-running for `gendrv`/`nav`/`semantic`/`control`/`mission`/`vision`, plus
`mission`'s live FSM state. See `06-process_api.md` and `11-robot_controller.md` for how
`RobotController.get_subsystems_status()` and `ProcessApi.get_subsystem_status()` /
`get_mission_state()` implement it — this file only registers the CLI-facing name.
