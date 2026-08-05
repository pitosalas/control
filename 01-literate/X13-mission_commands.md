---
version: "1.0"
generated: "2026-08-05"
---

# Mission Commands

`mission_commands.py` is the command registry for the `mission` domain — every command that
talks to the sibling `dome_mission` package (Nav2 goal-following and frontier exploration),
as opposed to navigation logic living inside `dome_control` itself. It returns a flat
dictionary of `CommandDef` entries keyed by dotted three-level names like
`"mission.go.start"`. The calling infrastructure in `CommandDispatcher` uses these keys to
route CLI input to the right method on `RobotController`.

The domain was renamed from `nav` to `mission` (see
`03-features/done/F19-dome-mission-intent-integration.md` and
`03-features/done/F21-cli-syntax-orthogonality.md` D3/D3a): the old name suggested this file
owned navigation, when in fact it only forwards intents to `dome_mission`'s FSM. The rename
also makes the bare `stop` command (a `dome_control` motor halt) and `mission.go.stop`
(a `dome_mission` goal cancel) unambiguous by name alone — they intentionally do not
coordinate with each other.

```python
import dome_control.commands.command_def as cd
import dome_control.commands.parameter_def as pd
```

Command definitions are pure data — no nodes, publishers, or subscriptions live here. The
ROS2 plumbing lives in `RobotController`; this file just describes the surface area.

---

## Mission group

The mission commands translate high-level user intent into ROS2 action or topic
interactions. They are split along a clear axis: commands that *cause* something to happen
publish an intent topic; the one command that *reads* something queries a status topic
instead. Every long-running behavior in this group follows the same explicit three-level
shape — `mission.<verb>.start` / `mission.<verb>.stop` — chosen over F21's other candidate
shapes specifically so `go` and `explore` read the same way at the CLI.

### mission.go.start — navigate to a labeled object

```python
"mission.go.start": cd.CommandDef(
    method_name="publish_intent_navigation_go",
    parameters=[pd.ParameterDef("label", str, True, None, "Object label to navigate to")],
    description="Navigate to nearest confirmed object with given label",
    group="mission"
),
```

`mission.go.start` is the only command in this file that takes a required parameter. The
`label` argument names the semantic object type (e.g. `"chair"`, `"door"`) that the robot
should navigate toward. The `RobotController` method `publish_intent_navigation_go` publishes
on the intent bus; `dome_mission` resolves the label to a pose by consulting the object
detection history and then sends a Nav2 `NavigateToPose` goal.

Requiring `label` as a positional CLI argument rather than a config variable is the right
tradeoff here: navigation targets change frequently during a session whereas the map name
does not.

### mission.go.stop — abort current navigation

```python
"mission.go.stop": cd.CommandDef(
    method_name="publish_intent_navigation_cancel",
    parameters=[],
    description="Cancel current navigation goal",
    group="mission"
),
```

A parameterless escape hatch. `publish_intent_navigation_cancel` publishes on the intent
topic; `dome_mission` cancels any active `NavigateToPose` or `FollowWaypoints` goal on
receipt. Keeping cancel as a separate command (rather than passing a flag to
`mission.go.start`) makes it keyboard-friendly and keeps it uncoordinated with the plain
`stop` command's motor halt, by design.

### mission.explore.start — start frontier exploration

```python
"mission.explore.start": cd.CommandDef(
    method_name="publish_intent_exploration_start",
    parameters=[],
    description="Start autonomous frontier exploration",
    group="mission"
),
```

Triggers the `explore_lite` (or equivalent) frontier exploration pipeline inside
`dome_mission`. Publishing an intent rather than calling the action server directly keeps
the command layer decoupled from the specific exploration library in use — the intent
subscriber owns that binding.

### mission.explore.stop — halt frontier exploration

```python
"mission.explore.stop": cd.CommandDef(
    method_name="publish_intent_exploration_stop",
    parameters=[],
    description="Stop autonomous frontier exploration",
    group="mission"
),
```

The counterpart to `mission.explore.start`. Publishes an intent that signals the exploration
node to cancel its active frontier goal and return to idle. This is kept separate from
`mission.go.stop` because exploration and point-to-point navigation can coexist in the
system model; each has its own cancel path.

### mission.explore.status — read exploration state

```python
"mission.explore.status": cd.CommandDef(
    method_name="explore_status",
    parameters=[],
    description="Read current /explore/status topic value",
    group="mission"
),
```

This command is intentionally different in character from every other command in this file.
All other mission commands *publish* an intent; `mission.explore.status` *reads* a topic.
The method name `explore_status` (no `publish_intent_` prefix) signals that distinction
explicitly.

The rationale: exploration is a long-running background process. The operator needs
observable feedback — is it still running, did it complete, did it fail? Polling
`/explore/status` gives a synchronous snapshot of `dome_mission`'s reported state without
requiring a separate monitoring terminal. The implementation in `RobotController`
subscribes to `/explore/status` with a short timeout and returns the most recent message
value to the CLI.
