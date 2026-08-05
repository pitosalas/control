---
version: "1.2"
generated: "2026-08-05"
---

# ProcessApi: Subprocess Lifecycle Management

`ProcessApi` owns everything that involves spawning, monitoring, and killing OS-level processes. It has two distinct modes of operation: **async launch** (fire-and-forget background processes) and **sync execution** (blocking commands that return output). Both are used by `RobotController` for different purposes.

## Data Structures

Two dataclasses carry all process state:

```python
@dataclass
class ProcessInfo:
    process: subprocess.Popen
    command: str
    output: List[str]
    is_running: bool
    pid: int
    start_time: float
    log_file: Optional[str] = None

@dataclass
class LaunchConfig:
    launch_type: str
    command_template: str
    description: str
    default_params: Dict[str, str]
```

`ProcessInfo` is the runtime state of a running process. `LaunchConfig` is the static template loaded from YAML config.

```
                   config.yaml
                       │
            ┌──────────▼──────────┐
            │   LaunchConfig[]    │ (templates, loaded at init)
            └──────────┬──────────┘
                       │ launch_by_type()
            ┌──────────▼──────────┐
            │    ProcessInfo[]    │ (live processes, in self.processes dict)
            └─────────────────────┘
```

## Loading Launch Templates

`ProcessApi.__init__` calls `_load_launch_configs` to convert the raw YAML dict into typed `LaunchConfig` objects:

```python
def _load_launch_configs(self):
    templates = self.config.get_launch_templates() if self.config else {}
    self.launch_configs = {}
    for launch_type, template in templates.items():
        self.launch_configs[launch_type] = LaunchConfig(
            launch_type=launch_type,
            command_template=template.get("command", ""),
            description=template.get("description", ""),
            default_params=template.get("default_params", {})
        )
```

## Async Launch: launch_command

The async path launches a process in a new session (`start_new_session=True`) so it survives terminal disconnection. A background thread captures all stdout to a log file and to the console:

```python
proc = subprocess.Popen(
    command, shell=True,
    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    stdin=subprocess.DEVNULL,
    text=True, bufsize=1,
    start_new_session=True,
)
```

`start_new_session=True` creates a new process group, which matters for `kill_ros_process`: if the PID is the group leader, `SIGTERM` is sent to `-pgid` (the whole group), not just the parent.

A UUID is returned as the process handle. `RobotController` stores this UUID in `launch_process_ids` keyed by launch type.

## Sync Execution: run_command_sync

Map saving and serialization need to run a command and wait for its result. `CommandConfig` bundles command, log name, and timeout:

```python
@dataclass
class CommandConfig:
    command: str
    log_name: Optional[str]
    timeout: float
```

```python
def run_command_sync(self, config: CommandConfig) -> tuple[bool, str, Optional[str]]:
    result = subprocess.run(
        config.command, shell=True,
        capture_output=True, text=True,
        timeout=config.timeout
    )
    return (result.returncode == 0, result.stdout + result.stderr, log_file_path)
```

Returns `(success, output, log_file_path)` so callers can report both the outcome and where to find details.

## Killing Processes

`kill_ros_process` handles the case where the target may be a process group leader — in which case `kill -TERM -PID` sends SIGTERM to the entire group:

```python
target = f"-{pid}" if is_group_leader else str(pid)
subprocess.run(["kill", "-TERM", target], check=True, timeout=2)
```

If SIGTERM fails, it escalates to SIGKILL. This graceful-then-force pattern avoids leaving zombie processes.

## Process Status Queries

`is_process_running` uses `poll()` to detect silent process death without blocking:

```python
def is_process_running(self, process_id: str) -> bool:
    proc_info = self.processes[process_id]
    proc_info.process.poll()
    if proc_info.process.returncode is not None:
        proc_info.is_running = False
        return False
    return True
```

`get_process_pid` provides the OS PID for external display:

```python
def get_process_pid(self, process_id: str) -> int | str:
    info = self.processes.get(process_id)
    return info.pid if info else "unknown"
```

## Listing ROS and Launch Processes

`list_ros_processes` and `list_launch_processes` shell out to `ps aux` and `ps axo` respectively, filtering by ROS-related keywords. These are display-only utilities — they scan all OS processes, not just managed ones:

```
ps aux → filter ros/nav2/slam/rviz keywords → tabular output
ps axo pid,pgid,ppid → filter ros2 launch → parent-only display
```

## Subsystem Status (F22)

`get_subsystem_status` answers a different question than `list_ros_processes`: not "what ROS-ish things are running" but "for each of these six named subsystems specifically, is it up, and what process(es) back it?" It reuses the same `ps aux` scan and a shared `_parse_ps_aux_line` helper (extracted from `list_ros_processes` so the two don't duplicate the column-splitting logic), but classifies each line against a per-subsystem keyword map instead of one flat list:

```python
SUBSYSTEM_KEYWORDS = {
    "gendrv": ["micro_ros_agent"],
    "nav": ["slam_manager_node", "explorer_manager_node", "slam_toolbox", "bt_navigator", "amcl"],
    "semantic": ["semantic_map_node"],
    "control": ["behavior_manager"],
    "mission": ["mission_node"],
    "vision": ["dome_vision_ros_node"],
}

def get_subsystem_status(self) -> dict[str, dict]:
    status = {name: {"running": False, "processes": []} for name in SUBSYSTEM_KEYWORDS}
    ...
    for line in result.stdout.split("\n")[1:]:
        info = self._parse_ps_aux_line(line)
        if info is None:
            continue
        line_lower = line.lower()
        for name, keywords in SUBSYSTEM_KEYWORDS.items():
            if any(kw in line_lower for kw in keywords):
                status[name]["running"] = True
                status[name]["processes"].append(info)
    return status
```

Every subsystem key is always present in the returned dict — even ones matching zero processes — so callers (`RobotController.get_subsystems_status`) never need a membership check, only a `["running"]` read. A single process can satisfy more than one subsystem's keyword list independently; the loop doesn't `break` on first match. On a `ps aux` failure (non-zero return code) or timeout, the method fails soft: it returns the same all-`False` shape rather than raising, so a caller building a status table never has to special-case a partial result.

`get_mission_state` fills in the one piece `ps aux` can't see: `mission`'s live FSM state (`IDLE`/`EXPLORING`/...), which only exists on `dome_mission`'s `/mission/state` topic, not in any process's command line. It mirrors `IntentApi.publish`'s reply-wait shape (`ros2_api/intent_api.py`) — subscribe, then `rclpy.spin_once` in a deadline loop — rather than inventing a new blocking-read pattern:

```python
def get_mission_state(self, timeout_s: float = 1.0) -> str:
    ...
    subscription = self.create_subscription(String, "/mission/state", on_state, qos)
    try:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and received["value"] is None:
            rclpy.spin_once(self, timeout_sec=0.1)
    finally:
        self.destroy_subscription(subscription)
    return received["value"] if received["value"] is not None else "unknown"
```

The subscription is transient — created and torn down within the call — because `ProcessApi` isn't otherwise subscribed to mission state; keeping a permanent subscription alive for a value only read on demand would be wasted upkeep. `"unknown"` is returned (not an exception) when nothing arrives within the deadline, since that's an expected steady state (mission not running, or running on a pre-F22 build without the publisher), not an error condition.

## Observations

- **Parameter formatting is complex.** `launch_by_type` has distinct code paths for `bl` commands, `ros2 run`, and launch files. This fragility could be replaced with a strategy object per command type.
- **`list_ros_processes` leaks shell knowledge.** The hardcoded keyword list `["ros2", "nav2", "slam", "rviz", ...]` will miss new components and is not tested. `SUBSYSTEM_KEYWORDS` is the same idea at finer grain, and inherits the same risk: a subsystem's binary name changing silently breaks its detection.
- **Circular import via string.** `kill_ros_process` and `list_*` import `CommandResponse` inline to avoid circular imports. The fix is to move `CommandResponse` to its own module.
- **`get_mission_state` is the only per-subsystem introspection.** The other five subsystems report only running/not-running; deeper state (Nav2/vision lifecycle, `/telemetry` snapshots) is deliberately out of scope for F22 and flagged as a follow-up in the feature file.
