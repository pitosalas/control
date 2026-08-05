---
version: "1.5"
generated: "2026-08-05"
---

# RobotController: Orchestrating Robot Operations

`RobotController` is the business logic layer. It sits between the command interfaces (CLI, REST) and the ROS2 API nodes (`MovementApi`, `ProcessApi`, `CalibrationApi`, `IntentApi`). Its job is to coordinate those APIs and return `CommandResponse` objects that callers can format however they like.

## Uniform Return Type

Every public method returns a `CommandResponse`:

```python
@dataclass
class CommandResponse:
    success: bool
    message: str
    data: dict | None = None
```

This is a deliberate design decision. Without it, callers need to handle `None`, exceptions, tuples, and strings. With it, the CLI can always check `result.success` and print `result.message`.

## Lazy API Initialization

ROS2 nodes are expensive to construct — they register with the DDS middleware and allocate system resources. `RobotController` defers construction until first use via properties:

```python
@property
def movement(self) -> MovementApi:
    if self.movement_node is None:
        self.movement_node = MovementApi(self.config)
    return self.movement_node
```

The backing variables (`movement_node`, `calibration_node`, `process_node`, `intent_node`, `speech_node`) start as `None`. When `get_robot_status` inspects them, it checks for `None` directly — a non-`None` node means it was constructed and is therefore "available".

```python
nodes_status = {
    "movement_api": "running" if self.movement_node else "not available",
    "calibration_api": "running" if self.calibration_node else "not available",
    "process_api": "running" if self.process_node else "not available",
}
```

## Launch Process Tracking

`RobotController.__init__` pre-populates `launch_process_ids` with `None` values for every known launch type:

```python
self.launch_process_ids = dict.fromkeys(self.config.get_launch_templates().keys())
```

This means iteration over `launch_process_ids` always covers all configured launch types, whether running or not — no surprises when listing status.

```
launch_process_ids = {
    "nav":  "3f2a-..."   # running — stores UUID from ProcessApi
    "slam": None         # not running
    "map":  None         # not running
}
```

## Launch Start and Stop

`launch_start` checks for a conflict (already running), then delegates to `ProcessApi.launch_by_type` and stores the returned UUID:

```python
def launch_start(self, launch_type: str, **kwargs) -> CommandResponse:
    if self.is_launch_running(launch_type):
        return CommandResponse(False, f"{launch_type} is already running, stop it first")
    try:
        process_id = self.process.launch_by_type(launch_type, **kwargs)
        self.launch_process_ids[launch_type] = process_id
        return CommandResponse(True, f"Started {launch_type}", {"process_id": process_id})
    except ValueError as e:
        return CommandResponse(False, str(e))
```

`launch_stop` mirrors this: look up the UUID, verify it is actually running, kill it, clear the UUID.

## Map Operations

Map save and serialize share a common pattern: build a shell command, wrap it in a `CommandConfig`, run it synchronously, and interpret the result:

```python
def map_path(self):
    map_name = self.config.get_variable("map_name")
    if not map_name:
        return None, None, None
    self.config.ensure_subdirs()
    maps_dir = self.config.get_maps_dir()
    return map_name, maps_dir, maps_dir / map_name

def run_map_command(self, config: CommandConfig, success_msg: str, error_prefix: str) -> CommandResponse:
    success, output, log_file = self.process.run_command_sync(config)
    if success:
        return CommandResponse(True, success_msg + (f"\nLog: {log_file}" if log_file else ""))
    ...
```

`map_path` is extracted because both `map_save` and `map_serialize` need the same three values: name, directory, and full path. Without the helper, that logic would be duplicated.

## Speech Output

`speak_text` publishes text to `/announcement` via `SpeechApi`, which `SpeechOutputNode` picks up and converts to audio. The `speech` property follows the same lazy init pattern as other APIs:

```python
@property
def speech(self) -> SpeechApi:
    if self.speech_node is None:
        self.speech_node = SpeechApi(self.config)
        time.sleep(0.5)
    return self.speech_node

def speak_text(self, text: str) -> CommandResponse:
    self.speech.speak(text)
    return CommandResponse(True, f"Speaking: {text!r}")
```

The 0.5s sleep on first construction serves the same DDS discovery purpose as the sleep on `intent` — allows the publisher to be seen by subscribers before the first message.

## Intent Publishing

The `publish_intent` methods dispatch named intents to the behavior manager via `IntentApi`:

```python
def publish_intent(self, name: str, slots: dict) -> CommandResponse:
    self.intent.publish(name, "cli", slots)
    return CommandResponse(True, f"Intent published: {name}")
```

The `intent` property has a 0.5-second sleep after first construction to allow DDS publisher-subscriber discovery before the first publish. This is a known ROS2 startup race condition.

Concrete intent helpers cover the full range of robot behaviors:

```python
def publish_intent_stop(self) -> CommandResponse:
    return self.publish_intent("stop", {})

def publish_intent_explore(self) -> CommandResponse:
    return self.publish_intent("explore", {})

def publish_intent_exploration_start(self) -> CommandResponse:
    return self.publish_intent("exploration_start", {})

def publish_intent_exploration_stop(self) -> CommandResponse:
    return self.publish_intent("exploration_stop", {})

def publish_intent_navigation_go(self, label: str) -> CommandResponse:
    return self.publish_intent("navigation_go", {"label": label})
```

Each of these publishes an intent and returns immediately. They make no assertion about what the behavior manager does with the intent — that is the behavior manager's concern.

## Reading Exploration State Without Publishing

Sometimes the operator needs to know what the explorer is currently doing without triggering any state change. `publish_intent_explore` is wrong for this because publishing an intent is a directive, not a query.

The solution is `explore_status`, which reads the `/explore/status` topic out-of-band using the `ros2 topic echo --once` CLI tool via `subprocess`:

```python
def explore_status(self) -> CommandResponse:
    try:
        result = subprocess.run(
            ["ros2", "topic", "echo", "--once", "/explore/status"],
            capture_output=True, text=True, timeout=3.0
        )
        value = result.stdout.strip().replace("data: ", "").strip("'\"")
        if value:
            return CommandResponse(True, f"explore status: {value}")
        return CommandResponse(False, "No status received from /explore/status (explore_manager_node running?)")
    except subprocess.TimeoutExpired:
        return CommandResponse(False, "Timeout reading /explore/status — explore_manager_node not running")
```

There are several deliberate choices here worth noting:

**Why `subprocess` instead of a ROS2 subscriber?** Creating a ROS2 subscriber node just to read one message would require spinning a node, waiting for the message, then destroying the node — a heavyweight operation for what amounts to a status poll. `subprocess.run` with `ros2 topic echo --once` is lighter: it reuses the existing `ros2` CLI, blocks until exactly one message arrives, and exits. The 3-second timeout ensures the call never hangs indefinitely.

**Why not `IntentApi`?** `IntentApi` is write-only — it publishes intents. Reading a topic that `explore_manager_node` writes requires a different path entirely.

**Why strip `data: ` from the output?** `ros2 topic echo` formats `std_msgs/msg/String` messages as `data: 'value'`. The strip removes both the key prefix and the surrounding quotes, leaving just the bare status string (e.g., `idle`, `exploring`, `stopped`).

**Timeout semantics.** A `subprocess.TimeoutExpired` exception means no message arrived within 3 seconds, which is the clearest signal that `explore_manager_node` is not running. The error message says so explicitly, guiding the operator to the root cause.

## Delegating to ProcessApi

Process management methods delegate entirely to `ProcessApi`:

```python
def list_ros_processes(self) -> CommandResponse:
    return self.process.list_ros_processes()

def kill_ros_process(self, pid: int) -> CommandResponse:
    return self.process.kill_ros_process(pid)
```

This keeps `RobotController` free of subprocess and `ps` parsing logic.

## Survey Start

`start_survey` uses `SurveyApi` to call the `/survey/start` Trigger service on `SpinSurveyNode`. The `survey` property follows the same lazy init pattern as other APIs:

```python
@property
def survey(self) -> SurveyApi:
    if self.survey_node is None:
        self.survey_node = SurveyApi()
    return self.survey_node

def start_survey(self) -> CommandResponse:
    ok, msg = self.survey.start()
    return CommandResponse(ok, msg)
```

No DDS discovery sleep is needed here because `SurveyApi.start()` calls `wait_for_service()` internally before sending the request.

## Subsystem Status Rollup (F22)

`get_subsystems_status` is a thin composition method: it asks `ProcessApi` for the six-subsystem snapshot, then enriches just the `mission` entry with a live FSM-state read, and wraps the result the way every other structured response is wrapped — a `CommandResponse` whose `data` carries a named key (`"subsystems"`) that the CLI layer (`simple_cli.py`) knows how to render as a table:

```python
def get_subsystems_status(self) -> CommandResponse:
    subsystems = self.process.get_subsystem_status()
    subsystems["mission"]["state"] = self.process.get_mission_state()
    return CommandResponse(True, "Subsystem status", {"subsystems": subsystems})
```

This follows the same "enrich one dict in place" shape as `get_robot_status`'s combination of speeds/processes/nodes — `RobotController` composes what several lower-level APIs each know a piece of, rather than any one of them owning the full picture. `mission` is singled out for the extra read because it's the only subsystem in F22's scope with no other externally observable state: the other five are correctly described by running/not-running alone, but `mission`'s FSM state is the whole point of checking it.

## Observations

- **`turn_degrees` breaks the pattern.** It returns `self.movement.turn_degrees(degrees)` directly instead of a `CommandResponse`. `MovementApi.turn_degrees` returns `None`, so callers get `None` back — a latent bug.
- **No error handling on movement calls.** Methods like `move_distance` call `self.movement.move_dist` without try/except. A ROS2 exception (e.g., node not initialized) would propagate unhandled to the CLI.
- **`list_maps` complexity.** The file-grouping logic (stem → extensions) is 20 lines that could be a helper. It also conflates map files and any other file in the directory.
- **`explore_status` uses `subprocess` while other status checks use ROS2 nodes.** This is a pragmatic inconsistency: the explorer publishes a string topic that is cheap to read with `ros2 topic echo`, whereas movement and process status require richer structured access. If the pattern proliferates, a thin `TopicReaderApi` wrapping `subprocess.run` would centralize the timeout and stripping logic.
