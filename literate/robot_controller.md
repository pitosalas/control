# robot_controller — Orchestration and Lazy API Initialization

## What it does and why

`RobotController` is the boundary between the command layer and the ROS2 API layer.
The CLI dispatcher calls methods on `RobotController`; `RobotController` delegates to
`MovementApi`, `CalibrationApi`, and `ProcessApi`. No command module imports a ROS2
API directly.

`CommandResponse` is the uniform return type for every method:

```python
@dataclass
class CommandResponse:
    success: bool
    message: str
    data: dict | None = None
```

The CLI prints `result.message` and optionally formats `result.data`. Having a single
return type means the dispatcher can handle success and failure uniformly without
knowing anything about what the method did.

## Lazy API initialization

ROS2 nodes are expensive to construct — they register with the ROS2 daemon,
allocate publisher/subscriber queues, and validate config. Constructing all three API
nodes at startup means any config error surfaces immediately and the process crashes
before the REPL appears, even if the user only wanted to type `config get`.

The solution is lazy initialization via properties:

```python
@property
def movement(self) -> MovementApi:
    if self._movement is None:
        self._movement = MovementApi(self.config)
    return self._movement
```

`__init__` sets `_movement = None`. The `MovementApi` node is only created on the
first call to a movement method. Config-only and process commands never touch the
movement node at all.

The `get_robot_status` method reports which nodes are live without triggering their
creation:

```python
nodes_status = {
    "movement_api": "running" if self._movement else "not available",
    ...
}
```

Using the private `_movement` attribute rather than the property avoids the
side-effect of instantiating the node just to check its status.

## Launch process tracking

The `launch_process_ids` dict maps launch type names to in-flight process IDs.
It is seeded from config at init time so the set of valid launch types is known
before any launch is attempted:

```python
launch_templates = self.config.get_launch_templates()
self.launch_process_ids = dict.fromkeys(launch_templates.keys())
```

`_is_launch_running`, `_check_launch_conflict`, `_start_launch`, and `_stop_launch`
form a small state machine around this dict. Each public launch method is a one-liner
that delegates to these helpers.

## Possible improvements

- `robot_controller.py` is ~650 lines, well over the 300-line target. The map methods
  (`map_save`, `map_serialize`, `list_maps`) and the process introspection methods
  (`list_ros_processes`, `list_launch_processes`, `kill_ros_process`) each belong in
  their own module.
- The `[DEBUG]` print statements in `_start_launch` should be removed or replaced
  with `log_debug` calls.
- `turn_degrees` delegates to `self.movement.turn_degrees` which returns `None`, not
  a `CommandResponse`. This inconsistency will surface as a missing message in the CLI.
