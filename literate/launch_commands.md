# Launch Commands — Command Registry for Launch Management

## Purpose

`build_launch_commands` registers the four commands that control ROS2 launch processes: `launch.list`, `launch.info`, `launch.start`, and `launch.stop`. This file is purely declarative — no logic, no branching, just a mapping from command names to their `CommandDef` schemas.

## Structure

```python
def build_launch_commands() -> dict[str, cd.CommandDef]:
    return {
        "launch.list": cd.CommandDef(...),
        "launch.info": cd.CommandDef(...),
        "launch.start": cd.CommandDef(...),
        "launch.stop":  cd.CommandDef(...),
    }
```

The dot-separated key (`"launch.start"`) encodes both the group and the subcommand in a single string. The dispatcher uses this key directly; the CLI splits on the dot to reconstruct `command.subcommand` form for user display.

## The launch.start Command

`launch.start` is the most complex entry — it accepts four parameters, three of which are optional:

```python
"launch.start": cd.CommandDef(
    method_name="launch_start",
    parameters=[
        pd.ParameterDef("launch_type", str, True,  None, "Launch type (nav, slam, map)"),
        pd.ParameterDef("use_sim_time", bool, False, None, "Use simulation time"),
        pd.ParameterDef("map",          str,  False, None, "Map filename without extension"),
        pd.ParameterDef("map_name",     str,  False, None, "Map name for map launch type"),
    ],
    ...
)
```

`launch_type` is the only required argument. The rest are optional flags passed through to the underlying `ros2 launch` command.

## Possible Improvements

- **`map` and `map_name` are redundant-looking.** The distinction between them (one for nav, one for map launch type) is not obvious from the parameter names alone. A description field that's only visible in `launch info` output is easy to miss.
- **No validation that `launch_type` is one of `nav`, `slam`, `map`.** This is caught at runtime by `ProcessApi`, but a `choices` field on `ParameterDef` would surface the error earlier with a better message.
- **`use_sim_time` as `bool` with `default=None`** is inconsistent — a bool with no default should probably default to `False`. The dispatcher will pass `None` if the flag is omitted, and the controller must handle that.
