# CommandDef and ParameterDef — The Command Registry Schema

## Purpose

`CommandDef` and `ParameterDef` are the schema types for the command registry. Every command the robot understands is described by a `CommandDef`; every argument that command accepts is described by a `ParameterDef`. These two dataclasses live across `command_def.py` and `parameter_def.py` but are documented together because they are meaningless in isolation.

## ParameterDef

```python
@dataclass
class ParameterDef:
    name: str
    param_type: type
    required: bool
    default: object
    description: str
```

`param_type` carries the Python type object (`str`, `int`, `float`, `bool`) rather than a string name. This lets the dispatcher coerce incoming string arguments without a lookup table — it calls `param_type(value)` directly.

`default` is `object` rather than a typed optional because a default can be any type, including `None`. The `required` field determines whether `default` is meaningful: if `required` is `True`, `default` is ignored.

## CommandDef

```python
@dataclass
class CommandDef:
    method_name: str
    parameters: list[pd.ParameterDef]
    description: str
    group: str
```

`method_name` is the string name of the method on `RobotController` that the dispatcher calls via `getattr`. This indirection means adding a new command never requires touching the dispatcher — only the command builder and the controller need to change.

`group` clusters commands for help display (e.g. `"launch"`, `"control"`, `"movement"`). It is not used for routing.

## How They Fit Together

```
build_launch_commands()  ──►  dict[str, CommandDef]
                                     │
                              CommandDispatcher
                                     │
                        getattr(robot_controller, method_name)(**params)
```

The command builder functions (`build_launch_commands`, `build_control_commands`, etc.) return plain dicts. The dispatcher merges these dicts into one registry at startup. Lookup is O(1) by command name string.

## Possible Improvements

- **`default` on `ParameterDef` is untyped.** A `Generic[T]` dataclass or `param_type: type[T]` with `default: T | None` would let type checkers catch mismatches between `param_type` and `default`.
- **`required=False` with `default=None` is the dominant pattern**, but nothing enforces that non-required params have a usable default. A `__post_init__` validator could catch `required=False, default=None` for non-nullable types.
- **`group` is a free string.** A `StrEnum` would prevent typos silently creating orphaned groups in the help output.
