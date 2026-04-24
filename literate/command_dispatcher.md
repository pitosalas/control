# CommandDispatcher — Central Command Registry and Executor

## Introduction

`CommandDispatcher` is the glue between the CLI parser and `RobotController`. It owns
a flat dictionary of all known commands, validates their parameters, and invokes the
matching method on the controller. No command module touches the controller directly;
no controller method knows anything about how it was invoked.

## Building the Registry

At construction, `_build_command_registry` merges dictionaries from every command
module into one flat map:

```python
def _build_command_registry(self) -> dict[str, cd.CommandDef]:
    commands = {}
    commands.update(mov_cmd.build_movement_commands())
    commands.update(ctrl_cmd.build_control_commands())
    commands.update(nav_cmd.build_navigation_commands())
    commands.update(lch_cmd.build_launch_commands())
    commands.update(sys_cmd.build_system_commands())
    commands.update(intent_cmd.build_intent_commands())
    return commands
```

Each `build_*` function returns `dict[str, CommandDef]`. Adding a new command group
means adding one import and one `update` call here — nothing else changes.

```
Registry key       →  CommandDef
─────────────────────────────────────────────────────
"move.distance"    →  method="move_distance",  params=[distance:float required]
"intent.stop"      →  method="publish_intent_stop", params=[]
"intent.count_objects" → method="publish_intent_count_objects", params=[object_type:str required]
```

## Execution Pipeline

`execute` is a three-stage pipeline: lookup → validate → invoke.

```python
def execute(self, command_name: str, params: dict[str, object]) -> rc.CommandResponse:
    if command_name not in self.commands:
        return rc.CommandResponse(success=False, message=f"Unknown command: {command_name}")

    command_def = self.commands[command_name]

    try:
        validated_params = self._validate_parameters(command_def, params)
    except ValueError as e:
        return rc.CommandResponse(success=False, message=f"Parameter error: {e!s}")

    try:
        method = getattr(self.robot_controller, command_def.method_name)
    except AttributeError:
        return rc.CommandResponse(success=False, message=f"Method {command_def.method_name} not found")

    try:
        result = method(**validated_params) if validated_params else method()
        ...
    except Exception as e:
        return rc.CommandResponse(success=False, message=f"Command execution error: {e!s}")
```

Each stage returns a failure `CommandResponse` on error, so the caller always gets the
same type regardless of what went wrong.

## Parameter Validation and Type Coercion

`_validate_parameters` iterates the `CommandDef`'s parameter list and:

1. Raises `ValueError` for any missing required parameter
2. Skips optional parameters that weren't supplied (no default injection if default is `None`)
3. Calls `_convert_parameter_value` to coerce strings to the declared type

```python
def _convert_parameter_value(self, param_def, value):
    if param_def.param_type == bool:
        if isinstance(value, str):
            return value.lower() in ("true", "1", "yes", "on")
        return bool(value)
    if param_def.param_type == str:
        return str(value)
    return param_def.param_type(value)
```

Boolean coercion from strings is the only special case — every other type just calls
its constructor on the raw value. This is intentionally simple: the CLI parser already
does basic tokenization; the dispatcher doesn't need to handle complex input forms.

## Possible Improvements

- **Duplicate method detection**: if two command modules register the same method name,
  the bug is silent. A validation pass at startup (`assert` all method names exist on
  `RobotController`) would catch mismatches immediately.
- **Group ordering**: `_build_command_registry` uses plain `dict.update`, so command
  group order in help output depends on insertion order. Explicit ordering would make
  the help text more predictable.
- **Schema-driven registration**: rather than one `build_*` function per module, a
  single YAML-driven registry would make adding commands a data change, not a code
  change.
