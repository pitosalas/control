# IntentCommands — CLI Command Definitions for Intent Publishing

## Introduction

`intent_commands.py` follows the same pure-data pattern used throughout this codebase: it returns a dictionary of `CommandDef` objects and contains no logic of its own. This keeps command registration decoupled from execution — the dispatcher and controller never need to know where a command definition came from.

## The Registry Pattern

All command modules expose a single `build_*` function returning `dict[str, CommandDef]`. The `CommandDispatcher` calls each builder and merges the results into one flat registry at startup:

```python
def build_intent_commands() -> dict[str, cd.CommandDef]:
    return {
        "intent.stop": cd.CommandDef(...),
        "intent.explore": cd.CommandDef(...),
        "intent.describe_scene": cd.CommandDef(...),
        "intent.count_objects": cd.CommandDef(...),
    }
```

Keys use dot notation (`intent.stop`) which the CLI parser splits into `command="intent"`, `subcommand="stop"`.

## Zero-Argument vs. Parameterized Intents

Three intents carry no slots and need no parameters:

```python
"intent.stop": cd.CommandDef(
    method_name="publish_intent_stop",
    parameters=[],
    description="Publish stop intent to /intent topic",
    group="intent"
),
```

`count_objects` is the only intent with a required slot — the object type to count:

```python
"intent.count_objects": cd.CommandDef(
    method_name="publish_intent_count_objects",
    parameters=[
        pd.ParameterDef("object_type", str, True, None, "Type of object to count")
    ],
    description="Publish count_objects intent with required object_type",
    group="intent"
),
```

Setting `required=True` and `default=None` means the dispatcher will reject the command with a parameter error if `object_type` is omitted — no extra validation needed in the controller.

## Grouping

All four commands share `group="intent"`. This makes them discoverable as a unit:

```
help intent        → lists all intent subcommands
help commands      → shows INTENT group in the full command table
```

## Possible Improvements

- **Slot validation**: `object_type` is currently a free string. A future improvement could restrict it to a known vocabulary list and return a helpful error on unknown types.
- **Dynamic intent registration**: if the behavior manager publishes its supported intents at startup, this registry could be built dynamically rather than hardcoded.
