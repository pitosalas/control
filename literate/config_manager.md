# ConfigManager — Robot Configuration Storage

## What It Does and Why

`ConfigManager` owns the robot's persistent configuration: a single YAML file on disk that stores calibration values, path settings, and runtime flags. Every other module receives a `ConfigManager` instance rather than reading the file directly — this keeps I/O isolated and makes testing straightforward.

## Two-Phase Construction

The most deliberate design choice here is separating object construction from initialization side effects. `__init__` does only assignments — it establishes the file path and an empty variable dict, nothing more:

```python
def __init__(self, config_file: str):
    self.config_file = Path(config_file)
    self.control_dir = self.config_file.parent
    self.variables: dict[str, object] = {}
```

Callers that want a fully loaded manager use the factory:

```python
@classmethod
def create(cls, config_file: str) -> "ConfigManager":
    manager = cls(config_file)
    manager.load_config()
    manager.ensure_subdirs()
    manager.detect_test_environment()
    return manager
```

This matters because tests can construct a bare `ConfigManager` and manually populate `variables` without touching the filesystem, while production code always calls `create()`.

## Storing Values: Type Coercion at the Boundary

All external input enters through `set_variable`. Values from CLI or YAML arrive as strings, but the config should store native Python types so callers never have to parse. The coercion chain:

```python
def set_variable(self, name: str, value: object):
    if not isinstance(value, str):
        self.variables[name] = value          # already native type
    elif value.lower() in ["true", "false"]:
        self.variables[name] = value.lower() == "true"
    elif self.is_number(value):
        self.variables[name] = self.convert_number(value)
    else:
        self.variables[name] = value          # keep as string
    self.save_config()
```

The write-through to disk on every `set_variable` call is intentional: the robot may be power-cycled at any time, and a partial write is safer than a missed write.

## Number Detection

`is_number` uses `float()` as the oracle — it handles integers, floats, and scientific notation in one shot:

```python
def is_number(self, value: str) -> bool:
    try:
        float(value)
        return True
    except ValueError:
        return False
```

`convert_number` then distinguishes int from float by the presence of a dot:

```python
def convert_number(self, value: str) -> int | float:
    if "." in value:
        return float(value)
    return int(value)
```

## Path Resolution

All paths in the config are relative to `~/.control/` unless absolute. `resolve_path` enforces this:

```python
def resolve_path(self, path_str: str) -> Path:
    path = Path(path_str).expanduser()
    if path.is_absolute():
        return path
    return (self.control_dir / path).resolve()
```

This means a config entry like `log_dir: logs` resolves to `~/.control/logs`, while `/tmp/logs` passes through unchanged.

## Test Environment Detection

`detect_test_environment` checks the live module registry for `pytest` or `unittest` and automatically enables dry-run mode:

```python
def detect_test_environment(self):
    if "pytest" in sys.modules or "unittest" in sys.modules:
        self.variables["dry_run"] = True
```

This prevents tests from sending actual ROS2 commands without requiring any test-specific setup from the caller. The tradeoff: any code importing pytest (even transitively) will trigger this, which could theoretically mask bugs in mixed environments.

## Data Flow

```
ConfigManager.create(path)
        │
        ├── load_config()       reads YAML → self.variables
        ├── ensure_subdirs()    creates maps/ and logs/ if absent
        └── detect_test_environment()  sets dry_run if under test

set_variable(name, value)
        │
        ├── coerce type
        ├── self.variables[name] = coerced_value
        └── save_config()       writes entire dict to YAML
```

## Possible Improvements

- **Write-through is expensive** for bulk updates. A `batch_set` context manager that defers `save_config()` until exit would help calibration routines that set many values at once.
- **`is_number` / `convert_number` are public** but only exist to serve `set_variable`. They could be module-level functions or a private static method — exposing them invites misuse.
- **No schema validation.** Unknown keys are silently accepted. A whitelist of valid variable names would catch typos at the boundary.
- **`detect_test_environment` is a heuristic.** An explicit `dry_run=True` parameter to `create()` would be cleaner and testable without relying on module introspection.
