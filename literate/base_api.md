# base_api — The ROS2 Node Foundation

## What it does and why

Every ROS2 API in this package needs a node identity, a logger, and access to the
configuration system. `BaseApi` is the single place those three concerns live. It
extends both `rclpy.node.Node` and Python's `ABC`, making every subclass a proper
ROS2 node that cannot be instantiated directly.

## The initialization contract

```python
def __init__(self, node_name: str, config_manager: cm.ConfigManager = None):
    super().__init__(node_name)
    self.config = config_manager or cm.ConfigManager()
```

A deliberate design decision: `BaseApi.__init__` does **not** call `rclpy.init()`.
That is the caller's responsibility. This follows the standard ROS2 convention and
avoids a subtle multi-node bug: if two `BaseApi` subclasses are constructed in the
same process and each tried to call `rclpy.init()`, the second call would silently
no-op. By moving `rclpy.init()` to the entry point (`main.py`), the lifetime of the
ROS2 context is explicit and owned by exactly one place.

## Shared utilities

The logging helpers (`log_debug`, `log_info`, `log_warn`, `log_error`) are thin
wrappers over the ROS2 logger. `log_info` uses `print` instead of the ROS2 logger
because ROS2's `INFO` level is often filtered in practice; print output always
appears on the terminal.

`check_bounds` provides a reusable safety guard for velocity and angle parameters:

```python
def check_bounds(self, value: float, min_val: float, max_val: float, name: str) -> bool:
    if not (min_val <= value <= max_val):
        self.log_warn(f"{name} out of bounds: {value}. Must be [{min_val}, {max_val}]")
        return False
    return True
```

Every movement method calls this before publishing to `/cmd_vel`. Returning `False`
rather than raising keeps caller code uniform: check the return, skip the publish.

## Possible improvements

- `log_info` using `print` is a workaround for ROS2 log filtering. A better fix is
  to set the node's logger level explicitly in the launch file or via `--ros-args`.
- The `config_manager` default fallback (`cm.ConfigManager()`) creates a second
  config object with default paths if none is passed. Callers should always pass one
  explicitly; the fallback masks misconfiguration.
