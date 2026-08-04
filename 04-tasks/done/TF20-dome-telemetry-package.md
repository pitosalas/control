# Tasks for Feature F20

Task file name must be `TFNN-<slug>.md` where `NN` matches the feature number.
For each task step, add a test when feasible. If not feasible, note why.

## T01 — Add `BatteryStats.msg` to `dome_telemetry_msgs`
**Status**: done
**Description**: New message file `dome_telemetry_msgs/msg/BatteryStats.msg`:
`std_msgs/Header header`, `float32 bus_voltage_v`, `float32 current_a`,
`float32 power_w`, `float32 percent`. Register in that package's
`CMakeLists.txt` alongside `OakStats.msg`. No behavior to unit test (msg
generation); verified by building and importing
`dome_telemetry_msgs.msg.BatteryStats` in T04's test.

## T02 — Scaffold the `dome_telemetry` ament_python package
**Status**: done
**Description**: New package directory `dome_telemetry/` at the workspace
`src/` level (sibling to `dome_control`), `package.xml` + `setup.py` +
`resource/dome_telemetry` marker, depending on `rclpy`,
`dome_telemetry_msgs`, `ament_index_python`. Follow the file-header and
package-layout conventions in `.claude/style_guide.md`. No test — scaffold
only, verified by `colcon build --packages-select dome_telemetry`.

## T03 — Move `ups_status.py` from `dome_control` to `dome_telemetry`
**Status**: done
**Description**: Move `dome_control/dome_control/ups_status.py` (INA219
driver, `SocEstimator`, `read_ups_stats`, `UpsStats`) to
`dome_telemetry/dome_telemetry/ups_status.py`. Remove it and its `smbus`
dependency from `dome_control` entirely. Move its existing test
(`test/test_telemetry_ups_stats.py`) to the new package's `test/` directory,
updating the import path; test bodies are unchanged (they already mock the
INA219 handle, no hardware needed).

## T04 — Implement `BatteryTelemetryNode`
**Status**: done
**Description**: `dome_telemetry/dome_telemetry/nodes/battery_telemetry_node.py`.
Opens the INA219 once at startup (fail-soft: log a warning and keep the node
alive with no publishes if the device is absent, matching `TelemetryNode`'s
existing fail-soft pattern), then on a single 60s timer reads UPS stats and
publishes a stamped `BatteryStats` on `/telemetry/battery`. No CSV logging, no
other timers, no other topics. Test: build the `BatteryStats` message from a
given `UpsStats` snapshot and assert field mapping (pure function, no ROS
context needed); mock the INA219 open failure path and assert the node stays
up with no publish.

## T05 — Switch `dome_control`'s `TelemetryNode` to subscribe to `/telemetry/battery`
**Status**: done
**Description**: Replace `UpsReader` (which opens the INA219 directly) with a
subscription to `/telemetry/battery` (`dome_telemetry_msgs/BatteryStats`),
following the existing `latest_oak` / `on_oak` pattern used for OAK stats.
`build_message()` maps the latest cached `BatteryStats` onto the combined
`Telemetry` message's `ups_*` fields; `None` (nothing received yet) leaves
them at zero, unchanged from current fail-soft behavior. Remove the `smbus`
exec dependency from `dome_control`'s `package.xml`. Test: update
`test_telemetry_ups_stats.py`'s dome_control-side coverage (now
`test_telemetry_node.py`'s UPS-field mapping) to feed a `BatteryStats` message
into `build_message()` instead of a `UpsStats`, asserting the same `ups_*`
output fields.

## T06 — Launch file + `gendrv.launch.py` wiring
**Status**: done
**Description**: `dome_telemetry/launch/robot.launch.py` (single `bl.node()`
call for `battery_telemetry_node`, `better_launch` style per repo convention).
Add `bl.include("dome_telemetry", "robot.launch.py")` to
`dome2/launch/gendrv.launch.py` so it starts with hardware driver bringup. No
test — launch wiring, verified manually per the feature's demo steps.

## T07 — Update docs
**Status**: done
**Description**: `02-doc/current.md`: note the new `dome_telemetry` package,
its launch point (via `gendrv.launch.py`), and the `TelemetryNode` UPS-source
change. Regenerate `01-literate/28-ups_status.md` for its new location and add
a literate doc for `battery_telemetry_node.py`, per `.claude/process.md`'s
literate-docs rule. No test — documentation only.
