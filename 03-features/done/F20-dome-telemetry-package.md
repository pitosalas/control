# Feature description for feature F20
## F20 — standalone dome_telemetry package for battery telemetry

**Priority**: Medium
**Done:** yes
**Tasks File Created:** yes
**Tests Written:** yes
**Test Passing:** yes
**Description**: Move battery telemetry out of `dome_control` into a new,
standalone ROS2 package `dome_telemetry`, launched alongside `gendrv` (hardware
driver bringup) rather than with the control/vision/voice stack. For now it
covers battery telemetry only — reading the INA219 UPS and publishing a
`BatteryStats` message once every 60 seconds. It is designed for near-zero
robot impact: one lightweight node, one 60s timer, no CSV logging, no other
sensor sources, no dependency on the rest of the stack being up.

`dome_control`'s existing combined `/telemetry` topic (`Telemetry.msg`, F17)
keeps its current shape. Its `TelemetryNode` currently owns the INA219 handle
directly (`UpsReader`); this changes it to subscribe to the new
`/telemetry/battery` topic instead — the same pattern already used for OAK
stats (subscribed from `/telemetry/oak` rather than opened directly), since
only one owner should hold a hardware handle. Battery fields in the combined
message will refresh every 60s instead of every publish tick; this is
acceptable since a 3S LiPo's charge state does not move meaningfully faster
than that.

`ups_status.py` (INA219 driver, `SocEstimator`, `read_ups_stats`) moves from
`dome_control` to `dome_telemetry`. `dome_control` no longer imports smbus or
opens I2C.

**New message**: `dome_telemetry_msgs/BatteryStats.msg` — header +
`bus_voltage_v`, `current_a`, `power_w`, `percent`. Added to the existing
`dome_telemetry_msgs` package (already hosts `OakStats`, same flat-field
Foxglove-friendly style).

**Launch**: `dome_telemetry` node is wired into `dome2/launch/gendrv.launch.py`
so it starts with hardware driver bringup, independently of whether
control/vision/voice are launched.

## How to Demo
**Setup**: Robot hardware with INA219 UPS wired; `bl dome2 gendrv.launch.py`
(or the relevant `dome2 robot.launch.py --options d` drivers-only run).

**Steps**:
1. Launch drivers: `bl dome2 gendrv.launch.py`.
2. `ros2 topic echo /telemetry/battery` — observe one message every ~60s with
   plausible `bus_voltage_v`/`percent` values.
3. Separately launch `dome_control` (`bl dome_control robot.launch.py`) and
   `ros2 topic echo /telemetry` — confirm `ups_*` fields are still populated,
   sourced from `/telemetry/battery` rather than a direct INA219 read.

**Expected output**: `/telemetry/battery` publishes independently of the
control stack, at 60s intervals; `dome_control`'s combined `/telemetry` still
carries battery fields when both are running.
