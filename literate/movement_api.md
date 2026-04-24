# movement_api — Velocity Control and Odometry

## What it does and why

`MovementApi` is the single path from high-level commands ("move 1 metre forward")
to ROS2 `/cmd_vel` Twist messages. It owns the publisher, the odometry subscriber,
and the battery subscriber. All motion logic — time calculation, direction signs,
safety limits — lives here so `RobotController` stays free of physics.

## Configuration as live properties

Speed limits are not stored as instance variables. They are properties that read
from the config store on each access:

```python
@property
def linear(self) -> float:
    return self.config.get_variable("linear_speed")
```

This means `config set linear_speed 0.4` takes effect on the next movement command
without restarting the node. The tradeoff is a dict lookup per call, which is
negligible compared to the movement duration.

`_validate_config` runs at init time to catch a missing YAML key early — before the
robot tries to move — rather than at the moment of first motion.

## The blocking loop and its guard

`cmd_vel_helper` is where all motion actually happens:

```python
def cmd_vel_helper(self, linear: float, angular: float, seconds: float):
    # Blocks the calling thread for `seconds`. Must not be called from a
    # lifecycle node spin context — set blocking_ok = False there to catch this.
    if not self.blocking_ok:
        raise RuntimeError("cmd_vel_helper called from a non-blocking context")
    ...
    while rclpy.ok() and (time.time() - start_time) < seconds:
        self.cmd_vel_pub.publish(twist)
        rclpy.spin_once(self, timeout_sec=sleep_duration)
        time.sleep(sleep_duration)
```

The loop publishes at 10 Hz and interleaves `spin_once` to service any incoming
callbacks (odometry, battery) during motion. This works correctly in a CLI context
where the calling thread can block. It would deadlock if called from inside a ROS2
timer or subscription callback — the spin inside the loop would re-enter the
executor.

The `blocking_ok` class attribute is the guard. A lifecycle node subclass sets
`blocking_ok = False` in its class body; any accidental call to `cmd_vel_helper`
from a spin context raises `RuntimeError` immediately rather than hanging silently.

## Safety limits

Every call checks velocity bounds before publishing:

```python
def check_velocity_limits(self, linear: float, angular: float) -> bool:
    linear_ok = self.check_bounds(linear, self.linear_min, self.linear_max, "linear velocity")
    angular_ok = self.check_bounds(angular, self.angular_min, self.angular_max, "angular velocity")
    return linear_ok and angular_ok
```

Limits come from config (`linear_min`, `linear_max`, `angular_min`, `angular_max`).
Tuning them in YAML is enough to change the robot's safe operating envelope.

## Possible improvements

- The blocking loop is the right design for a simple CLI but the wrong design for a
  lifecycle node. A timer-based approach (publish at each timer tick, cancel after N
  ticks) would work in both contexts.
- `stop()` calls `cmd_vel_helper(0, 0, 0)`, which skips the loop and publishes a
  zero twist immediately — correct, but not obvious from the implementation.
- `current_pose` and `current_voltage` are set by callbacks but never read internally.
  They are available for callers that need them, but adding a `get_pose()` accessor
  would make that contract explicit.
