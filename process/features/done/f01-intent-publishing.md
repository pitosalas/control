# Feature description for feature F01
## F01 — Add intent publishing (voice/speech MVP)
**Priority**: High
**Done:** yes
**Tests Written:** yes
**Test Passing:** yes
**Description**: Extend control/ so the existing CLI REPL can publish normalized
intent messages to the `/intent` ROS2 topic. This is the minimal wire between the
CLI and the behavior manager (brain). Uses `std_msgs/String` JSON as wire format;
migration to a custom `Intent.msg` is deferred until the vocabulary stabilizes.
No new parser, no new REPL, no new entry point — existing infrastructure reused.

## How to Demo
**Setup**: ROS2 environment sourced; robot or sim not required (topic publish works standalone).

**Steps**:
1. Terminal 1: `ros2 topic echo /intent std_msgs/msg/String`
2. Terminal 2: `ros2 run control run`
3. In REPL: type `intent stop`
4. In REPL: type `intent count_objects object_type=chair`

**Expected output**: Terminal 1 shows JSON messages, e.g.:
```
data: '{"name": "stop", "source": "cli", "slots": {}}'
data: '{"name": "count_objects", "source": "cli", "slots": {"object_type": "chair"}}'
```
