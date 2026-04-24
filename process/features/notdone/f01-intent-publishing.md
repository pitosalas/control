# Feature description for feature F01
## F01 — Add intent publishing (voice/speech MVP)
**Priority**: High
**Done:** no
**Tests Written:** no
**Test Passing:** no
**Description**: Extend control/ so the existing CLI REPL can publish normalized
intent messages to the `/intent` ROS2 topic. This is the minimal wire between the
CLI and the behavior manager (brain). Uses `std_msgs/String` JSON as wire format;
migration to a custom `Intent.msg` is deferred until the vocabulary stabilizes.
No new parser, no new REPL, no new entry point — existing infrastructure reused.
