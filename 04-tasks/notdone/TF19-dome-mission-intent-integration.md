# Tasks for Feature F19

## T01 — Retire the dead `explore` intent
**Status**: not done
**Description**: Remove CLI command `intent.explore` (`command_dispatcher.py`
`BEHAVIOR_COMMANDS["intent.explore"]`) and the `"explore"` stub branch in
`motion_behavior.py` (`MOTION_INTENTS` entry + the `pass  # not yet
implemented` case). `nav.explore` (F18) is the real, wired path to
`dome_mission` now — keep one unambiguous "start exploring" command, not two.
Test: `test_motion_behavior.py` asserts `"explore"` is no longer in
`MOTION_INTENTS`; `test_command_dispatcher_text.py` asserts `intent.explore`
no longer resolves to a command.

## T02 — Decide the coordinated-stop design
**Status**: not done
**Description**: Resolve the two sub-questions recorded in F19's "Open
question — coordinated stop": (a) should `dome_control`'s `stop` also
publish `exploration_stop` / `navigation_cancel` so an active `dome_mission`
goal is cancelled too, unconditionally or only when a mission is actually
active; (b) should that coordination live in `behavior_manager`
(`dome_control`) or should `dome_mission` itself treat `"stop"` as a cancel
signal by adding it to its own `NAME_TO_INTENT`. Output of this task is a
recorded decision (append to F19's Description or a short addendum here) —
no code. Blocks T03.

## T03 — Implement coordinated stop
**Status**: not done
**Description**: Wire whichever design T02 settles on. Test: unit test
exercising the wiring (mock intent publisher or mock mission-cancel path,
per T02's decision) asserts `stop` produces the decided cancel behavior.

## T04 — Docs
**Status**: not done
**Description**: Update `dome_control/02-doc/current.md` to document the
split `/intent` ownership — which names `dome_control` handles
(`stop`, `drive_square`, `turn_right`, `turn_left`, `get_status`,
`describe_scene`, `count_objects`) vs. which `dome_mission` handles
(`exploration_start`, `exploration_stop`, `navigation_go`,
`navigation_cancel`), and the resolved stop behavior from T02.

## T05 — Live verification on the Pi
**Status**: not done
**Description**: Run `behavior_manager` and `mission_node` together on
hardware. Confirm: `intent.explore` is gone from the CLI; `nav.explore`
still drives a real `dome_mission` explore session; `stop` behaves per the
T02 decision; no intent name is double-dispatched. Hardware/runtime-only —
no plain-suite test; record manual test notes (command, setup, expected vs.
actual observation).

## T06 — Test-writing task
**Status**: not done
**Description**: Fill in/adjust unit test coverage for T01 (removed
`"explore"` stub, removed `intent.explore` command) and T03 (coordinated
stop wiring); record T05's manual test notes after a real run on the Pi.
