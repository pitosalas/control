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
**Status**: done
**Description**: Resolve the two sub-questions recorded in F19's "Open
question — coordinated stop". **Decision (2026-08-04, see F19's
Decisions section): no automatic coordination.** `stop` stays a pure
`dome_control` motor halt; cancelling an active `dome_mission` goal remains
a separate, explicit command (`nav cancel` / `nav explore.stop` today,
renamed to `mission cancel` / `mission explore.stop` under F21). No code
change required in `dome_control`/`dome_mission` for this decision itself —
the cancel path already exists; only the CLI domain rename (F21) follows
from it.

## T03 — Implement coordinated stop
**Status**: not applicable — superseded by T02's decision
**Description**: Not needed. T02 decided against coordinated stop; no
wiring to implement. Left in place (not deleted) for traceability of why
this task isn't done.

## T04 — Docs
**Status**: not done
**Description**: Update `dome_control/02-doc/current.md` to document the
split `/intent` ownership — which names `dome_control` handles
(`stop`, `drive_square`, `turn_right`, `turn_left`, `get_status`,
`describe_scene`, `count_objects`) vs. which `dome_mission` handles
(`exploration_start`, `exploration_stop`, `navigation_go`,
`navigation_cancel`), and the resolved stop behavior from T02 (no
coordination; `stop` and `mission cancel` are separate commands).

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
