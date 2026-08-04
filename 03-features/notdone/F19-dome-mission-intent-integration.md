# Feature description for feature F19
## F19 — dome_mission `/intent` integration

**Priority**: High
**Done:** no
**Tasks File Created:** yes
**Tests Written:** no
**Test Passing:** no
**Description**: `dome_mission` (new sibling package) now also subscribes to
`/intent` and owns `exploration_start` / `exploration_stop` / `navigation_go`
/ `navigation_cancel` — driving Nav2 `NavigateToPose` and dome_nav's
`ExploreArea` action. `dome_control`'s `behavior_manager` was never updated
for that: it still owns `/intent` for its own behaviors (`stop`,
`drive_square`, `turn_right`, `turn_left`, `get_status`, `describe_scene`,
`count_objects`) and has one dead command left over from before
`dome_mission` existed. This feature closes the gap between the two
`/intent` owners — no code changes assumed yet, since the "stop" question
below needs a decision first.

## Findings (investigation, not yet acted on)

1. **No live double-dispatch today.** The two packages' intent-name sets are
   disjoint as of right now (`dome_control/dome_control/behaviors/
   motion_behavior.py:10` vs. `dome_mission/dome_mission/intent_parser.py`'s
   `NAME_TO_INTENT`). Confirmed by reading both handlers, not just grepping
   topic names.
2. **Dead command**: CLI `intent.explore` (`command_dispatcher.py:62`)
   publishes intent name `"explore"`, handled by `motion_behavior.py`'s
   `elif intent.name == "explore": pass  # not yet implemented`. The real,
   wired path is `nav.explore` (F18), which publishes `exploration_start` —
   the name `dome_mission` actually executes. Two CLI commands, both read as
   "start exploring," only one does anything.
3. **Open question — coordinated stop**: `stop` (`dome_control`) calls
   `rc.stop_robot()`, a direct cmd_vel halt. It does not know whether
   `dome_mission`'s FSM has an outstanding `ExploreArea` or
   `NavigateToPose` goal, so that goal stays active underneath the halted
   drive motors. Two sub-questions, undecided:
   - Should `dome_control`'s `stop` also publish `exploration_stop` /
     `navigation_cancel` so `dome_mission` cancels its goal too — and if so,
     unconditionally, or only when `dome_mission` reports an active mission?
   - Is `behavior_manager` the right place to own that coordination at all,
     or should `mission_node` itself also treat `stop` as a cancel signal
     (i.e. `dome_mission` adds `"stop"` to its own `NAME_TO_INTENT` alongside
     `navigation_cancel`/`exploration_stop`)?
4. **Not an issue**: `/explore/status` (`robot_controller.explore_status`)
   is still published live by `explorer_manager_node` post-T07 — no staleness
   there.
5. **Docs are silent**: `dome_control/02-doc/current.md` documents
   `behavior_manager` as if it were the only `/intent` owner; no mention of
   `dome_mission` anywhere in `dome_control`'s docs.

## How to Demo

**Setup**: both `behavior_manager` (dome_control) and `mission_node`
(dome_mission) running together on the Pi.

**Steps**:
1. `intent.explore` (CLI) — should no longer be offered/should visibly do
   nothing misleading (exact resolution depends on T01).
2. `nav explore` (CLI) — starts a real frontier-explore session via
   `dome_mission`, as today.
3. `stop` (CLI) mid-explore — resolution depends on the T02 design decision;
   demo confirms whichever behavior was chosen actually happens.
4. `ros2 node info /intent` (or equivalent) — confirms exactly the expected
   two subscribers, each handling its documented name set, no surprises.

**Expected output**: no dead/misleading commands left in the CLI; `stop`'s
interaction with an active mission matches the decision recorded in T02;
docs describe the split ownership accurately.
