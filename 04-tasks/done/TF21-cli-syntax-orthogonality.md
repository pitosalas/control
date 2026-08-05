# Tasks for Feature F21

Task file name must be `TFNN-<slug>.md` where `NN` matches the feature number.
For each task step, add a test when feasible. If not feasible, note why.

## T01 — Remove `move.distance` and `turn.degrees` as generic signed commands (D1)
**Status**: done
**Description**: Delete the `move.distance` and `turn.degrees` `CommandDef`
entries from `movement_commands.py`. `RobotController.move_distance` and
`RobotController.turn_degrees` become unreachable from the CLI — delete them
too unless something else still calls them directly (check
`MotionBehavior`/other internal callers before deleting). Keep
`move.forward`/`move.backward`/`turn.clockwise`/`turn.counterclockwise` and
`turn.radians` unchanged. Test: `dispatch_text("move distance 1")` and
`dispatch_text("turn degrees 90")` both return `Unknown command`; the four
directional commands and `turn.radians` still dispatch successfully.

## T02 — Rename `nav.explore` → `mission.explore.start` (D3, D3a)
**Status**: done
**Description**: In `mission_commands.py` (post-T05 split), rename the
`nav.explore` `CommandDef` key to `mission.explore.start` (domain rename
`nav` → `mission` per D3a folds in here — one rename, not two passes). No
change to `method_name` (`publish_intent_exploration_start`) or behavior.
Test: `dispatch_text("mission explore start")` publishes `exploration_start`;
`dispatch_text("nav explore")` and `dispatch_text("mission explore")` both
return `Unknown command`.

## T03 — Rename `nav.go` → `mission.go.start`, `nav.cancel` → `mission.go.stop` (D3, D3a)
**Status**: done
**Description**: Rename the `nav.go` `CommandDef` key to `mission.go.start`
(same `method_name`/params, `publish_intent_navigation_go`, `label`
parameter). Rename `nav.cancel` to `mission.go.stop` (same `method_name`,
`publish_intent_navigation_cancel`). Test: `dispatch_text("mission go start
kitchen")` publishes `navigation_go` with `label=kitchen`;
`dispatch_text("mission go stop")` publishes `navigation_cancel`;
`dispatch_text("nav go kitchen")` and `dispatch_text("nav cancel")` both
return `Unknown command`.

## T03a — Rename remaining `nav.*` commands to `mission.*` (D3a)
**Status**: done
**Description**: Rename `nav.explore.stop` → `mission.explore.stop` and
`nav.explore.status` → `mission.explore.status` (same `method_name`s,
no behavior change). Test: `dispatch_text("mission explore stop")`
publishes `exploration_stop`; `mission explore status` returns the same
value `nav explore status` did; both `nav.*` forms return `Unknown
command`.

## T04 — Document positional-only optional parameters (D4)
**Status**: done
**Description**: Add a short note to `02-doc/cli-reference.md`'s `launch.*`
section (already partially states this) and to `docs/launch.start.txt` (if
kept per the chores.md doc-replacement item) making explicit: optional
parameters must be supplied in the declared order or omitted entirely — there
is no way to skip to a later optional parameter. No behavior change. No test
— documentation only.

## T05 — Split `navigation_commands.py` into `map_commands.py` + `mission_commands.py` (D5, D3a)
**Status**: done
**Description**: Move the `map.*` `CommandDef` entries into a new
`map_commands.py` (`build_map_commands()`), and the `nav.*` entries (as
renamed by T02/T03/T03a to `mission.*`) into a new `mission_commands.py`
(`build_mission_commands()`). Delete `navigation_commands.py`. Update
`command_dispatcher.py`'s `_build_command_registry()` to import and call
both builders in place of `build_navigation_commands()`, and update the
`group=` metadata on the moved commands from `nav` to `mission`. Test:
`help commands` output (grouped by `group=`) shows `map` and `mission`
groups with the same command sets as the old `map`/`nav` groups (adjusted
for the T02/T03/T03a renames); `list_commands(group="map")` and
`list_commands(group="mission")` return those sets; `list_commands(group="nav")`
returns empty.

## T06 — Update `02-doc/cli-reference.md` for the renamed/removed commands
**Status**: done
**Description**: Reflect T01–T03a's renames and removals in the reference
doc written for this session (movement table loses `move.distance`/
`turn.degrees` rows; the `nav` table is renamed to `mission` and its rows
become `mission.go.start`/`mission.go.stop`/`mission.explore.start`/
`mission.explore.stop`/`mission.explore.status`). No test — documentation
only.
