# Feature description for feature F21
## F21 — CLI syntax orthogonality cleanup

**Priority**: Low
**Done:** no
**Tasks File Created:** yes
**Tests Written:** no
**Test Passing:** no
**Description**: The `domain.action[.subaction]` command grammar
(`command_dispatcher.py` + the `commands/*_commands.py` builders) is applied
inconsistently across domains — the same underlying shape (a directional pair,
a start/stop/status triad, an optional-parameter command) is expressed
differently depending on which domain happens to have it. This feature
proposes a consistent convention and the renames/removals needed to reach it.
Pure dead-code/bug fixes (duplicate `intent.*`/`scene.*` commands, the dropped
`scene.count` argument, `config.set`'s 0/1-as-boolean bug) are tracked
separately in `04-tasks/chores.md`, not here — this feature is only the
design-level inconsistencies that need a decision, not a fix.

Abbreviations (`ABBREV_TO_FULL`) are explicitly out of scope — reviewed and
kept as-is.

## Decisions (made 2026-08-04 — ready for a task list)

**D1 — Directional commands duplicate the signed generic command.**
`move.forward`/`move.backward` are `move.distance` with the sign pre-applied;
`turn.clockwise`/`turn.counterclockwise` are `turn.degrees` with the sign
pre-applied.
**Decision: keep the directional pairs only.** Drop `move.distance` and
`turn.degrees` as generic signed forms; `move.forward`/`move.backward` and
`turn.clockwise`/`turn.counterclockwise` become the sole way to express
movement/turning by amount. (`turn.radians` is unaffected — no directional
equivalent exists for it, and it isn't part of this duplication.)

**D2 — Units are a command-name suffix for `turn`, not for `move`.**
No change — `turn.degrees`/`turn.radians` staying a deliberate, documented
convention that `move` doesn't also need. (Note: D1 removes `turn.degrees` as
a *generic signed* command; `turn.radians` remains, so the degrees/radians
unit split survives via `turn.radians` alone once `turn.clockwise`/
`turn.counterclockwise` cover the degrees case.)

**D3 — Four different shapes for "start/stop/status of a long-running
behavior."**
**Decision: explicit 3-level everywhere.** `nav.explore` → `mission.explore.start`,
`nav.go` → `mission.go.start`, `nav.cancel` → `mission.go.stop`. `launch.start`/
`launch.stop` stay 2-level as-is (`launch_type` is a parameter there, not a
name slot — a different shape by necessity, not inconsistency). `survey.start`
is unaffected (still no stop/status — out of scope here).

**D3a — `nav` domain renamed to `mission` (added 2026-08-04, follows from
F19's T02 decision).** Every command in this domain talks to `dome_mission`,
not to navigation logic inside `dome_control` — the name `nav` was
misleading. Rename the domain itself: `nav.*` → `mission.*` throughout
(`mission.go.start`, `mission.go.stop`, `mission.explore.start`,
`mission.explore.stop`, `mission.explore.status`). This makes `stop`
(talks to `dome_control`, unchanged) and `mission.go.stop`/
`mission.explore.stop` (talks to `dome_mission`) unambiguous by name alone
— see F19's Decisions section for why `stop` and mission-cancel stay
separate, uncoordinated commands.

**D4 — `CommandDef`/`ParameterDef` model optional named parameters; the
parser only supports positional-by-index.**
**Decision: document positional-only behavior, no code change.** State
plainly (in `02-doc/cli-reference.md` and `launch.start`'s help text) that
optional parameters must be supplied in order or not at all.

**D5 — `group=` metadata doesn't match the file that defines it.**
**Decision: split `navigation_commands.py` into `map_commands.py` +
`mission_commands.py`.** Matches the one-file-one-group pattern every other
domain already follows; `build_map_commands()` and `build_mission_commands()`
replace `build_navigation_commands()`.

## How to Demo
**Setup**: `ros2 run dome_control run` (or non-interactive `dome_control run
<command>`).

**Steps**:
1. `move forward 1` / `move backward 1` / `turn clockwise 90` /
   `turn counterclockwise 90` still work; `move distance 1` and
   `turn degrees 90` now return `Unknown command`.
2. `nav go <label>` fails with `Unknown command` (domain renamed);
   `mission go start <label>` works. `mission explore start` works.
   `mission go stop` works (cancels an active mission goal). `stop` (no
   domain prefix) still halts motors only, per F19's Decisions.
3. `help commands` shows `map.*` and `mission.*` grouped exactly as `map.*`/
   `nav.*` did before (grouping output is unchanged — only the source file
   moved and the domain name changed).

**Expected output**: renamed/removed commands behave as listed above; nothing
else in the CLI changes.
