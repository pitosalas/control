# Chores

Simple bug fixes and refactors — no spec/behavior change, no feature/task pair.
Each entry is one line, flipped to `[x]` when applied. Still gets a test.

- [ ] Remove duplicate `intent.*` CLI commands (`intent.stop`, `intent.explore`,
      `intent.describe_scene`, `intent.count_objects`, `intent.list_objects`) —
      `scene.*` already covers the same actions via the same `BEHAVIOR_COMMANDS`
      targets. Keep `scene.*` as the sole spelling; delete the `intent.*` entries
      from `BEHAVIOR_COMMANDS`/`BEHAVIOR_COMMAND_DESCRIPTIONS` in
      `command_dispatcher.py`, and delete the now-fully-unreachable
      `RobotController.publish_intent_stop/explore/describe_scene/count_objects`
      methods (never called by any `CommandDef` today). Test: dispatching
      `"intent stop"` returns `Unknown command: intent.stop`; `scene.describe`/
      `scene.count`/`scene.objects` still dispatch successfully.
- [ ] Give `scene.describe`/`scene.count`/`scene.objects` real `CommandDef`
      parameter lists instead of the empty-`parameters=[]` placeholder used for
      the `BEHAVIOR_COMMANDS` bypass, so `scene.count <object_type>` actually
      forwards its argument as an intent slot instead of silently dropping it.
      Test: dispatch `"scene count trash"` and assert the published intent's
      slots include `object_type: "trash"`.
- [ ] Stop `config.set` swallowing numeric `0`/`1` as booleans. Root cause:
      `command_dispatcher.parse_value()` treats `"1"`/`"0"` as boolean
      true/false (matching `("true","yes","1")`/`("false","no","0")`) before
      the value ever reaches `ConfigManager`, which has its own — correct —
      numeric detection that never gets a chance to run. Narrow
      `parse_value()`'s boolean check to `true`/`false`/`yes`/`no` only,
      dropping `1`/`0` from it (the very next branch already handles numeric
      strings). Test: `config set count 1` stores integer `1`, not boolean
      `True`; `config set flag yes` still stores boolean `True`.
- [x] Move the control config from `~/.control/config.yaml` to
      `~/.dome/config/control.yaml`. Updated the default path in
      `simple_cli.py` and `behavior_manager_node.py`, the four hardcoded
      test paths (`test_f02_cleanup.py`, `test_launch_commands.py`,
      `test_robot_controller_launch.py`, `test_script_commands.py`), the
      example `config/control-config.yaml` and `config/mapper_params_online_async.yaml`
      map path, and `01-literate/14-simple_cli.md`. Test: existing suite
      exercises the default path computation; full suite passes locally with
      a real `~/.dome/config/control.yaml` in place (copied from
      `dome-docker/runtime-data/control/config.yaml`).
- [x] Drop the `CONTROL_CONFIG` env var override — config path is now fixed
      at `~/.dome/config/control.yaml` (no more per-invocation override).
      Removed the `os.environ.get("CONTROL_CONFIG", ...)` calls in
      `simple_cli.py` and `behavior_manager_node.py`; removed
      `TestT02ConfigPath` from `test_f02_cleanup.py` (it tested the env-var
      override we removed) and the matching lookup in
      `test_robot_controller_launch.py`. Test: new
      `test_config_path_is_fixed_under_dome_config` in
      `test_simple_cli_formatting.py` asserts `CONTROL_CONFIG` no longer
      appears in `SimpleCLI.__init__`'s source and the fixed path segments do.
- [ ] `README.md`'s "Available Commands" section and `docs/_index.txt` are
      stale and inaccurate against the actual command registry (documents a
      `--flag`/`-m` option syntax that doesn't exist, several commands that
      don't exist, and abbreviations that aren't in `ABBREV_TO_FULL`). Replace
      both with a short pointer to `02-doc/cli-reference.md` (the
      registry-verified reference) rather than hand-maintained prose that will
      drift again. No test — documentation only.
