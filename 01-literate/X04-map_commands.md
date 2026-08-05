---
version: "1.0"
generated: "2026-08-05"
---

# Map Commands

`map_commands.py` is the command registry for SLAM map persistence operations in
dome_control. It returns a flat dictionary of `CommandDef` entries keyed by dotted names
like `"map.save"`. The calling infrastructure in `CommandDispatcher` uses these keys to
route CLI input to the right method on `RobotController`.

The file imports one thin descriptor type rather than any ROS2 machinery:

```python
import dome_control.commands.command_def as cd
```

This is deliberate: command *definitions* are pure data. No nodes, no publishers, no
subscriptions live here. The ROS2 plumbing lives in `RobotController`; this file just
describes the surface area.

(This module was split out of the former `navigation_commands.py` — see
`03-features/done/F21-cli-syntax-orthogonality.md` D5 — once the `nav`/`map` groups it
mixed together no longer matched the file's own name.)

---

## Map group

Three commands handle persistence of the SLAM map. All three are parameterless because the
map name is read from the `map_name` config variable at execution time, not typed on the
command line. This avoids the error-prone pattern of re-typing a filename and ensures the
same name is used consistently across save and serialize operations.

```python
"map.save": cd.CommandDef(
    method_name="map_save",
    parameters=[],
    description="Save current map to maps/ folder (uses map_name variable)",
    group="map"
),
"map.list": cd.CommandDef(
    method_name="list_maps",
    parameters=[],
    description="List available maps in maps/ folder",
    group="map"
),
"map.serialize": cd.CommandDef(
    method_name="map_serialize",
    parameters=[],
    description="Save current map in SLAM Toolbox serialized format (uses map_name variable)",
    group="map"
),
```

`map.save` drives `nav2_map_server map_saver_cli`, producing the standard `.pgm`/`.yaml`
pair. `map.serialize` calls the SLAM Toolbox `SerializePoseGraph` service, which saves the
full graph (not just the occupancy grid) so that localization can resume exactly where it
left off. Both are synchronous shell-out operations wrapped in `ProcessApi.run_command_sync`
inside `RobotController`.

`map.list` is a read-only convenience command that scans the `maps/` folder and prints
available files. It requires no ROS2 interaction at all.
