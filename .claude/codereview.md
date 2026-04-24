# Code Review Checklist

Derived from `.claude/coding.md`. Apply to each Python file under review.

## Workflow
1. Apply this checklist to the target file.
2. Record each violation as a task in the feature's task file.
3. Commit process files (feature, tasks, this checklist) directly to `main`.
4. Create a branch for source code fixes (e.g. `review/f44-semantic-map`).
5. Fix each violation, run tests, open a PR.
6. Before opening the PR: regenerate the literate version of any changed file (`apply .claude/literate.md` to each changed Python source, save to `literate/<module>.md`).


## File header
- [ ] Shebang line: `#!/usr/bin/env python3`
- [ ] First comment: module name and one-line description
- [ ] Second comment: `Author: Pito Salas and Claude Code`
- [ ] Third comment: `Open Source Under MIT license`

## Imports
- [ ] All imports at top of file
- [ ] Absolute imports with module aliases (no relative imports)
- [ ] No ROS2 imports (`rclpy`, `sensor_msgs`, etc.) in library modules
- [ ] No unused imports

## Style
- [ ] Double quotes throughout; single quotes only when required
- [ ] No `from __future__ import annotations`
- [ ] No `Optional[X]` — use `X | None` instead
- [ ] No underscore prefix on private methods/variables

## Structure
- [ ] Functions and methods ≤ 50 lines
- [ ] File ≤ ~300 lines
- [ ] One class per file (dataclasses co-located with their constructing class are allowed)
- [ ] Identifiers ≤ ~15 chars and intention-revealing
- [ ] No if/else nested more than 1 deep
- [ ] No if statement with more than 3 branches
- [ ] No 1-line or 2-line methods
- [ ] No simple wrappers
- [ ] No functions with more than 3 arguments
- [ ] No default parameters on functions
- [ ] No variable assigned from a function result used only once

## Comments and docstrings
- [ ] Simple methods (name is self-explanatory, body is obvious): no docstring
- [ ] Complex methods (non-obvious algorithm, hidden invariant, subtle side-effect): multi-line docstring allowed — explain WHY the mechanism exists, what problem it solves, and any non-obvious preconditions or postconditions
- [ ] No comments that only restate WHAT the code does — add them only when the WHY would surprise a reader
- [ ] No task/fix/caller references in comments

## Type safety
- [ ] Public method parameters have type annotations where the type is non-obvious
- [ ] Return type annotated when the caller would otherwise have to guess
- [ ] Prefer simple types (`str`, `int`, `float`, `bool`) — avoid complex compound generics

## Naming
- [ ] Boolean variables/params named `is_X`, `has_X`, or `can_X`
- [ ] No single-letter variables except loop counters `i`, `j`

## Data and state
- [ ] No logic in `__init__` beyond assignment
- [ ] No side effects at module import time
- [ ] No mutable default arguments
- [ ] No in-place mutation of arguments unless obvious from the name
- [ ] Dataclass mutable defaults use `field(default_factory=...)`

## Design
- [ ] No god methods (each does one thing)
- [ ] No feature envy (method doesn't heavily reach into another class's internals)
- [ ] No data clumps (same 2–3 args traveling together everywhere → make a dataclass)

### Schema duplication
- [ ] No parallel dataclasses — if two classes share 5+ identical fields, extract a base class
- [ ] No hand-transcribed schemas — if a dataclass's fields also appear in a YAML config or in ROS2 `declare_parameter` blocks, those sites must loop over `dataclasses.fields()` instead of re-listing fields by hand
- [ ] Adding one field to a dataclass must not require edits in more than one other file (shotgun surgery smell)

### Cohesion
- [ ] Class has a single clear responsibility
- [ ] All methods belong to that responsibility — none doing unrelated work

### Coupling
- [ ] No unnecessary dependency on concrete classes — could a simpler type suffice?
- [ ] Parameters are as simple as possible (accept a float, not a whole object, if that's all that's needed)

### Abstraction level
- [ ] All methods in a class operate at the same level of abstraction
- [ ] High-level logic is not mixed with low-level detail in the same method

### Interface clarity
- [ ] Public API is minimal — only what callers actually need is exposed
- [ ] Internal helpers are clearly separated from the public interface

### Mutability
- [ ] Mutable state is necessary — could this be a stateless function or utility instead?
- [ ] Class invariants that should be enforced are enforced

### Dependency direction
- [ ] Module does not depend on things above it in the stack (e.g. library module does not import from ROS layer)

## Error handling
- [ ] No bare `except Exception:` — specific exception types only
- [ ] No silent swallowing: no `except X: pass`
- [ ] No defensive coding for impossible cases
- [ ] Validation only at system boundaries (user input, external APIs)

## Test coverage
- [ ] Every public method has at least one test
- [ ] Edge cases are covered: empty inputs, None values, boundary conditions
- [ ] Any non-obvious behaviour that was clarified by a comment also has a test that encodes that expectation
- [ ] If a bug was fixed during review, a regression test was added

## Packaging (standalone library)
- [ ] Uses `pyproject.toml` + `setuptools`; install with `pip3 install -e . --break-system-packages`
- [ ] Source dir name matches project dir name
- [ ] Dual-package repos include `package.xml`, `setup.py`, `resource/<package_name>`
- [ ] Library modules: no `main()`, no `argparse`; config via YAML dataclass; examples in `examples/` ≤40 lines
- [ ] No ROS2 imports (`rclpy`, `sensor_msgs`, etc.) outside `ros/` subpackage

## ROS2 package
- [ ] Nodes under `ros/` subpackage; only place that imports `rclpy` or ROS message types
- [ ] Lifecycle nodes (`rclpy.lifecycle.Node`) where appropriate
- [ ] Message conversion helpers in `ros/converters.py`, not in the node
- [ ] Launch files use `better_launch` (`@launch_this`, `bl.node`, `bl.group`, `bl.include`)
- [ ] ROS2 runtime deps declared as `exec_depend` in `package.xml`; not in `pyproject.toml`
- [ ] Tests in `tests/` run with plain `pytest` — no `colcon test` dependency

## Dead code
- [ ] No unreachable code after `return`
- [ ] No commented-out code blocks

## Code quality
- [ ] No duplicated logic (DRY) — **read every method body and actively look for repeated patterns, not just obvious copy-paste. Check: identical or near-identical method bodies, the same 3+ line sequence appearing in multiple places, repeated object construction patterns**
- [ ] No features not required by current spec (YAGNI)
- [ ] Prefer async/await over threading where possible
- [ ] Avoid nested comprehensions unless simple enough to read at a glance or there is a clear performance reason
- [ ] Look for helper function opportunities: if a block of 3+ lines appears more than once, or if a method does two distinct things that could be named separately, extract a helper
