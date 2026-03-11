# tk25_decision Repo Notes (Updated: 2026-03-10)

## 1) Repository Snapshot

- Root focus: ROS2 behavior-tree decision module for RoboCup@Home style tasks.
- Main package: `src/behavior_tree`.
- Architecture: task-specific BT compositions built from reusable `TemplateNodes` (Navigation, Vision, Audio, Manipulation).
- Mock framework: subsystem and node-level mock control in `behavior_tree/config.py` + `behavior_tree/mock_config.json`.

### Recent structural changes (already landed)

- Entry points were decoupled from a monolithic `behavior_tree/main.py` dispatcher.
- New shared launcher utility: `behavior_tree/runtime.py`.
- New task-local CLI launchers:
  - `behavior_tree/Receptionist/cli.py`
  - `behavior_tree/HelpMeCarry/cli.py`
  - `behavior_tree/ServeBreakfast/cli.py`
  - `behavior_tree/StoringGroceries/cli.py`
  - `behavior_tree/Inspection/cli.py`
  - `behavior_tree/Restaurant/cli.py`
  - `behavior_tree/grasp_intel_demo/cli.py`
  - `behavior_tree/cli_draw.py`
- Entry-point migration tests added: `src/behavior_tree/test/test_entrypoints_migration.py`.
- Stale `yanglaozhucan` code + entrypoints were removed from this branch.

---

## 2) Task Rulebook and Plan Documents

The detailed rulebook extraction notes, requirements summaries, broad plans, and code-refined plans are task-local:

- Storing Groceries:
  - `src/behavior_tree/behavior_tree/StoringGroceries/RULEBOOK_PLAN.md`
- Receptionist:
  - `src/behavior_tree/behavior_tree/Receptionist/RULEBOOK_PLAN.md`
- HRI:
  - `src/behavior_tree/behavior_tree/HRI/RULEBOOK_PLAN.md`
- PickAndPlace:
  - `src/behavior_tree/behavior_tree/PickAndPlace/RULEBOOK_PLAN.md`
- Restaurant:
  - `src/behavior_tree/behavior_tree/Restaurant/RULEBOOK_PLAN.md`

These were regenerated on 2026-03-10 after rulebook file mismatches were resolved.

---

## 3) Cross-Cutting Technical Observations

- Several task files still depend on absolute paths under `/home/tinker/.../constants.json`.
  - This is fragile across machines and should be replaced with package-relative lookup.
- `behavior_tree/config.py` currently contains a bug in `_normalize_node_mode`:
  - premature `return` leaves non-string handling unreachable.
- Mock mode architecture is strong and useful for deterministic integration testing.
- Entry-point migration tests are now in place and should be extended when new CLIs are added.

---

## 4) Suggested Near-Term Work Queue

1. Fix `config.py::_normalize_node_mode` logic.
2. Remove absolute-path constants loading in task scripts.
3. Implement StoringGroceries cereal + bag + tiny/heavy branches.
4. Restore Receptionist gaze-critical branches and runtime drink localization.
5. Add phase-level timers for both tasks (6:00 and 7:00 budget handling).
6. Add task-level acceptance tests with mocked services for rulebook main-goal checkpoints.
