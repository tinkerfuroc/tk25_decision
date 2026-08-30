# Bench provenance: a stale colcon install/ can silently shadow the source tree

Status: informational -- no code change needed in the GPSR bench itself beyond making
the problem visible (M-3, round-3 fix review). Read this before trusting a
`gpsr-bench tier0`/`tier1`/`tier2` result that looks wrong.

## The failure mode

`gpsr_bench.py` (and everything it calls -- `orchestrator`, `planner`, `validators`, ...)
is imported as `behavior_tree.GPSR.*`. If a colcon `install/` tree exists for this
package (e.g. `install/setup.bash` has been sourced in the shell, or ROS 2's own
overlay search puts `install/` ahead of the source checkout on `sys.path`),
`import behavior_tree` can resolve to the **installed copy**, not the checkout you are
editing -- even though every `from behavior_tree.GPSR....` import inside the bench code
still appears to work, because the installed copy has the same module layout.

A round-3 tier0 sweep reproduced exactly this: `goto(bedroom)` /
`deliver(recipient_location='laundry_room')` were rejected as "unknown location" against
`constants.rcw2026.json`, contradicting the worktree source (which resolves both via
`load_knowledge_from_constants`'s room-alias loader, committed in `cbf76f1`). The
installed `orchestrator.py` was dated Aug 18 -- **0** occurrences of the room-alias
block, which landed Aug 27. The bench plumbing (`gpsr_bench._knowledge`) was fine; the
*loader it called* was a stale pre-alias copy. The round-3 tier0 FAIL and the
"constants artefact" note that followed from it were invalid evidence against the
worktree HEAD.

## The fix (process, not code)

Run bench/tests with the source tree taking priority, either:

- `PYTHONPATH=<checkout>/src/behavior_tree` prepended before `install/setup.bash` (or
  without sourcing `install/setup.bash` at all, for a plain `pytest`/`gpsr-bench` run
  that doesn't need the rest of the ROS overlay), or
- a fresh `colcon build --symlink-install` so the installed copy is a symlink back to
  the checkout instead of a stale copy.

## What the code now does about it

`gpsr_bench.py` cannot fix a misconfigured `PYTHONPATH` for you, but it makes a stale
resolution visible instead of silently invalid:

- `main()` prints `[gpsr-bench] behavior_tree resolved from: <path>` before running any
  subcommand.
- Every tier's report `meta` carries the same path as `source_path`
  (`gpsr_bench._resolved_source_path()` -- the resolved `behavior_tree.__file__`), so a
  report generated against a stale install is identifiable after the fact, not just at
  the time it ran.

Before trusting a surprising bench result, check that line (or `report.json`'s
`meta.source_path`) points into `<checkout>/src/behavior_tree/behavior_tree/...`, not an
`install/` tree.
