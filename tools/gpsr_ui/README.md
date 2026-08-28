# GPSR Bench Run Viewer

A local, read-only web UI over the artifacts a GPSR bench battery writes to
`gpsr_runs/bench` — behaviour-tree telemetry, camera frame dumps and judge
sheets. It never modifies the corpus it reads.

## What it is for

Failure triage first. Most runs in this corpus fail, and the question that
matters is "what was the robot doing, and what did it see, at the moment it
went wrong." The run page's core interaction is the linked jump: click a
failure mark in the timeline ribbon and the behaviour-tree panel *and both*
camera tracks (head, arena) jump to that same moment together. Nothing else
in the UI is more load-bearing than that one click.

The `/live` dashboard is the secondary use: watching a battery that is
currently running — which entries are in flight, how stale their last event
is, and (best-effort) whether the sim stack behind them looks stuck.

## Run it

    tools/gpsr-ui

Then open http://localhost:8770. `/live` shows runs currently in flight.

The launcher execs `uvicorn gpsr_ui.app:app` with the pinned interpreter
`/home/tinker/tinker-sim/6.0.1/.venv/bin/python` (3.12.13) — the venv this
tool was built and tested against. It installs nothing at start; if the venv
is missing a dependency, that's a setup bug, not something the launcher
papers over.

## Configuration

| Variable | Default | Meaning |
|---|---|---|
| `GPSR_UI_BENCH_ROOT` | the tk25_decision GPSR bench dir (`.../GPSR/gpsr_runs/bench`) | corpus to read |
| `GPSR_UI_STATE_DIR` | `~/.cache/gpsr-ui` | derived-model cache (parsed events, not corpus data) |
| `GPSR_UI_SHEET_EVENTS` | unset | override path for the vendored classifier, for drift-checking against a newer upstream |
| `GPSR_UI_PORT` | `8770` | listen port |

## The read-only guarantee

Nothing is ever written inside `gpsr_runs/`. This is enforced two ways:

- **Structurally, at the `Settings` type boundary.** `Settings.__post_init__`
  rejects a `state_dir` that is equal to, or nested inside, `bench_root` --
  the derived-model cache cannot be pointed into the corpus even by
  misconfiguration, and this holds no matter how a `Settings` comes to exist.
  `load_settings()` (which builds the `Settings` the shipped launcher
  actually uses, from `GPSR_UI_STATE_DIR`/`GPSR_UI_BENCH_ROOT`) gets this for
  free by constructing one; so does any other code path that builds a
  `Settings` directly, including tests.
- **By test, in two complementary forms**, both in `tests/test_read_only.py`:
  - `test_indexing_a_synthetic_corpus_mutates_nothing_at_all` builds an
    entirely synthetic corpus in a tmp dir and asserts full file-set equality
    (mtimes, additions, removals — everything) before and after indexing it.
    Nothing else is writing to that corpus, so this is the strict form of the
    guarantee, and it's the only one that could catch an indexing bug that
    *creates* a stray file — a live corpus can't distinguish that from a
    battery's own legitimate write.
  - `test_indexing_the_real_corpus_mutates_nothing` runs the same indexing
    pass against the actual bench corpus, which a live battery may be writing
    to concurrently. It **must** tolerate that: a battery legitimately adds
    run directories, appends to `events.jsonl`, and (per finding 9 below)
    rewrites files inside a directory it has reused for a re-run. So this
    test excludes any run directory that is in flight at the start or the end
    of the test (see `_in_flight_run_dirs`) and then demands the stricter
    guarantee — no modification, no removal — of everything else. It cannot
    be as strict as the synthetic test; excluding in-flight runs is what
    makes it meaningful at all against a moving corpus instead of vacuously
    passing or randomly flaking.

## Things about the data that surprised us

This tool reads artifacts produced by another team's benchmark harness, and
nearly every hard bug in this project traced back to an assumption about
those artifacts that turned out to be wrong. These are the findings, and
what they cost us or forced us to do:

- **`tree_revision` is `0` in every event we've ever seen** — 289
  `tree.generated` events across 105 `events.jsonl` files, all zero. Do not
  read it as a version number. We surfaced this upstream; the vendored
  classifier (`vendor/sheet_events.py`, pinned at `9072c6e`) was fixed to key
  on generation *count* instead.

- **Two `tree.generated` epochs is normal, not a replan.** Every healthy
  `t2-2026` run gets exactly two: a skeleton tree at startup, then the
  executor materialising the real plan. We compute
  `tree_regenerations = max(0, epochs - 2)`. Using `epochs - 1` — the naive
  reading — would put a phantom regeneration on every single healthy run.

- **Tree regeneration is not the same phenomenon as an executor replan, and
  must never be labelled "replan."** The `DynamicExecutor` can replan
  internally without ever regenerating the tree — one observed run
  replan-looped for its entire 900 s timeout with only 2 epochs the whole
  time. So the epoch count *undercounts* replans; it is not a proxy for them.
  `gate_failures` (PRECONDITION/POSTCONDITION failures) is the
  replan-adjacent signal to watch instead.

- **`run.finished.status` is `"incomplete"` in all 105 runs**, including the
  one run in this corpus that PASSed. It is never a verdict. The actual
  outcome lives in `run.json`'s `verdict` field, which this UI reads instead.

- **Sim and wall clocks are not proportional.** Real-time factor drifts
  within a single run — measured 0.394 and 0.406 sim-seconds-per-wall-second
  on two runs via the exact frame/event join, not a constant. Frames cannot
  be placed on the event timeline by scaling one clock into the other.
  `frames/index.jsonl` carries the real join between frame and wall time;
  when it's absent we fall back to interpolation and the run page badges
  that run's clock `approximate` rather than presenting it as exact.

- **`run.json`'s `id` field is the corpus entry, not the attempt.** All
  attempt directories of one entry (e.g. the 11 attempts under
  `s2026-002-countPrsInRoom`) share a single `id`. The directory name — not
  `id` — is the attempt identity everywhere in this UI.

- **`announcements.txt` repeats itself.** One real file is 7967 lines long
  but contains only 11 distinct utterances (the robot re-announces on
  retries/ticks). Deduplicate before showing it to a person.

- **The two camera tracks record at very different, and correct, rates —
  they were never meant to be in lockstep.** Head publishes at 12 sim-Hz, so
  the recorder's 1-second gate almost always finds a fresh frame. Arena is
  deliberately throttled to 2 sim-Hz (it costs real-time factor), so its
  timestamps land on a 0.5 s grid and slots are often skipped. Arena settles
  at roughly 50–70% of head's frame count on the same run (measured 89 vs
  141, and 147 vs 245). Both tracks are independently correct against their
  own `index.jsonl`; a mismatched count between them is not a bug to chase.

- **The bench reuses a run directory in place on a re-run; it does not
  archive the previous attempt.** (`.attemptN-*` directories that do exist in
  the corpus are a manual convention someone applied by hand afterward, not
  something the bench does itself.) This has two consequences, both of which
  bit us before we accounted for them:

  - *Orphan frames.* Frame filenames embed the simulator timestamp, so a
    re-run's frames are *interleaved* with the previous occupant's rather
    than overwriting them. Only frames listed in `frames/index.jsonl` belong
    to the current run — anything else in the directory is a leftover.
    Measured contamination: `s2026-003` head had 141 files on disk for 89
    frames that actually belonged to the run; `s2026-004` had 245 for 147;
    `s2026-005` had 340 for 291. The orphans' timestamps were roughly 4600 s
    *earlier* than the real run's, so a naive filename sort put them at the
    *front* — before this was accounted for, the first 50–98 frames a user
    scrubbed through were footage from an entirely different run.
  - *Liveness.* A reused directory keeps the *previous* run's `run.json`
    sitting there, unchanged, for the entire duration of the new run while
    it's being written. So "no `run.json` present" is not a complete
    liveness test — it misses an in-flight re-run of an entry that has
    already succeeded once. The rule this UI uses instead: a run is in
    flight if `run.json` is absent, **or** `events.jsonl`/`orchestrator.log`
    is newer than the `run.json` sitting next to it, with a staleness guard
    (no new event in 60 s) so a truly abandoned run doesn't get stuck
    claiming to be live forever.

- **`run.json` is not the last file the bench writes, so "newest file in the
  directory" is not a liveness signal either.** `sheet.jpg` and
  `judge-sheet.jpg` are generated *after* `run.json`, as a post-processing
  step — true for 69 of the 72 completed runs in this corpus that have both.
  `run.json` is only the last file the *orchestrator* itself writes before
  handing off to that post-processing step.

## Environment gotchas (running the tests, not the app)

- This shell sources ROS Humble, which leaks Python 3.10 site-packages onto
  the path and breaks the 3.12 venv's imports (`ModuleNotFoundError: No
  module named 'lark'`). Clear `PYTHONPATH` before any pytest invocation:
  `PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest`.
- `node --test tests/js/` fails under Node 24, which treats a bare directory
  argument as a module specifier rather than a directory of tests. Use the
  quoted glob instead: `node --test 'tests/js/**/*.mjs'`.

## The vendored classifier

`gpsr_ui/vendor/sheet_events.py` is a vendored, unmodified copy of upstream
`tinker-sim`'s `tools/sheet_events.py`, pinned at commit `9072c6e` (the
commit that fixed REPLAN detection to key on tree-generation count instead
of the always-zero `tree_revision` — see above). **Do not edit this file.**
To pick up an upstream fix, re-copy the file and update the commit hash in
its header comment.

`GPSR_UI_SHEET_EVENTS` points the app at a different copy of the same module
shape instead, without touching the vendored file — useful for checking
whether upstream has drifted in a way that changes this UI's output before
committing to a re-vendor.

## Tests

    cd tools
    PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest
    node --test 'tests/js/**/*.mjs'

85 Python tests, 71 Node tests, as of this writing.

Tests marked `corpus` (see `pytest.ini`) read the real bench tree and are
skipped automatically when it is absent, or when `GPSR_UI_SKIP_CORPUS=1` is
set — set that while a battery is running if you want a clean, corpus-free
run of the suite.
