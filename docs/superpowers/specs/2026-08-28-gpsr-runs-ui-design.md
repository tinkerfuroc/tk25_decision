# GPSR Bench Run Viewer — Design

Date: 2026-08-28
Status: design, awaiting approval
Author: session `2614f450`, in coordination with session `gpsr command testing robustness` (which produces the runs)

## Purpose

The GPSR bench emits a run directory per corpus entry: telemetry, two camera
frame streams, contact sheets, announcements and logs. Reading a failure today
means grepping a 7 MB orchestrator log and opening a 960x7306 JPEG. This
document designs a local web UI that makes a run legible.

Primary job is **failure triage**: answer "why did `s2026-003-findObjInRoom`
fail?" in seconds rather than minutes. Secondary job is **demonstration**: the
same run view should be good enough to play a successful run back to an
audience. Triage wins where the two conflict.

The current t2-2026 tier stands at 1 PASS against 14 FAIL/TIMEOUT across 15 run
directories, so triage is where the time goes.

## Constraints

These are established facts, verified on disk or confirmed by the producing
session. They drive most of the design.

1. **`events.jsonl` is the only source of truth.** Schema
   `tinker.gpsr.telemetry` v1, at `debug/gpsr-*/events.jsonl`. The bench
   verdicts and contact sheets are built from it.
   `orchestrator.log` re-prints a whole-tree snapshot every tick (line count
   inflated ~1000x) and `bt_visualization_logs/` is GUI-oriented with nothing
   downstream depending on it. **Neither is parsed.**
2. **Frames and events are on different clocks.** Frame filenames
   `NNNN_<ms>.jpg` are sim-clock milliseconds; `occurred_at` is wall UTC. RTF
   varies 0.2–0.5 *within* a single run, so the two are not proportionally
   related. The run inspected spans 151 s of sim time across 383 s of wall time.
3. **`frames/index.jsonl` is the sync key**, one line per frame carrying both
   `stamp_s` and `wall`. It lands on all runs from the next battery onward.
   Existing runs do not have it.
4. **Writes into `runs/` are forbidden.** The bench appends to and archives run
   directories mid-battery; a collision would corrupt evidence.
5. **Re-running a corpus entry archives the old directory** as
   `<run>.attemptN-<slug>`. A dot-suffixed directory is an archived attempt; the
   bare name is current. `s2026-002-countPrsInRoom` already has 11 attempts.
6. **`run.json`'s `id` is the corpus entry, not the attempt.** All 11 attempt
   directories report `id: s2026-002-countPrsInRoom`. Attempt identity is the
   directory name.
7. **`tree_revision` is always 0, and `run.finished.status` is always
   `incomplete`.** Measured across the whole corpus: 105 `events.jsonl` files,
   289 `tree.generated` events, `tree_revision` is `0` in every one; all 105
   `run.finished` events report `incomplete`, including the run whose
   `run.json` says `PASS`. Two consequences, both load-bearing:
   - **Replans must be counted as `tree.generated` epochs**, not revision
     bumps. `group-000` in `t1-42` has 24 epochs. The vendored classifier
     emits its `REPLAN` judge event only when `tree_revision > 0`, so that
     lane would be **silently empty on every run in the corpus** if we relied
     on it. `telemetry.py` derives replans itself.
   - **`run.finished.status` is never displayed as an outcome.** The verdict
     is `run.json`'s `verdict` field.
8. **Scale.** 40 corpus entries per tier, plus attempt archives. 150–900 frames
   per camera per run (1 frame per sim-second, 900 s cap, two cameras). Whole
   bench corpus is currently 2.4 GB. Plain disk reads are adequate; no manifest
   step is needed.

## Approaches considered

**A. Telemetry-first single service.** One FastAPI app; a backend indexer
derives a per-run model from `events.jsonl`; a no-build ES-module frontend;
SSE for live updates. *Chosen.*

**B. Static pre-render plus a thin file server.** A build step emits per-run
JSON that the server simply serves. Faster cold loads and trivially shareable,
but requires re-running the build after every battery and structurally cannot
live-tail an in-flight run.

**C. Vite + React SPA.** Best UI ergonomics, and Node 24 is installed. Costs a
`node_modules` tree, a build step, and a second toolchain on a robot box.

A is chosen because live-tailing is a v1 requirement that B cannot meet, and
because its dependencies are already installed while C's are not.

## Deviation from the original location decision

The UI was initially to live in the sim worktree at
`/home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec/tools/`,
beside `sheet_events.py`. The session that owns that worktree asked us to stay
out of it, for two reasons worth recording:

- The worktree's lifetime is tied to that session; worktrees can be cleaned up
  when a session ends, which would take this code with it.
- Two sessions committing on one branch in one worktree entangles both git
  histories and index operations.

**Resolution.** Code lives at `tools/gpsr_ui/` in the `tk25_decision` repo. The
telemetry classifier is vendored rather than path-imported (see below). This
keeps the intent of the original choice — reuse the existing classifier, don't
reimplement it — while putting the code somewhere durable.

## Component design

### Layout

```
tools/gpsr_ui/
  app.py            FastAPI app, route wiring, CLI entrypoint
  corpus.py         tier/run discovery, attempt grouping, verdict summary
  telemetry.py      events.jsonl -> derived run model
  clock.py          sim <-> wall mapping (exact and approximate)
  frames.py         frame indexing, thumbnails, mp4 export
  live.py           in-flight detection, byte-offset tail, SSE
  cache.py          derived-model cache, annotation store
  vendor/
    sheet_events.py copy of tools/sheet_events.py @ 07497b0
  static/
    *.js            plain ES modules, no build step
    *.css
  templates/        Jinja2, one shell page per screen
tests/gpsr_ui/      pytest, run against the real corpus
```

Each backend module is independently testable and has one job. No module
reaches into another's internals; `app.py` is the only place they are composed.

### Runtime

Served by the durable sim virtualenv, which is outside any worktree and already
has every dependency:

```
cd tools    # so that gpsr_ui is importable as a package
/home/tinker/tinker-sim/6.0.1/.venv/bin/python -m uvicorn gpsr_ui.app:app --port 8770
```

A `tools/gpsr-ui` shell wrapper sets the interpreter path and working directory,
so in practice the command is one word. Configuration comes from the environment
rather than CLI flags, since uvicorn owns the command line:
`GPSR_UI_BENCH_ROOT` defaults to the GPSR bench directory, `GPSR_UI_STATE_DIR`
to `~/.cache/gpsr-ui/`, and `GPSR_UI_SHEET_EVENTS` overrides the vendored
classifier with a live path.

This virtualenv is Python 3.12 while the ROS-side tooling is Python 3.10. The
UI shares no code with the ROS stack and imports no ROS packages, so the split
is deliberate rather than a problem to solve.

### Vendoring the classifier

`sheet_events.py` is pure stdlib (`ast`, `json`, `re`, `dataclasses`,
`pathlib`, `typing`) and about 250 lines. It is copied to
`vendor/sheet_events.py` with the source commit `07497b0` recorded in a header
comment. `load_run_telemetry(run_dir)` returns `MilestoneEvent` and
`JudgeEvent` lists already carrying NAV/VISION/AUDIO/MANIP classification,
keepalive and bookkeeping exclusion, and `plan-step N action: params` context.
Reimplementing that classification would guarantee drift from the contact
sheets and bench verdicts.

`GPSR_UI_SHEET_EVENTS` overrides the vendored copy with a live path, for
checking against upstream changes. The producing session has committed to
messaging before changing the public surface.

### Read-only guarantee

Nothing is written inside `gpsr_runs/` — not even the sidecar `ui/` directory
that was offered. Every file is opened read-only. The derived-model cache and
triage annotations both live under `--state-dir`, keyed by
`<tier>/<directory name>`. This is stricter than required and reduces the
collision surface with a running battery to zero.

Annotations are a UI-local convenience. The bench does not read them, and the
design does not assume they survive a cache wipe.

### Clock mapping — `clock.py`

The single hardest correctness problem, and the one most likely to silently
mislead. One interface, two implementations:

- **Exact.** Join on `frames/index.jsonl`, which carries `stamp_s` and `wall`
  per frame. Used whenever the file is present.
- **Approximate.** Linear interpolation per camera label between
  `recorder-meta.json`'s `started_wall`/`ended_wall` and
  `first_stamp`/`last_stamp`. Accurate to within seconds, and wrong in the
  middle of a run wherever RTF deviates from its run-average.

The active mode is exposed through the API and **rendered as a visible badge**
in the UI. An approximate alignment must never be mistakable for a real one;
that misreading would produce false triage conclusions about what the robot
could see when a node failed.

### Derived run model — `telemetry.py`

A single pass over `events.jsonl` produces:

- **Tree epochs.** Each `tree.generated` payload is one epoch, in arrival
  order, carrying node id, name, type, parent, children, semantics and
  blackboard access. The inspected run has two: 84 nodes at sequence 3, then
  158 nodes at sequence 257, the difference being the `DynamicExecutor`
  materialising the plan. **Both report `tree_revision: 0`** — see constraint
  7 — so epochs are identified by ordinal, and the tree shown at a playhead is
  the latest epoch at or before it. Epochs after the first are **replans** and
  are first-class events in the UI, not silent redraws. Epoch count per run
  ranges from 1 to 24 across the corpus.
- **Node status timeline.** From `tree.node_states_changed`: per node, an
  ordered list of (tick, wall, status, feedback). Node status at any playhead
  position is the last transition at or before it.
- **Milestones and judge events**, from the vendored classifier.
- **Plan steps**, from step context stamped on feedback, anchored by the spoken
  `My plan: ...` announcement that new runs emit early in each task.
- **Run outcome**, from `run.json`'s `verdict`, `detail` and `seconds`.
  `run.started` and `run.configured` supply start time and mode;
  `run.finished` supplies only its wall time, never its `status` (constraint
  7).

`tree.tick_observed` is **not** used to build the model. In the inspected run
it carries 754 events against 168 `tree.node_states_changed`, mostly restating
node status the latter already reports; in other runs the two counts are equal.
Either way the status timeline comes from `node_states_changed`.
`tick_observed` is read only for `counter_deltas`, `retry_repeat_deltas` and
`active_action_context`, which appear nowhere else.

The model is cached under `--state-dir` keyed by (path, mtime, size), so a
finished run is parsed once.

### Screens

**Corpus browser.** All tiers (`t0-42`, `t1-42`, `t2-2026`, `t2plus-2026`,
`smoke-t2`), each run showing verdict, duration, template and feasibility class.
Archived `.attemptN-*` directories are grouped under their parent entry so the
11 attempts of `s2026-002` read as a history, newest first, with the slug
(`nav-fixed-count-fail`, `postcondition-loop`) shown as the attempt's label.
This page reads only `run.json` and directory metadata — it never opens a 5 MB
event log.

**Run view.** Three panels locked to one shared **wall-clock playhead**:

- *Timeline ribbon.* Wall-clock x-axis with lanes for plan steps, milestones
  (NAV/VISION/AUDIO/MANIP, red on FAILURE), judge events (precondition,
  postcondition, supervisor, correction), replan markers at each
  `tree.generated` epoch after the first, and frame coverage density. A budget
  bar shows elapsed against the 6-minute target and the 900 s timeout.
- *Tree panel.* SVG hierarchical layout of the current tree revision,
  time-travelled to the playhead. Keepalive and bookkeeping subtrees collapsed
  by default; the path to the active node auto-expanded. Clicking a node opens
  its type, semantics, blackboard read/write sets, and its full feedback history
  across ticks.
- *Stop-motion viewer.* Head (1280x720) and arena (960x540) as two synced
  tracks, with a preloaded window of surrounding frames, variable speed, and a
  scrub bar. Because frames are one per sim-second, playback rate is expressed
  in frames per second of wall time and defaults to 10, which reads as film
  rather than as a slideshow.

Clicking any failure anywhere moves the playhead, and the tree and both frame
tracks follow. That single gesture is the triage loop, and it is also what makes
the view usable as a demo.

**Live dashboard.** Described below.

### Live tail — `live.py`

In-flight detection: a run directory with **no `run.json`**. This was verified
against a completed run — `events.jsonl` closes at 06:58:50, `run.json` is
written at 06:58:52, last of all — and confirmed by the producing session as the
best signal that exists; there is no explicit per-run marker. A staleness guard
covers crashed and torn-down runs: no `run.json` and no `events.jsonl` growth
for 60 s means dead, not live.

Updates are pushed over SSE, with the backend tailing `events.jsonl` by byte
offset. The dashboard shows the active plan step, the last nav goal and its
outcome, replan count (epochs beyond the first), gate failures, elapsed against
budget, and the newest frame from each camera, auto-advancing.

Two pieces of context that live outside the run directory:

- **"Failed to make progress" count**, from
  `<sim worktree>/gpsr_stack_logs/<newest>/02-bridge.log`. Read-only, resolved
  at request time so a deleted worktree degrades to a missing panel rather than
  an error.
- **Announcements**, deduped to first-occurrence order.
  `announcements.txt` for the inspected run is 7967 lines containing 11
  distinct utterances, because older runs re-append the whole history each
  tick. Deduping is not a nicety; the raw file is unreadable.

### Frames — `frames.py`

Full-size frames are served with long-lived cache headers. Contact-sheet
thumbnails are generated on demand with PIL into `--state-dir`. mp4 export via
ffmpeg (present at `/usr/bin/ffmpeg`) is offered per camera per run for sharing
a run outside the tool.

## Error handling

The corpus is a live, partially-broken artifact tree, so degradation is the
normal case and each missing piece disables exactly one panel:

- Missing `frames/index.jsonl` → approximate clock, badged.
- Missing `recorder-meta.json` **and** missing `index.jsonl` → frames are shown
  as an unsynced strip, with the timeline link disabled. Per the producing
  session, a `recorder.log` traceback *without* `recorder-meta.json` indicates a
  real recorder crash and is surfaced as a warning; the
  `ExternalShutdownException` seen on healthy runs is benign rclpy teardown
  noise on SIGINT and is suppressed.
- Missing or truncated `events.jsonl` → run listed, run view degraded to
  whatever parsed. Trailing partial lines are expected while a run is live and
  are skipped, not treated as errors.
- Missing `run.json` → treated as in-flight, subject to the staleness guard.
- Unreadable frame → placeholder tile; playback continues.

## Testing

- **pytest against the real corpus.** It already contains PASS, FAIL and
  TIMEOUT runs and 11 archived attempts, which covers most branches. Golden
  tests pin the derived model for a small number of named runs.
- **A synthetic minimal run fixture** for edge cases the corpus lacks: empty
  event log, truncated final line, missing recorder metadata, a run with three
  tree epochs.
- **Clock mapping is tested directly**, both implementations, including the
  assertion that exact and approximate disagree on a real run — which
  documents why the badge exists.
- **Frontend logic** (tree layout, clock interpolation, playhead reducer) lives
  in pure ES modules with no DOM dependency and is tested with Node 24's
  built-in `node:test`. DOM wiring is not unit-tested.
- **Read-only enforcement is a test**, not just a convention: a test asserts
  that a full index-and-render pass over the corpus leaves every mtime under
  `gpsr_runs/` unchanged.

## Phasing

1. `corpus.py` + browser page. Makes 15 runs and their attempt history legible
   on its own.
2. `telemetry.py` + `clock.py` + derived model API, with the read-only test.
3. Run view: timeline ribbon and tree panel.
4. Stop-motion viewer and playhead linking.
5. `live.py` and the live dashboard.
6. Polish: mp4 export, annotations, bridge-log panel.

Phases 1–2 carry the correctness risk and should be reviewed before 3 begins.

## Explicitly out of scope

- Any write into `gpsr_runs/`.
- Editing, re-running or launching bench runs from the UI. It observes; the
  bench runner drives.
- Authentication, multi-user support, remote deployment. This is a local tool
  bound to localhost.
- Replacing `sheet.jpg` / `judge-sheet.jpg`. Those remain the portable
  artifact; this is the interactive complement.
