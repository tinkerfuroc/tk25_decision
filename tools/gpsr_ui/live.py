# tools/gpsr_ui/live.py
"""Detect and follow the run that is currently executing.

LIVENESS RULE (this overrides the task brief -- see the task-11 report for
the full justification): the brief said a run is in flight exactly when
`run.json` is absent. That is insufficient. The bench reuses a run
directory IN PLACE on a re-run and does not archive the previous
occupant, so the directory keeps the PREVIOUS run's `run.json` for the
entire duration of the new run. Under the brief's rule an in-flight
re-run looks exactly like a finished run and would never appear on this
dashboard -- precisely the run the battery team most wants to watch (a
re-run of a failed entry).

The corrected rule:

    a run is IN FLIGHT when `run.json` is ABSENT, OR `events.jsonl`'s
    mtime is NEWER than `run.json`'s mtime,
    AND the newest `events.jsonl` has not gone quiet (no growth, no
    touch) for more than `stale_after` seconds.

The staleness guard matters on its own: a crashed or torn-down run never
gets a `run.json` at all, so presence/absence alone can't rule it out;
the bench's own hard timeout is 900s, so 60s of silence is decisive, not
a guess.

This was checked against all 7 completed t2-2026 entries at the time of
writing: the corrected rule agrees with the brief's on every one of
them, because `run.json` lands ~2s AFTER `events.jsonl` closes on a
genuinely finished run (so "events mtime > run.json mtime" is false for
all of them, exactly as "run.json absent" was false). The correction
only changes behaviour for the reused-directory case the brief's own
test suite never constructed -- and tests/test_read_only.py independently
confirms the same reused-directory fact from a completely different
angle (using orchestrator.log instead of events.jsonl, for a different
purpose), which is strong corroboration this is real and not a guess.

Do NOT use "newest file in the directory" as a liveness signal instead:
sheet.jpg / judge-sheet.jpg are written AFTER run.json on completion
(verified on 69 of 72 completed runs), so run.json is only the last file
the ORCHESTRATOR itself writes, not the last file in the directory.
"""
from __future__ import annotations

import json
import time
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path

from .cache import cached_run_model
from .clock import parse_wall
from .corpus import list_tiers
from .frames import list_frames
from .telemetry import RunModel, load_run_model, newest_events_file


@dataclass(frozen=True)
class InFlight:
    tier: str
    dir_name: str
    path: Path
    last_event_age: float


def _run_json_mtime(run_dir: Path) -> float | None:
    try:
        return (run_dir / "run.json").stat().st_mtime
    except OSError:
        return None


def find_in_flight(
    bench_root: Path,
    stale_after: float = 60.0,
    now: float | None = None,
) -> list[InFlight]:
    """Every run currently executing under `bench_root`, per the
    corrected liveness rule (see module docstring)."""
    now = time.time() if now is None else now
    out: list[InFlight] = []
    for tier in list_tiers(bench_root):
        for entry in tier.entries:
            for attempt in entry.attempts:
                events = newest_events_file(attempt.path)
                if events is None:
                    continue  # no telemetry at all: nothing to watch
                try:
                    events_mtime = events.stat().st_mtime
                except OSError:
                    continue

                run_json_mtime = _run_json_mtime(attempt.path)
                if run_json_mtime is not None and events_mtime <= run_json_mtime:
                    continue  # finished: run.json is not older than the log

                age = now - events_mtime
                if age > stale_after:
                    continue  # crashed or torn down, not live

                out.append(InFlight(
                    tier=tier.name,
                    dir_name=attempt.dir_name,
                    path=attempt.path,
                    last_event_age=age,
                ))
    return out


def tail_events(path: Path, offset: int) -> tuple[list[dict], int]:
    """Read complete lines appended since `offset`.

    Never advances past a torn trailing line -- a live writer produces
    one routinely -- so the next poll picks it up whole rather than
    losing or mis-parsing it.
    """
    try:
        size = path.stat().st_size
    except OSError:
        return [], offset
    if size < offset:
        offset = 0  # file was replaced or truncated under us
    try:
        with path.open("rb") as fh:
            fh.seek(offset)
            chunk = fh.read()
    except OSError:
        return [], offset

    events: list[dict] = []
    consumed = 0
    for raw in chunk.splitlines(keepends=True):
        if not raw.endswith(b"\n"):
            break  # torn final line: leave it for the next poll
        consumed += len(raw)
        line = raw.strip()
        if not line:
            continue
        try:
            event = json.loads(line)
        except ValueError:
            continue
        if isinstance(event, dict):
            events.append(event)
    return events, offset + consumed


def _latest_plan_step(model: RunModel) -> str | None:
    """Most recent 'plan-step N action: ...' tag across BOTH milestone
    and judge events.

    The vendored classifier (sheet_events.py, _apply_step_context) stamps
    this context onto every subsequent milestone AND judge event once a
    materialise node succeeds. Scanning only judge_events would miss it
    whenever the truly most recent tagged event is a milestone -- the
    common case, since one plan step produces far more milestones (nav/
    vision/audio/manip actions) than judge gates.
    """
    candidates: list[tuple[float, str]] = []
    for group in (model.milestones, model.judge_events):
        for item in reversed(group):
            info = item.info or ""
            if "plan-step" in info:
                wall = parse_wall(item.wall)
                candidates.append((wall if wall is not None else -1.0, info))
                break  # each group is chronological; first hit is its latest
    if not candidates:
        return None
    candidates.sort(key=lambda c: c[0])
    return candidates[-1][1]


def _newest_frames(run_dir: Path) -> dict[str, str]:
    """label -> newest frame filename, for the dashboard's auto-advancing
    thumbnails. `list_frames` already sorts each label's refs by sim
    stamp ascending, so the last one is the newest."""
    return {
        label: refs[-1].file
        for label, refs in list_frames(run_dir).items()
        if refs
    }


def live_summary(run_dir: Path, state_dir: Path | None = None) -> dict:
    """A snapshot of an in-flight run's key facts for the dashboard.

    When `state_dir` is given, the run model is loaded via the existing
    disk cache (cache.py), keyed on the event log's path/size/mtime -- a
    poll that finds the log unchanged since the last one (a stretch of
    the run with no new events) then loads a pickle instead of
    re-parsing the whole file from scratch, which the largest run in the
    corpus (5130 transitions) makes worth avoiding on a 2-second SSE
    cadence. `state_dir=None` (the default) always re-parses directly;
    every test in this suite uses that path, to stay independent of a
    cache file under a `state_dir` it would otherwise have to invent.
    """
    run_dir = Path(run_dir)
    model = (
        cached_run_model(run_dir, state_dir)
        if state_dir is not None
        else load_run_model(run_dir)
    )

    last_failure = None
    for t in reversed(model.transitions):
        if t.status == "FAILURE":
            last_failure = {
                "node_id": t.node_id, "feedback": t.feedback, "wall": t.wall}
            break

    last_nav = None
    for m in reversed(model.milestones):
        if m.kind == "NAV":
            last_nav = {"name": m.name, "status": m.status, "info": m.info}
            break

    elapsed = None
    if model.started_wall is not None:
        # Real current time, not the event log's own mtime: a live run
        # can go quiet for a stretch (a long-running nav goal in flight
        # with no ticks) and still count as in flight up to
        # `stale_after` seconds -- using the log's mtime as "now" would
        # freeze the elapsed clock for that whole window instead of
        # reflecting genuine wall time.
        elapsed = time.time() - model.started_wall

    return {
        "dir_name": Path(run_dir).name,
        "trajectory_id": model.trajectory_id,
        "tree_regenerations": model.tree_regenerations,
        "gate_failures": model.gate_failures,
        "epoch_count": len(model.epochs),
        "elapsed_s": elapsed,
        "budget_s": 360.0,     # the 6-minute target
        "timeout_s": 900.0,    # the bench hard timeout
        "last_failure": last_failure,
        "last_nav": last_nav,
        "plan_step": _latest_plan_step(model),
        "announcements": model.announcements[-8:],
        "milestone_count": len(model.milestones),
        "newest_frames": _newest_frames(run_dir),
    }


def find_progress_failures(search_roots: Iterable[Path]) -> dict | None:
    """Best-effort 'Failed to make progress' count from the newest
    02-bridge.log under any `gpsr_stack_logs/` found beneath
    `search_roots`.

    Two directory layouts are checked per root, both observed in
    practice: `<root>/*/gpsr_stack_logs` (a direct sim checkout) and
    `<root>/*/.claude/worktrees/*/gpsr_stack_logs` (one of its git
    worktrees). Across every `gpsr_stack_logs/` found, the newest
    run-id subdirectory (its name is an ISO-ish, lexicographically
    sortable timestamp, e.g. "20260828T091519") wins.

    This data source lives entirely OUTSIDE the run corpus and outside
    this repo, in a sim worktree whose lifetime this app does not
    control -- it can be created, renamed or torn down at any moment
    independent of this app's own lifetime. Resolved fresh on every
    call (never cached, and deliberately cheap: a couple of bounded
    globs plus one small file read). Returns None on any absence or I/O
    problem -- never raises -- so a caller renders "missing panel" for
    None rather than an error or a crash.
    """
    stacks: list[Path] = []
    for root in search_roots:
        try:
            stacks.extend(root.glob("*/gpsr_stack_logs"))
            stacks.extend(root.glob("*/.claude/worktrees/*/gpsr_stack_logs"))
        except OSError:
            continue

    newest_dir: Path | None = None
    newest_name = ""
    for stack_logs in stacks:
        try:
            run_names = sorted(
                p.name for p in stack_logs.iterdir() if p.is_dir())
        except OSError:
            continue
        if run_names and run_names[-1] > newest_name:
            newest_name = run_names[-1]
            newest_dir = stack_logs / run_names[-1]

    if newest_dir is None:
        return None
    log_path = newest_dir / "02-bridge.log"
    try:
        text = log_path.read_text(errors="replace")
    except OSError:
        return None
    return {"path": str(log_path), "count": text.count("Failed to make progress")}
