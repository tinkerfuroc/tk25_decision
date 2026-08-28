# GPSR Bench Run Viewer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A local web UI that makes a GPSR bench run legible — browse the corpus, time-travel the behaviour tree, scrub both camera streams as stop-motion, and watch the in-flight run live.

**Architecture:** A FastAPI service reads run artifacts read-only from disk and derives a per-run model from `events.jsonl`, cached outside the corpus. The frontend is plain ES modules with no build step; every panel is driven by one shared wall-clock playhead. Live updates are pushed over SSE by tailing the active run's event log by byte offset.

**Tech Stack:** Python 3.12, FastAPI, uvicorn, Jinja2, Pillow, pytest. Frontend: vanilla ES modules, inline SVG, `node:test` for pure logic. ffmpeg for optional mp4 export.

**Spec:** `docs/superpowers/specs/2026-08-28-gpsr-runs-ui-design.md`

## Global Constraints

Every task's requirements implicitly include this section.

- **Never write inside `gpsr_runs/`.** Open every corpus file read-only. All derived state goes under `GPSR_UI_STATE_DIR` (default `~/.cache/gpsr-ui/`).
- **Interpreter:** `/home/tinker/tinker-sim/6.0.1/.venv/bin/python` (Python 3.12.13). It already has fastapi, uvicorn, jinja2, pydantic and Pillow. **Install nothing.** If a task seems to need a new package, stop and report instead.
- **`events.jsonl` is the only telemetry source.** Never parse `orchestrator.log` or `bt_visualization_logs/`.
- **`tree_revision` is `0` in all 289 `tree.generated` events across all 105 runs in the corpus.** Never read it.
- **Two `tree.generated` epochs is the NORMAL pair** — a skeleton tree at startup, then the `DynamicExecutor` materialising the plan. Tree regenerations are `max(0, epochs - 2)`. Every t2-2026 run has exactly 2; `t1-42/group-000` has 24.
- **Tree regeneration is not an executor replan.** The `DynamicExecutor` replans internally without regenerating the tree: `s2026-002.attempt11` replan-looped for the full 900 s timeout with only 2 epochs. Never label the epoch count "replans". On t2 runs the replan-adjacent signal is PRECONDITION/POSTCONDITION `FAILURE` judge events, exposed as `gate_failures`.
- **`run.finished.status` is `"incomplete"` in all 105 runs, including the PASS run.** Never display it as an outcome. The verdict is `run.json`'s `verdict` field.
- **Attempt identity is the directory name**, not `run.json`'s `id` — all 11 attempt dirs of `s2026-002-countPrsInRoom` carry the same `id`.
- **Unit tests must be hermetic.** The corpus is being actively appended to and archived by a running battery, so no unit test may assert against it. Corpus-dependent tests are marked `@pytest.mark.corpus` and skip when the corpus is absent.
- **Run tests from `tools/`:** `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest`.
- All Python is typed with `from __future__ import annotations` and passes `python -m compileall`.

## File Structure

| File | Responsibility |
|---|---|
| `tools/gpsr_ui/__init__.py` | Package marker |
| `tools/gpsr_ui/config.py` | Environment-derived settings (bench root, state dir, classifier override) |
| `tools/gpsr_ui/corpus.py` | Tier/run discovery, attempt grouping, verdict summary |
| `tools/gpsr_ui/clock.py` | Sim↔wall mapping, exact and approximate |
| `tools/gpsr_ui/telemetry.py` | `events.jsonl` → derived run model (epochs, status timeline, regenerations) |
| `tools/gpsr_ui/frames.py` | Frame discovery, thumbnails, mp4 export |
| `tools/gpsr_ui/cache.py` | Derived-model cache keyed by (path, mtime, size) |
| `tools/gpsr_ui/live.py` | In-flight detection, byte-offset tail, SSE generator |
| `tools/gpsr_ui/app.py` | FastAPI routes, composition root |
| `tools/gpsr_ui/vendor/sheet_events.py` | Vendored classifier, pinned at `9072c6e` |
| `tools/gpsr_ui/static/*.js` | Frontend modules, no build step |
| `tools/gpsr_ui/templates/*.html` | Jinja2 page shells |
| `tools/tests/` | pytest suite plus hermetic fixture builders |
| `tools/gpsr-ui` | Launcher shell script |

Backend modules never import each other except through `app.py`, with two exceptions: `telemetry.py` imports `vendor.sheet_events`, and everything may import `config.py`.

---

### Task 1: Package skeleton, config, and hermetic fixture builder

**Files:**
- Create: `tools/gpsr_ui/__init__.py`, `tools/gpsr_ui/config.py`
- Create: `tools/gpsr_ui/vendor/__init__.py`, `tools/gpsr_ui/vendor/sheet_events.py`
- Create: `tools/tests/__init__.py`, `tools/tests/conftest.py`
- Create: `tools/pytest.ini`, `tools/gpsr-ui`
- Test: `tools/tests/test_config.py`

**Interfaces:**
- Consumes: nothing.
- Produces: `Settings` dataclass with fields `bench_root: Path`, `state_dir: Path`, `sheet_events_path: Path | None`; `load_settings() -> Settings`. Pytest fixture `make_run(tmp_path, **kw) -> Path` that builds a synthetic run directory, and fixture `corpus_root` that returns the real bench root or skips.

- [ ] **Step 1: Vendor the classifier**

```bash
mkdir -p tools/gpsr_ui/vendor tools/tests
touch tools/gpsr_ui/__init__.py tools/tests/__init__.py
cp /home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec/tools/sheet_events.py \
   tools/gpsr_ui/vendor/sheet_events.py
printf '' > tools/gpsr_ui/vendor/__init__.py
```

Then prepend this header to `tools/gpsr_ui/vendor/sheet_events.py`, above its existing docstring:

```python
# VENDORED from tinker-sim tools/sheet_events.py at commit 9072c6e.
# Do not edit. To refresh, re-copy and update this commit hash.
# The upstream owner (session "gpsr command testing robustness") has
# committed to messaging before changing the public surface:
#   load_run_telemetry(run_dir) -> (list[MilestoneEvent], list[JudgeEvent], dict)
#   MilestoneEvent(wall, kind, name, status, info)
#   JudgeEvent(wall, kind, name, status, info)
#   meta["tree_generations"] -> int, the tree.generated epoch count
# This module emits REPLAN JudgeEvents itself when tree_generations >= 3.
# gpsr_ui.telemetry must NOT emit its own replan events on top of these.
```

- [ ] **Step 2: Write the failing test**

```python
# tools/tests/test_config.py
from __future__ import annotations

from pathlib import Path

from gpsr_ui.config import load_settings


def test_defaults_point_at_bench_and_user_cache(monkeypatch):
    monkeypatch.delenv("GPSR_UI_BENCH_ROOT", raising=False)
    monkeypatch.delenv("GPSR_UI_STATE_DIR", raising=False)
    monkeypatch.delenv("GPSR_UI_SHEET_EVENTS", raising=False)
    s = load_settings()
    assert s.bench_root.name == "bench"
    assert s.state_dir == Path.home() / ".cache" / "gpsr-ui"
    assert s.sheet_events_path is None


def test_environment_overrides_are_expanded(monkeypatch, tmp_path):
    monkeypatch.setenv("GPSR_UI_BENCH_ROOT", str(tmp_path / "b"))
    monkeypatch.setenv("GPSR_UI_STATE_DIR", str(tmp_path / "s"))
    monkeypatch.setenv("GPSR_UI_SHEET_EVENTS", str(tmp_path / "se.py"))
    s = load_settings()
    assert s.bench_root == tmp_path / "b"
    assert s.state_dir == tmp_path / "s"
    assert s.sheet_events_path == tmp_path / "se.py"


def test_state_dir_is_never_inside_the_corpus(monkeypatch, tmp_path):
    """The read-only guarantee starts here: refuse to write into gpsr_runs."""
    monkeypatch.setenv("GPSR_UI_BENCH_ROOT", str(tmp_path / "bench"))
    monkeypatch.setenv("GPSR_UI_STATE_DIR", str(tmp_path / "bench" / "ui"))
    import pytest
    with pytest.raises(ValueError, match="must not be inside"):
        load_settings()
```

- [ ] **Step 3: Run test to verify it fails**

Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest tests/test_config.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'gpsr_ui.config'`

- [ ] **Step 4: Write the implementation**

```python
# tools/gpsr_ui/config.py
from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

DEFAULT_BENCH_ROOT = Path(
    "/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree"
    "/GPSR/gpsr_runs/bench"
)


@dataclass(frozen=True)
class Settings:
    bench_root: Path
    state_dir: Path
    sheet_events_path: Path | None


def _path_env(name: str, default: Path | None) -> Path | None:
    raw = os.environ.get(name)
    if raw is None or not raw.strip():
        return default
    return Path(raw).expanduser()


def load_settings() -> Settings:
    bench_root = _path_env("GPSR_UI_BENCH_ROOT", DEFAULT_BENCH_ROOT)
    state_dir = _path_env("GPSR_UI_STATE_DIR", Path.home() / ".cache" / "gpsr-ui")
    assert bench_root is not None and state_dir is not None

    # Enforce the read-only guarantee structurally, not by convention.
    if bench_root == state_dir or bench_root in state_dir.parents:
        raise ValueError(
            f"GPSR_UI_STATE_DIR ({state_dir}) must not be inside the bench "
            f"corpus ({bench_root}); the viewer never writes there"
        )

    return Settings(
        bench_root=bench_root,
        state_dir=state_dir,
        sheet_events_path=_path_env("GPSR_UI_SHEET_EVENTS", None),
    )
```

```ini
# tools/pytest.ini
[pytest]
testpaths = tests
markers =
    corpus: test reads the real bench corpus; skipped when it is absent
```

- [ ] **Step 5: Run test to verify it passes**

Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest tests/test_config.py -v`
Expected: 3 passed

- [ ] **Step 6: Write the fixture builder**

This is what makes every later task hermetic. `make_run` writes a synthetic run directory with the same shape as a real one.

```python
# tools/tests/conftest.py
from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

REAL_BENCH = Path(
    "/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree"
    "/GPSR/gpsr_runs/bench"
)


def _event(seq: int, etype: str, wall: str, payload: dict) -> dict:
    return {
        "schema": "tinker.gpsr.telemetry",
        "schema_version": 1,
        "event_id": f"evt-{seq}",
        "trajectory_id": "gpsr-TEST-0001",
        "task_id": None,
        "trace_id": "trace-0001",
        "source_id": "gpsr-orchestrator:1",
        "sequence": seq,
        "occurred_at": wall,
        "monotonic_ns": 1_000_000_000 + seq,
        "event_type": etype,
        "phase": None,
        "parent_event_id": None,
        "causation_ids": [],
        "payload": payload,
    }


def _tree(node_names: list[str]) -> dict:
    """Root Sequence with one leaf per name. tree_revision is always 0,
    matching every tree.generated event in the real corpus."""
    nodes = [
        {
            "id": "executor/root",
            "node_id": "executor/root",
            "parent_id": None,
            "name": "GPSR orchestrator",
            "type": "Sequence",
            "status": "RUNNING",
            "children": [f"executor/root/{i}" for i in range(len(node_names))],
            "order": 0,
            "node_class": "composite",
            "semantics": {"category": "composite", "kind": "sequence"},
            "blackboard_access": {"read": [], "write": [], "exclusive": []},
            "action_context": {},
        }
    ]
    for i, name in enumerate(node_names):
        nodes.append(
            {
                "id": f"executor/root/{i}",
                "node_id": f"executor/root/{i}",
                "parent_id": "executor/root",
                "name": name,
                "type": "BtNode_Announce",
                "status": "SUCCESS",
                "children": [],
                "order": i + 1,
                "node_class": "leaf",
                "semantics": {"category": "leaf", "kind": "leaf"},
                "blackboard_access": {"read": [], "write": [], "exclusive": []},
                "action_context": {},
            }
        )
    return {
        "kind": "executor",
        "tree_id": "executor",
        "tree_revision": 0,
        "root_id": "executor/root",
        "nodes": nodes,
    }


@pytest.fixture
def make_run(tmp_path):
    """Build a synthetic run dir. Returns its Path.

    epochs: list of node-name lists, one per tree.generated event.
    transitions: list of (wall, node_id, status, feedback).
    frames: {label: [(index, stamp_ms)]}.
    """

    def _make(
        name: str = "s9999-000-testEntry",
        verdict: str | None = "PASS",
        seconds: float = 42.0,
        epochs: list[list[str]] | None = None,
        transitions: list[tuple[str, str, str, str]] | None = None,
        frames: dict[str, list[tuple[int, int]]] | None = None,
        recorder_meta: dict | None = None,
        index_lines: list[dict] | None = None,
        announcements: list[str] | None = None,
        finished: bool = True,
    ) -> Path:
        run = tmp_path / "t9-test" / "runs" / name
        debug = run / "debug" / "gpsr-20260828T000000000000Z-test"
        debug.mkdir(parents=True)

        epochs = epochs if epochs is not None else [["announce ready for gpsr"]]
        transitions = transitions or []

        events: list[dict] = [
            _event(1, "run.started", "2026-08-28T10:00:00.000000Z",
                   {"trajectory_id": "gpsr-TEST-0001"}),
            _event(2, "run.configured", "2026-08-28T10:00:00.100000Z",
                   {"expected_task_count": 1, "mode": "injected"}),
        ]
        seq = 3
        for i, names in enumerate(epochs):
            events.append(
                _event(seq, "tree.generated",
                       f"2026-08-28T10:00:0{i + 1}.000000Z", _tree(names))
            )
            seq += 1
        for wall, node_id, status, feedback in transitions:
            events.append(
                _event(seq, "tree.node_states_changed", wall, {
                    "tree_kind": "executor",
                    "tree_revision": 0,
                    "tick": seq,
                    "nodes": [{
                        "id": node_id, "node_id": node_id,
                        "visit_order": 0, "topology_order": 1,
                        "status": status, "feedback": feedback,
                        "node_class": "leaf",
                        "semantics": {"category": "leaf", "kind": "leaf"},
                        "blackboard_access": {
                            "read": [], "write": [], "exclusive": []},
                        "action_context": {},
                    }],
                })
            )
            seq += 1
        if finished:
            # status is always "incomplete" in the real corpus, even on PASS.
            events.append(
                _event(seq, "run.finished", "2026-08-28T10:05:00.000000Z",
                       {"trajectory_id": "gpsr-TEST-0001",
                        "status": "incomplete"})
            )

        with (debug / "events.jsonl").open("w") as fh:
            for e in events:
                fh.write(json.dumps(e) + "\n")

        for label, specs in (frames or {}).items():
            d = run / "frames" / label
            d.mkdir(parents=True)
            for idx, stamp_ms in specs:
                (d / f"{idx:04d}_{stamp_ms}.jpg").write_bytes(b"\xff\xd8\xff\xd9")
        if index_lines is not None:
            with (run / "frames" / "index.jsonl").open("w") as fh:
                for line in index_lines:
                    fh.write(json.dumps(line) + "\n")
        if recorder_meta is not None:
            (run / "recorder-meta.json").write_text(json.dumps(recorder_meta))
        if announcements is not None:
            (run / "announcements.txt").write_text(
                "\n".join(announcements) + "\n")

        # run.json is written LAST by the bench; its absence means in-flight.
        if verdict is not None:
            (run / "run.json").write_text(json.dumps({
                "id": name.split(".")[0], "text": "test command",
                "template": "testEntry", "feasibility": "A", "tier": "T9",
                "verdict": verdict, "detail": "", "seconds": seconds,
            }))
        return run

    return _make


@pytest.fixture
def corpus_root():
    if os.environ.get("GPSR_UI_SKIP_CORPUS"):
        pytest.skip("corpus tests disabled")
    if not REAL_BENCH.is_dir():
        pytest.skip(f"real bench corpus not present at {REAL_BENCH}")
    return REAL_BENCH
```

- [ ] **Step 7: Write the launcher**

```bash
# tools/gpsr-ui
#!/usr/bin/env bash
# Launch the GPSR bench run viewer. Reads the corpus read-only.
set -euo pipefail
cd "$(dirname "$0")"
exec /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m uvicorn \
    gpsr_ui.app:app --host 127.0.0.1 --port "${GPSR_UI_PORT:-8770}" "$@"
```

Then `chmod +x tools/gpsr-ui`.

- [ ] **Step 8: Verify the whole suite and the vendored import**

Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest -v`
Expected: 3 passed
Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -c "from gpsr_ui.vendor import sheet_events; print(sheet_events.load_run_telemetry.__doc__.splitlines()[0])"`
Expected: prints `Extract milestone/judge events and run meta from a GPSR run's telemetry.`

- [ ] **Step 9: Commit**

```bash
git add tools/
git commit -m "feat: gpsr_ui package skeleton, config, and hermetic test fixtures"
```

---

### Task 2: Corpus discovery and attempt grouping

**Files:**
- Create: `tools/gpsr_ui/corpus.py`
- Test: `tools/tests/test_corpus.py`

**Interfaces:**
- Consumes: `Settings` from `gpsr_ui.config`.
- Produces:
  - `Attempt` dataclass: `dir_name: str`, `entry_id: str`, `slug: str | None`, `path: Path`, `verdict: str | None`, `seconds: float | None`, `detail: str`, `is_current: bool`, `mtime: float`.
  - `Entry` dataclass: `entry_id: str`, `template: str | None`, `feasibility: str | None`, `text: str | None`, `attempts: list[Attempt]` (current first, then archived newest-first).
  - `Tier` dataclass: `name: str`, `path: Path`, `entries: list[Entry]`.
  - `split_dir_name(name: str) -> tuple[str, str | None]`
  - `list_tiers(bench_root: Path) -> list[Tier]`
  - `find_run(bench_root: Path, tier: str, dir_name: str) -> Path | None`

- [ ] **Step 1: Write the failing test**

```python
# tools/tests/test_corpus.py
from __future__ import annotations

import pytest

from gpsr_ui.corpus import Attempt, list_tiers, split_dir_name


def test_split_dir_name_separates_entry_from_attempt_slug():
    assert split_dir_name("s2026-002-countPrsInRoom") == (
        "s2026-002-countPrsInRoom", None)
    assert split_dir_name("s2026-002-countPrsInRoom.attempt7-rpp") == (
        "s2026-002-countPrsInRoom", "attempt7-rpp")
    assert split_dir_name("s2026-002-countPrsInRoom.attempt10-nav-fixed") == (
        "s2026-002-countPrsInRoom", "attempt10-nav-fixed")


def test_attempts_group_under_their_entry_with_current_first(make_run):
    run = make_run(name="s9999-000-testEntry", verdict="PASS")
    make_run(name="s9999-000-testEntry.attempt2-foo", verdict="FAIL")
    make_run(name="s9999-000-testEntry.attempt3-bar", verdict="TIMEOUT")
    bench_root = run.parents[2]

    tiers = list_tiers(bench_root)
    assert [t.name for t in tiers] == ["t9-test"]
    entries = tiers[0].entries
    assert len(entries) == 1, "three dirs, one corpus entry"

    entry = entries[0]
    assert entry.entry_id == "s9999-000-testEntry"
    assert len(entry.attempts) == 3
    assert entry.attempts[0].is_current is True
    assert entry.attempts[0].slug is None
    assert {a.slug for a in entry.attempts[1:]} == {
        "attempt2-foo", "attempt3-bar"}
    assert all(a.is_current is False for a in entry.attempts[1:])


def test_verdict_comes_from_run_json_not_run_finished(make_run):
    """run.finished.status is 'incomplete' even on PASS runs."""
    run = make_run(name="s9999-001-x", verdict="PASS")
    tiers = list_tiers(run.parents[2])
    assert tiers[0].entries[0].attempts[0].verdict == "PASS"


def test_missing_run_json_yields_none_verdict_not_a_crash(make_run):
    run = make_run(name="s9999-002-x", verdict=None)
    tiers = list_tiers(run.parents[2])
    attempt = tiers[0].entries[0].attempts[0]
    assert attempt.verdict is None
    assert attempt.seconds is None


def test_empty_bench_root_yields_no_tiers(tmp_path):
    assert list_tiers(tmp_path) == []


@pytest.mark.corpus
def test_real_corpus_groups_the_known_attempt_history(corpus_root):
    tiers = {t.name: t for t in list_tiers(corpus_root)}
    assert "t2-2026" in tiers
    entries = {e.entry_id: e for e in tiers["t2-2026"].entries}
    counts = entries["s2026-002-countPrsInRoom"]
    # The battery may add more attempts; assert the floor, not equality.
    assert len(counts.attempts) >= 11
    assert counts.attempts[0].is_current is True
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest tests/test_corpus.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'gpsr_ui.corpus'`

- [ ] **Step 3: Write the implementation**

```python
# tools/gpsr_ui/corpus.py
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

# Tier dirs hold their runs in "runs", plus historical siblings such as
# "runs-invalidated-20260826" which we surface as separate pseudo-tiers.
_RUNS_PREFIX = "runs"


@dataclass(frozen=True)
class Attempt:
    dir_name: str
    entry_id: str
    slug: str | None
    path: Path
    verdict: str | None
    seconds: float | None
    detail: str
    is_current: bool
    mtime: float


@dataclass
class Entry:
    entry_id: str
    template: str | None = None
    feasibility: str | None = None
    text: str | None = None
    attempts: list[Attempt] = field(default_factory=list)


@dataclass
class Tier:
    name: str
    path: Path
    entries: list[Entry] = field(default_factory=list)


def split_dir_name(name: str) -> tuple[str, str | None]:
    """`s2026-002-x.attempt7-rpp` -> (`s2026-002-x`, `attempt7-rpp`)."""
    entry_id, sep, slug = name.partition(".")
    return (entry_id, slug if sep else None)


def _read_run_json(run_dir: Path) -> dict:
    try:
        return json.loads((run_dir / "run.json").read_text())
    except (OSError, ValueError):
        return {}


def _attempt(run_dir: Path) -> Attempt:
    entry_id, slug = split_dir_name(run_dir.name)
    data = _read_run_json(run_dir)
    seconds = data.get("seconds")
    return Attempt(
        dir_name=run_dir.name,
        entry_id=entry_id,
        slug=slug,
        path=run_dir,
        verdict=data.get("verdict"),
        seconds=float(seconds) if isinstance(seconds, (int, float)) else None,
        detail=data.get("detail") or "",
        is_current=slug is None,
        mtime=run_dir.stat().st_mtime,
    )


def _corpus_metadata(tier_dir: Path) -> dict[str, dict]:
    """entry_id -> corpus.jsonl record, for template/feasibility/text."""
    out: dict[str, dict] = {}
    path = tier_dir / "corpus.jsonl"
    try:
        text = path.read_text()
    except OSError:
        return out
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rec = json.loads(line)
        except ValueError:
            continue
        if isinstance(rec, dict) and isinstance(rec.get("id"), str):
            out[rec["id"]] = rec
    return out


def _build_tier(name: str, runs_dir: Path, meta: dict[str, dict]) -> Tier:
    by_entry: dict[str, Entry] = {}
    for run_dir in sorted(p for p in runs_dir.iterdir() if p.is_dir()):
        attempt = _attempt(run_dir)
        entry = by_entry.get(attempt.entry_id)
        if entry is None:
            rec = meta.get(attempt.entry_id, {})
            entry = Entry(
                entry_id=attempt.entry_id,
                template=rec.get("template"),
                feasibility=rec.get("feasibility"),
                text=rec.get("text"),
            )
            by_entry[attempt.entry_id] = entry
        entry.attempts.append(attempt)

    for entry in by_entry.values():
        # Current attempt first; archived attempts newest-first behind it.
        entry.attempts.sort(key=lambda a: (a.is_current, a.mtime), reverse=True)

    return Tier(
        name=name,
        path=runs_dir,
        entries=sorted(by_entry.values(), key=lambda e: e.entry_id),
    )


def list_tiers(bench_root: Path) -> list[Tier]:
    tiers: list[Tier] = []
    try:
        tier_dirs = sorted(p for p in bench_root.iterdir() if p.is_dir())
    except OSError:
        return []

    for tier_dir in tier_dirs:
        meta = _corpus_metadata(tier_dir)
        for runs_dir in sorted(
            p for p in tier_dir.iterdir()
            if p.is_dir() and p.name.startswith(_RUNS_PREFIX)
        ):
            suffix = runs_dir.name[len(_RUNS_PREFIX):].lstrip("-")
            name = tier_dir.name if not suffix else f"{tier_dir.name}/{suffix}"
            tier = _build_tier(name, runs_dir, meta)
            if tier.entries:
                tiers.append(tier)
    return tiers


def find_run(bench_root: Path, tier: str, dir_name: str) -> Path | None:
    """Resolve a (tier, dir_name) pair to a run dir, refusing traversal."""
    if "/" in dir_name or dir_name in {"", ".", ".."}:
        return None
    for candidate in list_tiers(bench_root):
        if candidate.name != tier:
            continue
        for entry in candidate.entries:
            for attempt in entry.attempts:
                if attempt.dir_name == dir_name:
                    return attempt.path
    return None
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest tests/test_corpus.py -v`
Expected: 6 passed

- [ ] **Step 5: Commit**

```bash
git add tools/gpsr_ui/corpus.py tools/tests/test_corpus.py
git commit -m "feat: corpus discovery with attempt grouping"
```

---

### Task 3: Clock mapping

**Files:**
- Create: `tools/gpsr_ui/clock.py`
- Test: `tools/tests/test_clock.py`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `Clock` dataclass: `mode: str` (`"exact"` | `"approximate"` | `"none"`), `labels: list[str]`.
  - `Clock.sim_to_wall(label: str, stamp_s: float) -> float | None` (epoch seconds)
  - `Clock.wall_to_frame(label: str, wall: float) -> str | None` (frame filename)
  - `load_clock(run_dir: Path) -> Clock`
  - `parse_frame_name(name: str) -> tuple[int, float] | None` → `(index, stamp_s)`
  - `parse_wall(iso: str) -> float | None`

- [ ] **Step 1: Write the failing test**

```python
# tools/tests/test_clock.py
from __future__ import annotations

import json

from gpsr_ui.clock import load_clock, parse_frame_name, parse_wall


def test_parse_frame_name_reads_index_and_sim_seconds():
    assert parse_frame_name("0042_2123833.jpg") == (42, 2123.833)
    assert parse_frame_name("0000_2081833.jpg") == (0, 2081.833)
    assert parse_frame_name("not-a-frame.jpg") is None
    assert parse_frame_name("0001.jpg") is None


def test_parse_wall_handles_the_z_suffix_used_in_events():
    assert parse_wall("2026-08-28T10:52:30.735414Z") == parse_wall(
        "2026-08-28T10:52:30.735414+00:00")
    assert parse_wall("nonsense") is None


def test_exact_mode_when_index_jsonl_present(make_run):
    run = make_run(
        name="s9999-010-x",
        frames={"head": [(0, 1000), (1, 2000)]},
        index_lines=[
            {"label": "head", "file": "0000_1000.jpg",
             "stamp_s": 1.0, "wall": "2026-08-28T10:00:00.000000Z"},
            {"label": "head", "file": "0001_2000.jpg",
             "stamp_s": 2.0, "wall": "2026-08-28T10:00:05.000000Z"},
        ],
    )
    clock = load_clock(run)
    assert clock.mode == "exact"
    assert clock.labels == ["head"]
    # Exact join: 1 sim-second maps to 5 wall-seconds here (RTF 0.2).
    assert clock.sim_to_wall("head", 2.0) == parse_wall(
        "2026-08-28T10:00:05.000000Z")
    # At-or-BEFORE, never nearest: frame 1 is stamped 10:00:05, one second
    # after this query, and returning it would show a frame from the future.
    assert clock.wall_to_frame(
        "head", parse_wall("2026-08-28T10:00:04.000000Z")) == "0000_1000.jpg"


def test_approximate_mode_interpolates_from_recorder_meta(make_run):
    run = make_run(
        name="s9999-011-x",
        frames={"head": [(0, 1000), (1, 2000), (2, 3000)]},
        recorder_meta={
            "labels": {"head": {
                "frames": 3, "first_stamp": 1.0, "last_stamp": 3.0}},
            "started_wall": "2026-08-28T10:00:00+00:00",
            "ended_wall": "2026-08-28T10:00:10+00:00",
        },
    )
    clock = load_clock(run)
    assert clock.mode == "approximate"
    # Midpoint of a 2 sim-second span across 10 wall-seconds.
    assert clock.sim_to_wall("head", 2.0) == parse_wall(
        "2026-08-28T10:00:05+00:00")


def test_no_metadata_yields_mode_none_rather_than_a_crash(make_run):
    run = make_run(name="s9999-012-x", frames={"head": [(0, 1000)]})
    clock = load_clock(run)
    assert clock.mode == "none"
    assert clock.sim_to_wall("head", 1.0) is None


def test_index_jsonl_wins_over_recorder_meta(make_run):
    """Both present: the exact join must be preferred."""
    run = make_run(
        name="s9999-013-x",
        frames={"head": [(0, 1000)]},
        index_lines=[{"label": "head", "file": "0000_1000.jpg",
                      "stamp_s": 1.0, "wall": "2026-08-28T10:00:00.000000Z"}],
        recorder_meta={
            "labels": {"head": {
                "frames": 1, "first_stamp": 1.0, "last_stamp": 1.0}},
            "started_wall": "2026-08-28T09:00:00+00:00",
            "ended_wall": "2026-08-28T09:00:10+00:00",
        },
    )
    clock = load_clock(run)
    assert clock.mode == "exact"
    assert clock.sim_to_wall("head", 1.0) == parse_wall(
        "2026-08-28T10:00:00.000000Z")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest tests/test_clock.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'gpsr_ui.clock'`

- [ ] **Step 3: Write the implementation**

```python
# tools/gpsr_ui/clock.py
"""Map between the sim clock (frame filenames) and wall time (events).

The two are NOT proportionally related: real-time factor varies between
roughly 0.2 and 0.5 within a single run. Frames therefore cannot be placed
on the event timeline by scaling. Two strategies exist:

  exact        join on frames/index.jsonl, which carries stamp_s and wall
               per frame. Correct.
  approximate  linear interpolation between recorder-meta.json's
               started_wall/ended_wall and first_stamp/last_stamp. Wrong
               mid-run by seconds wherever RTF deviates from its average.

The mode is surfaced so the UI can badge it. An approximate alignment must
never be mistakable for a real one.
"""
from __future__ import annotations

import bisect
import json
import re
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

_FRAME_RE = re.compile(r"^(\d+)_(\d+)\.jpg$")


def parse_frame_name(name: str) -> tuple[int, float] | None:
    m = _FRAME_RE.match(name)
    if m is None:
        return None
    return int(m.group(1)), int(m.group(2)) / 1000.0


def parse_wall(iso: str | None) -> float | None:
    if not isinstance(iso, str) or not iso:
        return None
    try:
        return datetime.fromisoformat(iso.replace("Z", "+00:00")).timestamp()
    except ValueError:
        return None


@dataclass
class Clock:
    mode: str = "none"
    labels: list[str] = field(default_factory=list)
    # label -> (sorted sim stamps, parallel wall times, parallel filenames)
    _stamps: dict[str, list[float]] = field(default_factory=dict)
    _walls: dict[str, list[float]] = field(default_factory=dict)
    _files: dict[str, list[str]] = field(default_factory=dict)
    # label -> (first_stamp, last_stamp, started_wall, ended_wall)
    _linear: dict[str, tuple[float, float, float, float]] = field(
        default_factory=dict)

    def sim_to_wall(self, label: str, stamp_s: float) -> float | None:
        if self.mode == "exact":
            stamps = self._stamps.get(label)
            walls = self._walls.get(label)
            if not stamps or not walls:
                return None
            i = bisect.bisect_left(stamps, stamp_s)
            if i < len(stamps) and stamps[i] == stamp_s:
                return walls[i]
            return _interpolate(stamps, walls, stamp_s)
        if self.mode == "approximate":
            span = self._linear.get(label)
            if span is None:
                return None
            first, last, started, ended = span
            if last == first:
                return started
            frac = (stamp_s - first) / (last - first)
            return started + frac * (ended - started)
        return None

    def wall_to_frame(self, label: str, wall: float | None) -> str | None:
        """Filename of the frame whose wall time is nearest at-or-before."""
        if wall is None:
            return None
        files = self._files.get(label)
        if not files:
            return None
        walls = self._wall_series(label)
        if walls is None:
            return None
        i = bisect.bisect_right(walls, wall) - 1
        if i < 0:
            i = 0
        return files[i]

    def _wall_series(self, label: str) -> list[float] | None:
        if self.mode == "exact":
            return self._walls.get(label)
        if self.mode == "approximate":
            stamps = self._stamps.get(label)
            if not stamps:
                return None
            out = [self.sim_to_wall(label, s) for s in stamps]
            return [w for w in out if w is not None]
        return None


def _interpolate(xs: list[float], ys: list[float], x: float) -> float | None:
    if not xs:
        return None
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    i = bisect.bisect_left(xs, x)
    x0, x1 = xs[i - 1], xs[i]
    y0, y1 = ys[i - 1], ys[i]
    if x1 == x0:
        return y0
    return y0 + (x - x0) / (x1 - x0) * (y1 - y0)


def _scan_frames(run_dir: Path) -> dict[str, list[tuple[float, str]]]:
    out: dict[str, list[tuple[float, str]]] = {}
    frames_dir = run_dir / "frames"
    if not frames_dir.is_dir():
        return out
    for label_dir in sorted(p for p in frames_dir.iterdir() if p.is_dir()):
        items: list[tuple[float, str]] = []
        for f in label_dir.iterdir():
            parsed = parse_frame_name(f.name)
            if parsed is not None:
                items.append((parsed[1], f.name))
        if items:
            out[label_dir.name] = sorted(items)
    return out


def _load_exact(run_dir: Path, clock: Clock) -> bool:
    path = run_dir / "frames" / "index.jsonl"
    try:
        text = path.read_text()
    except OSError:
        return False

    rows: dict[str, list[tuple[float, float, str]]] = {}
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rec = json.loads(line)
        except ValueError:
            continue  # a live run may have a torn final line
        if not isinstance(rec, dict):
            continue
        label, file = rec.get("label"), rec.get("file")
        stamp, wall = rec.get("stamp_s"), parse_wall(rec.get("wall"))
        if not isinstance(label, str) or not isinstance(file, str):
            continue
        if not isinstance(stamp, (int, float)) or wall is None:
            continue
        rows.setdefault(label, []).append((float(stamp), wall, file))

    if not rows:
        return False
    for label, items in rows.items():
        items.sort()
        clock._stamps[label] = [i[0] for i in items]
        clock._walls[label] = [i[1] for i in items]
        clock._files[label] = [i[2] for i in items]
    clock.mode = "exact"
    clock.labels = sorted(rows)
    return True


def _load_approximate(run_dir: Path, clock: Clock) -> bool:
    try:
        meta = json.loads((run_dir / "recorder-meta.json").read_text())
    except (OSError, ValueError):
        return False
    started = parse_wall(meta.get("started_wall"))
    ended = parse_wall(meta.get("ended_wall"))
    labels = meta.get("labels")
    if started is None or ended is None or not isinstance(labels, dict):
        return False

    scanned = _scan_frames(run_dir)
    ok = False
    for label, info in labels.items():
        if not isinstance(info, dict):
            continue
        first, last = info.get("first_stamp"), info.get("last_stamp")
        if not isinstance(first, (int, float)):
            continue
        if not isinstance(last, (int, float)):
            continue
        clock._linear[label] = (float(first), float(last), started, ended)
        items = scanned.get(label, [])
        clock._stamps[label] = [i[0] for i in items]
        clock._files[label] = [i[1] for i in items]
        ok = True

    if ok:
        clock.mode = "approximate"
        clock.labels = sorted(clock._linear)
    return ok


def load_clock(run_dir: Path) -> Clock:
    clock = Clock()
    if _load_exact(run_dir, clock):
        return clock
    if _load_approximate(run_dir, clock):
        return clock
    scanned = _scan_frames(run_dir)
    clock.labels = sorted(scanned)
    for label, items in scanned.items():
        clock._stamps[label] = [i[0] for i in items]
        clock._files[label] = [i[1] for i in items]
    return clock
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest tests/test_clock.py -v`
Expected: 6 passed

- [ ] **Step 5: Commit**

```bash
git add tools/gpsr_ui/clock.py tools/tests/test_clock.py
git commit -m "feat: sim/wall clock mapping with exact and approximate modes"
```

---

### Task 4: Telemetry model

**Files:**
- Create: `tools/gpsr_ui/telemetry.py`
- Test: `tools/tests/test_telemetry.py`

**Interfaces:**
- Consumes: `gpsr_ui.vendor.sheet_events`, `gpsr_ui.clock.parse_wall`.
- Produces:
  - `TreeNode` dataclass: `id`, `name`, `type`, `parent_id`, `children: list[str]`, `node_class`, `reads: list[str]`, `writes: list[str]`.
  - `Epoch` dataclass: `ordinal: int`, `wall: float | None`, `sequence: int`, `root_id: str`, `nodes: dict[str, TreeNode]`.
  - `Transition` dataclass: `wall: float | None`, `tick: int`, `node_id: str`, `status: str`, `feedback: str`.
  - `RunModel` dataclass: `trajectory_id`, `epochs: list[Epoch]`, `transitions: list[Transition]`, `milestones`, `judge_events`, `tree_regenerations: int`, `gate_failures: int`, `started_wall`, `finished_wall`, `announcements: list[str]`.
  - `RunModel.epoch_at(wall) -> Epoch | None`, `RunModel.status_at(wall) -> dict[str, Transition]`
  - `load_run_model(run_dir: Path) -> RunModel`
  - `newest_events_file(run_dir: Path) -> Path | None`
  - `dedupe_announcements(lines: Iterable[str]) -> list[str]`

- [ ] **Step 1: Write the failing test**

```python
# tools/tests/test_telemetry.py
from __future__ import annotations

import pytest

from gpsr_ui.clock import parse_wall
from gpsr_ui.telemetry import dedupe_announcements, load_run_model


def _t(sec: int) -> str:
    return f"2026-08-28T10:0{sec // 60}:{sec % 60:02d}.000000Z"


def test_epochs_beyond_the_normal_pair_are_regenerations(make_run):
    """tree_revision is 0 corpus-wide, so epochs are counted, not read.
    Two epochs is the NORMAL pair (skeleton, then plan materialisation),
    so only the third onward is a genuine regeneration."""
    run = make_run(
        name="s9999-020-x",
        epochs=[["a"], ["a", "b"], ["a", "b", "c"]],
    )
    model = load_run_model(run)
    assert [e.ordinal for e in model.epochs] == [0, 1, 2]
    assert [len(e.nodes) for e in model.epochs] == [2, 3, 4]  # +1 for root
    assert model.tree_regenerations == 1


def test_the_normal_two_epoch_pair_is_not_a_regeneration(make_run):
    """Every t2-2026 run has exactly 2 epochs, including the one that
    replan-looped for the full 900s timeout. Reporting 1 here would put a
    phantom regeneration on every healthy run."""
    run = make_run(name="s9999-021-x", epochs=[["a"], ["a", "b"]])
    assert load_run_model(run).tree_regenerations == 0


def test_a_single_epoch_run_is_not_negative(make_run):
    run = make_run(name="s9999-021b-x", epochs=[["a"]])
    assert load_run_model(run).tree_regenerations == 0


def test_epoch_at_returns_the_latest_epoch_at_or_before_a_playhead(make_run):
    run = make_run(name="s9999-022-x", epochs=[["a"], ["a", "b"]])
    model = load_run_model(run)
    # Fixture stamps epoch i at 10:00:0(i+1).
    before = parse_wall("2026-08-28T10:00:00.500000Z")
    between = parse_wall("2026-08-28T10:00:01.500000Z")
    after = parse_wall("2026-08-28T10:00:09.000000Z")
    assert model.epoch_at(before) is None
    assert model.epoch_at(between).ordinal == 0
    assert model.epoch_at(after).ordinal == 1


def test_status_at_is_the_last_transition_at_or_before_the_playhead(make_run):
    run = make_run(
        name="s9999-023-x",
        epochs=[["announce ready for gpsr"]],
        transitions=[
            (_t(10), "executor/root/0", "RUNNING", "starting"),
            (_t(20), "executor/root/0", "SUCCESS", "done"),
        ],
    )
    model = load_run_model(run)
    at15 = model.status_at(parse_wall(_t(15)))
    assert at15["executor/root/0"].status == "RUNNING"
    at25 = model.status_at(parse_wall(_t(25)))
    assert at25["executor/root/0"].status == "SUCCESS"
    assert model.status_at(parse_wall(_t(5))) == {}


def test_nodes_carry_blackboard_access_and_parentage(make_run):
    run = make_run(name="s9999-024-x", epochs=[["announce ready for gpsr"]])
    model = load_run_model(run)
    nodes = model.epochs[0].nodes
    assert nodes["executor/root"].parent_id is None
    assert nodes["executor/root"].children == ["executor/root/0"]
    leaf = nodes["executor/root/0"]
    assert leaf.name == "announce ready for gpsr"
    assert leaf.type == "BtNode_Announce"
    assert leaf.node_class == "leaf"


def test_milestones_come_from_the_vendored_classifier(make_run):
    run = make_run(
        name="s9999-025-x",
        epochs=[["announce ready for gpsr"]],
        transitions=[(_t(10), "executor/root/0", "SUCCESS", "Hi, I am Tinker")],
    )
    model = load_run_model(run)
    assert [m.kind for m in model.milestones] == ["AUDIO"]
    assert model.milestones[0].name == "announce ready for gpsr"


def test_corrupt_and_truncated_lines_are_skipped_not_fatal(make_run):
    run = make_run(name="s9999-026-x", epochs=[["a"]])
    events = next((run / "debug").glob("gpsr-*")) / "events.jsonl"
    with events.open("a") as fh:
        fh.write("{not json at all\n")
        fh.write('{"event_type": "tree.gen')  # torn final line, no newline
    model = load_run_model(run)
    assert len(model.epochs) == 1
    assert model.tree_regenerations == 0


def test_missing_events_file_yields_an_empty_model(tmp_path):
    empty = tmp_path / "runs" / "nothing"
    empty.mkdir(parents=True)
    model = load_run_model(empty)
    assert model.epochs == []
    assert model.tree_regenerations == 0
    assert model.trajectory_id is None


def test_dedupe_announcements_keeps_first_occurrence_order():
    raw = ["hello", "world", "hello", "world", "bye", "hello"]
    assert dedupe_announcements(raw) == ["hello", "world", "bye"]


def test_announcements_are_deduped_in_the_model(make_run):
    run = make_run(
        name="s9999-027-x",
        announcements=["a", "b", "a", "b", "a", "c"],
    )
    assert load_run_model(run).announcements == ["a", "b", "c"]


@pytest.mark.corpus
def test_real_run_has_the_normal_epoch_pair_and_no_regeneration(corpus_root):
    run = corpus_root / "t2-2026" / "runs" / "s2026-002-countPrsInRoom"
    if not run.is_dir():
        pytest.skip("reference run has been re-run and archived")
    model = load_run_model(run)
    assert len(model.epochs) == 2
    assert [len(e.nodes) for e in model.epochs] == [84, 158]
    assert model.tree_regenerations == 0
    # 7967 raw announcement lines collapse to 11 distinct utterances.
    assert len(model.announcements) == 11
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest tests/test_telemetry.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'gpsr_ui.telemetry'`

- [ ] **Step 3: Write the implementation**

```python
# tools/gpsr_ui/telemetry.py
"""Derive a run model from events.jsonl.

Two facts about the corpus shape this module, both measured across all 105
events.jsonl files:

  * tree_revision is 0 in every one of the 289 tree.generated events. A
    replan is therefore an *extra tree.generated epoch*, not a revision
    bump. The vendored classifier emits its REPLAN judge event only when
    tree_revision > 0, so that lane is always empty; replans are derived
    here instead.
  * run.finished.status is "incomplete" in every run, including the one
    that passed. It is never an outcome. The verdict lives in run.json.
"""
from __future__ import annotations

import json
from collections.abc import Iterable
from dataclasses import dataclass, field
from pathlib import Path

from .clock import parse_wall
from .vendor import sheet_events

_TERMINAL = {"SUCCESS", "FAILURE"}


@dataclass(frozen=True)
class TreeNode:
    id: str
    name: str
    type: str
    parent_id: str | None
    children: list[str]
    node_class: str
    reads: list[str]
    writes: list[str]


@dataclass
class Epoch:
    ordinal: int
    wall: float | None
    sequence: int
    root_id: str
    nodes: dict[str, TreeNode]


@dataclass(frozen=True)
class Transition:
    wall: float | None
    tick: int
    node_id: str
    status: str
    feedback: str


@dataclass
class RunModel:
    trajectory_id: str | None = None
    epochs: list[Epoch] = field(default_factory=list)
    transitions: list[Transition] = field(default_factory=list)
    milestones: list = field(default_factory=list)
    judge_events: list = field(default_factory=list)
    tree_regenerations: int = 0
    gate_failures: int = 0
    started_wall: float | None = None
    finished_wall: float | None = None
    announcements: list[str] = field(default_factory=list)

    def epoch_at(self, wall: float | None) -> Epoch | None:
        if wall is None:
            return None
        found = None
        for epoch in self.epochs:
            if epoch.wall is not None and epoch.wall <= wall:
                found = epoch
            else:
                break
        return found

    def status_at(self, wall: float | None) -> dict[str, Transition]:
        """Last transition per node at or before `wall`."""
        out: dict[str, Transition] = {}
        if wall is None:
            return out
        for t in self.transitions:
            if t.wall is None or t.wall > wall:
                continue
            out[t.node_id] = t
        return out


def newest_events_file(run_dir: Path) -> Path | None:
    debug = Path(run_dir) / "debug"
    if not debug.is_dir():
        return None
    try:
        dirs = sorted(p for p in debug.glob("gpsr-*") if p.is_dir())
    except OSError:
        return None
    if not dirs:
        return None
    candidate = dirs[-1] / "events.jsonl"
    return candidate if candidate.is_file() else None


def dedupe_announcements(lines: Iterable[str]) -> list[str]:
    """First-occurrence order. Older runs re-append the whole history each
    tick: one real run is 7967 lines containing 11 distinct utterances."""
    seen: set[str] = set()
    out: list[str] = []
    for raw in lines:
        line = raw.strip()
        if not line or line in seen:
            continue
        seen.add(line)
        out.append(line)
    return out


def _node(raw: dict) -> TreeNode | None:
    node_id = raw.get("id") or raw.get("node_id")
    if not isinstance(node_id, str):
        return None
    access = raw.get("blackboard_access")
    access = access if isinstance(access, dict) else {}
    children = raw.get("children")
    return TreeNode(
        id=node_id,
        name=raw.get("name") or node_id,
        type=raw.get("type") or "",
        parent_id=raw.get("parent_id"),
        children=[c for c in (children or []) if isinstance(c, str)],
        node_class=raw.get("node_class") or "",
        reads=[r for r in (access.get("read") or []) if isinstance(r, str)],
        writes=[w for w in (access.get("write") or []) if isinstance(w, str)],
    )


def _iter_events(path: Path) -> Iterable[dict]:
    """Yield parsed events, skipping blank, corrupt and torn lines. A live
    run's final line is routinely incomplete; that is not an error."""
    try:
        handle = path.open("r")
    except OSError:
        return
    with handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                event = json.loads(line)
            except ValueError:
                continue
            if isinstance(event, dict):
                yield event


def load_run_model(run_dir: Path) -> RunModel:
    run_dir = Path(run_dir)
    model = RunModel()

    events_file = newest_events_file(run_dir)
    if events_file is not None:
        for event in _iter_events(events_file):
            etype = event.get("event_type")
            wall = parse_wall(event.get("occurred_at"))
            payload = event.get("payload")
            payload = payload if isinstance(payload, dict) else {}

            if model.trajectory_id is None:
                tid = event.get("trajectory_id")
                if isinstance(tid, str):
                    model.trajectory_id = tid

            if etype == "run.started":
                model.started_wall = wall
            elif etype == "run.finished":
                # payload["status"] is always "incomplete"; ignore it.
                model.finished_wall = wall
            elif etype == "tree.generated":
                raw_nodes = payload.get("nodes")
                if not isinstance(raw_nodes, list):
                    continue
                nodes: dict[str, TreeNode] = {}
                for raw in raw_nodes:
                    if isinstance(raw, dict):
                        node = _node(raw)
                        if node is not None:
                            nodes[node.id] = node
                if not nodes:
                    continue
                sequence = event.get("sequence")
                model.epochs.append(Epoch(
                    ordinal=len(model.epochs),
                    wall=wall,
                    sequence=sequence if isinstance(sequence, int) else -1,
                    root_id=payload.get("root_id") or "",
                    nodes=nodes,
                ))
            elif etype == "tree.node_states_changed":
                raw_nodes = payload.get("nodes")
                if not isinstance(raw_nodes, list):
                    continue
                tick = payload.get("tick")
                for raw in raw_nodes:
                    if not isinstance(raw, dict):
                        continue
                    status = raw.get("status")
                    node_id = raw.get("id") or raw.get("node_id")
                    if status not in _TERMINAL and status != "RUNNING":
                        continue
                    if not isinstance(node_id, str):
                        continue
                    model.transitions.append(Transition(
                        wall=wall,
                        tick=tick if isinstance(tick, int) else -1,
                        node_id=node_id,
                        status=status,
                        feedback=raw.get("feedback") or "",
                    ))

    # Two epochs is the NORMAL pair: skeleton at startup, then the
    # DynamicExecutor materialising the plan. Only beyond that is the
    # tree genuinely regenerated.
    model.tree_regenerations = max(0, len(model.epochs) - 2)
    model.gate_failures = sum(
        1 for j in judge_events
        if j.status == "FAILURE"
        and j.kind in ("PRECONDITION", "POSTCONDITION")
    )

    milestones, judge_events, _meta = sheet_events.load_run_telemetry(run_dir)
    model.milestones = milestones
    model.judge_events = judge_events

    try:
        raw = (run_dir / "announcements.txt").read_text()
    except (OSError, UnicodeDecodeError):
        raw = ""
    model.announcements = dedupe_announcements(raw.splitlines())

    return model
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest tests/test_telemetry.py -v`
Expected: 10 passed (the `corpus` test passes or skips)

- [ ] **Step 5: Commit**

```bash
git add tools/gpsr_ui/telemetry.py tools/tests/test_telemetry.py
git commit -m "feat: derive run model from events.jsonl with epoch-based regenerations"
```

---

### Task 5: Derived-model cache and the read-only guarantee test

**Files:**
- Create: `tools/gpsr_ui/cache.py`
- Test: `tools/tests/test_cache.py`, `tools/tests/test_read_only.py`

**Interfaces:**
- Consumes: `gpsr_ui.telemetry.load_run_model`, `gpsr_ui.config.Settings`.
- Produces: `cache_key(run_dir: Path) -> str`, `cached_run_model(run_dir: Path, state_dir: Path) -> RunModel`, `snapshot_mtimes(root: Path) -> dict[str, float]`.

The cache stores a marker keyed by (events path, mtime, size). A live run's log grows, so its key changes on every append and the model is recomputed — which is correct, not a bug.

- [ ] **Step 1: Write the failing test**

```python
# tools/tests/test_cache.py
from __future__ import annotations

from gpsr_ui.cache import cache_key, cached_run_model


def test_cache_key_changes_when_the_event_log_grows(make_run):
    run = make_run(name="s9999-030-x")
    first = cache_key(run)
    events = next((run / "debug").glob("gpsr-*")) / "events.jsonl"
    with events.open("a") as fh:
        fh.write("\n")
    assert cache_key(run) != first, "a growing live log must invalidate"


def test_cached_model_matches_a_fresh_one(make_run, tmp_path):
    run = make_run(name="s9999-031-x", epochs=[["a"], ["a", "b"]])
    state = tmp_path / "state"
    once = cached_run_model(run, state)
    twice = cached_run_model(run, state)
    assert once.tree_regenerations == twice.tree_regenerations == 0
    assert len(once.epochs) == len(twice.epochs) == 2


def test_cache_writes_only_into_the_state_dir(make_run, tmp_path):
    run = make_run(name="s9999-032-x")
    state = tmp_path / "state"
    before = {p: p.stat().st_mtime for p in run.rglob("*") if p.is_file()}
    cached_run_model(run, state)
    after = {p: p.stat().st_mtime for p in run.rglob("*") if p.is_file()}
    assert before == after
    assert state.is_dir() and any(state.rglob("*"))
```

```python
# tools/tests/test_read_only.py
from __future__ import annotations

import pytest

from gpsr_ui.cache import cached_run_model, snapshot_mtimes
from gpsr_ui.corpus import list_tiers


@pytest.mark.corpus
def test_indexing_the_real_corpus_mutates_nothing(corpus_root, tmp_path):
    """The read-only guarantee, enforced rather than asserted in prose."""
    before = snapshot_mtimes(corpus_root)

    tiers = list_tiers(corpus_root)
    assert tiers, "corpus should contain at least one tier"
    scanned = 0
    for tier in tiers:
        for entry in tier.entries:
            for attempt in entry.attempts:
                cached_run_model(attempt.path, tmp_path / "state")
                scanned += 1
                if scanned >= 12:
                    break
            if scanned >= 12:
                break
        if scanned >= 12:
            break

    after = snapshot_mtimes(corpus_root)
    # A live battery may ADD files; it must never be us. Assert that no
    # file we saw beforehand was modified or removed.
    for path, mtime in before.items():
        assert path in after, f"{path} disappeared during indexing"
        assert after[path] == mtime, f"{path} was modified during indexing"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest tests/test_cache.py tests/test_read_only.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'gpsr_ui.cache'`

- [ ] **Step 3: Write the implementation**

```python
# tools/gpsr_ui/cache.py
from __future__ import annotations

import hashlib
import pickle
from pathlib import Path

from .telemetry import RunModel, load_run_model, newest_events_file


def cache_key(run_dir: Path) -> str:
    """Identity of a run's telemetry: path, size and mtime of its event log.

    A live run's log grows, changing the key on every append, so the model
    is recomputed while it is in flight. That is the intended behaviour.
    """
    run_dir = Path(run_dir)
    events = newest_events_file(run_dir)
    parts = [str(run_dir.resolve())]
    if events is not None:
        try:
            st = events.stat()
            parts += [str(events), str(st.st_size), repr(st.st_mtime)]
        except OSError:
            parts.append("stat-failed")
    else:
        parts.append("no-events")
    return hashlib.sha256("\0".join(parts).encode()).hexdigest()[:32]


def cached_run_model(run_dir: Path, state_dir: Path) -> RunModel:
    key = cache_key(run_dir)
    target = Path(state_dir) / "models" / f"{key}.pickle"
    try:
        with target.open("rb") as fh:
            model = pickle.load(fh)
        if isinstance(model, RunModel):
            return model
    except (OSError, pickle.PickleError, AttributeError, EOFError):
        pass

    model = load_run_model(run_dir)
    try:
        target.parent.mkdir(parents=True, exist_ok=True)
        tmp = target.with_suffix(".tmp")
        with tmp.open("wb") as fh:
            pickle.dump(model, fh)
        tmp.replace(target)
    except (OSError, pickle.PickleError):
        pass  # a cache miss is never fatal
    return model


def snapshot_mtimes(root: Path) -> dict[Path, float]:
    """Every file under `root` with its mtime, for read-only assertions."""
    out: dict[Path, float] = {}
    for path in Path(root).rglob("*"):
        try:
            if path.is_file():
                out[path] = path.stat().st_mtime
        except OSError:
            continue
    return out
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest tests/test_cache.py tests/test_read_only.py -v`
Expected: 3 passed, 1 passed-or-skipped

- [ ] **Step 5: Commit**

```bash
git add tools/gpsr_ui/cache.py tools/tests/test_cache.py tools/tests/test_read_only.py
git commit -m "feat: derived-model cache with an enforced read-only guarantee"
```

---

### Task 6: FastAPI app, JSON API, and the corpus browser page

**Files:**
- Create: `tools/gpsr_ui/app.py`
- Create: `tools/gpsr_ui/templates/base.html`, `tools/gpsr_ui/templates/index.html`
- Create: `tools/gpsr_ui/static/app.css`
- Test: `tools/tests/test_app.py`

**Interfaces:**
- Consumes: everything above.
- Produces: `app` (FastAPI instance), `create_app(settings) -> FastAPI`. Routes:
  - `GET /` corpus browser HTML
  - `GET /api/tiers` → `{"tiers": [{"name", "entries": [{"entry_id", "template", "feasibility", "text", "attempts": [...]}]}]}`
  - `GET /api/run/{tier}/{dir_name}` → run model JSON
  - `GET /healthz` → `{"ok": true}`

- [ ] **Step 1: Write the failing test**

```python
# tools/tests/test_app.py
from __future__ import annotations

from fastapi.testclient import TestClient

from gpsr_ui.app import create_app
from gpsr_ui.config import Settings


def _client(bench_root, tmp_path):
    settings = Settings(
        bench_root=bench_root,
        state_dir=tmp_path / "state",
        sheet_events_path=None,
    )
    return TestClient(create_app(settings))


def test_healthz(make_run, tmp_path):
    run = make_run(name="s9999-040-x")
    client = _client(run.parents[2], tmp_path)
    assert client.get("/healthz").json() == {"ok": True}


def test_tiers_api_groups_attempts(make_run, tmp_path):
    run = make_run(name="s9999-041-x", verdict="PASS")
    make_run(name="s9999-041-x.attempt2-foo", verdict="FAIL")
    client = _client(run.parents[2], tmp_path)

    body = client.get("/api/tiers").json()
    entries = body["tiers"][0]["entries"]
    assert len(entries) == 1
    assert len(entries[0]["attempts"]) == 2
    assert entries[0]["attempts"][0]["is_current"] is True


def test_run_api_returns_epochs_and_regeneration_count(make_run, tmp_path):
    run = make_run(name="s9999-042-x", epochs=[["a"], ["a", "b"]])
    client = _client(run.parents[2], tmp_path)

    body = client.get("/api/run/t9-test/s9999-042-x").json()
    assert body["tree_regenerations"] == 0
    assert len(body["epochs"]) == 2
    assert body["verdict"] == "PASS"
    assert body["clock_mode"] == "none"


def test_run_api_404s_for_an_unknown_run(make_run, tmp_path):
    run = make_run(name="s9999-043-x")
    client = _client(run.parents[2], tmp_path)
    assert client.get("/api/run/t9-test/nope").status_code == 404


def test_run_api_rejects_path_traversal(make_run, tmp_path):
    run = make_run(name="s9999-044-x")
    client = _client(run.parents[2], tmp_path)
    assert client.get("/api/run/t9-test/..%2F..%2Fetc").status_code == 404


def test_index_page_renders_the_entry(make_run, tmp_path):
    run = make_run(name="s9999-045-x", verdict="PASS")
    client = _client(run.parents[2], tmp_path)
    page = client.get("/")
    assert page.status_code == 200
    assert "s9999-045-x" in page.text
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest tests/test_app.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'gpsr_ui.app'`

- [ ] **Step 3: Write the implementation**

```python
# tools/gpsr_ui/app.py
from __future__ import annotations

from dataclasses import asdict
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

from .cache import cached_run_model
from .clock import load_clock
from .config import Settings, load_settings
from .corpus import find_run, list_tiers

_HERE = Path(__file__).parent


def _attempt_json(attempt) -> dict:
    data = asdict(attempt)
    data["path"] = str(attempt.path)
    return data


def _run_json(run_dir: Path, model, clock, attempt) -> dict:
    return {
        "dir_name": run_dir.name,
        "trajectory_id": model.trajectory_id,
        "verdict": attempt.verdict if attempt else None,
        "seconds": attempt.seconds if attempt else None,
        "detail": attempt.detail if attempt else "",
        "started_wall": model.started_wall,
        "finished_wall": model.finished_wall,
        "tree_regenerations": model.tree_regenerations,
        "gate_failures": model.gate_failures,
        "clock_mode": clock.mode,
        "clock_labels": clock.labels,
        "announcements": model.announcements,
        "epochs": [
            {
                "ordinal": e.ordinal,
                "wall": e.wall,
                "sequence": e.sequence,
                "root_id": e.root_id,
                "nodes": [asdict(n) for n in e.nodes.values()],
            }
            for e in model.epochs
        ],
        "transitions": [asdict(t) for t in model.transitions],
        "milestones": [asdict(m) for m in model.milestones],
        "judge_events": [asdict(j) for j in model.judge_events],
    }


def create_app(settings: Settings | None = None) -> FastAPI:
    settings = settings or load_settings()
    app = FastAPI(title="GPSR Bench Run Viewer")
    app.state.settings = settings

    templates = Jinja2Templates(directory=str(_HERE / "templates"))
    app.mount(
        "/static", StaticFiles(directory=str(_HERE / "static")), name="static")

    @app.get("/healthz")
    def healthz() -> dict:
        return {"ok": True}

    @app.get("/api/tiers")
    def api_tiers() -> JSONResponse:
        tiers = list_tiers(settings.bench_root)
        return JSONResponse({
            "tiers": [
                {
                    "name": t.name,
                    "entries": [
                        {
                            "entry_id": e.entry_id,
                            "template": e.template,
                            "feasibility": e.feasibility,
                            "text": e.text,
                            "attempts": [_attempt_json(a) for a in e.attempts],
                        }
                        for e in t.entries
                    ],
                }
                for t in tiers
            ]
        })

    def _resolve(tier: str, dir_name: str):
        run_dir = find_run(settings.bench_root, tier, dir_name)
        if run_dir is None:
            raise HTTPException(status_code=404, detail="run not found")
        attempt = None
        for candidate in list_tiers(settings.bench_root):
            if candidate.name != tier:
                continue
            for entry in candidate.entries:
                for a in entry.attempts:
                    if a.dir_name == dir_name:
                        attempt = a
        return run_dir, attempt

    @app.get("/api/run/{tier}/{dir_name}")
    def api_run(tier: str, dir_name: str) -> JSONResponse:
        run_dir, attempt = _resolve(tier, dir_name)
        model = cached_run_model(run_dir, settings.state_dir)
        clock = load_clock(run_dir)
        return JSONResponse(_run_json(run_dir, model, clock, attempt))

    @app.get("/")
    def index(request: Request):
        return templates.TemplateResponse(
            "index.html",
            {"request": request, "tiers": list_tiers(settings.bench_root)},
        )

    return app


app = create_app()
```

```html
<!-- tools/gpsr_ui/templates/base.html -->
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{% block title %}GPSR Bench{% endblock %}</title>
  <link rel="stylesheet" href="/static/app.css">
</head>
<body>
  <header class="topbar"><a href="/">GPSR Bench Run Viewer</a></header>
  <main>{% block content %}{% endblock %}</main>
</body>
</html>
```

```html
<!-- tools/gpsr_ui/templates/index.html -->
{% extends "base.html" %}
{% block content %}
{% for tier in tiers %}
  <section class="tier">
    <h2>{{ tier.name }}</h2>
    <table class="entries">
      <thead>
        <tr><th>entry</th><th>template</th><th>class</th>
            <th>verdict</th><th>seconds</th><th>attempts</th></tr>
      </thead>
      <tbody>
      {% for entry in tier.entries %}
        {% set current = entry.attempts[0] %}
        <tr class="entry-row">
          <td><a href="/run/{{ tier.name }}/{{ current.dir_name }}">
              {{ entry.entry_id }}</a></td>
          <td>{{ entry.template or "" }}</td>
          <td>{{ entry.feasibility or "" }}</td>
          <td class="verdict v-{{ (current.verdict or 'none')|lower }}">
              {{ current.verdict or "in flight" }}</td>
          <td>{{ "%.0f"|format(current.seconds) if current.seconds else "" }}</td>
          <td>
            {% if entry.attempts|length > 1 %}
              <details>
                <summary>{{ entry.attempts|length - 1 }} archived</summary>
                <ul>
                {% for a in entry.attempts[1:] %}
                  <li><a href="/run/{{ tier.name }}/{{ a.dir_name }}">
                      {{ a.slug }}</a>
                      <span class="verdict v-{{ (a.verdict or 'none')|lower }}">
                      {{ a.verdict or "?" }}</span></li>
                {% endfor %}
                </ul>
              </details>
            {% endif %}
          </td>
        </tr>
      {% endfor %}
      </tbody>
    </table>
  </section>
{% endfor %}
{% endblock %}
```

```css
/* tools/gpsr_ui/static/app.css */
:root {
  --bg: #14161a; --fg: #e7e9ee; --dim: #8b93a3; --line: #2a2f38;
  --pass: #4ec9a0; --fail: #e06c75; --timeout: #d9a441; --none: #8b93a3;
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--fg);
  font: 14px/1.5 ui-monospace, SFMono-Regular, Menlo, monospace; }
.topbar { padding: 10px 16px; border-bottom: 1px solid var(--line); }
.topbar a { color: var(--fg); text-decoration: none; }
main { padding: 16px; }
.tier h2 { font-size: 15px; color: var(--dim); margin: 20px 0 8px; }
table.entries { width: 100%; border-collapse: collapse; }
table.entries th { text-align: left; color: var(--dim); font-weight: 400;
  border-bottom: 1px solid var(--line); padding: 4px 8px; }
table.entries td { padding: 4px 8px; border-bottom: 1px solid var(--line);
  vertical-align: top; }
a { color: #6cb6ff; }
.verdict { font-weight: 600; }
.v-pass { color: var(--pass); } .v-fail { color: var(--fail); }
.v-timeout, .v-error { color: var(--timeout); } .v-none { color: var(--none); }
ul { margin: 4px 0; padding-left: 18px; }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest tests/test_app.py -v`
Expected: 6 passed

- [ ] **Step 5: Smoke-test against the real corpus**

Run: `cd tools && ./gpsr-ui &` then `curl -s localhost:8770/healthz` and `curl -s localhost:8770/ | head -40`
Expected: `{"ok":true}` and HTML listing real tier names. Stop the server afterwards.

- [ ] **Step 6: Commit**

```bash
git add tools/gpsr_ui/app.py tools/gpsr_ui/templates tools/gpsr_ui/static tools/tests/test_app.py
git commit -m "feat: FastAPI app with corpus browser and run API"
```

---

### Task 7: Frames API

**Files:**
- Create: `tools/gpsr_ui/frames.py`
- Modify: `tools/gpsr_ui/app.py` (add three routes)
- Test: `tools/tests/test_frames.py`

**Interfaces:**
- Consumes: `gpsr_ui.clock`.
- Produces: `list_frames(run_dir) -> dict[str, list[FrameRef]]`; `FrameRef` dataclass `index: int`, `stamp_s: float`, `file: str`, `wall: float | None`; `frame_path(run_dir, label, file) -> Path | None`. Routes `GET /api/run/{tier}/{dir_name}/frames`, `GET /frame/{tier}/{dir_name}/{label}/{file}`.

- [ ] **Step 1: Write the failing test**

```python
# tools/tests/test_frames.py
from __future__ import annotations

from gpsr_ui.frames import frame_path, list_frames


def test_frames_are_listed_per_label_in_stamp_order(make_run):
    run = make_run(
        name="s9999-050-x",
        frames={"head": [(1, 2000), (0, 1000)], "arena": [(0, 1000)]},
    )
    frames = list_frames(run)
    assert sorted(frames) == ["arena", "head"]
    assert [f.stamp_s for f in frames["head"]] == [1.0, 2.0]
    assert frames["head"][0].file == "0000_1000.jpg"


def test_frames_carry_wall_times_when_the_clock_is_exact(make_run):
    run = make_run(
        name="s9999-051-x",
        frames={"head": [(0, 1000)]},
        index_lines=[{"label": "head", "file": "0000_1000.jpg",
                      "stamp_s": 1.0, "wall": "2026-08-28T10:00:00.000000Z"}],
    )
    assert list_frames(run)["head"][0].wall is not None


def test_a_run_with_only_one_camera_is_fine(make_run):
    """s2026-003-findObjInRoom in the real corpus has head but no arena."""
    run = make_run(name="s9999-052-x", frames={"head": [(0, 1000)]})
    assert list(list_frames(run)) == ["head"]


def test_a_run_with_no_frames_yields_an_empty_mapping(make_run):
    assert list_frames(make_run(name="s9999-053-x")) == {}


def test_frame_path_refuses_traversal(make_run):
    run = make_run(name="s9999-054-x", frames={"head": [(0, 1000)]})
    assert frame_path(run, "head", "0000_1000.jpg") is not None
    assert frame_path(run, "head", "../../run.json") is None
    assert frame_path(run, "../head", "0000_1000.jpg") is None
    assert frame_path(run, "head", "nope.jpg") is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest tests/test_frames.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'gpsr_ui.frames'`

- [ ] **Step 3: Write the implementation**

```python
# tools/gpsr_ui/frames.py
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .clock import load_clock, parse_frame_name


@dataclass(frozen=True)
class FrameRef:
    index: int
    stamp_s: float
    file: str
    wall: float | None


def list_frames(run_dir: Path) -> dict[str, list[FrameRef]]:
    run_dir = Path(run_dir)
    frames_dir = run_dir / "frames"
    if not frames_dir.is_dir():
        return {}
    clock = load_clock(run_dir)

    out: dict[str, list[FrameRef]] = {}
    for label_dir in sorted(p for p in frames_dir.iterdir() if p.is_dir()):
        refs: list[FrameRef] = []
        for f in label_dir.iterdir():
            parsed = parse_frame_name(f.name)
            if parsed is None:
                continue
            index, stamp_s = parsed
            refs.append(FrameRef(
                index=index,
                stamp_s=stamp_s,
                file=f.name,
                wall=clock.sim_to_wall(label_dir.name, stamp_s),
            ))
        if refs:
            out[label_dir.name] = sorted(refs, key=lambda r: r.stamp_s)
    return out


def frame_path(run_dir: Path, label: str, file: str) -> Path | None:
    """Resolve a frame, refusing anything that escapes the frames dir."""
    if "/" in label or "/" in file or ".." in label or ".." in file:
        return None
    if parse_frame_name(file) is None:
        return None
    candidate = Path(run_dir) / "frames" / label / file
    try:
        resolved = candidate.resolve()
        root = (Path(run_dir) / "frames").resolve()
    except OSError:
        return None
    if root not in resolved.parents:
        return None
    return resolved if resolved.is_file() else None
```

Add to `create_app` in `tools/gpsr_ui/app.py`, after `api_run`:

```python
    @app.get("/api/run/{tier}/{dir_name}/frames")
    def api_frames(tier: str, dir_name: str) -> JSONResponse:
        run_dir, _ = _resolve(tier, dir_name)
        frames = list_frames(run_dir)
        return JSONResponse({
            "labels": {
                label: [asdict(r) for r in refs]
                for label, refs in frames.items()
            }
        })

    @app.get("/frame/{tier}/{dir_name}/{label}/{file}")
    def frame(tier: str, dir_name: str, label: str, file: str):
        run_dir, _ = _resolve(tier, dir_name)
        path = frame_path(run_dir, label, file)
        if path is None:
            raise HTTPException(status_code=404, detail="frame not found")
        return FileResponse(
            path,
            media_type="image/jpeg",
            # Frames are immutable once written; cache them hard.
            headers={"Cache-Control": "public, max-age=31536000, immutable"},
        )
```

And extend the imports at the top of `app.py`:

```python
from fastapi.responses import FileResponse, JSONResponse
from .frames import frame_path, list_frames
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest tests/test_frames.py tests/test_app.py -v`
Expected: 11 passed

- [ ] **Step 5: Commit**

```bash
git add tools/gpsr_ui/frames.py tools/gpsr_ui/app.py tools/tests/test_frames.py
git commit -m "feat: frame listing and serving with traversal guards"
```

---

### Task 8: Frontend core — playhead, timeline ribbon, run page

**Files:**
- Create: `tools/gpsr_ui/static/playhead.js`, `tools/gpsr_ui/static/timeline.js`, `tools/gpsr_ui/static/run.js`
- Create: `tools/gpsr_ui/templates/run.html`
- Modify: `tools/gpsr_ui/app.py` (add `GET /run/{tier}/{dir_name}`)
- Modify: `tools/gpsr_ui/static/app.css`
- Test: `tools/tests/js/test_playhead.mjs`, `tools/tests/js/test_timeline.mjs`

**Interfaces:**
- Consumes: `/api/run/...` and `/api/run/.../frames`.
- Produces: `playhead.js` exports `createPlayhead({start, end})` with `get()`, `set(wall)`, `subscribe(fn)`, `clamp(wall)`. `timeline.js` exports `buildLanes(model)` returning `[{id, label, items: [{wall, kind, status, name, info}]}]` and `xOf(wall, start, end, width)`.

- [ ] **Step 1: Write the failing JS tests**

```javascript
// tools/tests/js/test_playhead.mjs
import test from "node:test";
import assert from "node:assert/strict";
import { createPlayhead } from "../../gpsr_ui/static/playhead.js";

test("clamps to the run bounds", () => {
  const p = createPlayhead({ start: 100, end: 200 });
  assert.equal(p.clamp(50), 100);
  assert.equal(p.clamp(250), 200);
  assert.equal(p.clamp(150), 150);
});

test("notifies subscribers on change and not on a repeat", () => {
  const p = createPlayhead({ start: 0, end: 10 });
  const seen = [];
  p.subscribe((v) => seen.push(v));
  p.set(5);
  p.set(5);
  p.set(7);
  assert.deepEqual(seen, [5, 7]);
});

test("set clamps out-of-range values", () => {
  const p = createPlayhead({ start: 0, end: 10 });
  p.set(99);
  assert.equal(p.get(), 10);
});
```

```javascript
// tools/tests/js/test_timeline.mjs
import test from "node:test";
import assert from "node:assert/strict";
import { buildLanes, xOf } from "../../gpsr_ui/static/timeline.js";

test("xOf maps wall time onto pixel width", () => {
  assert.equal(xOf(100, 100, 200, 500), 0);
  assert.equal(xOf(200, 100, 200, 500), 500);
  assert.equal(xOf(150, 100, 200, 500), 250);
});

test("xOf is stable when the run has zero duration", () => {
  assert.equal(xOf(100, 100, 100, 500), 0);
});

test("buildLanes has no separate replan lane", () => {
  // The vendored classifier emits REPLAN as a judge event once a run has
  // 3+ tree generations. A second lane built from epochs would double-count,
  // and would also mislabel the normal 2-epoch pair as a replan.
  const model = {
    started_wall: 0,
    finished_wall: 100,
    epochs: [{ ordinal: 0, wall: 0 }, { ordinal: 1, wall: 40 },
             { ordinal: 2, wall: 70 }],
    milestones: [], judge_events: [],
  };
  const lanes = buildLanes(model);
  assert.equal(lanes.find((l) => l.id === "replan"), undefined);
});

test("buildLanes surfaces classifier REPLAN events in the judge lane", () => {
  const model = {
    started_wall: 0, finished_wall: 100, epochs: [], milestones: [],
    judge_events: [
      { wall: "1970-01-01T00:00:40Z", kind: "REPLAN", status: "SUCCESS",
        name: "replan", info: "tree regenerated #1 (230 nodes)" },
    ],
  };
  const judge = buildLanes(model).find((l) => l.id === "judge");
  assert.equal(judge.items.length, 1);
  assert.equal(judge.items[0].kind, "REPLAN");
});

test("buildLanes separates milestones by kind and marks failures", () => {
  const model = {
    started_wall: 0, finished_wall: 10, epochs: [],
    milestones: [
      { wall: "1970-01-01T00:00:01Z", kind: "NAV", status: "FAILURE",
        name: "goto target", info: "blocked" },
      { wall: "1970-01-01T00:00:02Z", kind: "AUDIO", status: "SUCCESS",
        name: "announce", info: "hi" },
    ],
    judge_events: [],
  };
  const lanes = buildLanes(model);
  const nav = lanes.find((l) => l.id === "NAV");
  assert.equal(nav.items.length, 1);
  assert.equal(nav.items[0].status, "FAILURE");
});
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd tools && node --test tests/js/`
Expected: FAIL with `Cannot find module .../static/playhead.js`

- [ ] **Step 3: Write the implementation**

```javascript
// tools/gpsr_ui/static/playhead.js
// The single source of time for every panel. Everything — tree state,
// frame selection, event highlighting — derives from this one value.
export function createPlayhead({ start, end }) {
  let value = start;
  const subscribers = [];

  const clamp = (v) => {
    if (!Number.isFinite(v)) return start;
    if (v < start) return start;
    if (v > end) return end;
    return v;
  };

  return {
    clamp,
    get: () => value,
    set(next) {
      const clamped = clamp(next);
      if (clamped === value) return;
      value = clamped;
      for (const fn of subscribers) fn(value);
    },
    subscribe(fn) {
      subscribers.push(fn);
      return () => {
        const i = subscribers.indexOf(fn);
        if (i >= 0) subscribers.splice(i, 1);
      };
    },
  };
}
```

```javascript
// tools/gpsr_ui/static/timeline.js
const MILESTONE_KINDS = ["NAV", "VISION", "AUDIO", "MANIP"];

export function parseWall(value) {
  if (typeof value === "number") return value;
  if (typeof value !== "string") return null;
  const ms = Date.parse(value);
  return Number.isNaN(ms) ? null : ms / 1000;
}

export function xOf(wall, start, end, width) {
  if (end <= start) return 0;
  return ((wall - start) / (end - start)) * width;
}

// tree_revision is 0 everywhere in the corpus, so it is never consulted.
// Replan events arrive already classified in model.judge_events.
export function buildLanes(model) {
  const lanes = [];

  for (const kind of MILESTONE_KINDS) {
    lanes.push({
      id: kind,
      label: kind,
      items: (model.milestones || [])
        .filter((m) => m.kind === kind)
        .map((m) => ({
          wall: parseWall(m.wall), kind, status: m.status,
          name: m.name, info: m.info,
        }))
        .filter((m) => m.wall !== null),
    });
  }

  lanes.push({
    id: "judge",
    label: "judge",
    items: (model.judge_events || [])
      .map((j) => ({
        wall: parseWall(j.wall), kind: j.kind, status: j.status,
        name: j.name, info: j.info,
      }))
      .filter((j) => j.wall !== null),
  });

  // No replan lane: the vendored classifier already emits REPLAN as a judge
  // event when a run has 3+ tree generations. Two generations is the normal
  // skeleton-then-materialise pair, not a replan.
  return lanes;
}
```

```javascript
// tools/gpsr_ui/static/run.js
import { createPlayhead } from "./playhead.js";
import { buildLanes, parseWall, xOf } from "./timeline.js";

const SVG_NS = "http://www.w3.org/2000/svg";
const LANE_HEIGHT = 18;
const STATUS_CLASS = { SUCCESS: "ok", FAILURE: "bad", RUNNING: "run" };

function el(tag, attrs) {
  const node = document.createElementNS(SVG_NS, tag);
  for (const [k, v] of Object.entries(attrs)) node.setAttribute(k, v);
  return node;
}

function runBounds(model) {
  const walls = [];
  if (model.started_wall) walls.push(model.started_wall);
  if (model.finished_wall) walls.push(model.finished_wall);
  for (const m of model.milestones || []) {
    const w = parseWall(m.wall);
    if (w !== null) walls.push(w);
  }
  if (walls.length === 0) return { start: 0, end: 1 };
  return { start: Math.min(...walls), end: Math.max(...walls) };
}

function renderRibbon(svg, lanes, bounds, playhead) {
  const width = svg.clientWidth || 900;
  svg.setAttribute("height", lanes.length * LANE_HEIGHT + 24);
  svg.replaceChildren();

  lanes.forEach((lane, row) => {
    const y = row * LANE_HEIGHT + 12;
    svg.appendChild(el("line", {
      x1: 0, x2: width, y1: y, y2: y, class: "lane-rule",
    }));
    const label = el("text", { x: 2, y: y - 4, class: "lane-label" });
    label.textContent = lane.label;
    svg.appendChild(label);

    for (const item of lane.items) {
      const mark = el("circle", {
        cx: xOf(item.wall, bounds.start, bounds.end, width),
        cy: y, r: 4,
        class: `mark ${STATUS_CLASS[item.status] || "run"}`,
      });
      const title = el("title", {});
      title.textContent = `${item.name}\n${item.info || ""}`;
      mark.appendChild(title);
      mark.addEventListener("click", () => playhead.set(item.wall));
      svg.appendChild(mark);
    }
  });

  const cursor = el("line", {
    y1: 0, y2: lanes.length * LANE_HEIGHT + 12, class: "cursor",
    x1: 0, x2: 0,
  });
  svg.appendChild(cursor);

  const move = (wall) => {
    const x = xOf(wall, bounds.start, bounds.end, width);
    cursor.setAttribute("x1", x);
    cursor.setAttribute("x2", x);
  };
  move(playhead.get());
  playhead.subscribe(move);

  svg.addEventListener("click", (event) => {
    if (event.target !== svg) return;
    const rect = svg.getBoundingClientRect();
    const frac = (event.clientX - rect.left) / rect.width;
    playhead.set(bounds.start + frac * (bounds.end - bounds.start));
  });
}

export async function boot({ tier, dirName }) {
  const base = `/api/run/${encodeURIComponent(tier)}/${encodeURIComponent(dirName)}`;
  const model = await (await fetch(base)).json();

  const badge = document.getElementById("clock-badge");
  badge.textContent = `clock: ${model.clock_mode}`;
  badge.className = `badge clock-${model.clock_mode}`;
  badge.title = model.clock_mode === "approximate"
    ? "Frame times interpolated from recorder-meta.json. Accurate to seconds "
      + "only; RTF varies 0.2-0.5 within a run."
    : model.clock_mode === "exact"
      ? "Frames joined to wall time via frames/index.jsonl."
      : "No frame/wall mapping available for this run.";

  document.getElementById("regen-count").textContent = model.tree_regenerations;
  document.getElementById("gate-failures").textContent = model.gate_failures;
  document.getElementById("verdict").textContent = model.verdict || "in flight";
  document.getElementById("verdict").className =
    `verdict v-${(model.verdict || "none").toLowerCase()}`;

  const bounds = runBounds(model);
  const playhead = createPlayhead(bounds);
  renderRibbon(
    document.getElementById("ribbon"), buildLanes(model), bounds, playhead);

  window.__gpsr = { model, playhead, bounds, base };
  return window.__gpsr;
}
```

```html
<!-- tools/gpsr_ui/templates/run.html -->
{% extends "base.html" %}
{% block title %}{{ dir_name }}{% endblock %}
{% block content %}
<h2 class="run-title">{{ tier }} / {{ dir_name }}</h2>
<div class="run-meta">
  <span id="verdict">…</span>
  <span id="clock-badge" class="badge">clock: …</span>
  <span>tree regens: <b id="regen-count">…</b></span>
  <span>gate failures: <b id="gate-failures">…</b></span>
</div>
<svg id="ribbon" class="ribbon" width="100%"></svg>
<div id="panels"></div>
<script type="module">
  import { boot } from "/static/run.js";
  boot({ tier: {{ tier|tojson }}, dirName: {{ dir_name|tojson }} });
</script>
{% endblock %}
```

Add to `create_app` in `app.py`:

```python
    @app.get("/run/{tier}/{dir_name}")
    def run_page(request: Request, tier: str, dir_name: str):
        _resolve(tier, dir_name)  # 404s before rendering
        return templates.TemplateResponse(
            "run.html",
            {"request": request, "tier": tier, "dir_name": dir_name},
        )
```

Append to `app.css`:

```css
.run-title { font-size: 15px; margin: 0 0 6px; }
.run-meta { display: flex; gap: 14px; align-items: center;
  margin-bottom: 12px; color: var(--dim); }
.badge { padding: 1px 7px; border: 1px solid var(--line); border-radius: 3px; }
.clock-exact { color: var(--pass); border-color: var(--pass); }
.clock-approximate { color: var(--timeout); border-color: var(--timeout); }
.clock-none { color: var(--none); }
.ribbon { width: 100%; background: #0f1114; border: 1px solid var(--line); }
.lane-rule { stroke: var(--line); stroke-width: 1; }
.lane-label { fill: var(--dim); font-size: 9px; }
.mark { cursor: pointer; }
.mark.ok { fill: var(--pass); } .mark.bad { fill: var(--fail); }
.mark.run { fill: var(--dim); }
.cursor { stroke: #6cb6ff; stroke-width: 1; }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd tools && node --test tests/js/`
Expected: 6 passing
Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest -v`
Expected: all passing

- [ ] **Step 5: Verify the page renders against a real run**

Run: `cd tools && ./gpsr-ui &`, then open `http://localhost:8770/run/t2-2026/s2026-002-countPrsInRoom`.
Expected: verdict `PASS`, clock badge reads `approximate` in amber, tree regens `0` (two epochs is the normal pair), and the ribbon shows NAV/VISION/AUDIO/MANIP/judge lanes with clickable marks. Stop the server afterwards.

- [ ] **Step 6: Commit**

```bash
git add tools/gpsr_ui/static tools/gpsr_ui/templates/run.html tools/gpsr_ui/app.py tools/tests/js
git commit -m "feat: run page with shared playhead and timeline ribbon"
```

---

### Task 9: Tree panel with time travel

**Files:**
- Create: `tools/gpsr_ui/static/tree.js`
- Modify: `tools/gpsr_ui/static/run.js` (mount the tree panel)
- Modify: `tools/gpsr_ui/static/app.css`
- Test: `tools/tests/js/test_tree.mjs`

**Interfaces:**
- Consumes: run model JSON.
- Produces: `tree.js` exports `layoutTree(nodes, rootId)` → `{positions: Map<id, {x, y, depth}>, width, height}`, `statusAt(transitions, wall)` → `Map<id, {status, feedback}>`, and `isBookkeeping(node)`.

- [ ] **Step 1: Write the failing test**

```javascript
// tools/tests/js/test_tree.mjs
import test from "node:test";
import assert from "node:assert/strict";
import { isBookkeeping, layoutTree, statusAt } from "../../gpsr_ui/static/tree.js";

const NODES = [
  { id: "r", name: "root", type: "Sequence", parent_id: null,
    children: ["a", "b"], node_class: "composite" },
  { id: "a", name: "leaf a", type: "BtNode_Announce", parent_id: "r",
    children: [], node_class: "leaf" },
  { id: "b", name: "leaf b", type: "BtNode_GotoAction", parent_id: "r",
    children: [], node_class: "leaf" },
];

test("layout assigns depth by ancestry and separates siblings", () => {
  const { positions, height } = layoutTree(NODES, "r");
  assert.equal(positions.get("r").depth, 0);
  assert.equal(positions.get("a").depth, 1);
  assert.equal(positions.get("b").depth, 1);
  assert.notEqual(positions.get("a").y, positions.get("b").y);
  assert.ok(height > 0);
});

test("layout tolerates a missing root without throwing", () => {
  const { positions } = layoutTree(NODES, "nope");
  assert.equal(positions.size, 0);
});

test("layout tolerates a cycle without hanging", () => {
  const cyclic = [
    { id: "x", parent_id: null, children: ["y"], name: "x", node_class: "c" },
    { id: "y", parent_id: "x", children: ["x"], name: "y", node_class: "c" },
  ];
  const { positions } = layoutTree(cyclic, "x");
  assert.equal(positions.size, 2, "each node placed exactly once");
});

test("statusAt returns the last transition at or before the playhead", () => {
  const transitions = [
    { wall: 10, node_id: "a", status: "RUNNING", feedback: "go" },
    { wall: 20, node_id: "a", status: "SUCCESS", feedback: "done" },
    { wall: 30, node_id: "b", status: "FAILURE", feedback: "blocked" },
  ];
  assert.equal(statusAt(transitions, 15).get("a").status, "RUNNING");
  assert.equal(statusAt(transitions, 25).get("a").status, "SUCCESS");
  assert.equal(statusAt(transitions, 25).has("b"), false);
  assert.equal(statusAt(transitions, 5).size, 0);
});

test("keepalive and bookkeeping nodes are flagged for collapsing", () => {
  assert.equal(isBookkeeping({ name: "nav keepalive 0" }), true);
  assert.equal(isBookkeeping({ name: "keepalive gap 2" }), true);
  assert.equal(isBookkeeping({ name: "clear plan_index" }), true);
  assert.equal(isBookkeeping({ name: "goto target" }), false);
});
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd tools && node --test tests/js/test_tree.mjs`
Expected: FAIL with `Cannot find module .../static/tree.js`

- [ ] **Step 3: Write the implementation**

```javascript
// tools/gpsr_ui/static/tree.js
const ROW = 16;
const COL = 168;
const BOOKKEEPING = [/^nav keepalive/i, /^keepalive gap/i,
  /^say keepalive/i, /^clear /i, /^set /i, /^latch /i];

export function isBookkeeping(node) {
  const name = (node && node.name) || "";
  return BOOKKEEPING.some((re) => re.test(name));
}

// Depth-first layout: depth sets the column, a running counter sets the row.
// Guards against cycles because the corpus is machine-generated and a bad
// edge must not hang the page.
export function layoutTree(nodes, rootId) {
  const byId = new Map(nodes.map((n) => [n.id, n]));
  const positions = new Map();
  if (!byId.has(rootId)) return { positions, width: 0, height: 0 };

  const seen = new Set();
  let row = 0;
  let maxDepth = 0;

  const walk = (id, depth) => {
    if (seen.has(id)) return;
    seen.add(id);
    const node = byId.get(id);
    if (!node) return;
    positions.set(id, { x: depth * COL, y: row * ROW, depth });
    row += 1;
    maxDepth = Math.max(maxDepth, depth);
    for (const child of node.children || []) walk(child, depth + 1);
  };
  walk(rootId, 0);

  return { positions, width: (maxDepth + 1) * COL, height: row * ROW };
}

export function statusAt(transitions, wall) {
  const out = new Map();
  for (const t of transitions) {
    if (t.wall === null || t.wall === undefined || t.wall > wall) continue;
    out.set(t.node_id, { status: t.status, feedback: t.feedback });
  }
  return out;
}
```

Append to `run.js`, and call `mountTree` at the end of `boot`:

```javascript
import { isBookkeeping, layoutTree, statusAt } from "./tree.js";

function mountTree(container, model, playhead) {
  const panel = document.createElement("div");
  panel.className = "tree-panel";
  const svg = document.createElementNS(SVG_NS, "svg");
  svg.setAttribute("class", "tree");
  const detail = document.createElement("pre");
  detail.className = "node-detail";
  detail.textContent = "click a node";
  panel.append(svg, detail);
  container.appendChild(panel);

  let currentEpoch = null;

  const draw = (wall) => {
    // The tree shown is the latest epoch at or before the playhead.
    let epoch = null;
    for (const e of model.epochs) {
      if (e.wall !== null && e.wall <= wall) epoch = e;
    }
    if (epoch === null) epoch = model.epochs[0];
    if (!epoch) return;

    if (epoch !== currentEpoch) {
      currentEpoch = epoch;
      svg.replaceChildren();
      const { positions, width, height } = layoutTree(epoch.nodes, epoch.root_id);
      svg.setAttribute("width", width + 200);
      svg.setAttribute("height", height + 12);
      epoch._positions = positions;

      const byId = new Map(epoch.nodes.map((n) => [n.id, n]));
      for (const [id, pos] of positions) {
        const node = byId.get(id);
        const g = document.createElementNS(SVG_NS, "g");
        g.setAttribute("class",
          `node ${isBookkeeping(node) ? "bookkeeping" : ""}`);
        g.setAttribute("transform", `translate(${pos.x},${pos.y + 10})`);
        g.dataset.nodeId = id;

        const dot = document.createElementNS(SVG_NS, "circle");
        dot.setAttribute("r", 3.5);
        dot.setAttribute("class", "node-dot");
        const text = document.createElementNS(SVG_NS, "text");
        text.setAttribute("x", 8);
        text.setAttribute("y", 3);
        text.textContent = node.name;
        g.append(dot, text);

        g.addEventListener("click", () => {
          const st = statusAt(model.transitions, playhead.get()).get(id);
          detail.textContent = [
            `id      : ${node.id}`,
            `name    : ${node.name}`,
            `type    : ${node.type}`,
            `class   : ${node.node_class}`,
            `reads   : ${(node.reads || []).join(", ") || "-"}`,
            `writes  : ${(node.writes || []).join(", ") || "-"}`,
            "",
            `status  : ${st ? st.status : "(not yet ticked)"}`,
            `feedback: ${st ? st.feedback : "-"}`,
          ].join("\n");
        });
        svg.appendChild(g);
      }
      const header = document.createElementNS(SVG_NS, "text");
      header.setAttribute("class", "epoch-label");
      header.setAttribute("x", 0);
      header.setAttribute("y", 8);
      header.textContent =
        `epoch ${epoch.ordinal} — ${epoch.nodes.length} nodes`;
      svg.appendChild(header);
    }

    const states = statusAt(model.transitions, wall);
    for (const g of svg.querySelectorAll("g.node")) {
      const st = states.get(g.dataset.nodeId);
      g.setAttribute("data-status", st ? st.status : "NONE");
    }
  };

  draw(playhead.get());
  playhead.subscribe(draw);
}
```

At the end of `boot`, before the `return`:

```javascript
  mountTree(document.getElementById("panels"), model, playhead);
```

Append to `app.css`:

```css
.tree-panel { display: grid; grid-template-columns: 1fr 320px; gap: 12px;
  margin-top: 12px; }
.tree { background: #0f1114; border: 1px solid var(--line);
  overflow: auto; max-height: 60vh; }
.tree text { fill: var(--fg); font-size: 10px; }
.epoch-label { fill: var(--dim) !important; }
.node { cursor: pointer; }
.node.bookkeeping { opacity: 0.35; }
.node-dot { fill: var(--none); }
.node[data-status="SUCCESS"] .node-dot { fill: var(--pass); }
.node[data-status="FAILURE"] .node-dot { fill: var(--fail); }
.node[data-status="RUNNING"] .node-dot { fill: #6cb6ff; }
.node-detail { background: #0f1114; border: 1px solid var(--line);
  padding: 8px; margin: 0; font-size: 11px; white-space: pre-wrap;
  overflow: auto; max-height: 60vh; }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd tools && node --test tests/js/`
Expected: 11 passing

- [ ] **Step 5: Verify against a real run**

Run: `cd tools && ./gpsr-ui &`, open `/run/t2-2026/s2026-002-countPrsInRoom`.
Expected: the tree shows `epoch 0 — 84 nodes` at the start; dragging the playhead past the second epoch switches it to `epoch 1 — 158 nodes`. Node dots change colour as the playhead moves. Clicking a node fills the detail pane. Stop the server afterwards.

- [ ] **Step 6: Commit**

```bash
git add tools/gpsr_ui/static tools/tests/js/test_tree.mjs
git commit -m "feat: BT tree panel with epoch time travel and node detail"
```

---

### Task 10: Stop-motion frame viewer

**Files:**
- Create: `tools/gpsr_ui/static/frames.js`
- Modify: `tools/gpsr_ui/static/run.js` (mount the viewer)
- Modify: `tools/gpsr_ui/static/app.css`
- Test: `tools/tests/js/test_frames.mjs`

**Interfaces:**
- Consumes: `/api/run/.../frames`.
- Produces: `frames.js` exports `frameAt(refs, wall)` → the ref at-or-before `wall`, `preloadWindow(refs, index, radius, urlFor)` → array of `Image`, and `createPlayer({onTick})` with `play(fps)`, `pause()`, `isPlaying()`.

- [ ] **Step 1: Write the failing test**

```javascript
// tools/tests/js/test_frames.mjs
import test from "node:test";
import assert from "node:assert/strict";
import { createPlayer, frameAt } from "../../gpsr_ui/static/frames.js";

const REFS = [
  { index: 0, stamp_s: 1, file: "0000_1000.jpg", wall: 100 },
  { index: 1, stamp_s: 2, file: "0001_2000.jpg", wall: 105 },
  { index: 2, stamp_s: 3, file: "0002_3000.jpg", wall: 112 },
];

test("frameAt picks the frame at or before the playhead", () => {
  assert.equal(frameAt(REFS, 104).file, "0000_1000.jpg");
  assert.equal(frameAt(REFS, 105).file, "0001_2000.jpg");
  assert.equal(frameAt(REFS, 999).file, "0002_3000.jpg");
});

test("frameAt before the first frame returns the first, not null", () => {
  assert.equal(frameAt(REFS, 1).file, "0000_1000.jpg");
});

test("frameAt on an empty list returns null", () => {
  assert.equal(frameAt([], 100), null);
});

test("frameAt ignores refs with no wall time", () => {
  const mixed = [{ file: "a.jpg", wall: null }, { file: "b.jpg", wall: 50 }];
  assert.equal(frameAt(mixed, 60).file, "b.jpg");
});

test("player reports its state and stops cleanly", () => {
  const player = createPlayer({ onTick: () => {} });
  assert.equal(player.isPlaying(), false);
  player.play(10);
  assert.equal(player.isPlaying(), true);
  player.pause();
  assert.equal(player.isPlaying(), false);
});
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd tools && node --test tests/js/test_frames.mjs`
Expected: FAIL with `Cannot find module .../static/frames.js`

- [ ] **Step 3: Write the implementation**

```javascript
// tools/gpsr_ui/static/frames.js
export function frameAt(refs, wall) {
  let found = null;
  for (const ref of refs) {
    if (ref.wall === null || ref.wall === undefined) continue;
    if (ref.wall <= wall) found = ref;
    else if (found !== null) break;
  }
  if (found === null) {
    for (const ref of refs) {
      if (ref.wall !== null && ref.wall !== undefined) return ref;
    }
  }
  return found;
}

export function preloadWindow(refs, index, radius, urlFor) {
  const images = [];
  const lo = Math.max(0, index - radius);
  const hi = Math.min(refs.length - 1, index + radius);
  for (let i = lo; i <= hi; i += 1) {
    const img = new Image();
    img.src = urlFor(refs[i]);
    images.push(img);
  }
  return images;
}

// Frames are one per sim-second. Playing them back at wall-clock speed would
// be a slideshow, so the player advances by frame index at a chosen fps.
export function createPlayer({ onTick }) {
  let timer = null;
  return {
    isPlaying: () => timer !== null,
    play(fps) {
      if (timer !== null) return;
      timer = setInterval(onTick, Math.max(1, Math.round(1000 / fps)));
    },
    pause() {
      if (timer === null) return;
      clearInterval(timer);
      timer = null;
    },
  };
}
```

Append to `run.js` and call `mountFrames` at the end of `boot`:

```javascript
import { createPlayer, frameAt, preloadWindow } from "./frames.js";

async function mountFrames(container, base, model, playhead) {
  const payload = await (await fetch(`${base}/frames`)).json();
  const labels = Object.keys(payload.labels || {});
  const panel = document.createElement("div");
  panel.className = "frames-panel";

  if (labels.length === 0) {
    panel.innerHTML = "<p class='muted'>no frames recorded for this run</p>";
    container.appendChild(panel);
    return;
  }

  const tracks = labels.map((label) => {
    const refs = payload.labels[label];
    const wrap = document.createElement("figure");
    const img = document.createElement("img");
    img.loading = "eager";
    const cap = document.createElement("figcaption");
    wrap.append(img, cap);
    panel.appendChild(wrap);
    return { label, refs, img, cap };
  });

  const controls = document.createElement("div");
  controls.className = "frame-controls";
  const button = document.createElement("button");
  button.textContent = "play";
  const speed = document.createElement("select");
  for (const fps of [2, 5, 10, 20, 40]) {
    const option = document.createElement("option");
    option.value = String(fps);
    option.textContent = `${fps} fps`;
    if (fps === 10) option.selected = true;
    speed.appendChild(option);
  }
  controls.append(button, speed);
  panel.appendChild(controls);
  container.appendChild(panel);

  const urlFor = (label) => (ref) =>
    `/frame/${encodeURIComponent(model.tier)}/`
    + `${encodeURIComponent(model.dir_name)}/`
    + `${encodeURIComponent(label)}/${encodeURIComponent(ref.file)}`;

  const render = (wall) => {
    for (const track of tracks) {
      const ref = frameAt(track.refs, wall);
      if (!ref) continue;
      const url = urlFor(track.label)(ref);
      if (track.img.getAttribute("src") !== url) track.img.src = url;
      track.cap.textContent =
        `${track.label} · frame ${ref.index} · sim ${ref.stamp_s.toFixed(3)}s`;
      preloadWindow(track.refs, track.refs.indexOf(ref), 4, urlFor(track.label));
    }
  };
  render(playhead.get());
  playhead.subscribe(render);

  const primary = tracks[0];
  const player = createPlayer({
    onTick: () => {
      const ref = frameAt(primary.refs, playhead.get());
      const next = primary.refs[primary.refs.indexOf(ref) + 1];
      if (!next || next.wall === null) {
        player.pause();
        button.textContent = "play";
        return;
      }
      playhead.set(next.wall);
    },
  });
  button.addEventListener("click", () => {
    if (player.isPlaying()) {
      player.pause();
      button.textContent = "play";
    } else {
      player.play(Number(speed.value));
      button.textContent = "pause";
    }
  });
}
```

In `boot`, add `model.tier = tier; model.dir_name = dirName;` right after the fetch, and `await mountFrames(document.getElementById("panels"), base, model, playhead);` before the `return`.

Append to `app.css`:

```css
.frames-panel { display: flex; flex-wrap: wrap; gap: 12px; margin-top: 12px;
  align-items: flex-start; }
.frames-panel figure { margin: 0; }
.frames-panel img { max-width: 46vw; border: 1px solid var(--line);
  background: #000; display: block; }
.frames-panel figcaption { color: var(--dim); font-size: 11px;
  padding-top: 4px; }
.frame-controls { display: flex; gap: 8px; align-items: center; width: 100%; }
.frame-controls button, .frame-controls select {
  background: #1b1f26; color: var(--fg); border: 1px solid var(--line);
  padding: 3px 10px; font: inherit; cursor: pointer; }
.muted { color: var(--dim); }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd tools && node --test tests/js/`
Expected: 16 passing

- [ ] **Step 5: Verify the linked triage loop end to end**

Run: `cd tools && ./gpsr-ui &`, open `/run/t2-2026/s2026-002-countPrsInRoom`.
Expected: two camera tracks (head and arena, 152 frames each). Pressing play advances both. Clicking a red mark in the ribbon jumps the tree, the detail pane and both images to that moment. Then open `/run/t2-2026/s2026-003-findObjInRoom`, which has a head camera but no arena, and confirm a single track renders without error. Stop the server afterwards.

- [ ] **Step 6: Commit**

```bash
git add tools/gpsr_ui/static tools/tests/js/test_frames.mjs
git commit -m "feat: two-track stop-motion viewer linked to the playhead"
```

---

### Task 11: Live tail and dashboard

**Files:**
- Create: `tools/gpsr_ui/live.py`
- Create: `tools/gpsr_ui/static/live.js`, `tools/gpsr_ui/templates/live.html`
- Modify: `tools/gpsr_ui/app.py` (add three routes)
- Test: `tools/tests/test_live.py`

**Interfaces:**
- Consumes: `gpsr_ui.corpus.list_tiers`, `gpsr_ui.telemetry`.
- Produces: `find_in_flight(bench_root, stale_after=60.0, now=None) -> list[InFlight]`; `InFlight` dataclass `tier: str`, `dir_name: str`, `path: Path`, `last_event_age: float`; `tail_events(path, offset) -> tuple[list[dict], int]`; `live_summary(run_dir) -> dict`. Routes `GET /live`, `GET /api/live`, `GET /api/live/stream`.

- [ ] **Step 1: Write the failing test**

```python
# tools/tests/test_live.py
from __future__ import annotations

import json
import time

from gpsr_ui.live import find_in_flight, live_summary, tail_events


def test_a_run_without_run_json_is_in_flight(make_run):
    run = make_run(name="s9999-060-x", verdict=None, finished=False)
    found = find_in_flight(run.parents[2])
    assert [f.dir_name for f in found] == ["s9999-060-x"]


def test_a_finished_run_is_not_in_flight(make_run):
    run = make_run(name="s9999-061-x", verdict="PASS")
    assert find_in_flight(run.parents[2]) == []


def test_a_stale_run_without_run_json_is_dead_not_live(make_run):
    """A crashed run never gets its run.json; staleness is the guard."""
    run = make_run(name="s9999-062-x", verdict=None, finished=False)
    future = time.time() + 3600
    assert find_in_flight(run.parents[2], stale_after=60.0, now=future) == []


def test_tail_events_resumes_from_a_byte_offset(make_run):
    run = make_run(name="s9999-063-x", verdict=None, finished=False)
    events = next((run / "debug").glob("gpsr-*")) / "events.jsonl"

    first, offset = tail_events(events, 0)
    assert len(first) > 0
    again, offset2 = tail_events(events, offset)
    assert again == []
    assert offset2 == offset

    with events.open("a") as fh:
        fh.write(json.dumps({"event_type": "run.heartbeat",
                             "payload": {"tick": 99}}) + "\n")
    more, _ = tail_events(events, offset)
    assert len(more) == 1
    assert more[0]["payload"]["tick"] == 99


def test_tail_events_does_not_consume_a_torn_final_line(make_run):
    run = make_run(name="s9999-064-x", verdict=None, finished=False)
    events = next((run / "debug").glob("gpsr-*")) / "events.jsonl"
    _, offset = tail_events(events, 0)

    with events.open("a") as fh:
        fh.write('{"event_type": "tree.gen')  # no trailing newline yet
    torn, torn_offset = tail_events(events, offset)
    assert torn == []
    assert torn_offset == offset, "offset must not advance past a partial line"

    with events.open("a") as fh:
        fh.write('erated", "payload": {"nodes": []}}\n')
    complete, _ = tail_events(events, torn_offset)
    assert len(complete) == 1
    assert complete[0]["event_type"] == "tree.generated"


def test_live_summary_reports_regenerations_and_elapsed(make_run):
    run = make_run(
        name="s9999-065-x", verdict=None, finished=False,
        epochs=[["a"], ["a", "b"], ["a", "b", "c"]],
        transitions=[("2026-08-28T10:00:30.000000Z", "executor/root/0",
                      "FAILURE", "goto target failed")],
    )
    summary = live_summary(run)
    assert summary["tree_regenerations"] == 1
    assert summary["elapsed_s"] is not None
    assert summary["last_failure"]["feedback"] == "goto target failed"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest tests/test_live.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'gpsr_ui.live'`

- [ ] **Step 3: Write the implementation**

```python
# tools/gpsr_ui/live.py
"""Detect and follow the run that is currently executing.

Liveness signal: the bench writes run.json LAST, after the event log closes
(verified on a real run: events.jsonl at 06:58:50, run.json at 06:58:52).
So a run directory without run.json is in flight. There is no explicit
per-run marker. A crashed or torn-down run never gets its run.json, so a
staleness threshold on the event log is the guard; the bench timeout is
900 s, and anything quiet for a minute with no run.json is gone.
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path

from .clock import parse_wall
from .corpus import list_tiers
from .telemetry import load_run_model, newest_events_file


@dataclass(frozen=True)
class InFlight:
    tier: str
    dir_name: str
    path: Path
    last_event_age: float


def find_in_flight(
    bench_root: Path,
    stale_after: float = 60.0,
    now: float | None = None,
) -> list[InFlight]:
    now = time.time() if now is None else now
    out: list[InFlight] = []
    for tier in list_tiers(bench_root):
        for entry in tier.entries:
            for attempt in entry.attempts:
                if attempt.verdict is not None:
                    continue  # run.json exists: finished
                if (attempt.path / "run.json").exists():
                    continue
                events = newest_events_file(attempt.path)
                if events is None:
                    continue
                try:
                    age = now - events.stat().st_mtime
                except OSError:
                    continue
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
    """Read complete lines from `offset`. Never advances past a partial
    trailing line, which a live writer produces routinely."""
    events: list[dict] = []
    try:
        size = path.stat().st_size
    except OSError:
        return [], offset
    if size < offset:
        offset = 0  # file was replaced or truncated
    try:
        with path.open("rb") as fh:
            fh.seek(offset)
            chunk = fh.read()
    except OSError:
        return [], offset

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


def live_summary(run_dir: Path) -> dict:
    model = load_run_model(run_dir)

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
        events = newest_events_file(run_dir)
        if events is not None:
            try:
                elapsed = events.stat().st_mtime - model.started_wall
            except OSError:
                elapsed = None

    plan_step = None
    for j in reversed(model.judge_events):
        if "plan-step" in (j.info or ""):
            plan_step = j.info
            break

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
        "plan_step": plan_step,
        "announcements": model.announcements[-8:],
        "milestone_count": len(model.milestones),
    }
```

Add to `create_app` in `app.py`:

```python
    @app.get("/api/live")
    def api_live() -> JSONResponse:
        found = find_in_flight(settings.bench_root)
        return JSONResponse({
            "in_flight": [
                {
                    "tier": f.tier, "dir_name": f.dir_name,
                    "last_event_age": f.last_event_age,
                    "summary": live_summary(f.path),
                }
                for f in found
            ]
        })

    @app.get("/api/live/stream")
    async def api_live_stream():
        async def events():
            # Byte offsets per run, so each poll reads only what was
            # appended since the last one rather than re-reading the log.
            offsets: dict[str, int] = {}
            while True:
                found = find_in_flight(settings.bench_root)
                items = []
                for f in found:
                    key = str(f.path)
                    log = newest_events_file(f.path)
                    new_events: list[dict] = []
                    if log is not None:
                        new_events, offsets[key] = tail_events(
                            log, offsets.get(key, 0))
                    items.append({
                        "tier": f.tier,
                        "dir_name": f.dir_name,
                        "summary": live_summary(f.path),
                        "new_event_types": [
                            e.get("event_type") for e in new_events],
                    })
                live_keys = {str(f.path) for f in found}
                for stale in set(offsets) - live_keys:
                    del offsets[stale]
                yield f"data: {json.dumps({'in_flight': items})}\n\n"
                await asyncio.sleep(2.0)

        return StreamingResponse(events(), media_type="text/event-stream")

    @app.get("/live")
    def live_page(request: Request):
        return templates.TemplateResponse("live.html", {"request": request})
```

Extend `app.py` imports:

```python
import asyncio
import json
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from .live import find_in_flight, live_summary, tail_events
from .telemetry import newest_events_file
```

```html
<!-- tools/gpsr_ui/templates/live.html -->
{% extends "base.html" %}
{% block title %}live{% endblock %}
{% block content %}
<h2 class="run-title">In flight</h2>
<div id="live"><p class="muted">waiting for a run…</p></div>
<script type="module">
  import { boot } from "/static/live.js";
  boot();
</script>
{% endblock %}
```

```javascript
// tools/gpsr_ui/static/live.js
function fmt(seconds) {
  if (seconds === null || seconds === undefined) return "-";
  const m = Math.floor(seconds / 60);
  const s = Math.floor(seconds % 60);
  return `${m}m${String(s).padStart(2, "0")}s`;
}

function card(item) {
  const s = item.summary;
  const overBudget = s.elapsed_s !== null && s.elapsed_s > s.budget_s;
  const pct = s.elapsed_s === null
    ? 0 : Math.min(100, (s.elapsed_s / s.timeout_s) * 100);
  return `
    <article class="live-card">
      <h3><a href="/run/${item.tier}/${item.dir_name}">${item.dir_name}</a></h3>
      <div class="budget"><div class="budget-fill ${
        overBudget ? "over" : ""}" style="width:${pct}%"></div></div>
      <dl>
        <dt>elapsed</dt><dd class="${overBudget ? "bad" : ""}">${
          fmt(s.elapsed_s)} / ${fmt(s.budget_s)} target</dd>
        <dt>plan step</dt><dd>${s.plan_step || "-"}</dd>
        <dt>last nav</dt><dd>${
          s.last_nav ? `${s.last_nav.name} — ${s.last_nav.status}` : "-"}</dd>
        <dt>tree regens</dt><dd>${s.tree_regenerations}</dd>
        <dt>gate failures</dt><dd class="${
          s.gate_failures ? "bad" : ""}">${s.gate_failures}</dd>
        <dt>last failure</dt><dd class="bad">${
          s.last_failure ? s.last_failure.feedback : "none"}</dd>
      </dl>
      <pre class="announce">${(s.announcements || []).join("\n")}</pre>
    </article>`;
}

export function boot() {
  const root = document.getElementById("live");
  const source = new EventSource("/api/live/stream");
  source.onmessage = (event) => {
    const data = JSON.parse(event.data);
    root.innerHTML = data.in_flight.length === 0
      ? "<p class='muted'>no run in flight</p>"
      : data.in_flight.map(card).join("");
  };
  source.onerror = () => {
    root.insertAdjacentHTML("afterbegin",
      "<p class='muted'>stream interrupted; retrying…</p>");
  };
}
```

Append to `app.css`:

```css
.live-card { border: 1px solid var(--line); padding: 12px; margin-bottom: 12px;
  background: #0f1114; }
.live-card h3 { margin: 0 0 8px; font-size: 14px; }
.live-card dl { display: grid; grid-template-columns: 110px 1fr;
  gap: 2px 10px; margin: 0; }
.live-card dt { color: var(--dim); }
.live-card dd { margin: 0; }
.budget { height: 4px; background: var(--line); margin-bottom: 10px; }
.budget-fill { height: 100%; background: var(--pass); }
.budget-fill.over { background: var(--timeout); }
.bad { color: var(--fail); }
.announce { color: var(--dim); font-size: 11px; margin: 8px 0 0;
  white-space: pre-wrap; }
```

Add a link to the live page in `base.html`'s topbar:

```html
  <header class="topbar">
    <a href="/">GPSR Bench Run Viewer</a>
    <a href="/live" class="topbar-live">live</a>
  </header>
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest tests/test_live.py -v`
Expected: 6 passed

- [ ] **Step 5: Verify the full suite and the live page**

Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest -v && node --test tests/js/`
Expected: everything passing
Run: `cd tools && ./gpsr-ui &`, open `/live`.
Expected: "no run in flight" when idle; a card with an advancing elapsed bar while a battery is running. Stop the server afterwards.

- [ ] **Step 6: Commit**

```bash
git add tools/gpsr_ui/live.py tools/gpsr_ui/static/live.js tools/gpsr_ui/templates tools/gpsr_ui/app.py tools/tests/test_live.py
git commit -m "feat: live tail and in-flight dashboard over SSE"
```

---

### Task 12: README and final verification

**Files:**
- Create: `tools/gpsr_ui/README.md`
- Test: full suite

- [ ] **Step 1: Write the README**

```markdown
# GPSR Bench Run Viewer

A local, read-only web UI over the GPSR bench run artifacts.

## Run it

    tools/gpsr-ui

Then open http://localhost:8770. `/live` shows the run currently executing.

## Configuration

| Variable | Default | Meaning |
|---|---|---|
| `GPSR_UI_BENCH_ROOT` | the tk25_decision GPSR bench dir | corpus to read |
| `GPSR_UI_STATE_DIR` | `~/.cache/gpsr-ui` | derived-model cache |
| `GPSR_UI_SHEET_EVENTS` | unset | override the vendored classifier |
| `GPSR_UI_PORT` | `8770` | listen port |

## Guarantees

Nothing is ever written inside `gpsr_runs/`. `tests/test_read_only.py`
enforces this by snapshotting mtimes across the corpus before and after a
full indexing pass. `GPSR_UI_STATE_DIR` is rejected at startup if it points
inside the corpus.

## Things about the data that surprised us

* **`tree_revision` is `0` everywhere** — all 289 `tree.generated` events
  across all 105 runs. We surfaced this; upstream `sheet_events` was fixed
  in `9072c6e` to key REPLAN on generation count instead, and we vendor that.
* **Two tree generations is normal, not a replan.** A skeleton tree, then the
  plan materialised. `tree_regenerations` is `max(0, epochs - 2)`.
* **Tree regeneration and executor replan are different phenomena.** The
  `DynamicExecutor` replans without regenerating the tree, so epoch count
  *undercounts* replans on t2 runs — `s2026-002.attempt11` replan-looped for
  900 s with only 2 epochs. For those runs, watch `gate_failures`
  (PRECONDITION/POSTCONDITION failures) instead.
* **`run.finished.status` is `"incomplete"` everywhere**, including the run
  that passed. The verdict lives in `run.json`.
* **Sim and wall clocks are not proportional.** RTF varies 0.2–0.5 within a
  single run, so frames cannot be placed on the event timeline by scaling.
  `frames/index.jsonl` carries the real join; without it we interpolate and
  badge the run `approximate`.
* **`run.json`'s `id` is the corpus entry, not the attempt.** All 11 attempt
  directories of `s2026-002-countPrsInRoom` share one `id`; the directory
  name is the attempt identity.
* **`announcements.txt` repeats itself.** One run is 7967 lines containing
  11 distinct utterances.

## Tests

    cd tools
    PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest
    node --test tests/js/

Tests marked `corpus` read the real bench tree and skip when it is absent.
Set `GPSR_UI_SKIP_CORPUS=1` to skip them explicitly — useful while a
battery is running.
```

- [ ] **Step 2: Run the complete suite**

Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest -v`
Expected: all passing
Run: `cd tools && node --test tests/js/`
Expected: all passing
Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m compileall -q gpsr_ui`
Expected: no output

- [ ] **Step 3: Confirm the read-only guarantee against the live corpus**

Run: `cd tools && PYTHONPATH= /home/tinker/tinker-sim/6.0.1/.venv/bin/python -m pytest tests/test_read_only.py -v`
Expected: passed (or skipped if the corpus is absent)

- [ ] **Step 4: Commit**

```bash
git add tools/gpsr_ui/README.md
git commit -m "docs: README for the GPSR bench run viewer"
```

---

## Deferred to a follow-up

Deliberately not in this plan. Each is independently useful and none blocks the rest.

- **mp4 export** via ffmpeg. Needs a job model so a 900-frame encode does not block a request.
- **Triage annotations** in `GPSR_UI_STATE_DIR`. Wanted, but the read-only story should be proven in daily use first.
- **The "Failed to make progress" panel** from `gpsr_stack_logs/<newest>/02-bridge.log`. Deferred because that path lives in a worktree whose lifetime we do not control; the UI must degrade to a missing panel, and that deserves its own design.
- **Thumbnail generation** with Pillow. Only pays off once a run browser shows frame previews.
- **The budget bar on the run view's timeline ribbon.** The spec puts it there; Task 11 builds one on the live dashboard, where it earns its place, and Task 8 leaves the ribbon without one. A finished run already shows its duration next to the verdict, so the bar adds little there. Noted as a conscious deviation rather than an oversight.
- **Cross-attempt comparison** — diffing the 11 attempts of `s2026-002` against each other. The most valuable follow-up, and large enough to need its own spec.
