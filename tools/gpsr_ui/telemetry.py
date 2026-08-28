# tools/gpsr_ui/telemetry.py
"""Derive a run model from events.jsonl.

Two facts about the corpus shape this module, both measured across all 105
events.jsonl files:

  * tree_revision is 0 in every one of the 289 tree.generated events. A
    replan is therefore an *extra tree.generated epoch*, not a revision
    bump. The vendored classifier emits its REPLAN judge event only when
    tree_revision > 0 OR tree_generations >= 3, so replans surface there
    already; this module must not emit its own replan events on top.
  * run.finished.status is "incomplete" in every run, including the one
    that passed. It is never an outcome. The verdict lives in run.json,
    which this module does not read -- only run.finished's wall time is
    used, as `finished_wall`.
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
        """Latest epoch at-or-before `wall`.

        Epochs are appended in file-arrival order, which is chronological,
        so a strictly-later epoch (wall > playhead) means every epoch after
        it is later still and the scan can stop. An epoch whose wall failed
        to parse (None) carries no ordering information -- it must not be
        allowed to end the scan early, only to be skipped, or a single
        unparseable timestamp partway through the list would hide every
        valid epoch that follows it.
        """
        if wall is None:
            return None
        found: Epoch | None = None
        for epoch in self.epochs:
            if epoch.wall is None:
                continue
            if epoch.wall <= wall:
                found = epoch
            else:
                break
        return found

    def status_at(self, wall: float | None) -> dict[str, Transition]:
        """Last transition per node at or before `wall`.

        A node that never ticked at or before `wall` (or never ticked at
        all) simply has no key in the returned mapping -- callers use
        dict.get, not indexing, for nodes that may not have started.
        """
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
                # Only the wall time is meaningful here.
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
    # tree genuinely regenerated. tree_revision is 0 corpus-wide, so this
    # counts arrival-order epochs rather than reading a revision number.
    model.tree_regenerations = max(0, len(model.epochs) - 2)

    # Milestones/judge events come from the vendored classifier, which
    # must run before gate_failures can be derived from its judge_events.
    milestones, judge_events, _meta = sheet_events.load_run_telemetry(run_dir)
    model.milestones = milestones
    model.judge_events = judge_events

    # Replan-adjacent signal: PRECONDITION/POSTCONDITION judge gates that
    # FAILED. This is not the same thing as a tree regeneration (see
    # module docstring) -- the executor can replan internally, without
    # ever regenerating the tree, so this is the metric that reflects it.
    model.gate_failures = sum(
        1 for j in judge_events
        if j.status == "FAILURE"
        and j.kind in ("PRECONDITION", "POSTCONDITION")
    )

    try:
        raw = (run_dir / "announcements.txt").read_text()
    except (OSError, UnicodeDecodeError):
        raw = ""
    model.announcements = dedupe_announcements(raw.splitlines())

    return model
