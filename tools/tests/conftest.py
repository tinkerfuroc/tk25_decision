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

        if len(epochs) > 59:
            raise ValueError(
                f"make_run supports at most 59 tree.generated epochs "
                f"(got {len(epochs)}); each is stamped at second (i+1) "
                f"of 2026-08-28T10:00:00Z and seconds only go to :59"
            )

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
                       f"2026-08-28T10:00:{i + 1:02d}.000000Z", _tree(names))
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
