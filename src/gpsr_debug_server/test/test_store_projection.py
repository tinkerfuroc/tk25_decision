"""Focused contract tests for the standalone debugger backend."""
from __future__ import annotations

import json

import pytest

from gpsr_debug_server.retention import RetentionPolicy
from gpsr_debug_server.store import DebugStore, EventConflictError


def event(trajectory_id: str, sequence: int, event_id: str, event_type: str, payload: dict | None = None, *, at: str | None = None) -> dict:
    return {
        "trajectory_id": trajectory_id,
        "sequence": sequence,
        "event_id": event_id,
        "type": event_type,
        "occurred_at": at or f"2026-08-01T00:00:{sequence:02d}Z",
        "payload": payload or {},
    }


def test_append_is_idempotent_immutable_and_causally_queryable() -> None:
    with DebugStore(":memory:") as store:
        started = event("trace-1", 1, "start", "trajectory.started")
        assert store.append_event(started) is True
        assert store.append_event(started) is False

        conflicting = event("trace-1", 1, "different", "trajectory.started")
        with pytest.raises(EventConflictError):
            store.append_event(conflicting)

        second = event("trace-1", 2, "plan", "plan.committed", {"revision": 1, "steps": []})
        second["causation_id"] = "start"
        assert store.append_event(second) is True
        assert [item["event_id"] for item in store.events("trace-1")] == ["start", "plan"]
        assert [item["event_id"] for item in store.causal_events("trace-1", "plan")] == ["start"]


def test_checkpoint_delta_projects_agents_plans_trees_outcomes_and_interventions() -> None:
    with DebugStore(":memory:", checkpoint_interval=2) as store:
        assert store.append_event(event("trace-2", 1, "start", "trajectory.started", {"mode": "mock"}))
        assert store.append_event(event("trace-2", 2, "agent", "agent.registered", {"agent_id": "planner", "status": "idle"}))
        assert store.append_event(
            event("trace-2", 3, "plan", "plan.committed", {"revision": 1, "plan_id": "p1", "steps": [{"action": "go"}]})
        )
        assert store.append_event(
            event(
                "trace-2",
                4,
                "tree",
                "tree.generated",
                {"revision": "r1", "tree_id": "planned-r1", "nodes": [{"id": "root", "name": "Root"}], "edges": []},
            )
        )
        assert store.append_event(
            event(
                "trace-2",
                5,
                "node",
                "tree.node_states_changed",
                {"revision": "r1", "changed_nodes": [{"id": "root", "state": "RUNNING", "feedback": "planning"}]},
            )
        )
        assert store.append_event(
            event("trace-2", 6, "intervention", "intervention.requested", {"request_id": "req-1", "command": "pause"})
        )
        assert store.append_event(
            event("trace-2", 7, "finish", "trajectory.finished", {"status": "succeeded", "summary": "done"})
        )

        historical = store.trajectory_snapshot("trace-2", at_sequence=4)
        assert historical["status"] == "running"
        snapshot = store.trajectory_snapshot("trace-2")
        assert snapshot["status"] == "succeeded"
        assert snapshot["agents"]["planner"]["status"] == "idle"
        assert snapshot["plans"]["active_revision"] == "1"
        assert snapshot["trees"]["revisions"]["r1"]["nodes"]["root"]["status"] == "RUNNING"
        assert snapshot["trees"]["revisions"]["r1"]["nodes"]["root"]["feedback"] == "planning"
        assert snapshot["interventions"][0]["command"] == "pause"
        assert snapshot["outcomes"][0]["status"] == "succeeded"
        assert json.loads(json.dumps(snapshot)) == snapshot

        document = store.tree_document("trace-2", "r1")
        assert document["tree_id"] == "planned-r1"
        assert document["nodes"]["root"]["status"] == "RUNNING"


def test_names_pins_and_active_runs_are_excluded_from_retention() -> None:
    policy = RetentionPolicy(max_age_days=7, max_bytes=10**9)
    with DebugStore(":memory:", retention_policy=policy) as store:
        for trajectory_id, extra in (("old", {}), ("named", {"name": "keep me"}), ("pinned", {"pinned": True})):
            assert store.append_event(event(trajectory_id, 1, f"{trajectory_id}-start", "trajectory.started", extra, at="2026-07-01T00:00:00Z"))
            assert store.append_event(
                event(trajectory_id, 2, f"{trajectory_id}-end", "trajectory.completed", {"status": "succeeded"}, at="2026-07-01T00:01:00Z")
            )
        assert store.append_event(event("active", 1, "active-start", "trajectory.started", at="2026-07-01T00:00:00Z"))

        report = store.retention(now="2026-08-03T00:00:00Z")
        assert report["deleted_trajectory_ids"] == ["old"]
        assert {row["trajectory_id"] for row in store.list_trajectories(limit=10)} == {"named", "pinned", "active"}
        assert store.trajectory_snapshot("named")["name"] == "keep me"
        assert store.trajectory_snapshot("pinned")["pinned"] is True


def test_causal_order_wins_when_packets_arrive_out_of_order_and_budget_applies() -> None:
    policy = RetentionPolicy(max_age_days=30, max_bytes=1)
    with DebugStore(":memory:", checkpoint_interval=1, retention_policy=policy) as store:
        # A reconnect can deliver the terminal event before the original start.
        assert store.append_event(
            event("reordered", 2, "reordered-end", "trajectory.completed", {"status": "succeeded"}, at="2026-08-01T00:02:00Z")
        )
        assert store.append_event(event("reordered", 1, "reordered-start", "trajectory.started", at="2026-08-01T00:01:00Z"))
        snapshot = store.trajectory_snapshot("reordered")
        assert snapshot["started_at"] == "2026-08-01T00:01:00Z"
        assert snapshot["status"] == "succeeded"
        assert store.list_trajectories()[0]["completed"] is True

        report = store.retention(now="2026-08-02T00:00:00Z")
        assert report["deleted_trajectory_ids"] == ["reordered"]


def test_monotonic_append_does_not_rebuild_the_whole_projection(monkeypatch) -> None:
    with DebugStore(":memory:", checkpoint_interval=100) as store:
        def unexpected_refresh(_trajectory_id):
            raise AssertionError("monotonic ingestion must stay incremental")

        monkeypatch.setattr(store, "_refresh_trajectory_lifecycle_locked", unexpected_refresh)
        assert store.append_event(event("stream", 1, "start", "trajectory.started"))
        assert store.append_event(event("stream", 2, "tick", "tree.tick_observed", {"tick": 1}))
        assert store.append_event(
            event("stream", 3, "end", "trajectory.completed", {"status": "succeeded"})
        )
        assert store.trajectory_snapshot("stream")["status"] == "succeeded"


def test_pagination_and_manual_deletion_have_json_safe_results() -> None:
    with DebugStore(":memory:") as store:
        for trajectory_id in ("one", "two", "three"):
            assert store.append_event(event(trajectory_id, 1, trajectory_id, "trajectory.started"))
        page = store.list_trajectories(limit=2)
        assert len(page) == 2
        next_page = store.list_trajectories(limit=2, cursor=page[-1]["cursor"])
        assert {item["trajectory_id"] for item in page}.isdisjoint(item["trajectory_id"] for item in next_page)
        renamed = store.set_trajectory_name("one", "  diagnostic  ")
        assert renamed["name"] == "diagnostic"
        assert store.delete_trajectory("one") is True
        assert store.delete_trajectory("one") is False
        assert json.loads(json.dumps(store.list_trajectories())) == store.list_trajectories()


def test_task_command_and_step_status_are_projected_for_the_dashboard() -> None:
    with DebugStore(":memory:") as store:
        task_id = "trace-task/task-1"
        assert store.append_event(event("trace-task", 1, "start", "run.started"))
        assert store.append_event(
            event(
                "trace-task",
                2,
                "command",
                "task.command_received",
                {"task_id": task_id, "command": "bring water"},
            )
        )
        assert store.append_event(
            event(
                "trace-task",
                3,
                "plan",
                "plan.committed",
                {"revision": 1, "steps": [{"action": "goto", "params": {}}]},
            )
        )
        assert store.append_event(
            event(
                "trace-task",
                4,
                "step-start",
                "step.started",
                {"task_id": task_id, "step_id": "plan-r1/step-0000", "action": "goto"},
            )
        )
        assert store.append_event(
            event(
                "trace-task",
                5,
                "step-end",
                "step.finished",
                {
                    "task_id": task_id,
                    "step_id": "plan-r1/step-0000",
                    "action": "goto",
                    "outcome": "succeeded",
                },
            )
        )
        snapshot = store.trajectory_snapshot("trace-task")
        assert snapshot["command"] == "bring water"
        assert snapshot["tasks"][task_id]["command"] == "bring water"
        assert snapshot["tasks"][task_id]["steps"]["plan-r1/step-0000"]["status"] == "succeeded"
        assert snapshot["plans"]["revisions"]["1"]["steps"][0]["status"] == "succeeded"
        assert not any(item["type"] == "task.command_received" for item in snapshot["unknown_events"])


def test_orchestrator_plan_list_is_normalized_to_projected_steps() -> None:
    with DebugStore(":memory:") as store:
        assert store.append_event(
            event(
                "trace-plan-list",
                1,
                "plan",
                "plan.committed",
                {
                    "plan_revision": 4,
                    "plan": [{"action": "announce", "params": {"text": "ready"}}],
                },
            )
        )

        revision = store.trajectory_snapshot("trace-plan-list")["plans"]["revisions"]["4"]
        assert revision["steps"] == [
            {"action": "announce", "params": {"text": "ready"}},
        ]


def test_stale_projection_checkpoint_is_rebuilt_from_immutable_events() -> None:
    with DebugStore(":memory:", checkpoint_interval=1) as store:
        assert store.append_event(event("upgrade", 1, "start", "run.started"))
        assert store.append_event(
            event(
                "upgrade",
                2,
                "command",
                "task.command_received",
                {"task_id": "upgrade/task-1", "command": "find water"},
            )
        )
        row = store._connection.execute(
            "SELECT snapshot_json FROM checkpoints WHERE trajectory_id = ? ORDER BY sequence DESC LIMIT 1",
            ("upgrade",),
        ).fetchone()
        stale = json.loads(row["snapshot_json"])
        stale["projection_version"] = 1
        stale.pop("tasks", None)
        stale.pop("command", None)
        store._connection.execute(
            "UPDATE checkpoints SET snapshot_json = ? WHERE trajectory_id = ?",
            (json.dumps(stale), "upgrade"),
        )

        rebuilt = store.trajectory_snapshot("upgrade")
        assert rebuilt["projection_version"] == 4
        assert rebuilt["command"] == "find water"
        assert rebuilt["tasks"]["upgrade/task-1"]["command"] == "find water"


def test_proposal_preserves_proposer_and_records_votes_separately() -> None:
    with DebugStore(":memory:") as store:
        assert store.append_event(
            event(
                "proposal-trace",
                1,
                "proposal",
                "proposal.created",
                {"proposal_id": "p1", "agent_id": "planner", "summary": "change route"},
            )
        )
        assert store.append_event(
            event(
                "proposal-trace",
                2,
                "vote",
                "vote.cast",
                {"proposal_id": "p1", "agent_id": "critic", "vote": "approve"},
            )
        )
        proposal = store.trajectory_snapshot("proposal-trace")["proposals"][0]
        assert proposal["agent_id"] == "planner"
        assert proposal["proposer_agent_id"] == "planner"
        assert proposal["votes"][0]["agent_id"] == "critic"
