import json

from behavior_tree.GPSR.bench.events import TaskResult, parse_events, slot_of


def _ev(event_type, task_id=None, payload=None, at="2026-08-23T00:00:00Z", seq=0):
    return {"schema": "tinker.gpsr.telemetry", "schema_version": 1, "event_id": str(seq),
            "trajectory_id": "gpsr-x", "task_id": task_id, "trace_id": None, "source_id": "t",
            "sequence": seq, "occurred_at": at, "monotonic_ns": seq, "event_type": event_type,
            "phase": None, "parent_event_id": None, "causation_ids": [], "payload": payload or {}}


def test_slot_of_parses_task_suffix():
    assert slot_of("gpsr-x/task-3") == 3
    assert slot_of(None) is None
    assert slot_of("gpsr-x") is None


def test_parse_events_groups_by_slot_and_keeps_step_order(tmp_path):
    lines = [
        _ev("run.started", seq=0),
        _ev("step.finished", "gpsr-x/task-0", {"action": "goto", "outcome": "succeeded"}, "2026-08-23T00:00:01Z", 1),
        _ev("planner.error", "gpsr-x/task-1", {"error": "boom"}, seq=2),
        _ev("step.finished", "gpsr-x/task-0", {"action": "find_person", "outcome": "failed"}, seq=3),
        _ev("task.finished", "gpsr-x/task-0", {"status": "failed", "reason": "find_person(...) failed"}, "2026-08-23T00:00:05Z", 4),
        _ev("step.finished", "gpsr-x/task-1", {"action": "announce", "outcome": "succeeded"}, seq=5),
    ]
    path = tmp_path / "events.jsonl"
    path.write_text("\n".join(json.dumps(l) for l in lines) + "\n")

    results = parse_events(path)
    assert set(results) == {0, 1}
    t0 = results[0]
    assert t0 == TaskResult(slot=0, status="failed", reason="find_person(...) failed",
                            steps=[("goto", "succeeded"), ("find_person", "failed")],
                            planner_errors=0, first_seen="2026-08-23T00:00:01Z",
                            finished_at="2026-08-23T00:00:05Z")
    assert results[1].status is None
    assert results[1].planner_errors == 1


def test_parse_events_tolerates_partial_last_line(tmp_path):
    path = tmp_path / "events.jsonl"
    path.write_text(json.dumps(_ev("step.finished", "g/task-0", {"action": "goto", "outcome": "succeeded"})) + "\n{\"trunc")
    assert parse_events(path)[0].steps == [("goto", "succeeded")]


def _tree_generated(name_by_id, kind="executor", at="2026-08-23T00:00:00Z", seq=0):
    """A `tree.generated` event: the only telemetry event that carries a node's human name
    (e.g. "executor task 1") alongside its id — `tree.node_states_changed` (below) carries only
    ids, so a listener has to remember this id -> name mapping to interpret those."""
    nodes = [{"id": node_id, "node_id": node_id, "name": name} for node_id, name in name_by_id.items()]
    return _ev("tree.generated", None, {"kind": kind, "nodes": nodes}, at, seq)


def _node_states_changed(status_by_id, kind="executor", at="2026-08-23T00:00:00Z", seq=0):
    nodes = [{"id": node_id, "node_id": node_id, "status": status} for node_id, status in status_by_id.items()]
    return _ev("tree.node_states_changed", None, {"tree_kind": kind, "nodes": nodes}, at, seq)


def test_parse_events_derives_a_terminal_status_from_executor_node_states(tmp_path):
    """The production two-layer flow never emits task.finished/step.finished (only the legacy
    flow does), so a tier-1 group run through it would otherwise time out even when every task
    actually succeeded. Fall back to the per-tick 'executor task N' DynamicExecutor node status:
    `tree.generated` gives the id -> "executor task N" name mapping once; `tree.node_states_changed`
    gives id -> status per tick, with no name of its own (matches the real orchestrator's telemetry
    shape, verified against a captured events.jsonl)."""
    lines = [
        _tree_generated({"executor/root/7/0/13": "executor task 1",
                         "executor/root/7/1/13": "executor task 2"}, at="2026-08-23T00:00:00Z", seq=0),
        _node_states_changed({"executor/root/7/0/13": "RUNNING",
                              "executor/root/7/1/13": "RUNNING"}, at="2026-08-23T00:00:01Z", seq=1),
        _node_states_changed({"executor/root/7/0/13": "SUCCESS",
                              "executor/root/7/1/13": "FAILURE"}, at="2026-08-23T00:00:02Z", seq=2),
        _node_states_changed({"executor/root/7/0/13": "INVALID"}, at="2026-08-23T00:00:03Z", seq=3),
    ]
    path = tmp_path / "events.jsonl"
    path.write_text("\n".join(json.dumps(l) for l in lines) + "\n")

    results = parse_events(path)
    assert results[1].status == "succeeded"
    assert results[1].reason == "executor node SUCCESS"
    assert results[1].first_seen == "2026-08-23T00:00:01Z"
    assert results[1].finished_at == "2026-08-23T00:00:02Z"
    assert results[2].status == "failed"
    assert results[2].reason == "executor node FAILURE"


def test_parse_events_lets_a_real_task_finished_override_node_derived_status(tmp_path):
    lines = [
        _tree_generated({"executor/root/7/0/13": "executor task 1"}, at="2026-08-23T00:00:00Z", seq=0),
        _node_states_changed({"executor/root/7/0/13": "SUCCESS"}, at="2026-08-23T00:00:01Z", seq=1),
        _ev("task.finished", "gpsr-x/task-1", {"status": "failed", "reason": "postcondition unmet"},
           "2026-08-23T00:00:02Z", 2),
        _node_states_changed({"executor/root/7/0/13": "SUCCESS"}, at="2026-08-23T00:00:03Z", seq=3),
    ]
    path = tmp_path / "events.jsonl"
    path.write_text("\n".join(json.dumps(l) for l in lines) + "\n")

    results = parse_events(path)
    assert results[1].status == "failed"
    assert results[1].reason == "postcondition unmet"


def test_parse_events_ignores_node_states_from_a_non_executor_tree(tmp_path):
    """Only the "executor" tree's node states should be read as task-completion signals."""
    lines = [
        _tree_generated({"planner/root/0": "executor task 1"}, kind="planner", at="2026-08-23T00:00:00Z", seq=0),
        _node_states_changed({"planner/root/0": "SUCCESS"}, kind="planner", at="2026-08-23T00:00:01Z", seq=1),
    ]
    path = tmp_path / "events.jsonl"
    path.write_text("\n".join(json.dumps(l) for l in lines) + "\n")

    assert parse_events(path) == {}
