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
