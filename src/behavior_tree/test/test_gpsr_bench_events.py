import json

from behavior_tree.GPSR.bench.events import TaskResult, _DIAG_RE, parse_events, slot_of


def _ev(event_type, task_id=None, payload=None, at="2026-08-23T00:00:00Z", seq=0):
    return {"schema": "tinker.gpsr.telemetry", "schema_version": 1, "event_id": str(seq),
            "trajectory_id": "gpsr-x", "task_id": task_id, "trace_id": None, "source_id": "t",
            "sequence": seq, "occurred_at": at, "monotonic_ns": seq, "event_type": event_type,
            "phase": None, "parent_event_id": None, "causation_ids": [], "payload": payload or {}}


def test_parse_events_keeps_split_accepted_targets(tmp_path):
    # J14 (round-3 adversarial review, tier0 #7): split.accepted carries the
    # full target contracts so tier0/tier2 reports can audit them.
    targets = [
        {"id": "t0", "desc": "grab the coke", "object": "coke", "location": "kitchen",
         "depends_on": [], "preconditions": [], "postconditions": ["held(coke)"]},
        {"id": "t1", "desc": "deliver the coke to me", "object": "coke", "location": "",
         "depends_on": ["t0"], "preconditions": ["held(coke)"],
         "postconditions": ["delivered(coke,me)"]},
    ]
    lines = [
        _ev("split.accepted", "gpsr-x/task-2", {"slot": 2, "targets": targets}, seq=0),
    ]
    path = tmp_path / "events.jsonl"
    path.write_text("\n".join(json.dumps(l) for l in lines) + "\n")
    results = parse_events(path)
    assert results[2].split_targets == targets


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


def test_parse_events_keeps_last_gate_verified_invalid_reason_per_target(tmp_path):
    # I6 (round-3 adversarial review): gate.verified is a NEW event type
    # carrying the gate's per-fact verdict + rationale -- parse_events keeps
    # the LAST INVALID/UNKNOWN one per target_index so tier2 diagnostics can
    # show WHY a gate failed, not just that it failed.
    lines = [
        _ev("gate.verified", "gpsr-x/task-0",
            {"slot": 0, "target_index": 0, "phase": "postcondition",
             "fact": "counted(drinks)", "verdict": "VALID", "confidence": 1.0,
             "reason": "count artifact target provenance matches"}, seq=0),
        _ev("gate.verified", "gpsr-x/task-0",
            {"slot": 0, "target_index": 1, "phase": "precondition",
             "fact": "held(spam)", "verdict": "UNKNOWN", "confidence": 1.0,
             "reason": "UNKNOWN: stronger verifier not installed and no sufficient artifact"},
            seq=1),
        _ev("gate.verified", "gpsr-x/task-0",
            {"slot": 0, "target_index": 1, "phase": "postcondition",
             "fact": "counted(drinks)", "verdict": "INVALID", "confidence": 1.0,
             "reason": "count artifact target provenance mismatch"}, seq=2),
    ]
    path = tmp_path / "events.jsonl"
    path.write_text("\n".join(json.dumps(l) for l in lines) + "\n")

    results = parse_events(path)
    # target 0's only event was VALID -- never recorded.
    assert 0 not in results[0].gate_reasons
    # target 1's LAST invalid/unknown reason wins.
    assert results[0].gate_reasons[1] == (
        "postcondition:counted(drinks) INVALID (count artifact target provenance mismatch)"
    )


def test_parse_events_pops_gate_reason_on_a_later_valid_l4(tmp_path):
    # L-4 (round-3 fix review): a target that FAILED a gate check but then
    # RECOVERED (a later gate.verified for the same target/fact is VALID)
    # must not still show the stale failure reason in diagnostics.
    lines = [
        _ev("gate.verified", "gpsr-x/task-0",
            {"slot": 0, "target_index": 1, "phase": "postcondition",
             "fact": "counted(drinks)", "verdict": "INVALID", "confidence": 1.0,
             "reason": "count artifact target provenance mismatch"}, seq=0),
        _ev("gate.verified", "gpsr-x/task-0",
            {"slot": 0, "target_index": 1, "phase": "postcondition",
             "fact": "counted(drinks)", "verdict": "VALID", "confidence": 1.0,
             "reason": "count artifact target provenance matches"}, seq=1),
    ]
    path = tmp_path / "events.jsonl"
    path.write_text("\n".join(json.dumps(l) for l in lines) + "\n")

    results = parse_events(path)
    assert 1 not in results[0].gate_reasons


def test_parse_events_keeps_invalid_reason_when_a_different_facts_valid_l4m1(tmp_path):
    # L-4-M1 (round-3 fix2 review): since J3, BtNode_TargetPostconditionCheck
    # verifies EVERY fact and emits one gate.verified per fact ("no more
    # fail-fast") -- a target with sources [answered(x) INVALID, counted(y)
    # VALID] (declared postconditions first, then deferred preconditions --
    # exactly the J8 report target's shape) must not have the INVALID
    # answered() reason popped just because an UNRELATED fact on the same
    # target went VALID in the same tick. Only a VALID for the SAME fact
    # may pop it.
    lines = [
        _ev("gate.verified", "gpsr-x/task-0",
            {"slot": 0, "target_index": 1, "phase": "postcondition",
             "fact": "answered(what colour is the box)", "verdict": "INVALID",
             "confidence": 1.0, "reason": "answer artifact question provenance mismatch"}, seq=0),
        _ev("gate.verified", "gpsr-x/task-0",
            {"slot": 0, "target_index": 1, "phase": "postcondition",
             "fact": "counted(drinks)", "verdict": "VALID", "confidence": 1.0,
             "reason": "count artifact target provenance matches"}, seq=1),
    ]
    path = tmp_path / "events.jsonl"
    path.write_text("\n".join(json.dumps(l) for l in lines) + "\n")

    results = parse_events(path)
    assert results[0].gate_reasons[1] == (
        "postcondition:answered(what colour is the box) INVALID "
        "(answer artifact question provenance mismatch)"
    )


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


def test_parse_events_captures_the_last_precondition_unmet_feedback_from_a_nested_node(tmp_path):
    """F6: the node-derived fallback status is the only T1 evidence (no step.finished from the
    production two-layer flow), so its reason should be the real diagnostic, not just a bare
    'executor node FAILURE'. A real failure's feedback lives on a leaf several levels under its
    "executor task N" ancestor (verified against a captured t1-42 events.jsonl), not on the
    top-level node itself, so this must match by id-prefix."""
    lines = [
        _tree_generated({"executor/root/7/0/13": "executor task 1"}, at="2026-08-23T00:00:00Z", seq=0),
        _node_states_changed({"executor/root/7/0/13": "RUNNING"}, at="2026-08-23T00:00:01Z", seq=1),
        _ev("tree.node_states_changed", None, {"tree_kind": "executor", "nodes": [
            {"id": "executor/root/7/0/13/0/0", "node_id": "executor/root/7/0/13/0/0",
             "status": "FAILURE", "feedback": "precondition unmet: at_robot(kitchen) (INVALID)"}]},
            at="2026-08-23T00:00:02Z", seq=2),
        _node_states_changed({"executor/root/7/0/13": "FAILURE"}, at="2026-08-23T00:00:03Z", seq=3),
    ]
    path = tmp_path / "events.jsonl"
    path.write_text("\n".join(json.dumps(l) for l in lines) + "\n")

    results = parse_events(path)
    assert results[1].status == "failed"
    assert results[1].reason == "precondition unmet: at_robot(kitchen) (INVALID)"


# ---------------------------------------------------------------------------
# H3 (round-2 review, run 004): _DIAG_RE must also recognise the reasons the
# H1/E2 escape ladder and the H1 search_object sweep produce -- without this,
# a genuine "unrecoverable" or "swept N of M spots" diagnostic never
# overwrites diag_by_slot, so a node-derived failure's reason falls back to
# the bare, uninformative "executor node FAILURE".
# ---------------------------------------------------------------------------

def test_diag_re_matches_unrecoverable_reason():
    text = "unrecoverable: no untried establisher for ['object_seen(pudding_box)']"
    assert _DIAG_RE.search(text)


def test_diag_re_matches_search_object_swept_reason():
    text = "search_object: swept 1 of 6 spots at bedroom, nothing found"
    assert _DIAG_RE.search(text)


def test_parse_events_captures_an_unrecoverable_feedback_from_a_nested_node(tmp_path):
    lines = [
        _tree_generated({"executor/root/7/0/13": "executor task 1"}, at="2026-08-23T00:00:00Z", seq=0),
        _node_states_changed({"executor/root/7/0/13": "RUNNING"}, at="2026-08-23T00:00:01Z", seq=1),
        _ev("tree.node_states_changed", None, {"tree_kind": "executor", "nodes": [
            {"id": "executor/root/7/0/13/0/0", "node_id": "executor/root/7/0/13/0/0",
             "status": "FAILURE",
             "feedback": "unrecoverable: no untried establisher for ['object_seen(pudding_box)']"}]},
            at="2026-08-23T00:00:02Z", seq=2),
        _node_states_changed({"executor/root/7/0/13": "FAILURE"}, at="2026-08-23T00:00:03Z", seq=3),
    ]
    path = tmp_path / "events.jsonl"
    path.write_text("\n".join(json.dumps(l) for l in lines) + "\n")

    results = parse_events(path)
    assert results[1].status == "failed"
    assert results[1].reason == "unrecoverable: no untried establisher for ['object_seen(pudding_box)']"


def test_parse_events_captures_a_search_object_swept_feedback_from_a_nested_node(tmp_path):
    lines = [
        _tree_generated({"executor/root/7/0/13": "executor task 1"}, at="2026-08-23T00:00:00Z", seq=0),
        _node_states_changed({"executor/root/7/0/13": "RUNNING"}, at="2026-08-23T00:00:01Z", seq=1),
        _ev("tree.node_states_changed", None, {"tree_kind": "executor", "nodes": [
            {"id": "executor/root/7/0/13/0/0", "node_id": "executor/root/7/0/13/0/0",
             "status": "FAILURE",
             "feedback": "search_object: swept 1 of 6 spots at bedroom, nothing found"}]},
            at="2026-08-23T00:00:02Z", seq=2),
        _node_states_changed({"executor/root/7/0/13": "FAILURE"}, at="2026-08-23T00:00:03Z", seq=3),
    ]
    path = tmp_path / "events.jsonl"
    path.write_text("\n".join(json.dumps(l) for l in lines) + "\n")

    results = parse_events(path)
    assert results[1].status == "failed"
    assert results[1].reason == "search_object: swept 1 of 6 spots at bedroom, nothing found"


def test_parse_events_ignores_node_states_from_a_non_executor_tree(tmp_path):
    """Only the "executor" tree's node states should be read as task-completion signals."""
    lines = [
        _tree_generated({"planner/root/0": "executor task 1"}, kind="planner", at="2026-08-23T00:00:00Z", seq=0),
        _node_states_changed({"planner/root/0": "SUCCESS"}, kind="planner", at="2026-08-23T00:00:01Z", seq=1),
    ]
    path = tmp_path / "events.jsonl"
    path.write_text("\n".join(json.dumps(l) for l in lines) + "\n")

    assert parse_events(path) == {}
