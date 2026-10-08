"""Regression tests for person-directed communication physical truth."""
from __future__ import annotations

import json
from pathlib import Path

from bench.truth import (combine_verdicts, execution_integrity_audit,
                         physical_checks, plan_execution_audit)
from bench.tier2 import _write_announcements


def _run_dir(tmp_path: Path, *, approach: bool, speech_pose: dict | None = None) -> Path:
    run = tmp_path
    (run / "debug" / "trace").mkdir(parents=True)
    (run / "announcements.txt").write_text("It is Thursday.\n", encoding="utf-8")
    (run / "run_truth.json").write_text(json.dumps({
        "persons": {"kitchen": {"xyz": [1.85, -1.55, 0.0]}},
        "robot": {"last_xyz": [1.85, -1.55, 0.0], "last_yaw_deg": 90.0},
    }), encoding="utf-8")
    events = []
    def emit(kind: str, payload: dict, sequence: int) -> None:
        events.append({
            "event_type": kind,
            "task_id": "trace/task-1",
            "sequence": sequence,
            "payload": payload,
        })
    steps = [{"action": "goto", "params": {"location": "kitchen"}},
             {"action": "find_person", "params": {"descriptor": "waving person"}}]
    if approach:
        steps.append({"action": "approach_person", "params": {}})
    emit("plan.materialized", {"target_index": 0, "revision": 1, "steps": steps}, 1)
    for i, step in enumerate(steps):
        emit("step.finished", {
            "step_index": i, "action": step["action"], "params": step["params"],
            "outcome": "succeeded",
        }, i + 2)
    answer_steps = [{"action": "announce", "params": {"text": "It is Thursday."}}]
    answer_payload = {"step_index": 0, "action": "announce", "params": {"text": "It is Thursday."},
                     "outcome": "succeeded"}
    if speech_pose is not None:
        answer_payload["robot_pose"] = speech_pose
    emit("plan.materialized", {"target_index": 1, "revision": 1,
                                "steps": answer_steps}, 20)
    emit("step.finished", answer_payload, 21)
    (run / "debug" / "trace" / "events.jsonl").write_text(
        "\n".join(json.dumps(event) for event in events) + "\n", encoding="utf-8")
    return run


def _targets() -> list[dict]:
    return [
        {
            "id": "t1", "desc": "find the waving person in the kitchen",
            "location": "kitchen", "postconditions": ["person_found(waving person)"],
        },
        {
            "id": "t2", "desc": "tell the time to the waving person",
            "location": "kitchen", "preconditions": ["person_found(waving person)"],
            "postconditions": ["answered(time)"],
        },
    ]


def _gates() -> dict[tuple[int, str], str]:
    return {
        (0, "person_found(waving person)"): "VALID",
        (1, "answered(time)"): "VALID",
    }


def test_person_answer_without_approach_is_physical_failure(tmp_path: Path) -> None:
    checks = physical_checks(_run_dir(tmp_path, approach=False), _targets(),
                             gate_verdicts=_gates())
    answer = next(check for check in checks if check["fact"] == "answered(time)")
    assert answer["verdict"] == "FAIL"
    assert "approach_person" in answer["reason"]
    assert combine_verdicts("PASS", checks, False)[0:2] == ("FAIL", "FAIL")


def test_approach_without_speech_geometry_does_not_pass(tmp_path: Path) -> None:
    checks = physical_checks(_run_dir(tmp_path, approach=True), _targets(),
                             gate_verdicts=_gates())
    answer = next(check for check in checks if check["fact"] == "answered(time)")
    assert answer["verdict"] == "UNKNOWN"
    assert answer["critical"] is True
    assert combine_verdicts("PASS", checks, False)[0:2] == ("UNKNOWN", "UNKNOWN")


def test_person_answer_requires_near_facing_speech_pose(tmp_path: Path) -> None:
    pose = {"xyz": [1.85, -2.05, 0.0], "yaw_deg": 90.0}
    checks = physical_checks(_run_dir(tmp_path, approach=True, speech_pose=pose), _targets(),
                             gate_verdicts=_gates())
    answer = next(check for check in checks if check["fact"] == "answered(time)")
    assert answer["verdict"] == "PASS"
    assert answer["distance_m"] < 1.5
    assert answer["heading_error_deg"] < 55.0


def test_operator_report_after_person_independent_task_needs_no_approach(tmp_path: Path) -> None:
    targets = [
        {"id": "t1", "desc": "count objects", "location": "kitchen",
         "postconditions": ["counted(objects)"]},
        {"id": "t2", "desc": "report the result to the operator", "location": "kitchen",
         "postconditions": ["answered(result)"]},
    ]
    gates = {(1, "answered(result)"): "VALID"}
    checks = physical_checks(_run_dir(tmp_path, approach=False), targets,
                             gate_verdicts=gates)
    answer = next(check for check in checks if check["fact"] == "answered(result)")
    assert answer["verdict"] == "PASS"
    assert "requires_approach" not in answer


def test_plan_audit_rejects_answer_without_announce_or_approach(tmp_path: Path) -> None:
    """A successful llm_fallback is not an explicit person-directed answer."""
    run = tmp_path
    (run / "debug" / "trace").mkdir(parents=True)
    events = [
        {"event_type": "plan.materialized", "task_id": "trace/task-1",
         "payload": {"target_index": 0, "steps": [
             {"action": "goto", "params": {"location": "kitchen"}},
             {"action": "find_person", "params": {}}]}},
        {"event_type": "plan.materialized", "task_id": "trace/task-1",
         "payload": {"target_index": 1, "steps": [
             {"action": "llm_fallback", "params": {"question": "What day?"}}]}},
    ]
    (run / "debug" / "trace" / "events.jsonl").write_text(
        "\n".join(json.dumps(event) for event in events) + "\n")
    targets = [
        {"desc": "find a person in the kitchen", "postconditions": ["person_found(person)"]},
        {"desc": "say what day is tomorrow", "location": "kitchen",
         "postconditions": ["answered(what_day)"]},
    ]
    audit = plan_execution_audit(run, targets)
    assert audit["verdict"] == "FAIL"
    assert {issue["kind"] for issue in audit["issues"]} == {"missing_announce", "missing_approach"}


def test_execution_integrity_detects_startup_replay(tmp_path: Path) -> None:
    (tmp_path / "announcements.txt").write_text(
        "Hi, I am Tinker. I am ready for GPSR. Please open the door\n"
        "The door is open. Entering the arena\n"
        "Hi, I am Tinker. I am ready for GPSR. Please open the door\n"
        "I am planning, please wait\n"
        "I am planning, please wait\n")
    audit = execution_integrity_audit(tmp_path)
    assert audit["verdict"] == "FAIL"
    assert any(issue["kind"] == "startup_repeat" for issue in audit["issues"])
    assert any(issue["kind"] == "planning_repeat" for issue in audit["issues"])


def test_execution_integrity_uses_orchestrator_log_when_speech_capture_is_incomplete(
    tmp_path: Path,
) -> None:
    """A truncated speech artifact must not hide a root restart in the trace."""
    (tmp_path / "announcements.txt").write_text(
        "Hi, I am Tinker. I am ready for GPSR. Please open the door\n",
        encoding="utf-8",
    )
    (tmp_path / "orchestrator.log").write_text(
        "Finished announcing Hi, I am Tinker. I am ready for GPSR. Please open the door\n"
        "Finished announcing Hi, I am Tinker. I am ready for GPSR. Please open the door\n"
        "Finished announcing The door is open. Entering the arena\n"
        "Finished announcing The door is open. Entering the arena\n",
        encoding="utf-8",
    )
    audit = execution_integrity_audit(tmp_path)
    assert audit["verdict"] == "FAIL"
    ready = next(issue for issue in audit["issues"] if issue["kind"] == "startup_repeat")
    assert "orchestrator.log" in ready["sources"]
    assert ready["orchestrator_count"] == 2


def test_execution_integrity_ignores_stale_tree_snapshots(tmp_path: Path) -> None:
    """A finished BT node printed in every tick is one activation, not a replay."""
    snapshot = (
        "{-} GPSR orchestrator [*]\n"
        "    --> announce ready for gpsr [✓] -- Finished announcing Hi, I am Tinker. I am ready for GPSR. Please open the door.\n"
        "    --> announce entering arena [✓] -- Finished announcing The door is open. Entering the arena.\n"
        "    --> announce going [✓] -- Finished announcing Going to command point\n"
        "    --> announce planning task 1 [✓] -- Finished announcing I am planning, please wait.\n"
    )
    (tmp_path / "orchestrator.log").write_text(snapshot + snapshot, encoding="utf-8")
    events = [
        {"event_type": "tree.generated", "payload": {"nodes": [
            {"id": "ready", "name": "announce ready for gpsr"},
            {"id": "enter", "name": "announce entering arena"},
            {"id": "going", "name": "announce going"},
            {"id": "plan", "name": "announce planning task 1"},
        ]}},
    ]
    for seq, (node_id, name) in enumerate((
        ("ready", "announce ready for gpsr"),
        ("enter", "announce entering arena"),
        ("going", "announce going"),
        ("plan", "announce planning task 1"),
    ), 1):
        events.append({"event_type": "tree.node_states_changed", "sequence": seq,
                       "payload": {"nodes": [{"id": node_id, "name": name,
                                                  "status": "SUCCESS",
                                                  "feedback": "Finished announcing "
                                                  + ("Going to command point" if node_id == "going" else "")}]}})
    (tmp_path / "debug" / "trace").mkdir(parents=True)
    (tmp_path / "debug" / "trace" / "events.jsonl").write_text(
        "\n".join(json.dumps(event) for event in events) + "\n", encoding="utf-8")
    audit = execution_integrity_audit(tmp_path)
    assert audit["verdict"] == "PASS"
    assert audit["raw_orchestrator_counts"]["ready"] == 2
    assert audit["orchestrator_counts"]["ready"] == 1
    assert audit["orchestrator_counts"]["plan"] == 1


def test_execution_integrity_catches_repeated_real_speech_with_stale_snapshot(
    tmp_path: Path,
) -> None:
    """A replayed speech stream is a real startup replay even when telemetry is stale."""
    snapshot = (
        "{-} GPSR orchestrator [*]\n"
        "    --> announce ready for gpsr [✓] -- Finished announcing Hi, I am Tinker. I am ready for GPSR. Please open the door.\n"
        "    --> announce entering arena [✓] -- Finished announcing The door is open. Entering the arena.\n"
    )
    (tmp_path / "orchestrator.log").write_text(snapshot * 20, encoding="utf-8")
    (tmp_path / "announcements.txt").write_text(
        "Hi, I am Tinker. I am ready for GPSR. Please open the door\n"
        "The door is open. Entering the arena\n"
        "Hi, I am Tinker. I am ready for GPSR. Please open the door\n"
        "The door is open. Entering the arena\n"
        "I am planning, please wait for a while\n"
        "I am planning, please wait\n",
        encoding="utf-8",
    )
    events = [
        {"event_type": "tree.node_states_changed", "sequence": 1,
         "payload": {"nodes": [{"id": "ready", "name": "announce ready for gpsr",
                                  "status": "SUCCESS"}]}},
        {"event_type": "tree.node_states_changed", "sequence": 2,
         "payload": {"nodes": [{"id": "enter", "name": "announce entering arena",
                                  "status": "SUCCESS"}]}},
    ]
    (tmp_path / "debug" / "trace").mkdir(parents=True)
    (tmp_path / "debug" / "trace" / "events.jsonl").write_text(
        "\n".join(json.dumps(event) for event in events) + "\n", encoding="utf-8")

    audit = execution_integrity_audit(tmp_path)

    assert audit["verdict"] == "FAIL"
    repeats = [issue for issue in audit["issues"] if issue["kind"] == "startup_repeat"]
    assert {tuple(issue["sources"]) for issue in repeats} == {("announcements.txt",)}
    assert all(issue["orchestrator_count"] == 1 for issue in repeats)


def test_execution_integrity_detects_state_transition_replay_without_telemetry(
    tmp_path: Path,
) -> None:
    """The no-telemetry fallback still catches a real SUCCESS replay."""
    def block(status: str) -> str:
        return (
            "{-} GPSR orchestrator [*]\n"
            f"    --> announce ready for gpsr [{status}] -- Finished announcing Hi, I am Tinker. I am ready for GPSR. Please open the door.\n"
        )
    (tmp_path / "orchestrator.log").write_text(
        block("✓") + block("-") + block("✓"), encoding="utf-8")
    audit = execution_integrity_audit(tmp_path)
    assert audit["verdict"] == "FAIL"
    assert audit["snapshot_counts"]["ready"] == 2
    assert any(issue["kind"] == "startup_repeat" for issue in audit["issues"])


def test_write_announcements_uses_speech_events_without_snapshot_repetition(tmp_path: Path) -> None:
    """The viewer artifact contains one line per speech activation, not per BT tick."""
    run = tmp_path
    (run / "debug" / "trace").mkdir(parents=True)
    events = []
    for sequence, text in enumerate(("I am ready", "I counted 1"), 1):
        events.append({
            "event_type": "speech.finished",
            "sequence": sequence,
            "payload": {"outcome": "succeeded", "text": text},
        })
    (run / "debug" / "trace" / "events.jsonl").write_text(
        "\n".join(json.dumps(event) for event in events) + "\n", encoding="utf-8")
    (run / "orchestrator.log").write_text(
        "Finished announcing I am ready\n" * 50, encoding="utf-8")
    _write_announcements(run)
    assert (run / "announcements.txt").read_text(encoding="utf-8").splitlines() == [
        "I am ready", "I counted 1"
    ]
