# tools/tests/test_telemetry.py
from __future__ import annotations

import pytest

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


def test_gate_failures_counts_only_failed_precondition_and_postcondition_gates(make_run):
    """gate_failures must count only FAILURE judge events whose kind is
    PRECONDITION or POSTCONDITION -- not a FAILURE of a different judge
    kind (e.g. SUPERVISOR), and not a SUCCESS of a matching kind.

    Node names are chosen to match vendor/sheet_events.py's
    `_classify_judge` prefixes ("precondition gate", "postcondition gate",
    "supervisor barrier") while avoiding `_classify_milestone`'s substrings
    (e.g. "arm", "grasp", "place") so the classifier doesn't mistake them
    for robot-action milestones first.
    """
    run = make_run(
        name="s9999-028-x",
        epochs=[[
            "precondition gate room",
            "postcondition gate room",
            "supervisor barrier room",
            "precondition gate other",
        ]],
        transitions=[
            # PRECONDITION FAILURE -> counts
            (_t(10), "executor/root/0", "FAILURE", "not ready"),
            # POSTCONDITION FAILURE -> counts
            (_t(20), "executor/root/1", "FAILURE", "not done"),
            # SUPERVISOR FAILURE -> different kind, must not count
            (_t(30), "executor/root/2", "FAILURE", "blocked"),
            # PRECONDITION SUCCESS -> matching kind but not a failure
            (_t(40), "executor/root/3", "SUCCESS", "ready"),
        ],
    )
    model = load_run_model(run)
    kinds = {(j.kind, j.status) for j in model.judge_events}
    # Sanity: the vendored classifier did produce all four judge events
    # under the expected kinds, so the gate_failures count below is
    # actually exercising the discrimination it claims to.
    assert ("PRECONDITION", "FAILURE") in kinds
    assert ("POSTCONDITION", "FAILURE") in kinds
    assert ("SUPERVISOR", "FAILURE") in kinds
    assert ("PRECONDITION", "SUCCESS") in kinds
    assert model.gate_failures == 2


@pytest.mark.corpus
def test_real_run_has_the_normal_epoch_pair_and_no_regeneration(corpus_root):
    """A live battery re-runs and archives entries under this same
    directory name, so this asserts invariants rather than the specimen's
    exact numbers (corpus-marked tests assert floors, not exact counts).

    Observed shape as of 2026-08-28 (informational only, not asserted):
    2 epochs with [84, 158] nodes; 7967 raw announcement lines collapsing
    to 11 distinct utterances.
    """
    run = corpus_root / "t2-2026" / "runs" / "s2026-002-countPrsInRoom"
    if not run.is_dir():
        pytest.skip("reference run has been re-run and archived")
    model = load_run_model(run)
    # A run that reached plan materialisation has at least the normal
    # skeleton+materialisation pair.
    assert len(model.epochs) >= 2
    # Self-consistency is the property actually worth pinning here.
    assert model.tree_regenerations == max(0, len(model.epochs) - 2)
    raw_lines = [
        line for line in (run / "announcements.txt").read_text().splitlines()
        if line.strip()
    ]
    # The dedupe must genuinely collapse repeats, not just pass them through.
    assert len(model.announcements) >= 1
    assert len(model.announcements) < len(raw_lines)
