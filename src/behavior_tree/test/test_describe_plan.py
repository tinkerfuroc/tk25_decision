"""``describe_plan`` turns a materialized action plan into a spoken summary.

Tinker used to go silent between "Starting task N now." and its first move —
the operator had no idea what the robot thought the task meant until it
either succeeded or failed. ``describe_plan`` is the pure text half of the
fix: given the ordered list of ``{action, params}`` step dicts the executor
already carries (the same shape that reaches ``materialise_params`` /
``BtNode_TargetPostconditionCheck``'s ``action_plan``), it renders one short
spoken sentence. The orchestrator speaks it right after "announce start task
N" via a new "announce plan N" node (see ``create_announce_task_plan`` /
``BtNode_BuildTaskPlanSpeech`` in orchestrator.py).

Run with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 (ROS pytest plugins break
collection).
"""

from __future__ import annotations

import sys
from pathlib import Path

SRC = Path(__file__).resolve().parents[1]
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from behavior_tree.GPSR.orchestrator import describe_plan  # noqa: E402


def test_empty_plan():
    assert describe_plan([]) == "My plan is ready."
    assert describe_plan(None) == "My plan is ready."


def test_s2026_002_plan_goto_count_goto_announce():
    # The real battery-run plan (s2026-002, 2026-08-28): go find + count
    # people pointing left in the bedroom, come back, and report.
    steps = [
        {"action": "goto", "params": {"location": "bedroom"}},
        {"action": "count", "params": {"object": "persons pointing to the left"}},
        {"action": "goto", "params": {"location": "start_position"}},
        {"action": "announce", "params": {}},
    ]
    assert describe_plan(steps) == (
        "My plan: go to the bedroom, then count the persons pointing to the left, "
        "then go to the start_position, then report the result."
    )


def test_unknown_action_humanizes_the_name():
    steps = [{"action": "scan_room", "params": {}}]
    assert describe_plan(steps) == "My plan: scan room."


def test_missing_param_drops_the_article_phrase():
    steps = [{"action": "goto", "params": {}}]
    assert describe_plan(steps) == "My plan: go to the destination."

    steps = [{"action": "goto", "params": None}]
    assert describe_plan(steps) == "My plan: go to the destination."


def test_all_named_action_templates():
    steps = [
        {"action": "find_person", "params": {"descriptor": "the man in red"}},
        {"action": "ask_person", "params": {}},
        {"action": "describe_person", "params": {}},
        {"action": "grasp", "params": {"object": "cup"}},
        {"action": "deliver", "params": {}},
        {"action": "follow", "params": {}},
        {"action": "guide", "params": {}},
        {"action": "record_position", "params": {"label": "sofa"}},
    ]
    assert describe_plan(steps) == (
        "My plan: find the man in red, then ask the person, then describe the person, "
        "then pick up the cup, then hand it over, then follow the person, "
        "then guide the person, then remember this spot."
    )


def test_find_person_falls_back_through_person_then_recipient():
    # descriptor is the real key materialise_params/describe_step read first;
    # person/recipient are the fallbacks, in that order.
    assert describe_plan([{"action": "find_person", "params": {"person": "Alice"}}]) == (
        "My plan: find Alice."
    )
    assert describe_plan([{"action": "find_person", "params": {"recipient": "Bob"}}]) == (
        "My plan: find Bob."
    )
    assert describe_plan([{"action": "find_person", "params": {
        "descriptor": "the man in red", "person": "Alice",
    }}]) == "My plan: find the man in red."


def test_guide_names_the_destination_when_given():
    assert describe_plan([{"action": "guide", "params": {"location": "kitchen"}}]) == (
        "My plan: guide the person to the kitchen."
    )
    assert describe_plan([{"action": "guide", "params": {}}]) == (
        "My plan: guide the person."
    )


def test_more_than_eight_steps_truncates_with_and_more():
    steps = [{"action": "goto", "params": {"location": f"room{i}"}} for i in range(10)]
    result = describe_plan(steps)
    assert result == (
        "My plan: go to the room0, then go to the room1, then go to the room2, "
        "then go to the room3, then go to the room4, then go to the room5, "
        "then go to the room6, then go to the room7, and more."
    )
    assert "room8" not in result
    assert "room9" not in result


def test_never_crashes_on_malformed_steps():
    steps = [
        "not a dict",
        {"no_action_key": True},
        {"action": None, "params": {}},
        {"action": "goto"},  # no params key at all
    ]
    # Must not raise — best-effort text only.
    result = describe_plan(steps)
    assert isinstance(result, str)
    assert result.startswith("My plan")


def test_build_task_plan_speech_missing_key_falls_back_to_ready():
    """The execute-phase build node reads SAVED_PLAN_PREFIX+slot directly
    (see create_announce_task_plan) — a slot whose plan was never saved
    (e.g. a fresh/unused slot number) must not crash the announce, just
    speak the same "My plan is ready." fallback describe_plan([]) gives."""
    import py_trees as pytree

    from behavior_tree.GPSR.orchestrator import BtNode_BuildTaskPlanSpeech

    node = BtNode_BuildTaskPlanSpeech(slot=999)
    node.setup()

    status = node.update()

    assert status == pytree.common.Status.SUCCESS
    assert node.feedback_message == "My plan is ready."
