"""Planner semantic regressions that must hold in every benchmark environment."""

from __future__ import annotations

try:
    from .planner import (
        _append_report_target_if_needed,
        _drop_dangling_goto_before_self_nav,
        _ensure_operator_delivery_return_leg,
        _kept_indices,
        _repair_operator_report_location,
        GPSRPlanner,
    )
except ImportError:  # pragma: no cover - direct invocation from this directory
    from planner import (
        _append_report_target_if_needed,
        _drop_dangling_goto_before_self_nav,
        _ensure_operator_delivery_return_leg,
        _kept_indices,
        _repair_operator_report_location,
        GPSRPlanner,
    )

try:
    from .planner_validators import validate_plan
except ImportError:  # pragma: no cover - direct invocation from this directory
    from planner_validators import validate_plan


def test_report_target_stays_at_operator_when_benchmark_override_is_set(monkeypatch):
    """A simulator workaround must not remove the operator return leg."""
    monkeypatch.setenv("GPSR_REPORT_AT_CURRENT", "1")
    targets = [{
        "id": "t0",
        "desc": "count how many items are on the side table",
        "object": "items",
        "location": "side_table",
        "depends_on": [],
        "preconditions": [],
        "postconditions": ["counted(items)"],
    }]

    result = _append_report_target_if_needed(targets)
    report = result[-1]

    assert report["location"] == "start_position"
    assert report["postconditions"] == [
        "at_robot(start_position)",
        "answered(the requested information)",
    ]


def test_explicit_operator_report_is_pinned_to_start_position():
    """An LLM supplied report target must include the return leg contract."""
    targets = [{
        "id": "count", "desc": "count waving persons in the laundry_room",
        "object": "waving persons", "location": "laundry_room",
        "depends_on": [], "preconditions": [],
        "postconditions": ["counted(waving_persons)"],
    }, {
        "id": "report", "desc": "report the result to the operator",
        "object": "", "location": "laundry_room", "depends_on": ["count"],
        "preconditions": ["counted(waving_persons)"],
        "postconditions": ["answered(the count)"],
    }]

    result = _repair_operator_report_location(targets)
    report = result[1]
    assert report["location"] == "start_position"
    assert report["postconditions"] == [
        "at_robot(start_position)", "answered(the count)",
    ]


def test_combined_count_and_operator_report_stays_at_task_location():
    """A one-target count/report clause must gather before returning."""
    targets = [{
        "id": "count_and_report",
        "desc": "tell me how many waving persons are in the laundry_room",
        "object": "waving persons", "location": "laundry_room",
        "depends_on": [], "preconditions": [],
        "postconditions": ["answered(how many waving persons are in the laundry_room)"],
    }]

    result = _repair_operator_report_location(targets)

    assert result[0]["location"] == "laundry_room"
    assert result[0]["postconditions"] == [
        "answered(how many waving persons are in the laundry_room)",
    ]


def test_person_directed_report_keeps_approach_pose():
    """Reporting an observation to Alex is speech at Alex, not the operator."""
    targets = [{
        "id": "report", "desc": "report the colour to Alex",
        "object": "", "location": "living_room", "depends_on": [],
        "preconditions": [], "postconditions": ["answered(the colour)"],
    }]

    result = _repair_operator_report_location(targets)
    assert result == targets
    assert result[0]["location"] == "living_room"
    assert result[0]["postconditions"] == ["answered(the colour)"]


def test_split_command_repairs_explicit_operator_report(monkeypatch):
    """The repair runs in the real top-layer split path."""
    planner = GPSRPlanner(max_attempts=1)
    planner._offline_mock = False
    monkeypatch.setattr(planner, "_new_client", lambda: object())
    split = {
        "targets": [{
            "id": "count", "desc": "count waving persons in the laundry_room",
            "object": "waving persons", "location": "laundry_room",
            "depends_on": [], "preconditions": [],
            "postconditions": ["counted(waving_persons)"],
        }, {
            "id": "report", "desc": "report the result to the operator",
            "object": "", "location": "laundry_room", "depends_on": ["count"],
            "preconditions": ["counted(waving_persons)"],
            "postconditions": ["answered(the count)"],
        }],
    }

    monkeypatch.setattr(
        "behavior_tree.GPSR.planner._call_llm",
        lambda *args, **kwargs: (split, None),
    )
    result = planner.split_command(
        "tell me how many waving persons are in the laundry_room", slot=0,
    )

    assert result[1]["location"] == "start_position"
    assert result[1]["postconditions"] == [
        "at_robot(start_position)", "answered(the count)",
    ]


def test_operator_delivery_keeps_explicit_return_leg_and_validates():
    """A bring-me plan must preserve the return goto before delivery."""
    plan = [
        {"action": "goto", "params": {"location": "shelf"}},
        {"action": "grasp", "params": {"object": "bowl", "from_shelf": True}},
        {"action": "goto", "params": {"location": "start_position"}},
        {"action": "deliver", "params": {
            "object": "bowl", "recipient": "me",
            "recipient_location": "start_position",
        }},
    ]

    cleaned, dropped = _drop_dangling_goto_before_self_nav(plan)
    assert cleaned == plan
    assert dropped == []

    ok, reason = validate_plan(
        plan,
        "give me a bowl from the shelf",
        {"goto", "grasp", "deliver"},
        known_locations={"shelf", "start_position"},
        postconditions=["delivered(bowl,me)", "at_robot(start_position)"],
    )
    assert ok, reason


def test_goto_before_non_operator_delivery_remains_rejected():
    """The explicit-return exception must not weaken ordinary self-nav checks."""
    plan = [
        {"action": "goto", "params": {"location": "living_room"}},
        {"action": "deliver", "params": {
            "object": "bowl", "recipient": "Alex",
            "recipient_location": "living_room",
        }},
    ]
    ok, reason = validate_plan(
        plan,
        "deliver the bowl to Alex in the living room",
        {"goto", "deliver"},
        known_locations={"living_room"},
    )
    assert not ok
    assert reason and "redundant" in reason


def test_operator_delivery_helper_inserts_return_and_fills_destination():
    plan = [
        {"action": "goto", "params": {"location": "shelf"}},
        {"action": "grasp", "params": {"object": "bowl"}},
        {"action": "deliver", "params": {"object": "bowl", "recipient": "me"}},
    ]

    result = _ensure_operator_delivery_return_leg(
        plan, command="bring me the bowl from the shelf",
    )

    assert [step["action"] for step in result] == ["goto", "grasp", "goto", "deliver"]
    assert result[2]["params"] == {"location": "start_position"}
    assert result[3]["params"]["recipient_location"] == "start_position"


def test_named_delivery_recipient_ignores_unrelated_operator_report():
    """An earlier tell-me target must not reroute a later delivery to Alex."""
    for destination in (None, "living_room"):
        params = {"object": "apple", "recipient": "Alex"}
        if destination is not None:
            params["recipient_location"] = destination
        expected_params = dict(params)
        plan = [{"action": "deliver", "params": params}]

        result = _ensure_operator_delivery_return_leg(
            plan,
            target={"desc": "deliver the apple to Alex"},
            command="tell me the time then deliver the apple to Alex",
        )

        assert [step["action"] for step in result] == ["deliver"]
        assert result[0]["params"] == expected_params


def test_missing_delivery_recipient_still_infers_operator_return():
    """Operator wording remains useful when a delivery omitted its recipient."""
    for context in (
        {"target": {"desc": "deliver the bowl to the operator"}},
        {"command": "deliver the bowl to me"},
    ):
        result = _ensure_operator_delivery_return_leg(
            [{"action": "deliver", "params": {"object": "bowl"}}],
            **context,
        )

        assert [step["action"] for step in result] == ["goto", "deliver"]
        assert result[0]["params"] == {"location": "start_position"}
        assert result[1]["params"]["recipient_location"] == "start_position"


def test_kept_indices_marks_planner_insertions_without_crashing():
    """Synthetic return navigation has no raw LLM step index."""
    raw = [{"action": "deliver", "params": {"recipient": "me"}}]
    repaired = _ensure_operator_delivery_return_leg(raw)

    assert _kept_indices(raw, repaired) == [-1, 0]


def test_goto_before_operator_delivery_without_matching_destination_is_dropped():
    missing_destination = [
        {"action": "goto", "params": {"location": "start_position"}},
        {"action": "deliver", "params": {"object": "bowl", "recipient": "me"}},
    ]
    mismatched_destination = [
        {"action": "goto", "params": {"location": "living_room"}},
        {"action": "deliver", "params": {
            "object": "bowl", "recipient": "me", "recipient_location": "start_position",
        }},
    ]

    for plan in (missing_destination, mismatched_destination):
        cleaned, dropped = _drop_dangling_goto_before_self_nav(plan)
        assert [step["action"] for step in cleaned] == ["deliver"]
        assert dropped == ["contract:goto->dangling"]


def test_count_report_plan_keeps_return_and_announce():
    plan = [
        {"action": "goto", "params": {"location": "laundry_room"}},
        {"action": "count", "params": {"object": "waving persons", "location": "laundry_room"}},
        {"action": "goto", "params": {"location": "start_position"}},
        {"action": "announce", "params": {}},
    ]
    ok, reason = validate_plan(
        plan,
        "tell me how many waving persons are in the laundry_room",
        {"goto", "count", "announce"},
        known_locations={"laundry_room", "start_position"},
        postconditions=["at_robot(start_position)", "answered(the count)"],
    )
    assert ok, reason
