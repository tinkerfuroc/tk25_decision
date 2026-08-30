from __future__ import annotations

import threading

import pytest

from behavior_tree.GPSR import planner as planner_module
from behavior_tree.GPSR.planner import (
    GPSRPlanner,
    TOP_LAYER_SYSTEM_PROMPT,
    _append_report_target_if_needed,
    _build_lower_layer_user_prompt,
    _log_split_acceptance,
    _merge_transport_targets,
    _normalise_targets,
    _reject_person_object_handling,
    _self_navigates_toward,
)
from behavior_tree.GPSR.planner_validators import validate_dag


def test_normalise_targets_emits_canonical_defaults_and_stable_ids():
    targets = _normalise_targets(["  find cup  ", {"desc": "place", "object": "cup"}])
    assert targets == [
        {"id": "t0", "desc": "find cup", "object": "", "location": "", "depends_on": [], "preconditions": [], "postconditions": []},
        {"id": "t1", "desc": "place", "object": "cup", "location": "", "depends_on": [], "preconditions": [], "postconditions": []},
    ]


def test_normalise_targets_maps_legacy_positional_dependencies_to_ids():
    targets = _normalise_targets([
        {"id": "acquire", "desc": "acquire"},
        {"id": "move", "desc": "move", "depends_on": 0},
        {"desc": "place", "depends_on": [0, " move ", 0]},
    ])
    assert [t["id"] for t in targets] == ["acquire", "move", "t2"]
    assert targets[1]["depends_on"] == ["acquire"]
    assert targets[2]["depends_on"] == ["acquire", "move"]


def test_normalise_targets_preserves_duplicate_explicit_ids_for_dag_rejection():
    targets = _normalise_targets([
        {"id": "same", "desc": "one"},
        {"id": "same", "desc": "two"},
    ])
    assert [t["id"] for t in targets] == ["same", "same"]
    assert validate_dag(targets)[0] is False


def test_normalise_targets_invalid_condition_shape_is_rejected_by_contract():
    target = _normalise_targets([{"desc": "one", "postconditions": "held(cup)"}])
    assert target[0]["postconditions"] == "held(cup)"
    assert planner_module._validate_target_contract(target)[0] is False


def test_normalise_targets_invalid_dependency_remains_rejected_by_dag():
    targets = _normalise_targets([
        {"desc": "one"},
        {"desc": "two", "depends_on": 99},
    ])
    assert validate_dag(targets)[0] is False


@pytest.mark.parametrize("raw", [123, "target", {"desc": "target"}, ("target",)])
def test_normalise_targets_rejects_non_list_collections(raw):
    assert _normalise_targets(raw) == []


def test_normalise_targets_preserves_malformed_dependencies_and_conditions():
    target = _normalise_targets([{
        "desc": "target",
        "depends_on": [" ", "-1", " ", 7],
        "preconditions": ["  " , 42],
        "postconditions": ["\t", None],
    }])[0]
    assert target["depends_on"] == ["", "-1", 7]
    assert target["preconditions"] == ["", 42]
    assert target["postconditions"] == ["", None]
    assert planner_module._validate_target_contract([target])[0] is False


def test_normalise_targets_strips_explicit_ids_before_dag_validation():
    targets = _normalise_targets([
        {"id": " same ", "desc": "one"},
        {"id": "same", "desc": "two"},
    ])
    assert [target["id"] for target in targets] == ["same", "same"]
    assert "duplicate" in (validate_dag(targets)[1] or "")


def test_normalise_targets_drops_preconditions_no_earlier_target_establishes():
    targets = _normalise_targets([
        {
            "desc": "Get a spam from the laundry_desk",
            "preconditions": ["at_robot(laundry_desk)"],
            "postconditions": ["held(spam)"],
        },
        {
            "desc": "Deliver the spam to me",
            "preconditions": ["held(spam)", "at_robot(start_position)"],
            "postconditions": ["delivered(spam,me)"],
        },
    ])
    assert targets[0]["preconditions"] == []
    assert targets[1]["preconditions"] == ["held(spam)"]


def test_retracting_addition_guards_zero_arity_facts_m6(monkeypatch):
    # M-6 (round-3 fix review): `_retracting_addition` indexes
    # `fact.args[0]`/`removed.args[0]` for held/placed/delivered -- safe
    # today only because `parse_fact` enforces arity for every predicate in
    # its closed vocabulary, but this is called on raw postcondition text
    # BEFORE `_validate_target_contract` runs, so it must never crash on a
    # malformed/zero-arity fact even if `parse_fact`'s vocabulary ever
    # widens. Monkeypatch parse_fact to simulate a parseable-yet-zero-arity
    # `held`/`placed` fact and confirm no IndexError.
    from behavior_tree.GPSR.validators import Fact

    def fake_parse_fact(text):
        if text == "held()":
            return Fact("held", (), text), None
        if text == "placed(x,t)":
            return Fact("placed", ("x", "t"), text), None
        return None, "unparseable"

    monkeypatch.setattr(planner_module, "parse_fact", fake_parse_fact)
    # removed fact has no args -- must not IndexError on removed.args[0]
    assert planner_module._retracting_addition("held()", ["placed(x,t)"]) is None
    # addition has no args -- must not IndexError on fact.args[0]
    assert planner_module._retracting_addition("placed(x,t)", ["held()"]) is None


# ---------------------------------------------------------------------------
# J8: counted()/answered() results must be reported to the operator
# ---------------------------------------------------------------------------

def test_j8_two_count_targets_get_a_report_target_e05():
    # edge corpus e05: "count how many mugs are on the shelf and then on
    # the side table" -- both targets count, neither reports.
    targets = [
        {"id": "t0", "desc": "count how many mugs are on the shelf",
         "object": "", "location": "shelf", "depends_on": [],
         "preconditions": [], "postconditions": ["counted(mugs)"]},
        {"id": "t1", "desc": "count how many mugs are on the side table",
         "object": "", "location": "side_table", "depends_on": [],
         "preconditions": [], "postconditions": ["counted(mugs)"]},
    ]
    result = _append_report_target_if_needed(targets)
    assert len(result) == 3
    report = result[2]
    assert report["id"] == "t2"
    assert report["desc"] == "report the result to the operator"
    assert report["depends_on"] == ["t0", "t1"]
    # L-3 (round-3 fix review): both targets declare the identical
    # counted(mugs) fact -- deduped, preserving first-seen order, so the
    # gate does not check the same fact twice.
    assert report["preconditions"] == ["counted(mugs)"]
    assert report["postconditions"] == [
        "at_robot(start_position)", "answered(the requested information)",
    ]
    # the appended target's contract is well-formed on its own.
    ok, reason = planner_module._validate_target_contract(result)
    assert ok, reason


def test_j8_dedup_preserves_first_seen_order_l3():
    # L-3 (round-3 fix review): a non-adjacent repeat of an EARLIER fact
    # must not move it -- the dedup keeps first-seen order, not sorted or
    # last-seen order.
    targets = [
        {"id": "t0", "desc": "count the mugs on the shelf", "object": "",
         "location": "shelf", "depends_on": [], "preconditions": [],
         "postconditions": ["counted(mugs)"]},
        {"id": "t1", "desc": "count the bowls on the shelf", "object": "",
         "location": "shelf", "depends_on": [], "preconditions": [],
         "postconditions": ["counted(bowls)"]},
        {"id": "t2", "desc": "count the mugs on the side table", "object": "",
         "location": "side_table", "depends_on": [], "preconditions": [],
         "postconditions": ["counted(mugs)"]},
    ]
    result = _append_report_target_if_needed(targets)
    report = result[-1]
    assert report["preconditions"] == ["counted(mugs)", "counted(bowls)"]
    assert report["depends_on"] == ["t0", "t1", "t2"]


def test_j8_single_count_target_gets_a_report_target_e06():
    # edge corpus e06: "go to the laundry desk and count the mugs on the
    # side table" -- one job, one target, no report.
    targets = [
        {"id": "t0", "desc": "go to the laundry desk and count the mugs on the side table",
         "object": "mugs", "location": "side_table", "depends_on": [],
         "preconditions": [], "postconditions": ["counted(mugs)"]},
    ]
    result = _append_report_target_if_needed(targets)
    assert len(result) == 2
    assert result[1]["depends_on"] == ["t0"]
    assert result[1]["preconditions"] == ["counted(mugs)"]


def test_j8_command_that_already_tells_gets_no_report_target():
    targets = [
        {"id": "t0", "desc": "count the mugs on the shelf and tell me the number",
         "object": "", "location": "shelf", "depends_on": [],
         "preconditions": [], "postconditions": ["counted(mugs)"]},
    ]
    result = _append_report_target_if_needed(targets)
    assert result == targets


def test_j8_no_counted_or_answered_facts_gets_no_report_target():
    targets = [
        {"id": "t0", "desc": "go to the kitchen", "object": "", "location": "kitchen",
         "depends_on": [], "preconditions": [], "postconditions": ["at_robot(kitchen)"]},
    ]
    result = _append_report_target_if_needed(targets)
    assert result == targets


def test_j8_answered_with_no_speech_verb_or_addressee_gets_a_report_target():
    # M-1 (round-3 fix review): an answered() whose target desc neither
    # addresses a person nor uses a speech verb is exactly the "gathered but
    # never spoken" case J8 exists for (e.g. "find out what day it is").
    targets = [
        {"id": "t0", "desc": "find out what day it is",
         "object": "", "location": "", "depends_on": [],
         "preconditions": [], "postconditions": ["answered(the day)"]},
    ]
    result = _append_report_target_if_needed(targets)
    assert len(result) == 2
    assert result[1]["depends_on"] == ["t0"]
    assert result[1]["preconditions"] == ["answered(the day)"]


def test_j8_meet_and_say_your_teams_name_gets_no_report_target():
    # M-1 (round-3 fix review): negative test from the ruling -- "meet Alex
    # in the laundry_room and say your teams name" encodes the speech act as
    # answered(...) (that's how the split represents "say ..."), but "say"
    # IS the spoken delivery -- appending a synthetic report target would
    # send the robot back to start_position to re-announce to nobody.
    targets = [
        {"id": "t0", "desc": "meet Alex in the laundry_room",
         "object": "", "location": "laundry_room", "depends_on": [],
         "preconditions": [], "postconditions": ["person_found(alex)"]},
        {"id": "t1", "desc": "say your teams name",
         "object": "", "location": "", "depends_on": ["t0"],
         "preconditions": ["person_found(alex)"], "postconditions": ["answered(teams name)"]},
    ]
    result = _append_report_target_if_needed(targets)
    assert result == targets


def test_j8_answered_addressed_to_a_person_by_preposition_gets_no_report_target():
    # M-1: the desc need not use one of the speech verbs -- addressing the
    # answered() at a person by "to/for (the) person/someone/guest/<name>"
    # is on its own evidence the answer was already spoken to them.
    targets = [
        {"id": "t0", "desc": "reply to the person raising their right arm",
         "object": "", "location": "", "depends_on": [],
         "preconditions": [], "postconditions": ["answered(the requested information)"]},
    ]
    result = _append_report_target_if_needed(targets)
    assert result == targets


# ---------------------------------------------------------------------------
# J9: persons are never object-handled (held/placed/delivered)
# ---------------------------------------------------------------------------

def test_j9_person_object_handling_split_is_rejected():
    # edge corpus guideClothPrs: the split treated a person like an object.
    targets = [
        {"id": "t0", "desc": "Take the person wearing a gray jacket from the sofa",
         "object": "", "location": "sofa", "depends_on": [],
         "preconditions": [], "postconditions": ["held(person wearing a gray jacket)"]},
        {"id": "t1", "desc": "Take the person wearing a gray jacket to the side_table",
         "object": "", "location": "side_table", "depends_on": ["t0"],
         "preconditions": ["held(person wearing a gray jacket)"],
         "postconditions": ["at_robot(side_table)"]},
        {"id": "t2", "desc": "Place the person wearing a gray jacket at the side_table",
         "object": "", "location": "side_table", "depends_on": ["t1"],
         "preconditions": ["held(person wearing a gray jacket)"],
         "postconditions": ["placed(person wearing a gray jacket,side_table)"]},
    ]
    reason = _reject_person_object_handling(targets)
    assert reason is not None
    assert "persons are guided, not grasped" in reason
    assert "find_person + guide" in reason


def test_j9_normal_guide_split_passes():
    targets = [
        {"id": "t0", "desc": "find the person wearing a gray jacket at the sofa",
         "object": "", "location": "sofa", "depends_on": [],
         "preconditions": [], "postconditions": ["person_found(person wearing a gray jacket)"]},
        {"id": "t1", "desc": "guide the person wearing a gray jacket to the side_table",
         "object": "", "location": "side_table", "depends_on": ["t0"],
         "preconditions": ["person_found(person wearing a gray jacket)"],
         "postconditions": ["at_robot(side_table)"]},
    ]
    assert _reject_person_object_handling(targets) is None


def test_j9_ordinary_object_handling_target_passes():
    targets = [
        {"id": "t0", "desc": "grab the coke from the kitchen", "object": "coke",
         "location": "kitchen", "depends_on": [], "preconditions": [],
         "postconditions": ["held(coke)"]},
    ]
    assert _reject_person_object_handling(targets) is None


def test_j9_deliver_to_a_person_recipient_passes():
    # H-1 (round-3 fix review): the desc names a person because it names the
    # RECIPIENT, but the HANDLED thing (fact.args[0], and target["object"])
    # is spam -- this is an ordinary deliver-to-person target, not a person
    # being object-handled, and must not be rejected.
    targets = [
        {"id": "t0", "desc": "give the spam to the person raising their left arm",
         "object": "spam", "location": "kitchen", "depends_on": [],
         "preconditions": [],
         "postconditions": ["delivered(spam,person raising their left arm)"]},
    ]
    assert _reject_person_object_handling(targets) is None


# ---------------------------------------------------------------------------
# X1 (round-3 fix review, source-pinned tier0 sweep): held/placed/delivered
# are for PHYSICAL OBJECTS only -- "tell <info> to <person>" must use
# answered(<what>), never delivered(<information>, <person>).
# ---------------------------------------------------------------------------

def test_x1_delivered_country_to_person_is_rejected(monkeypatch):
    from behavior_tree.GPSR import orchestrator as orch
    monkeypatch.setattr(orch, "KNOWN_OBJECT_NAMES", set())
    monkeypatch.setattr(planner_module, "KNOWN_OBJECT_NAMES", set())
    targets = [
        {"id": "t0", "desc": "tell the person raising their left arm the country",
         "object": "", "location": "", "depends_on": [], "preconditions": [],
         "postconditions": ["delivered(country,person_raising_their_left_arm)"]},
    ]
    ok, reason = planner_module._validate_target_contract(targets)
    assert ok is False
    assert "delivered/placed/held are for physical objects" in reason
    assert "answered(<what>)" in reason


def test_x1_delivered_gesture_to_person_is_rejected(monkeypatch):
    from behavior_tree.GPSR import orchestrator as orch
    monkeypatch.setattr(orch, "KNOWN_OBJECT_NAMES", set())
    monkeypatch.setattr(planner_module, "KNOWN_OBJECT_NAMES", set())
    targets = [
        {"id": "t0", "desc": "tell the person at the sofa the gesture",
         "object": "", "location": "", "depends_on": [], "preconditions": [],
         "postconditions": ["delivered(gesture,person_at_sofa)"]},
    ]
    reason = planner_module._reject_non_object_delivery(targets)
    assert reason is not None
    assert "delivered/placed/held are for physical objects" in reason


def test_x1_delivered_known_object_to_operator_passes(monkeypatch):
    from behavior_tree.GPSR import orchestrator as orch
    monkeypatch.setattr(orch, "KNOWN_OBJECT_NAMES", {"spam"})
    monkeypatch.setattr(planner_module, "KNOWN_OBJECT_NAMES", {"spam"})
    targets = [
        {"id": "t0", "desc": "bring me the spam", "object": "spam", "location": "",
         "depends_on": [], "preconditions": [], "postconditions": ["delivered(spam,me)"]},
    ]
    assert planner_module._reject_non_object_delivery(targets) is None
    ok, _reason = planner_module._validate_target_contract(targets)
    assert ok is True


def test_x1_delivered_generic_kitchen_item_passes(monkeypatch):
    # "item" is a generic object noun (not a specific arena name), but still
    # names a graspable THING, unlike "country"/"gesture".
    from behavior_tree.GPSR import orchestrator as orch
    monkeypatch.setattr(orch, "KNOWN_OBJECT_NAMES", set())
    monkeypatch.setattr(planner_module, "KNOWN_OBJECT_NAMES", set())
    targets = [
        {"id": "t0", "desc": "bring emma a kitchen item", "object": "kitchen item",
         "location": "", "depends_on": [], "preconditions": [],
         "postconditions": ["delivered(kitchen item,emma)"]},
    ]
    assert planner_module._reject_non_object_delivery(targets) is None


# ---------------------------------------------------------------------------
# J11: merge a pure-transport target into the following self-navigating
# place/deliver target
# ---------------------------------------------------------------------------

def test_j11_003_shaped_4_target_split_merges_to_3():
    targets = [
        {"id": "t0", "desc": "Locate the kitchen item in the room", "object": "kitchen_item",
         "location": "", "depends_on": [], "preconditions": [],
         "postconditions": ["object_seen(kitchen_item)"]},
        {"id": "t1", "desc": "Grasp the kitchen item", "object": "kitchen_item",
         "location": "", "depends_on": ["t0"], "preconditions": ["object_seen(kitchen_item)"],
         "postconditions": ["held(kitchen_item)"]},
        {"id": "t2", "desc": "Take the kitchen item to the kitchen_table", "object": "kitchen_item",
         "location": "kitchen_table", "depends_on": ["t1"], "preconditions": ["held(kitchen_item)"],
         "postconditions": ["at_robot(kitchen_table)"]},
        {"id": "t3", "desc": "Place the kitchen item on the kitchen_table", "object": "kitchen_item",
         "location": "kitchen_table", "depends_on": ["t2"],
         "preconditions": ["held(kitchen_item)", "at_robot(kitchen_table)"],
         "postconditions": ["placed(kitchen_item,kitchen_table)"]},
    ]
    result = _merge_transport_targets(targets)
    assert [t["id"] for t in result] == ["t0", "t1", "t3"]
    merged_target = result[-1]
    assert merged_target["depends_on"] == ["t1"]
    assert merged_target["postconditions"] == ["placed(kitchen_item,kitchen_table)"]
    assert merged_target["desc"] == "Place the kitchen item on the kitchen_table"
    # M-3 (round-3 fix review): the merge alone leaves t3's
    # at_robot(kitchen_table) precondition dangling -- t2, the ONLY target
    # that established it, is now gone. split_command re-runs the same
    # ledger-based pruning _normalise_targets applies so it is dropped too
    # (held(kitchen_item), still established by t1, stays).
    planner_module._prune_unestablishable_preconditions(result)
    assert merged_target["preconditions"] == ["held(kitchen_item)"]


def test_j11_deliver_to_a_room_shaped_recipient_merges_too():
    # 029: "bring the spam to the laundry_room" -- the transport target's
    # destination Y ("laundry_room") IS the deliver's recipient argument.
    targets = [
        {"id": "t0", "desc": "Grasp the spam", "object": "spam", "location": "",
         "depends_on": [], "preconditions": [], "postconditions": ["held(spam)"]},
        {"id": "t1", "desc": "Take the spam to the laundry_room", "object": "spam",
         "location": "laundry_room", "depends_on": ["t0"], "preconditions": ["held(spam)"],
         "postconditions": ["at_robot(laundry_room)"]},
        {"id": "t2", "desc": "Deliver the spam to the laundry_room", "object": "spam",
         "location": "laundry_room", "depends_on": ["t1"], "preconditions": ["held(spam)"],
         "postconditions": ["delivered(spam,laundry_room)"]},
    ]
    result = _merge_transport_targets(targets)
    assert [t["id"] for t in result] == ["t0", "t2"]
    assert result[-1]["depends_on"] == ["t0"]


def test_l2_self_navigates_toward_desc_fallback_requires_a_destination_preposition():
    # L-2 (round-3 fix review): the desc-fallback match used to fire on Y
    # appearing ANYWHERE in the desc -- "bring me the coke from the
    # kitchen" (Y = kitchen is the SOURCE, not the destination) would also
    # match. Tightened to `\b(to|at|on|onto)\s+(the\s+)?Y\b`. Recipient arg
    # and target["location"] are both deliberately non-matching here so
    # only the desc fallback is exercised.
    target = {
        "desc": "Deliver the spam to the kitchen", "location": "",
        "postconditions": ["delivered(spam,someone)"],
    }
    assert _self_navigates_toward(target, "kitchen") is True

    target_source = {
        "desc": "Bring me the coke from the kitchen", "location": "",
        "postconditions": ["delivered(coke,someone)"],
    }
    assert _self_navigates_toward(target_source, "kitchen") is False


def test_j11_transport_followed_by_find_object_is_not_merged():
    # find_object is not self-navigating -- nothing to merge.
    targets = [
        {"id": "t0", "desc": "Go to the kitchen_table with the kitchen item",
         "object": "kitchen_item", "location": "kitchen_table", "depends_on": [],
         "preconditions": [], "postconditions": ["at_robot(kitchen_table)"]},
        {"id": "t1", "desc": "Find another item at the kitchen_table",
         "object": "item2", "location": "kitchen_table", "depends_on": ["t0"],
         "preconditions": ["at_robot(kitchen_table)"], "postconditions": ["object_seen(item2)"]},
    ]
    result = _merge_transport_targets(targets)
    assert result == targets


# ---------------------------------------------------------------------------
# J14: log target contracts + emit split.accepted at split acceptance
# ---------------------------------------------------------------------------

def test_j14_logs_one_line_per_target_and_emits_split_accepted(tmp_path, capsys):
    from behavior_tree.GPSR.telemetry import GpsrTelemetry
    from behavior_tree.GPSR import telemetry as telemetry_module
    import json

    telemetry = GpsrTelemetry(tmp_path, trajectory_id="t-test", enabled=True)
    telemetry_module.set_default_telemetry(telemetry)
    try:
        targets = [
            {"id": "t0", "desc": "grab the coke", "object": "coke", "location": "kitchen",
             "depends_on": [], "preconditions": [], "postconditions": ["held(coke)"]},
        ]
        _log_split_acceptance(targets, slot=0)
        captured = capsys.readouterr()
        assert "[split] t0 'grab the coke' " in captured.out
        assert "pre=[] post=['held(coke)'] depends_on=[]" in captured.out
        telemetry.close()
        lines = [
            json.loads(line)
            for line in (tmp_path / "debug" / "t-test" / "events.jsonl").read_text().splitlines()
        ]
        accepted = [e for e in lines if e["event_type"] == "split.accepted"]
        assert len(accepted) == 1
        assert accepted[0]["payload"]["slot"] == 0
        assert accepted[0]["payload"]["targets"] == targets
        assert accepted[0]["task_id"] == "t-test/task-1"
    finally:
        telemetry_module.set_default_telemetry(None)


def test_j14_log_line_appears_without_telemetry_configured(capsys):
    targets = [
        {"id": "t0", "desc": "go to the kitchen", "object": "", "location": "kitchen",
         "depends_on": [], "preconditions": [], "postconditions": ["at_robot(kitchen)"]},
    ]
    _log_split_acceptance(targets)  # no slot, no telemetry configured -- must not raise
    captured = capsys.readouterr()
    assert "[split] t0 'go to the kitchen' " in captured.out


def test_fact_store_applies_transitions_and_returns_defensive_copies():
    planner = GPSRPlanner()
    planner.record_facts(2, ["held(cup)", "at_robot(kitchen)", "held(cup)"])
    planner.record_facts(2, ["placed(cup,table)", "at_robot(balcony)"])
    assert planner.get_facts(2) == ["placed(cup,table)", "at_robot(balcony)"]
    facts = planner.get_facts(2)
    facts.append("held(other)")
    assert planner.get_facts(2) == ["placed(cup,table)", "at_robot(balcony)"]
    planner.reset()
    assert planner.get_facts(2) == []


def test_fact_store_concurrent_writers_do_not_lose_facts():
    planner = GPSRPlanner()
    barrier = threading.Barrier(2)

    def write(fact):
        barrier.wait()
        planner.record_facts(0, [fact])

    threads = [threading.Thread(target=write, args=(fact,)) for fact in ("held(cup)", "at_robot(kitchen)")]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert set(planner.get_facts(0)) == {"held(cup)", "at_robot(kitchen)"}


def test_top_prompt_describes_schema_vocabulary_and_relocation_transitions():
    for field in ("id", "depends_on", "preconditions", "postconditions"):
        assert field in TOP_LAYER_SYSTEM_PROMPT
    rule_three = TOP_LAYER_SYSTEM_PROMPT.split("3.", 1)[1].split("4.", 1)[0]
    assert "depends_on to the index" not in rule_three
    assert "earlier target's ID" in rule_three or "earlier target ID" in rule_three
    for fact in ("at_robot", "object_seen", "person_found", "held", "placed", "delivered", "counted", "answered"):
        assert fact in TOP_LAYER_SYSTEM_PROMPT
    assert "three" in TOP_LAYER_SYSTEM_PROMPT
    assert "postcondition held(plant)" in TOP_LAYER_SYSTEM_PROMPT
    assert "postcondition at_robot(balcony)" in TOP_LAYER_SYSTEM_PROMPT


def test_lower_prompt_includes_verified_facts_or_none():
    prompt = _build_lower_layer_user_prompt("cmd", "target", "plant", "balcony", [], [], verified_facts=["held(plant)"])
    assert "Verified world-state facts" in prompt
    assert "  - held(plant)" in prompt
    assert "deterministic target gates" in prompt
    assert "(none yet)" in _build_lower_layer_user_prompt("cmd", "target", "", "", [], [])


def test_lower_prompt_lists_completed_steps_when_given():
    # J3: the postcondition gate's partial commit (a completed grasp) must be
    # rendered so a replan does not repeat it.
    prompt = _build_lower_layer_user_prompt(
        "cmd", "target", "spam", "", [], [],
        completed_steps=[{"action": "grasp", "params": {"object": "spam"}}],
    )
    assert "Already completed in this target (do not repeat):" in prompt
    assert "grasp({'object': 'spam'})" in prompt


def test_lower_prompt_omits_completed_steps_block_when_none():
    prompt = _build_lower_layer_user_prompt("cmd", "target", "", "", [], [])
    assert "Already completed in this target" not in prompt


@pytest.mark.parametrize("bad_targets", [123, "target", {"desc": "target"}, ("target",)])
def test_split_retries_non_list_targets_then_accepts_valid(monkeypatch, bad_targets):
    planner = GPSRPlanner(max_attempts=2)
    planner._offline_mock = False
    responses = [
        {"targets": bad_targets},
        {"targets": [{"id": "a", "desc": "good", "depends_on": [], "preconditions": [], "postconditions": []}]},
    ]
    monkeypatch.setattr(planner, "_new_client", lambda: object())
    monkeypatch.setattr(planner_module, "_call_llm", lambda *args: (responses.pop(0), None))
    assert planner.split_command("get a cup")[0]["desc"] == "good"


def test_split_all_non_list_attempts_fall_back_without_exception(monkeypatch):
    planner = GPSRPlanner(max_attempts=2)
    planner._offline_mock = False
    responses = [{"targets": 1}, {"targets": {"desc": "bad"}}]
    monkeypatch.setattr(planner, "_new_client", lambda: object())
    monkeypatch.setattr(planner_module, "_call_llm", lambda *args: (responses.pop(0), None))
    assert planner.split_command("first | second") == planner_module._offline_mock_targets("first | second")


def test_split_all_attempts_fail_still_logs_contract_and_emits_telemetry(monkeypatch, tmp_path, capsys):
    # M-4 (round-3 fix review): the deterministic fallback split (every LLM
    # attempt rejected) had no per-target contract log line or
    # split.accepted telemetry event, unlike the two LLM-accepted paths --
    # a fallback run could not be audited any better than a crash.
    from behavior_tree.GPSR.telemetry import GpsrTelemetry
    from behavior_tree.GPSR import telemetry as telemetry_module

    planner = GPSRPlanner(max_attempts=1)
    planner._offline_mock = False
    monkeypatch.setattr(planner, "_new_client", lambda: object())
    monkeypatch.setattr(planner_module, "_call_llm", lambda *args: (None, "boom"))

    telemetry = GpsrTelemetry(tmp_path, trajectory_id="t-test", enabled=True)
    telemetry_module.set_default_telemetry(telemetry)
    try:
        targets = planner.split_command("get a cup", slot=0)
        assert targets == planner_module._offline_mock_targets("get a cup")
        captured = capsys.readouterr()
        assert f"[split] {targets[0]['id']} " in captured.out
        telemetry.close()
        import json
        lines = [
            json.loads(line)
            for line in (tmp_path / "debug" / "t-test" / "events.jsonl").read_text().splitlines()
        ]
        assert any(e["event_type"] == "split.accepted" for e in lines)
    finally:
        telemetry_module.set_default_telemetry(None)


def test_split_retries_invalid_graph_or_condition_then_accepts_valid(monkeypatch):
    planner = GPSRPlanner(max_attempts=2)
    planner._offline_mock = False
    responses = [
        {"targets": [{"id": "a", "desc": "bad", "depends_on": ["missing"], "postconditions": ["unknown(x)"]}]},
        # "item" (not "cup") -- X1's held/placed/delivered-are-for-physical-
        # objects check accepts the generic object noun regardless of
        # whether KNOWN_OBJECT_NAMES happens to be populated in this test's
        # process, keeping this test about retry mechanics, not X1.
        {"targets": [{"id": "a", "desc": "good", "depends_on": [], "preconditions": [], "postconditions": ["held(item)"]}]},
    ]
    reasons = []
    monkeypatch.setattr(planner, "_new_client", lambda: object())
    monkeypatch.setattr(planner_module, "_call_llm", lambda *args: (responses.pop(0), None))
    result = planner.split_command("get a cup")
    assert result[0]["desc"] == "good"


def test_plan_target_refreshes_verified_facts_inside_each_retry(monkeypatch):
    planner = GPSRPlanner(max_attempts=2)
    planner._offline_mock = False
    prompts = []
    planner.record_facts(0, ["held(cup)"])
    monkeypatch.setattr(planner, "_new_client", lambda: object())
    original_prompt = planner_module._build_lower_layer_user_prompt

    def capture_prompt(*args, **kwargs):
        prompts.append(kwargs.get("verified_facts"))
        return original_prompt(*args, **kwargs)

    monkeypatch.setattr(planner_module, "_build_lower_layer_user_prompt", capture_prompt)
    monkeypatch.setattr(planner_module, "_call_llm", lambda *args: ({
        "plan": [{"action": "announce", "params": {"text": "ok"}}]
    }, None))
    validations = iter([(False, "retry"), (True, None)])

    def validate(*args, **kwargs):
        result = next(validations)
        if result[0] is False:
            planner.record_facts(0, ["at_robot(kitchen)"])
        return result

    monkeypatch.setattr(planner_module, "validate_plan", validate)
    planner.plan_target(0, 0, "say ok")
    assert prompts == [["held(cup)"], ["held(cup)", "at_robot(kitchen)"]]


def test_build_target_subtree_post_gate_includes_preserved_prefix_without_executing_it():
    planner = GPSRPlanner()
    with planner._lock:
        planner._slot_context[0] = {"command": "cmd", "targets": [{
            "id": "t0", "desc": "finish cup", "object": "cup", "location": "kitchen",
            "depends_on": [], "preconditions": [],
            "postconditions": ["at_robot(kitchen)", "held(cup)"],
        }]}
    prefix = [{"action": "goto", "params": {"location": "kitchen"}}]
    remainder = [{"action": "grasp", "params": {"object": "cup"}}]
    tree = planner.build_target_subtree(0, 0, remainder, completed_steps=prefix)
    gate = tree.children[-1]
    assert gate._action_plan == prefix + remainder
    assert sum("step0" in child.name for child in tree.children) == 1


def test_build_target_subtree_pre_gate_sees_self_establishers_in_preserved_prefix():
    # Supervisor replacement: place(kitchen_table) already ran, only [announce]
    # remains. The precondition gate must still treat at_robot(kitchen_table)
    # as self-established (deferred), not check it at entry and fail.
    planner = GPSRPlanner()
    with planner._lock:
        planner._slot_context[0] = {"command": "cmd", "targets": [{
            "id": "t0", "desc": "place coke", "object": "coke", "location": "kitchen_table",
            "depends_on": [], "preconditions": ["at_robot(kitchen_table)"],
            "postconditions": [],
        }]}
    prefix = [{"action": "place", "params": {"location": "kitchen_table"}}]
    remainder = [{"action": "announce", "params": {"text": "done"}}]
    tree = planner.build_target_subtree(0, 0, remainder, completed_steps=prefix)
    pre = tree.children[0]
    assert type(pre).__name__ == "BtNode_TargetPreconditionCheck"
    assert "at_robot(kitchen_table)" in pre._self_established
    post = tree.children[-1]
    assert type(post).__name__ == "BtNode_TargetPostconditionCheck"
    assert post._action_plan == prefix + remainder


def test_build_target_subtree_defensively_copies_preserved_prefix():
    planner = GPSRPlanner()
    with planner._lock:
        planner._slot_context[0] = {"command": "cmd", "targets": [{
            "id": "t0", "desc": "finish", "object": "", "location": "",
            "depends_on": [], "preconditions": [], "postconditions": ["held(cup)"],
        }]}
    prefix = [{"action": "goto", "params": {"location": "kitchen"}}]
    tree = planner.build_target_subtree(0, 0, [], completed_steps=prefix)
    prefix[0]["params"]["location"] = "mutated"
    assert tree.children[-1]._action_plan[0]["params"]["location"] == "kitchen"


def test_build_target_subtree_places_pre_and_post_gates_around_announcement(monkeypatch):
    planner = GPSRPlanner()
    with planner._lock:
        planner._slot_context[0] = {"command": "cmd", "targets": [{
            "id": "t0", "desc": "go", "object": "plant", "location": "balcony",
            "depends_on": [], "preconditions": ["held(plant)"],
            "postconditions": ["at_robot(balcony)"],
        }]}
    tree = planner.build_target_subtree(0, 0, [])
    assert type(tree.children[0]).__name__ == "BtNode_TargetPreconditionCheck"
    assert type(tree.children[1]).__name__ == "BtNode_AnnounceFromBB"
    assert type(tree.children[-1]).__name__ == "BtNode_TargetPostconditionCheck"


def test_build_target_subtree_without_conditions_preserves_prior_shape():
    planner = GPSRPlanner()
    with planner._lock:
        planner._slot_context[0] = {"command": "cmd", "targets": [{
            "id": "t0", "desc": "say", "object": "", "location": "",
            "depends_on": [], "preconditions": [], "postconditions": [],
        }]}
    tree = planner.build_target_subtree(0, 0, [])
    assert [type(child).__name__ for child in tree.children] == ["BtNode_AnnounceFromBB"]


def test_request_plan_all_canonicalizes_plain_strings(monkeypatch):
    planner = GPSRPlanner()
    started = []
    monkeypatch.setattr(planner, "plan_target", lambda *args, **kwargs: started.append(args))
    planner.request_plan_all(4, ["one", {"desc": "two", "id": "second", "depends_on": []}], command="cmd")
    assert planner._get_slot_context(4)["targets"] == [
        {"id": "t0", "desc": "one", "object": "", "location": "", "depends_on": [], "preconditions": [], "postconditions": []},
        {"id": "second", "desc": "two", "object": "", "location": "", "depends_on": [], "preconditions": [], "postconditions": []},
    ]


def test_request_plan_all_clears_facts_for_new_command(monkeypatch):
    planner = GPSRPlanner()
    planner.record_facts(4, ["held(old)"])
    monkeypatch.setattr(planner, "plan_target", lambda *args, **kwargs: None)
    planner.request_plan_all(4, ["new target"], command="new command")
    assert planner.get_facts(4) == []


def test_request_plan_all_same_command_replan_preserves_facts(monkeypatch):
    planner = GPSRPlanner()
    planner.record_facts(4, ["held(cup)"])
    monkeypatch.setattr(planner, "plan_target", lambda *args, **kwargs: None)
    planner.request_plan_all(4, ["target"], command="command")
    planner.record_facts(4, ["at_robot(kitchen)"])
    planner.replan_target(4, 0, "retry")
    assert planner.get_facts(4) == ["at_robot(kitchen)"]


def test_replan_target_threads_completed_steps_to_plan_target(monkeypatch):
    # J3: the postcondition gate's partial commit (steps that already
    # succeeded) must reach plan_target's completed_steps kwarg so the
    # lower-layer prompt can list them and the plan-validator can seed them.
    planner = GPSRPlanner()
    monkeypatch.setattr(planner, "plan_target", lambda *args, **kwargs: None)
    planner.request_plan_all(4, ["get the spam"], command="cmd")

    captured = {}
    done = threading.Event()

    def fake_plan_target(*args, **kwargs):
        captured["kwargs"] = kwargs
        done.set()

    monkeypatch.setattr(planner, "plan_target", fake_plan_target)
    completed = [{"action": "grasp", "params": {"object": "spam"}}]
    planner.replan_target(4, 0, "postcondition unmet: placed(spam,table) (UNKNOWN)",
                           completed_steps=completed)
    assert done.wait(timeout=2.0)
    assert captured["kwargs"]["completed_steps"] == completed


def test_plan_target_threads_completed_steps_to_build_target_subtree_on_accept(monkeypatch):
    # H-3 (round-3 fix review): unlike replace_target_plan, plan_target's
    # ACCEPTED path never passed completed_steps into build_target_subtree
    # -- the gates for an LLM-produced replan then never saw the target's
    # own already-succeeded steps, so an own postcondition they establish
    # (J2: no ledger shortcut for a target's own postconditions) could never
    # verify and the target looped to SKIP.
    planner = GPSRPlanner()
    planner._offline_mock = False
    monkeypatch.setattr(planner, "_new_client", lambda: object())
    monkeypatch.setattr(planner_module, "_call_llm", lambda *a: (
        {"plan": [{"action": "announce", "params": {"text": "ok", "acknowledgement": True}}]}, None,
    ))
    monkeypatch.setattr(planner_module, "validate_plan", lambda *a, **k: (True, None))
    captured = {}
    original_build = planner.build_target_subtree

    def capture_build(*args, **kwargs):
        captured["kwargs"] = kwargs
        return original_build(*args, **kwargs)

    monkeypatch.setattr(planner, "build_target_subtree", capture_build)
    completed = [{"action": "grasp", "params": {"object": "spam"}}]
    planner.plan_target(0, 0, "place spam", completed_steps=completed)
    assert captured["kwargs"].get("completed_steps") == completed


def test_plan_target_threads_completed_steps_to_build_target_subtree_on_fallback(monkeypatch):
    # H-3: same threading on the "every attempt failed" deterministic
    # fallback path.
    planner = GPSRPlanner(max_attempts=1)
    planner._offline_mock = False
    monkeypatch.setattr(planner, "_new_client", lambda: object())
    monkeypatch.setattr(planner_module, "_call_llm", lambda *a: (None, "boom"))
    captured = {}
    original_build = planner.build_target_subtree

    def capture_build(*args, **kwargs):
        captured["kwargs"] = kwargs
        return original_build(*args, **kwargs)

    monkeypatch.setattr(planner, "build_target_subtree", capture_build)
    completed = [{"action": "grasp", "params": {"object": "spam"}}]
    planner.plan_target(0, 0, "place spam", completed_steps=completed)
    assert captured["kwargs"].get("completed_steps") == completed


def test_drop_completed_duplicate_steps_drops_a_reemitted_completed_step():
    # H-3: a re-emitted step element-wise identical to an already-completed
    # one (e.g. a replan re-grasping an object the target already holds) is
    # dropped, tagged `contract:completed-step` like the other deterministic
    # guard drops.
    completed = [{"action": "grasp", "params": {"object": "x"}}]
    plan = [
        {"action": "grasp", "params": {"object": "x"}},
        {"action": "place", "params": {"location": "t"}},
    ]
    kept, dropped = planner_module._drop_completed_duplicate_steps(plan, completed)
    assert kept == [{"action": "place", "params": {"location": "t"}}]
    assert dropped == ["contract:completed-step"]


def test_drop_completed_duplicate_steps_is_a_no_op_without_completed_steps():
    plan = [{"action": "grasp", "params": {"object": "x"}}]
    kept, dropped = planner_module._drop_completed_duplicate_steps(plan, None)
    assert kept == plan
    assert dropped == []


def test_drop_completed_duplicate_steps_only_consumes_one_match_per_completed_step():
    # A plan that legitimately repeats the SAME action twice (e.g. grasp,
    # place at A, then re-grasp for a second delivery) is not over-dropped
    # -- only as many matching re-emissions as there are completed steps.
    completed = [{"action": "grasp", "params": {"object": "x"}}]
    plan = [
        {"action": "grasp", "params": {"object": "x"}},
        {"action": "grasp", "params": {"object": "x"}},
    ]
    kept, dropped = planner_module._drop_completed_duplicate_steps(plan, completed)
    assert kept == [{"action": "grasp", "params": {"object": "x"}}]
    assert dropped == ["contract:completed-step"]


def test_request_plan_all_rejects_invalid_contract_before_mutating_or_spawning(monkeypatch):
    planner = GPSRPlanner()
    planner.record_facts(4, ["held(old)"])
    planner._slot_context[4] = {"command": "old", "targets": [{"id": "old"}]}
    planner._cache[(4, 0)] = {"ready": True, "plan": [{"action": "announce"}]}
    started = []
    monkeypatch.setattr(planner, "plan_target", lambda *args, **kwargs: started.append(args))

    with pytest.raises(ValueError, match="unknown dependency"):
        planner.request_plan_all(4, [{"id": "a", "desc": "a", "depends_on": ["missing"]}])

    assert started == []
    assert planner.get_facts(4) == ["held(old)"]
    assert planner._get_slot_context(4)["command"] == "old"
    assert planner._cache[(4, 0)]["ready"] is True


def test_request_plan_all_rejects_dict_missing_depends_on_but_accepts_plain_strings(monkeypatch):
    planner = GPSRPlanner()
    monkeypatch.setattr(planner, "plan_target", lambda *args, **kwargs: None)

    with pytest.raises(ValueError, match="depends_on is required"):
        planner.request_plan_all(0, [{"id": "a", "desc": "a"}])

    planner.request_plan_all(1, ["legacy target"])
    assert planner.get_targets(1)[0]["depends_on"] == []


@pytest.mark.parametrize("bad_target", [
    {"id": "a", "desc": "a", "depends_on": None},
    {"id": "a", "desc": "a", "depends_on": [], "postconditions": ["not-a-fact"]},
])
def test_request_plan_all_rejects_malformed_contract_without_mutation(monkeypatch, bad_target):
    planner = GPSRPlanner()
    planner.record_facts(2, ["held(old)"])
    planner._slot_context[2] = {"command": "old", "targets": []}
    started = []
    monkeypatch.setattr(planner, "plan_target", lambda *args, **kwargs: started.append(args))

    with pytest.raises(ValueError):
        planner.request_plan_all(2, [bad_target])

    assert started == []
    assert planner.get_facts(2) == ["held(old)"]
    assert planner._get_slot_context(2)["command"] == "old"


def test_dependency_ancestor_targets_include_only_transitive_declared_dependencies():
    targets = _normalise_targets([
        {"id": "independent", "desc": "independent", "depends_on": []},
        {"id": "root", "desc": "root", "depends_on": []},
        {"id": "middle", "desc": "middle", "depends_on": ["root"]},
        {"id": "leaf", "desc": "leaf", "depends_on": ["middle"]},
    ])
    assert [t["id"] for t in planner_module._dependency_ancestor_targets(targets, 3)] == ["root", "middle"]
    assert planner_module._dependency_ancestor_targets(targets, 1) == []


def test_request_plan_all_passes_only_dependency_ancestors_to_workers(monkeypatch):
    planner = GPSRPlanner()
    started = []
    monkeypatch.setattr(planner, "plan_target", lambda *args, **kwargs: started.append(args))
    targets = [
        {"id": "independent", "desc": "independent", "depends_on": []},
        {"id": "root", "desc": "root", "depends_on": []},
        {"id": "middle", "desc": "middle", "depends_on": ["root"]},
        {"id": "leaf", "desc": "leaf", "depends_on": ["middle"]},
    ]
    planner.request_plan_all(0, targets, command="cmd")
    assert {args[1]: [t["id"] for t in args[6]] for args in started} == {
        0: [], 1: [], 2: ["root"], 3: ["root", "middle"],
    }


def test_prior_validation_context_is_independent_of_cache_readiness(monkeypatch):
    planner = GPSRPlanner()
    targets = _normalise_targets([
        {"id": "find", "desc": "find person", "depends_on": [], "postconditions": ["person_found(Alex)"]},
        {"id": "guide", "desc": "guide Alex", "depends_on": ["find"]},
    ])
    with planner._lock:
        planner._slot_context[0] = {"command": "cmd", "targets": targets}
    expected = planner_module._deterministic_prior_plan(targets, 1)
    planner._cache[(0, 0)] = {"plan": [{"action": "find_person", "params": {"descriptor": "Alex"}}]}
    assert planner_module._flatten_prior_plans(planner, 0, 1) == expected
    planner._cache.clear()
    assert planner_module._flatten_prior_plans(planner, 0, 1) == expected


def test_get_targets_returns_defensive_snapshot_of_slot_context(monkeypatch):
    planner = GPSRPlanner()
    monkeypatch.setattr(planner, "plan_target", lambda *args, **kwargs: None)
    planner.request_plan_all(5, [{
        "id": "acquire",
        "desc": "acquire cup",
        "depends_on": [],
    }, {
        "id": "place",
        "desc": "place cup",
        "depends_on": ["acquire"],
    }], command="command")

    snapshot = planner.get_targets(5)
    snapshot[0]["desc"] = "mutated"
    snapshot[1]["depends_on"].append("unexpected")
    snapshot.append({"id": "extra"})

    assert planner.get_targets(5) == [
        {"id": "acquire", "desc": "acquire cup", "object": "", "location": "", "depends_on": [], "preconditions": [], "postconditions": []},
        {"id": "place", "desc": "place cup", "object": "", "location": "", "depends_on": ["acquire"], "preconditions": [], "postconditions": []},
    ]

def test_two_layer_planner_honours_offline_planner_override(monkeypatch):
    import behavior_tree.GPSR.orchestrator as orch
    import behavior_tree.GPSR.planner as planner_mod

    monkeypatch.setattr(orch, "is_full_mock_mode", lambda: True)

    monkeypatch.delenv("GPSR_OFFLINE_PLANNER", raising=False)
    assert planner_mod.GPSRPlanner()._offline_mock is True

    monkeypatch.setenv("GPSR_OFFLINE_PLANNER", "0")
    assert planner_mod.GPSRPlanner()._offline_mock is False

    monkeypatch.setenv("GPSR_OFFLINE_PLANNER", "1")
    assert planner_mod.GPSRPlanner()._offline_mock is True


# ---------------------------------------------------------------------------
# I4 (round-3 adversarial review, M7): a planning deadline + a daemon thread
# that never dies silently.
# ---------------------------------------------------------------------------

def test_llm_timeout_s_defaults_and_reads_env_override(monkeypatch):
    from behavior_tree.GPSR import config as config_module

    # L-3 (round-3 fix review): 45s was close to observed reasoning-model
    # latency (tier0: ~15-20s per serial call) -- default raised to 90s.
    monkeypatch.delenv("GPSR_LLM_TIMEOUT_S", raising=False)
    assert config_module._resolve_llm_timeout_s() == 90.0

    monkeypatch.setenv("GPSR_LLM_TIMEOUT_S", "12.5")
    assert config_module._resolve_llm_timeout_s() == 12.5

    monkeypatch.setenv("GPSR_LLM_TIMEOUT_S", "not-a-number")
    assert config_module._resolve_llm_timeout_s() == 90.0


def test_new_client_passes_timeout_and_bounded_retries(monkeypatch):
    planner = GPSRPlanner()
    planner._offline_mock = False
    captured = {}

    class _FakeOpenAI:
        def __init__(self, **kwargs):
            captured.update(kwargs)

    monkeypatch.setattr(planner_module.openai, "OpenAI", _FakeOpenAI)

    planner._new_client()

    assert captured.get("timeout") == planner_module.LLM_TIMEOUT_S
    assert captured.get("max_retries") == 1


def test_plan_target_survives_a_crashing_call_llm_with_fallback_and_error(monkeypatch):
    # I4: code AFTER `_call_llm` (grouping, fallback subtree build) is not
    # the only thing that can raise -- ANY exception in the planning body
    # must leave the cache entry READY with the guaranteed fallback plan and
    # `error` set, never not-ready forever.
    planner = GPSRPlanner(max_attempts=2)
    planner._offline_mock = False
    monkeypatch.setattr(planner, "_new_client", lambda: object())

    def _crashing_call_llm(*args, **kwargs):
        raise RuntimeError("boom: LLM call crashed")

    monkeypatch.setattr(planner_module, "_call_llm", _crashing_call_llm)

    planner.plan_target(0, 0, "tell me what day it is", command="tell me what day it is")

    assert planner.is_target_ready(0, 0) is True
    error = planner.get_error(0, 0)
    assert error is not None
    assert error.startswith("planner crashed: ")
    assert "boom: LLM call crashed" in error
    plan = planner.get_action_plan(0, 0)
    assert plan and plan[0]["action"] == "announce"
    assert plan[0]["params"].get("acknowledgement") is True
