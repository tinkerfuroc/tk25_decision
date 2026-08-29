from __future__ import annotations

import pytest

from behavior_tree.GPSR import action_contracts as ac
from behavior_tree.GPSR.small_trees import ACTION_FACTORIES
from behavior_tree.GPSR.orchestrator import _TARGET_GATE_EVIDENCE_KEYS


def test_every_factory_has_a_contract_and_vice_versa():
    assert set(ac.ACTION_CONTRACTS) == set(ACTION_FACTORIES)


def test_records_are_known_evidence_names():
    known = {name for name, _ in _TARGET_GATE_EVIDENCE_KEYS}
    for contract in ac.ACTION_CONTRACTS.values():
        assert set(contract.records) <= known, contract.action


def test_contract_for_unknown_action_raises():
    with pytest.raises(KeyError):
        ac.contract_for("teleport")


def test_place_and_deliver_self_establish_at_robot():
    assert ac.contract_for("place").self_establishes == {"at_robot": "location"}
    assert ac.contract_for("deliver").self_establishes == {"at_robot": "recipient_location"}
    assert ac.contract_for("goto").self_establishes == {"at_robot": "location"}
    assert ac.contract_for("grasp").self_establishes == {}


def test_self_established_facts_from_step():
    assert ac.self_established_facts({"action": "place", "params": {"location": "kitchen table"}}) == ["at_robot(kitchen_table)"]
    assert ac.self_established_facts({"action": "deliver", "params": {"object": "coke", "recipient": "Susan"}}) == []
    assert ac.self_established_facts({"action": "grasp", "params": {"object": "coke"}}) == []


def test_established_facts_deliver_goto_grasp_place():
    assert ac.established_facts(
        {"action": "deliver", "params": {"object": "spam", "recipient": "me"}}
    ) == ["delivered(spam,me)"]
    assert ac.established_facts(
        {"action": "goto", "params": {"location": "kitchen"}}
    ) == ["at_robot(kitchen)"]
    assert ac.established_facts(
        {"action": "grasp", "params": {"object": "coke"}}
    ) == ["held(coke)"]
    assert ac.established_facts(
        {"action": "place", "params": {"object": "plant", "location": "balcony"}}
    ) == ["placed(plant,balcony)"]


def test_established_facts_missing_param_yields_empty():
    # deliver requires both object and recipient for its template — missing
    # either one means the fact cannot be derived, so skip it (never guess).
    assert ac.established_facts({"action": "deliver", "params": {"object": "spam"}}) == []
    assert ac.established_facts({"action": "deliver", "params": {}}) == []
    assert ac.established_facts({"action": "goto", "params": {}}) == []


def test_established_facts_unknown_action_yields_empty():
    assert ac.established_facts({"action": "teleport", "params": {"location": "kitchen"}}) == []
    assert ac.established_facts({"action": None, "params": {}}) == []


def test_established_facts_normalises_args():
    assert ac.established_facts(
        {"action": "goto", "params": {"location": "Laundry Desk"}}
    ) == ["at_robot(laundry_desk)"]
    assert ac.established_facts(
        {"action": "deliver", "params": {"object": "  Spam  ", "recipient": "Me"}}
    ) == ["delivered(spam,me)"]


def test_established_facts_zero_arity_template_never_emits_unparsable_fact(monkeypatch):
    # MINOR-4 guard: a future zero-arity `establishes` template (no params to
    # resolve) must never produce the unparsable f"{predicate}()" — it is
    # skipped, same as a template with an unresolvable param.
    fake = ac.ActionContract("beep", establishes=("beeped()",))
    monkeypatch.setitem(ac.ACTION_CONTRACTS, "beep", fake)
    assert ac.established_facts({"action": "beep", "params": {}}) == []


def test_established_facts_no_self_establish_actions_are_empty():
    # approach_person/describe_person/etc have no `establishes` templates at all.
    assert ac.established_facts({"action": "approach_person", "params": {}}) == []
    assert ac.established_facts({"action": "find_person", "params": {"person": "Alex"}}) == ["person_found(alex)"]


def test_count_established_facts_stays_counted_only_no_question_param():
    # H1: count now ALSO establishes answered(question), but it has no
    # `question` param (only `object`/`location`), so the answered(question)
    # template can never resolve -- established_facts() for count stays
    # exactly ["counted(object)"], same as before H1.
    assert ac.established_facts({"action": "count", "params": {"object": "drinks"}}) == ["counted(drinks)"]
    assert ac.established_facts({"action": "count", "params": {}}) == []


def test_llm_vlm_fallback_established_facts_resolve_answered_when_question_present():
    # H1: unlike count/announce, vlm_fallback/llm_fallback DO carry a
    # `question` param, so their answered(question) template CAN resolve --
    # deliberately, per the fix brief.
    assert ac.established_facts(
        {"action": "llm_fallback", "params": {"question": "what day is today"}}
    ) == ["answered(what_day_is_today)"]
    assert ac.established_facts({"action": "llm_fallback", "params": {}}) == []
    assert ac.established_facts(
        {"action": "vlm_fallback", "params": {"question": "what colour is the mug"}}
    ) == ["answered(what_colour_is_the_mug)"]
    assert ac.established_facts({"action": "vlm_fallback", "params": {}}) == []


def test_self_navigating_destinations_excludes_goto():
    # M-4 (round-2 review): guide navigates (small_trees.create_guide,
    # "retry guide goto") but declared no self_establishes -- a guide
    # target's at_robot(<dest>) postcondition then demanded an EXTRA goto,
    # and the model could place it before guide (robot walks off alone).
    # guide's param name is "location" (ACTION_CATALOGUE_DESCRIPTION /
    # materialise_params: "guide(location: str) Lead a person to location").
    assert ac.self_navigating_destinations() == {
        "deliver": "recipient_location",
        "guide": "location",
        "place": "location",
        "search_object": "location",
    }


def test_render_self_satisfied_rule_mentions_each_self_navigator():
    text = ac.render_self_satisfied_rule()
    for action in ("goto", "place", "deliver", "search_object", "guide"):
        assert action in text
    assert "at_robot" in text


import py_trees
from py_trees.common import Access

from behavior_tree.GPSR.orchestrator import materialise_params
from behavior_tree.GPSR.small_trees import bb_keys


@pytest.fixture(autouse=True)
def _clear_blackboard_after_materialise_tests():
    # _bb() below clears the shared py_trees Blackboard singleton on entry,
    # but leaves whatever it wrote behind on exit. Without a teardown clear
    # here, that state (LAST_NAV_LOCATION, TARGET_LOCATION, ...) leaks into
    # any test module that runs afterwards in the same pytest session and
    # doesn't clear the blackboard itself (e.g. test_gpsr_target_gates.py,
    # which alphabetically follows this file).
    yield
    py_trees.blackboard.Blackboard.clear()


def _bb():
    py_trees.blackboard.Blackboard.clear()
    bb = py_trees.blackboard.Client(name="t")
    for key in (bb_keys.LAST_NAV_LOCATION, bb_keys.TARGET_LOCATION, bb_keys.TARGET_POSE,
                bb_keys.GRASP_ASK_REFEREE, bb_keys.GRASP_REFEREE_LOCATION,
                bb_keys.GRASP_REFEREE_POSE, bb_keys.GRASP_REFEREE_IS_APPLIANCE,
                bb_keys.TARGET_OBJECT_NAME, bb_keys.TARGET_OBJECT_PROMPT,
                bb_keys.TARGET_PERSON_PROMPT, bb_keys.ANNOUNCE_TEXT, bb_keys.ASK_QUESTION,
                bb_keys.VLM_QUESTION, bb_keys.LLM_QUESTION, bb_keys.CURRENT_DYNLABEL):
        bb.register_key(key, access=Access.WRITE)
        bb.register_key(key, access=Access.READ)
    for key in (bb_keys.REPORT_INFO, bb_keys.START_POSE, bb_keys.DYNAMIC_LOCATIONS):
        bb.register_key(key, access=Access.READ)
    from behavior_tree.GPSR.orchestrator import SEARCH_POSE_KEYS
    for key in SEARCH_POSE_KEYS:
        bb.register_key(key, access=Access.WRITE)
    bb.set(bb_keys.LAST_NAV_LOCATION, "", overwrite=True)
    return bb


@pytest.mark.parametrize("action,params,expected", [
    ("goto", {"location": "kitchen_table"}, "kitchen_table"),
    ("place", {"location": "kitchen_table"}, "kitchen_table"),
    ("deliver", {"object": "coke", "recipient": "Susan", "recipient_location": "living_room"}, "living_room"),
    ("search_object", {"object": "coke", "location": "kitchen"}, "kitchen"),
])
def test_materialise_records_last_nav_for_self_navigators(action, params, expected):
    bb = _bb()
    materialise_params(bb, action, params)
    assert bb.get(bb_keys.LAST_NAV_LOCATION) == expected


def test_materialise_does_not_touch_last_nav_for_grasp():
    bb = _bb()
    bb.set(bb_keys.LAST_NAV_LOCATION, "shelf", overwrite=True)
    materialise_params(bb, "grasp", {"object": "coke"})
    assert bb.get(bb_keys.LAST_NAV_LOCATION) == "shelf"


def test_materialise_deliver_without_location_leaves_last_nav():
    bb = _bb()
    bb.set(bb_keys.LAST_NAV_LOCATION, "office", overwrite=True)
    materialise_params(bb, "deliver", {"object": "coke", "recipient": "Susan"})
    assert bb.get(bb_keys.LAST_NAV_LOCATION) == "office"


from behavior_tree.GPSR.planner_validators import validate_plan
from behavior_tree.GPSR.validators import (
    Verdict, VerificationContext, _action_verdict, parse_fact,
)


def test_validate_plan_rejects_goto_immediately_before_guide():
    # M-4: guide is now a real self-navigator, so the goto-before-self-nav
    # rule applies to it too -- a goto right before guide is redundant
    # (guide navigates to `location` itself) and the model must instead put
    # the destination on the guide step directly.
    plan = [
        {"action": "find_person", "params": {"person": "sarah"}},
        {"action": "approach_person", "params": {}},
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "guide", "params": {"location": "kitchen"}},
    ]
    ok, reason = validate_plan(
        plan, "guide sarah to the kitchen",
        {"find_person", "approach_person", "goto", "guide"},
    )
    assert not ok
    assert "guide" in reason and "redundant" in reason


def test_action_verdict_accepts_at_robot_after_guide():
    # M-4: guide's self_establishes now makes the generic at_robot
    # action-verdict fallback apply to it, same as goto/place/deliver.
    fact, _ = parse_fact("at_robot(kitchen)")
    ctx = VerificationContext(
        phase="postcondition", established_facts=frozenset(),
        completed_steps=(
            {"action": "guide", "params": {"location": "kitchen"}, "succeeded": True},
        ),
        target_object="", target_location="",
    )
    result = _action_verdict(fact, ctx)
    assert result is not None and result.verdict is Verdict.VALID


def test_validate_plan_rejects_goto_before_each_self_navigator(monkeypatch):
    fake = ac.ActionContract("teleport", self_establishes={"at_robot": "location"},
                             records=("last_nav_location",), self_navigating=True)
    monkeypatch.setitem(ac.ACTION_CONTRACTS, "teleport", fake)
    plan = [
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "teleport", "params": {"location": "kitchen"}},
    ]
    ok, reason = validate_plan(plan, "teleport to the kitchen", {"goto", "teleport"})
    assert not ok and "teleport" in reason and "redundant" in reason


def test_action_verdict_accepts_at_robot_after_place_and_deliver():
    fact, _ = parse_fact("at_robot(kitchen_table)")
    ctx = VerificationContext(
        phase="postcondition", established_facts=frozenset(),
        completed_steps=(
            {"action": "place", "params": {"location": "kitchen_table"}, "succeeded": True},
        ),
        target_object="", target_location="",
    )
    result = _action_verdict(fact, ctx)
    assert result is not None and result.verdict is Verdict.VALID

    fact, _ = parse_fact("at_robot(living_room)")
    ctx = VerificationContext(
        phase="postcondition", established_facts=frozenset(),
        completed_steps=(
            {"action": "deliver", "params": {"object": "coke", "recipient": "Susan",
                                             "recipient_location": "living_room"}, "succeeded": True},
        ),
        target_object="", target_location="",
    )
    result = _action_verdict(fact, ctx)
    assert result is not None and result.verdict is Verdict.VALID


def test_action_verdict_rejects_at_robot_after_grasp():
    fact, _ = parse_fact("at_robot(kitchen_table)")
    ctx = VerificationContext(
        phase="postcondition", established_facts=frozenset(),
        completed_steps=({"action": "grasp", "params": {"object": "coke"}, "succeeded": True},),
        target_object="", target_location="",
    )
    assert _action_verdict(fact, ctx) is None


def test_deterministic_intent_derives_from_registry():
    from behavior_tree.GPSR.planner import _deterministic_target_intent
    intents = _deterministic_target_intent({"desc": "x", "postconditions": ["placed(plant,balcony)", "at_robot(balcony)"]})
    assert [i["action"] for i in intents] == ["place", "goto"]
    assert intents[1]["params"] == {"location": "balcony"}
    assert _deterministic_target_intent({"desc": "x", "postconditions": ["object_seen(coke)"]})[0]["action"] == "find_object"
    assert _deterministic_target_intent({"desc": "x", "postconditions": ["answered(name)"]})[0]["action"] == "ask_person"


def test_split_prompt_rule_7_is_rendered_from_registry():
    from behavior_tree.GPSR import planner
    prompt = planner.TOP_LAYER_SYSTEM_PROMPT
    assert ac.render_self_satisfied_rule() in prompt
    assert "preconditions held(plant), at_robot(balcony)" not in prompt
    assert "precondition held(plant) and\n       postcondition placed(plant,balcony)" in prompt or "precondition held(plant)" in prompt
