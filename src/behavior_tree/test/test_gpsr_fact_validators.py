from __future__ import annotations

from dataclasses import dataclass

import pytest

from behavior_tree.GPSR import validators
from behavior_tree.GPSR.validators import (
    Fact,
    Verdict,
    VerificationContext,
    VerificationResult,
    canonical_fact,
    check_all,
    parse_fact,
    register_tier2_hook,
)


@pytest.mark.parametrize(
    ("text", "expected"),
    [("Held( Plant Pot )", "held(plant_pot)"), ("placed( Red Cup, Kitchen Table )", "placed(red_cup,kitchen_table)")],
)
def test_parse_fact_normalizes_predicate_and_arguments(text, expected):
    fact, error = parse_fact(text)
    assert error is None
    assert fact is not None
    assert canonical_fact(fact) == expected


def test_canonical_fact_preserves_raw_independently():
    fact = Fact("held", ("plant pot",), "Held( Plant Pot )")
    assert canonical_fact(fact) == "held(plant_pot)"


@pytest.mark.parametrize(
    "text",
    [
        "unknown(foo)",
        "held()",
        "held(plant, pot)",
        "placed(plant)",
        "held(plant(foo))",
        "held(plant) trailing",
        "held(plant))(extra)",
    ],
)
def test_parse_fact_rejects_unknown_bad_arity_empty_nested_and_trailing(text):
    fact, error = parse_fact(text)
    assert fact is None
    assert error


def _context(phase="precondition", **kwargs):
    return VerificationContext(phase=phase, **kwargs)


def _check(text, *, evidence=None, context=None):
    results, facts = check_all([text], evidence or {}, context or _context())
    assert len(results) == 1
    return results[0], facts


@pytest.mark.parametrize(
    ("existing", "additions", "expected"),
    [
        (["at_robot(kitchen)", "held(cup)"], ["at_robot(balcony)"], ["held(cup)", "at_robot(balcony)"]),
        (["held(cup)", "placed(cup,table)"], ["placed(cup,shelf)"], ["placed(cup,table)", "placed(cup,shelf)"]),
        (["placed(cup,table)"], ["held(cup)"], ["held(cup)"]),
        (["delivered(cup,alex)"], ["held(cup)"], ["held(cup)"]),
        (["legacy fact", "held(cup)", "held(cup)"], ["Held( CUP )", "legacy fact"], ["legacy fact", "held(cup)"]),
        (["at_robot(kitchen)"], ["at_robot(balcony)", "at_robot(lab)"], ["at_robot(lab)"]),
    ],
)
def test_apply_fact_transitions_updates_current_state_and_preserves_legacy(
    existing, additions, expected
):
    assert validators.apply_fact_transitions(existing, additions) == expected


def test_established_at_robot_is_invalid_when_navigation_evidence_contradicts_it():
    result, _ = _check(
        "at_robot(kitchen)",
        evidence={"last_nav_location": "balcony"},
        context=_context(established_facts=frozenset({"at_robot(kitchen)"})),
    )
    assert result.verdict is Verdict.INVALID


def test_established_held_is_valid():
    result, facts = _check(
        "held(plant pot)",
        context=_context(established_facts=frozenset({"held(plant_pot)"})),
    )
    assert facts[0].predicate == "held"
    assert result == VerificationResult(Verdict.VALID, 1.0, "established fact: held(plant_pot)")


def test_missing_held_precondition_is_unknown():
    result, _ = _check("held(plant pot)")
    assert result.verdict is Verdict.UNKNOWN
    assert "stronger verifier not installed" in result.evidence


def test_count_zero_is_valid_but_missing_count_is_unknown():
    present, _ = _check("counted(cups)", evidence={"count_value": 0})
    absent, _ = _check("counted(cups)")
    assert present.verdict is Verdict.VALID
    assert present.confidence == 1.0
    assert absent.verdict is Verdict.UNKNOWN


class _Detection:
    def __init__(self, objects):
        self.objects = objects


@pytest.mark.parametrize(
    ("predicate", "evidence_key", "response"),
    [
        ("object_seen", "object_detection", {"objects": ["cup"]}),
        ("object_seen", "object_detection", _Detection(["cup"])),
        ("person_found", "person_detection", {"objects": ("Alex",)}),
        ("person_found", "person_detection", _Detection(["Alex"])),
    ],
)
def test_detection_artifacts_are_valid_when_objects_are_nonempty(predicate, evidence_key, response):
    requested = "cup" if predicate == "object_seen" else "alex"
    result, _ = _check(f"{predicate}({requested})", evidence={evidence_key: response})
    assert result.verdict is Verdict.VALID


@pytest.mark.parametrize(
    ("predicate", "evidence_key"),
    [("object_seen", "object_detection"), ("person_found", "person_detection")],
)
def test_detection_artifacts_are_unknown_when_unset_or_empty(predicate, evidence_key):
    unset, _ = _check(f"{predicate}(target)")
    empty, _ = _check(f"{predicate}(target)", evidence={evidence_key: {"objects": []}})
    assert unset.verdict is Verdict.UNKNOWN
    assert empty.verdict is Verdict.UNKNOWN


@pytest.mark.parametrize(
    ("predicate", "evidence_key", "requested", "label", "expected"),
    [
        ("object_seen", "object_detection", "bottle", "cup", Verdict.INVALID),
        ("object_seen", "object_detection", "cup", "cup", Verdict.VALID),
        ("person_found", "person_detection", "susan", "alex", Verdict.INVALID),
        ("person_found", "person_detection", "alex", "alex", Verdict.VALID),
    ],
)
def test_recognizable_detection_labels_must_match_requested_fact(
    predicate, evidence_key, requested, label, expected
):
    result, _ = _check(
        f"{predicate}({requested})",
        evidence={evidence_key: {"objects": [{"label": label}]}},
    )
    assert result.verdict is expected


@pytest.mark.parametrize(
    ("label", "requested", "expected"),
    [
        # run 005: specialist find_person label "person Liam" -> normalized
        # "person_liam"; requested name "liam" is one of its '_'-tokens.
        ("person_liam", "liam", True),
        # run 003: category-prompted object detection label
        # "kitchen item.round white table" -> normalized
        # "kitchen_item.round_white_table"; requested "kitchen_item" is the
        # label's class prefix (text before the first '.').
        ("kitchen_item.round_white_table", "kitchen_item", True),
        ("drink.coke", "drink", True),
        ("person_bob", "liam", False),
        # token rule is pinned VALID: "red" is a whole '_'-token of
        # "red_jacket".
        ("red_jacket", "red", True),
        # but never a substring match: "red" is not a whole token of
        # "bred_jacket".
        ("bred_jacket", "red", False),
        ("cup", "cup", True),
        ("cup", "bottle", False),
    ],
)
def test_label_matches_equality_class_prefix_and_token_rules(label, requested, expected):
    assert validators._label_matches(label, requested) is expected


def test_person_found_matches_specialist_name_token_label():
    """run 005 evidence: specialist find_person reported "person Liam"."""
    result, _ = _check(
        "person_found(Liam)",
        evidence={"person_detection": {"objects": [{"label": "person Liam"}]}},
    )
    assert result.verdict is Verdict.VALID


def test_object_seen_matches_category_prompted_class_prefix_label():
    """run 003 evidence: search_object reported three kitchen-item instances."""
    result, _ = _check(
        "object_seen(kitchen item)",
        evidence={
            "object_detection": {
                "objects": [
                    {"label": "kitchen item.round white table"},
                    {"label": "kitchen item.trash can"},
                    {"label": "kitchen item.refrigerator"},
                ]
            }
        },
    )
    assert result.verdict is Verdict.VALID


def test_opaque_nonempty_detection_keeps_weak_v1_validity():
    result, _ = _check(
        "object_seen(bottle)",
        evidence={"object_detection": {"objects": [{"score": 0.9}]}},
    )
    assert result.verdict is Verdict.VALID
    assert "identity unavailable" in result.evidence


def test_waving_specialist_descriptor_is_provenance_not_named_identity():
    waving = {"waving_persons": [{"x": 1}], "person_provenance": "waving_specialist"}
    valid, _ = _check("person_found(waving_person)", evidence=waving)
    unrelated, _ = _check("person_found(alex)", evidence=waving)
    assert valid.verdict is Verdict.VALID
    assert unrelated.verdict is Verdict.UNKNOWN


def test_counted_matches_provenance_and_zero_is_preserved():
    matching, _ = _check(
        "counted(cups)", evidence={"count_value": 0, "count_target": "cups"}
    )
    mismatching, _ = _check(
        "counted(cups)", evidence={"count_value": 2, "count_target": "plates"}
    )
    fallback, _ = _check("counted(cups)", evidence={"count_value": 0})
    assert matching.verdict is Verdict.VALID
    assert mismatching.verdict is Verdict.INVALID
    assert fallback.verdict is Verdict.VALID


def test_answered_matches_question_provenance_with_weak_fallback():
    matching, _ = _check(
        "answered(what color)",
        evidence={"llm_answer": "blue", "question_provenance": "What color?"},
    )
    mismatching, _ = _check(
        "answered(what color)",
        evidence={"llm_answer": "blue", "question_provenance": "What shape?"},
    )
    fallback, _ = _check("answered(what color)", evidence={"llm_answer": "blue"})
    assert matching.verdict is Verdict.VALID
    assert mismatching.verdict is Verdict.INVALID
    assert fallback.verdict is Verdict.VALID


def test_person_answer_uses_ask_question_provenance():
    matching, _ = _check(
        "answered(what color)",
        evidence={"person_answer": "blue", "ask_question": "What color?"},
    )
    mismatching, _ = _check(
        "answered(what color)",
        evidence={"person_answer": "blue", "ask_question": "What shape?"},
    )
    assert matching.verdict is Verdict.VALID
    assert mismatching.verdict is Verdict.INVALID


def test_answered_provenance_normalizes_whitespace_and_underscores():
    result, _ = _check(
        "answered(what color)",
        evidence={"qa_answer": "blue", "qa_question": " What   color? "},
    )
    assert result.verdict is Verdict.VALID


def test_answered_requires_a_nonempty_answer_string():
    valid, _ = _check("answered(what color)", evidence={"llm_answer": "blue"})
    empty, _ = _check(
        "answered(what color)",
        evidence={"qa_answer": "", "person_answer": "  ", "llm_answer": None, "vlm_answer": ""},
    )
    assert valid.verdict is Verdict.VALID
    assert empty.verdict is Verdict.UNKNOWN


def _step(action, **params):
    return {"action": action, "params": params, "status": "succeeded"}


@pytest.mark.parametrize(
    ("fact_text", "step", "context_kwargs"),
    [
        ("held(plant pot)", _step("grasp", object="plant pot"), {}),
        ("placed(plant pot,kitchen table)", _step("place", object="plant pot", location="kitchen table"), {"target_object": "plant pot"}),
        ("delivered(plant pot,alex)", _step("deliver", object="plant pot", recipient="alex"), {}),
        ("at_robot(kitchen)", _step("goto", location="kitchen"), {}),
    ],
)
def test_successful_matching_action_verdict_establishes_postcondition(fact_text, step, context_kwargs):
    result, _ = _check(
        fact_text,
        context=_context(phase="postcondition", completed_steps=(step,), **context_kwargs),
    )
    assert result.verdict is Verdict.VALID
    assert result.confidence == 0.5
    assert "action-verdict fallback" in result.evidence
    assert "stronger verifier not installed" in result.evidence


@pytest.mark.parametrize(
    ("fact_text", "step", "context_kwargs"),
    [
        ("held(plant pot)", _step("grasp", object="mug"), {}),
        ("placed(plant pot,kitchen table)", _step("place", object="plant pot", location="desk"), {"target_object": "plant pot"}),
        ("delivered(plant pot,alex)", _step("deliver", object="mug", recipient="alex"), {}),
        ("at_robot(kitchen)", _step("goto", location="bedroom"), {}),
    ],
)
def test_mismatched_action_verdict_is_unknown(fact_text, step, context_kwargs):
    result, _ = _check(
        fact_text,
        context=_context(phase="postcondition", completed_steps=(step,), **context_kwargs),
    )
    assert result.verdict is Verdict.UNKNOWN


def test_place_can_use_step_object_when_context_target_is_empty():
    result, _ = _check(
        "placed(plant pot,kitchen table)",
        context=_context(
            phase="postcondition",
            completed_steps=(_step("place", object="plant pot", location="kitchen table"),),
        ),
    )
    assert result.verdict is Verdict.VALID


def test_explicit_place_object_wins_over_conflicting_context_target():
    result, _ = _check(
        "placed(plant pot,kitchen table)",
        context=_context(
            phase="postcondition",
            target_object="mug",
            completed_steps=(_step("place", object="plant pot", location="kitchen table"),),
        ),
    )
    assert result.verdict is Verdict.VALID


def test_conflicting_explicit_place_object_cannot_be_hidden_by_context_target():
    result, _ = _check(
        "placed(plant pot,kitchen table)",
        context=_context(
            phase="postcondition",
            target_object="plant pot",
            completed_steps=(_step("place", object="mug", location="kitchen table"),),
        ),
    )
    assert result.verdict is Verdict.UNKNOWN


def test_missing_place_object_uses_matching_context_target():
    result, _ = _check(
        "placed(plant pot,kitchen table)",
        context=_context(
            phase="postcondition",
            target_object="plant pot",
            completed_steps=(_step("place", location="kitchen table"),),
        ),
    )
    assert result.verdict is Verdict.VALID


def test_announce_cannot_establish_held():
    result, _ = _check(
        "held(plant pot)",
        context=_context(phase="postcondition", completed_steps=(_step("announce", text="done"),)),
    )
    assert result.verdict is Verdict.UNKNOWN


def test_action_fallback_is_prohibited_in_precondition_phase():
    result, _ = _check(
        "held(plant pot)",
        context=_context(completed_steps=(_step("grasp", object="plant pot"),)),
    )
    assert result.verdict is Verdict.UNKNOWN


def test_at_robot_navigation_intent_mismatch_is_invalid():
    result, _ = _check("at_robot(kitchen)", evidence={"last_nav_location": "bedroom"})
    assert result.verdict is Verdict.INVALID
    assert "mismatch" in result.evidence


def test_at_robot_matching_navigation_intent_is_low_confidence_valid():
    result, _ = _check(
        "at_robot(kitchen)",
        evidence={"last_nav_location": "kitchen"},
        context=_context(phase="postcondition"),
    )
    assert result.verdict is Verdict.VALID
    assert result.confidence <= 0.5
    assert "intent/action" in result.evidence


def test_tier2_result_is_authoritative_and_invalid_overrides_action_verdict():
    hook_result = VerificationResult(Verdict.INVALID, 0.9, "tier2 says grasp failed")
    register_tier2_hook("held", lambda fact, evidence, context: hook_result)
    try:
        result, _ = _check(
            "held(plant pot)",
            context=_context(
                phase="postcondition",
                completed_steps=(_step("grasp", object="plant pot"),),
            ),
        )
        assert result is hook_result
    finally:
        register_tier2_hook("held", None)


def test_tier2_hook_none_allows_fallback_and_cleanup_prevents_leakage():
    register_tier2_hook("held", lambda fact, evidence, context: None)
    try:
        result, _ = _check(
            "held(plant pot)",
            context=_context(
                phase="postcondition",
                completed_steps=(_step("grasp", object="plant pot"),),
            ),
        )
        assert result.verdict is Verdict.VALID
    finally:
        register_tier2_hook("held", None)
    result, _ = _check(
        "held(plant pot)",
        context=_context(
            phase="postcondition",
            completed_steps=(_step("grasp", object="plant pot"),),
        ),
    )
    assert result.confidence == 0.5
    assert "action-verdict fallback" in result.evidence
    assert "stronger verifier not installed" in result.evidence


def test_multiple_facts_are_evaluated_without_short_circuiting():
    results, facts = check_all(
        ["held(plant pot)", "counted(cups)", "object_seen(mug)"],
        {"count_value": 0, "object_detection": ["mug"]},
        _context(),
    )
    assert len(results) == 3
    assert len(facts) == 3
    assert [result.verdict for result in results] == [Verdict.UNKNOWN, Verdict.VALID, Verdict.VALID]
