"""L1a — reject a find_object/grasp object param that invents attributes.

Sim battery run 016 (2026-08-27): the LLM's plan named ``object: "red bowl"``
for a command that only said "bowl" (the spawned YCB bowl is white). Vision
scanned for "red bowl" four times, never matched (nothing red is in the
scene), and the target burned its whole replan budget on identical-plan
rejections.

``validate_plan`` gains a check: a ``find_object``/``grasp`` step's
``object`` param is rejected when it (i) is not itself a known object name /
category word / generic placeholder, (ii) contains a token that IS a known
object name (a reducible subset), and (iii) has other tokens that do not
appear in the command text — i.e. the LLM invented a descriptive attribute
the command never named. A param that copies an attribute the command DOES
use (e.g. "red mug" when the command says "red mug") must still pass.
"""
from __future__ import annotations

from behavior_tree.GPSR.planner_validators import validate_plan


KNOWN_OBJECTS = {"bowl", "mug", "coke", "apple"}


def test_invented_attribute_object_is_rejected():
    plan = [
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "find_object", "params": {"object": "red bowl"}},
    ]
    ok, reason = validate_plan(
        plan, "bring me the bowl from the kitchen", {"goto", "find_object"},
        known_locations={"kitchen"},
        known_objects=KNOWN_OBJECTS,
    )
    assert not ok
    assert reason == (
        'object "red bowl" adds attributes not in the command; use "bowl"'
    )


def test_grasp_with_invented_attribute_is_also_rejected():
    plan = [
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "grasp", "params": {"object": "red bowl"}},
    ]
    ok, reason = validate_plan(
        plan, "bring me the bowl from the kitchen", {"goto", "grasp"},
        known_locations={"kitchen"},
        known_objects=KNOWN_OBJECTS,
    )
    assert not ok
    assert 'use "bowl"' in reason


def test_attribute_named_in_command_is_accepted():
    plan = [
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "find_object", "params": {"object": "red mug"}},
    ]
    ok, reason = validate_plan(
        plan, "bring me the red mug from the kitchen", {"goto", "find_object"},
        known_locations={"kitchen"},
        known_objects=KNOWN_OBJECTS,
    )
    assert ok, reason


def test_plain_known_object_is_accepted():
    plan = [
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "find_object", "params": {"object": "bowl"}},
    ]
    ok, reason = validate_plan(
        plan, "bring me the bowl from the kitchen", {"goto", "find_object"},
        known_locations={"kitchen"},
        known_objects=KNOWN_OBJECTS,
    )
    assert ok, reason


def test_unknown_multiword_object_with_no_known_subset_is_accepted():
    # "vision system is open-vocabulary" -- an object that is not itself known
    # AND contains no known-object token subset is not this rule's business
    # (nothing to reduce it to); leave it to the vision system.
    plan = [
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "find_object", "params": {"object": "blue gizmo"}},
    ]
    ok, reason = validate_plan(
        plan, "bring me the blue gizmo from the kitchen", {"goto", "find_object"},
        known_locations={"kitchen"},
        known_objects=KNOWN_OBJECTS,
    )
    assert ok, reason


def test_generic_placeholder_object_is_accepted():
    plan = [
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "find_object", "params": {"object": "item"}},
    ]
    ok, reason = validate_plan(
        plan, "bring me the item from the kitchen", {"goto", "find_object"},
        known_locations={"kitchen"},
        known_objects=KNOWN_OBJECTS,
    )
    assert ok, reason


def test_generic_word_inside_a_legitimate_object_name_is_not_a_false_match():
    # W-1 (round-4 review, HIGH): "Cheez-It" tokenizes to ["cheez", "it"] --
    # _GENERIC_OBJECT_WORDS must NEVER enter the known-object token-subset
    # match, or "it" gets treated as a matched known-object subset and the
    # LLM is told to `use "it"` -- a nonsensical, actively counterproductive
    # suggestion, and a false rejection of a legitimate free-text object
    # that copies the command's own wording.
    plan = [
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "find_object", "params": {"object": "box of cheez it"}},
    ]
    # No category word ("snack") in the command -- kept isolated to the
    # attribute-guard rule under test; a category word would separately
    # trigger the UNRELATED category-to-instance-collapse rule.
    ok, reason = validate_plan(
        plan, "bring me the cheez it from the kitchen",
        {"goto", "find_object"},
        known_locations={"kitchen"},
        known_objects=KNOWN_OBJECTS,
    )
    assert ok, reason


def test_no_rejection_reason_ever_suggests_a_generic_word():
    from behavior_tree.GPSR.planner_validators import _GENERIC_OBJECT_WORDS

    candidates = [
        "box of cheez it",       # "it"
        "the item box",          # "item" -- not itself a known object either
        "some thing container",  # "thing"
        "an object holder",      # "object"
    ]
    for obj in candidates:
        plan = [
            {"action": "goto", "params": {"location": "kitchen"}},
            {"action": "find_object", "params": {"object": obj}},
        ]
        ok, reason = validate_plan(
            plan, "bring me the bowl from the kitchen",
            {"goto", "find_object"},
            known_locations={"kitchen"},
            known_objects=KNOWN_OBJECTS,
        )
        if not ok:
            for word in _GENERIC_OBJECT_WORDS:
                assert f'use "{word}"' not in reason, (obj, reason)


def test_multiword_known_object_with_invented_attribute_is_rejected():
    # W-2 (round-4 review, MEDIUM): constants.json's real possible_objects
    # catalogue stores multi-word names underscore-joined ("white_shirt");
    # an invented attribute on top of one must be caught the same way a
    # single-word known object's is, not silently accepted because the
    # known name's tokens were never a literal subset-of-tokens match
    # against a single underscore-joined string.
    known_objects = KNOWN_OBJECTS | {"white_shirt"}
    plan = [
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "find_object", "params": {"object": "dirty white shirt"}},
    ]
    ok, reason = validate_plan(
        plan, "bring me the white shirt from the kitchen",
        {"goto", "find_object"},
        known_locations={"kitchen"},
        known_objects=known_objects,
    )
    assert not ok
    assert "white shirt" in reason or "white_shirt" in reason
    assert "dirty" in reason


def test_multiword_known_object_named_exactly_is_accepted():
    known_objects = KNOWN_OBJECTS | {"instant_noodles"}
    plan = [
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "find_object", "params": {"object": "instant noodles"}},
    ]
    ok, reason = validate_plan(
        plan, "bring me the instant noodles from the kitchen",
        {"goto", "find_object"},
        known_locations={"kitchen"},
        known_objects=known_objects,
    )
    assert ok, reason


def test_pluralized_invented_attribute_object_is_rejected():
    # W-3 (round-4 review, LOW/MEDIUM): the LLM pluralizing ("red bowls" vs.
    # the command's singular "bowl") must not be enough to dodge the guard
    # -- a near-exact repeat of run-016's shape.
    plan = [
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "find_object", "params": {"object": "red bowls"}},
    ]
    ok, reason = validate_plan(
        plan, "bring me the bowl from the kitchen", {"goto", "find_object"},
        known_locations={"kitchen"},
        known_objects=KNOWN_OBJECTS,
    )
    assert not ok
    assert 'use "bowl"' in reason


def test_check_is_inactive_when_known_objects_not_supplied():
    # Opt-in, like known_locations: a caller that doesn't pass known_objects
    # gets today's behaviour (no rejection) — only the two-layer per-target
    # planner is expected to pass it.
    plan = [
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "find_object", "params": {"object": "red bowl"}},
    ]
    ok, reason = validate_plan(
        plan, "bring me the bowl from the kitchen", {"goto", "find_object"},
        known_locations={"kitchen"},
    )
    assert ok, reason
