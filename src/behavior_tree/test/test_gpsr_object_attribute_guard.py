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
