"""Offline tests for the plan-time small-tree modification protocol."""
from __future__ import annotations

import pytest

from behavior_tree.GPSR import small_trees
from behavior_tree.GPSR.modifiable_nodes import (
    TEMPLATES,
    ModificationValidationError,
    apply_modifications,
    diff_trees,
    group_modifications_by_step,
    validate_modifications,
    validate_plan_modifications,
)
from behavior_tree.GPSR.tree_serialization import serialize_tree


def _find_node_id(tree, kind, *, node_type=None, name_fragment=None):
    doc = serialize_tree(tree, kind=kind)
    for node in doc["nodes"]:
        if node_type is not None and node.get("type") != node_type:
            continue
        if name_fragment is not None and name_fragment not in str(node.get("name", "")):
            continue
        return node["node_id"]
    raise AssertionError(f"no node in {kind} tree matching type={node_type} name~{name_fragment}")


def _validate(tree, action, mods):
    return validate_modifications(mods, action, tree)


# ---------------------------------------------------------------------------
# Template registry
# ---------------------------------------------------------------------------

def test_registry_is_closed_and_schemas_validate():
    # every registered handler has a template spec with the same name
    from behavior_tree.GPSR.modifiable_nodes import REGISTRY
    assert set(REGISTRY) == set(TEMPLATES)
    # required params are enforced
    spec = TEMPLATES["attribute-person-specialist"]
    with pytest.raises(ModificationValidationError):
        spec.validate_params({})
    with pytest.raises(ModificationValidationError):
        spec.validate_params({"gate": 123})
    spec.validate_params({"gate": "red jacket", "prompt": "person wearing a red jacket"})


# ---------------------------------------------------------------------------
# validate_modifications: acceptance + rejection
# ---------------------------------------------------------------------------

def test_validate_accepts_attribute_specialist_on_find_person():
    tree = small_trees.create_find_person()
    sweep_id = _find_node_id(tree, "small/find_person", name_fragment="pantilt sweep")
    ok, reason = _validate(tree, "find_person", [{
        "template": "attribute-person-specialist",
        "target_node_id": sweep_id,
        "params": {"gate": "red jacket", "prompt": "person wearing a red jacket"},
        "reason": "needs a colour specialist",
    }])
    assert ok, reason


def test_validate_rejects_unknown_template():
    tree = small_trees.create_find_person()
    ok, reason = _validate(tree, "find_person", [{
        "template": "not-a-template", "target_node_id": "small/find_person/root/3",
        "params": {},
    }])
    assert not ok and "unknown template" in reason


def test_validate_rejects_missing_target_node():
    tree = small_trees.create_find_person()
    ok, reason = _validate(tree, "find_person", [{
        "template": "attribute-person-specialist",
        "target_node_id": "small/find_person/root/999",
        "params": {"gate": "red jacket"},
    }])
    assert not ok and "not found" in reason


def test_validate_rejects_template_on_wrong_role():
    tree = small_trees.create_find_person()
    sweep_id = _find_node_id(tree, "small/find_person", name_fragment="pantilt sweep")
    # vlm-template only applies to VLM query nodes
    ok, reason = _validate(tree, "find_person", [{
        "template": "vlm-template",
        "target_node_id": sweep_id,
        "params": {"question_template": "How many {value}?"},
    }])
    assert not ok and "does not apply" in reason


def test_validate_rejects_bad_param_schema():
    tree = small_trees.create_count()
    vlm_id = _find_node_id(tree, "small/count", node_type="BtNode_VLMQuery")
    ok, reason = _validate(tree, "count", [{
        "template": "vlm-template",
        "target_node_id": vlm_id,
        "params": {"question_template": "missing the value placeholder"},
        "reason": "x",
    }])
    assert not ok and "rejected" in reason
    # the rejection is actionable: it names the requirement, not just "schema"
    assert "{value}" in reason


# ---------------------------------------------------------------------------
# vlm-template scope: count only, not vlm_fallback (Task B §1)
# ---------------------------------------------------------------------------

def test_vlm_template_applies_only_to_count():
    assert TEMPLATES["vlm-template"].applies_to == ("count_vlm_branch",)


def test_modification_targets_blurb_lists_no_vlm_fallback_id():
    # Other templates (e.g. announce-text) may legitimately target a
    # vlm_fallback node (it has an announce leaf) — only vlm-template's own
    # line must never offer a vlm_fallback id.
    from behavior_tree.GPSR.planner import _modification_targets_blurb
    blurb = _modification_targets_blurb()
    vlm_template_lines = [
        line for line in blurb.splitlines() if line.startswith("- vlm-template:")
    ]
    assert vlm_template_lines, "vlm-template line missing from blurb"
    assert "small/vlm_fallback/" not in vlm_template_lines[0]


def test_vlm_template_rejected_on_vlm_fallback_query_node():
    tree = small_trees.create_vlm_fallback()
    vlm_id = _find_node_id(tree, "small/vlm_fallback", node_type="BtNode_VLMQuery")
    ok, reason = _validate(tree, "vlm_fallback", [{
        "template": "vlm-template",
        "target_node_id": vlm_id,
        "params": {"question_template": "How many {value}?"},
    }])
    assert not ok and "does not apply" in reason


def test_validate_rejects_non_list_and_empty_list():
    tree = small_trees.create_announce()
    assert not _validate(tree, "announce", {"template": "x"})[0]
    assert _validate(tree, "announce", [])[0]


def test_validate_requires_modification_action_to_match_step():
    tree = small_trees.create_find_person()
    sweep_id = _find_node_id(tree, "small/find_person", name_fragment="pantilt sweep")
    ok, reason = _validate(tree, "find_person", [{
        "template": "attribute-person-specialist",
        "action": "find_object",  # mismatched
        "target_node_id": sweep_id,
        "params": {"gate": "red jacket"},
    }])
    assert not ok and "does not match" in reason


# ---------------------------------------------------------------------------
# plan-level validation (the planner accept loop)
# ---------------------------------------------------------------------------

def test_validate_plan_modifications_groups_and_checks():
    plan = [{"action": "find_person", "params": {}},
            {"action": "announce", "params": {}}]
    tree = small_trees.create_find_person()
    sweep_id = _find_node_id(tree, "small/find_person", name_fragment="pantilt sweep")
    mods = [{
        "template": "attribute-person-specialist",
        "action": "find_person",
        "target_node_id": sweep_id,
        "params": {"gate": "red jacket"},
        "reason": "colour specialist",
    }]
    ok, reason = validate_plan_modifications(mods, plan)
    assert ok, reason
    grouped = group_modifications_by_step(plan, mods)
    assert list(grouped) == [0]


def test_validate_plan_modifications_rejects_unmatched():
    plan = [{"action": "find_person", "params": {}}]
    mods = [{
        "template": "attribute-person-specialist",
        "target_node_id": "small/find_person/root/3",
        "params": {"gate": "red jacket"},
        # no action / no step_index -> unmatched
    }]
    ok, reason = validate_plan_modifications(mods, plan)
    assert not ok and "could not be matched" in reason


def test_validate_plan_modifications_requires_step_index_on_duplicate_action():
    plan = [{"action": "find_person", "params": {}},
            {"action": "find_person", "params": {}}]
    tree = small_trees.create_find_person()
    sweep_id = _find_node_id(tree, "small/find_person", name_fragment="pantilt sweep")
    # no step_index on a duplicated action -> rejected
    ok, reason = validate_plan_modifications(
        [{"template": "attribute-person-specialist", "target_node_id": sweep_id,
          "params": {"gate": "red jacket"}}],
        plan,
    )
    assert not ok and "could not be matched" in reason
    # with step_index it passes
    ok, reason = validate_plan_modifications(
        [{"template": "attribute-person-specialist", "target_node_id": sweep_id,
          "step_index": 1, "params": {"gate": "red jacket"}}],
        plan,
    )
    assert ok, reason


# ---------------------------------------------------------------------------
# apply_modifications: audit tuples + revert on error
# ---------------------------------------------------------------------------

def test_apply_returns_audit_tuple_for_specialist():
    tree = small_trees.create_find_person()
    sweep_id = _find_node_id(tree, "small/find_person", name_fragment="pantilt sweep")
    mods = [{
        "template": "attribute-person-specialist",
        "target_node_id": sweep_id,
        "params": {"gate": "red jacket", "prompt": "person wearing a red jacket"},
        "reason": "red jacket specialist",
    }]
    assert _validate(tree, "find_person", mods)[0]
    new_tree, applied = apply_modifications(tree, "find_person", mods)
    assert len(applied) == 1
    template, node_id, reason = applied[0]
    assert template == "attribute-person-specialist"
    assert node_id == sweep_id
    assert reason == "red jacket specialist"
    # the new tree gained the gated branch
    names = {n.name for n in new_tree.iterate()}
    assert "red jacket person branch" in names


def test_apply_targets_single_vlm_node_and_diff_is_focused():
    tree = small_trees.create_count()
    vlm_id = _find_node_id(tree, "small/count", node_type="BtNode_VLMQuery")
    mods = [{
        "template": "vlm-template",
        "target_node_id": vlm_id,
        "params": {"question_template": "How many {value} here? Answer with a number."},
        "reason": "clearer count prompt",
    }]
    assert _validate(tree, "count", mods)[0]
    new_tree, applied = apply_modifications(tree, "count", mods)
    assert len(applied) == 1
    # param-only change on a leaf: structural diff stays focused (empty) — the
    # (template, node_id, reason) audit tuple is the record of the change.
    assert diff_trees(tree, new_tree, "small/count") == []
    # the targeted node really changed
    node = new_tree
    for part in vlm_id[len("small/count/root"):].strip("/").split("/"):
        node = node.children[int(part)]
    assert node._template == "How many {value} here? Answer with a number."


def test_apply_reverts_and_records_nothing_on_handler_error():
    # A directive that passes validate but whose handler raises is a silent
    # no-op: the previous subtree is kept and nothing is recorded.
    tree = small_trees.create_announce()
    announce_id = _find_node_id(tree, "small/announce", node_type="BtNode_Announce")
    mods = [{
        "template": "announce-text",
        "target_node_id": announce_id,
        "params": {"text": "hi"},
        "reason": "x",
    }]
    # create_announce's leaf is bb_source-sourced, so the handler rejects it
    assert _validate(tree, "announce", mods)[0]
    new_tree, applied = apply_modifications(tree, "announce", mods)
    assert new_tree is tree and applied == []


def test_apply_with_no_modifications_is_identity():
    tree = small_trees.create_find_object()
    new_tree, applied = apply_modifications(tree, "find_object", None)
    assert new_tree is tree and applied == []
    new_tree2, applied2 = apply_modifications(tree, "find_object", [])
    assert new_tree2 is tree and applied2 == []


# ---------------------------------------------------------------------------
# pan-tilt-sweep + search-spots rebuilds
# ---------------------------------------------------------------------------

def test_pan_tilt_sweep_rebuild():
    tree = small_trees.create_find_object()
    sweep_id = _find_node_id(tree, "small/find_object", name_fragment="pantilt sweep")
    mods = [{
        "template": "pan-tilt-sweep",
        "target_node_id": sweep_id,
        "params": {"pan_deg": [0.0, 90.0], "tilt_deg": [0.0]},
        "reason": "object is far right",
    }]
    assert _validate(tree, "find_object", mods)[0]
    new_tree, applied = apply_modifications(tree, "find_object", mods)
    assert len(applied) == 1
    names = {n.name for n in new_tree.iterate()}
    assert "find_object pan=+90 tilt=+0" in names


def test_search_spots_capacity():
    tree = small_trees.create_search_object()
    root_id = "small/search_object/root"
    mods = [{
        "template": "search-spots",
        "target_node_id": root_id,
        "params": {"capacity": 3},
        "reason": "this room has 3 spots",
    }]
    assert _validate(tree, "search_object", mods)[0]
    new_tree, applied = apply_modifications(tree, "search_object", mods)
    assert len(applied) == 1
    spot_count = sum(1 for n in new_tree.children if "search spot" in str(n.name))
    assert spot_count == 3


# ---------------------------------------------------------------------------
# diff_trees
# ---------------------------------------------------------------------------

def test_diff_identical_trees_is_empty():
    tree = small_trees.create_find_person()
    assert diff_trees(tree, tree, "small/find_person") == []


def test_diff_reports_added_for_specialist():
    tree = small_trees.create_find_person()
    sweep_id = _find_node_id(tree, "small/find_person", name_fragment="pantilt sweep")
    mods = [{
        "template": "attribute-person-specialist",
        "target_node_id": sweep_id,
        "params": {"gate": "red jacket"},
        "reason": "x",
    }]
    new_tree, _ = apply_modifications(tree, "find_person", mods)
    diff = diff_trees(tree, new_tree, "small/find_person")
    added = [record for record in diff if record["reason"] == "added"]
    assert added, "specialist insertion must add nodes"
    assert any("red jacket specialist scan" in str(record.get("name", "")) for record in added)


# ---------------------------------------------------------------------------
# No-gpsr_trace fallback (review L1): ModParamSpec must construct even when
# the module falls back to its own plain ParamSpec.
# ---------------------------------------------------------------------------

def test_mod_param_spec_constructs_under_no_gpsr_trace_fallback(monkeypatch):
    import importlib
    import sys

    # Force `from gpsr_trace.ir import ParamSpec` (and the events import) to
    # raise ImportError, then reload the module under test so it takes the
    # `except ImportError` branch — this is what "old installs without
    # gpsr_trace" actually hit at import time.
    monkeypatch.setitem(sys.modules, "gpsr_trace", None)
    monkeypatch.setitem(sys.modules, "gpsr_trace.ir", None)
    monkeypatch.setitem(sys.modules, "gpsr_trace.events", None)
    import behavior_tree.GPSR.modifiable_nodes as mn

    try:
        importlib.reload(mn)
        # The fallback ParamSpec is a frozen dataclass with (types, required,
        # validator) — before the L1 fix, ModParamSpec's generated __init__
        # only accepted `requirement`, so this call raised TypeError.
        spec = mn.ModParamSpec((list, tuple), validator=lambda v: isinstance(v, list), requirement="must be a list")
        assert spec.accepts([1, 2, 3])
        assert not spec.accepts("nope")
        assert spec.requirement == "must be a list"
        # A plain (non-Mod) fallback ParamSpec still works too.
        plain = mn.ParamSpec((list, tuple))
        assert plain.accepts([1])
        assert not plain.accepts(3)
    finally:
        # Restore the real gpsr_trace-backed module for every other test.
        monkeypatch.undo()
        importlib.reload(mn)
