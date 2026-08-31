"""L1b — scan-time query reduction (round-4 battery fix, run 016).

Even with the L1a plan-validation guard, an object query can still reach the
vision scan unknown (a "red bowl" the LLM happened to word before any replan
caught it, or a genuinely spawned-but-undescribed attribute). Sim battery run
016: the LLM's plan named ``object: "red bowl"`` for a command that only
said "bowl" (the spawned YCB bowl is white) -- four full pan/tilt sweeps for
"red bowl" never matched, burning the whole replan budget.

``reduce_unknown_object_query`` (orchestrator.py) computes the token-subset
reduction ("red bowl" -> "bowl"); ``BtNode_ReduceObjectQuery``
(small_trees.py) is a one-shot guard that applies it on the blackboard
exactly once per find_object subtree instance, wired as a second branch of a
Selector alongside the original pantilt sweep in ``create_find_object`` (see
that function) so a full-sweep failure gets ONE retry with the reduced query
before failing for good.

Run with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 (ROS pytest plugins break
collection).
"""
from __future__ import annotations

from unittest import mock

import pytest


# ---------------------------------------------------------------------------
# reduce_unknown_object_query (pure function)
# ---------------------------------------------------------------------------

@pytest.fixture()
def known_objects(monkeypatch):
    from behavior_tree.GPSR import orchestrator as orch
    names = {"bowl", "mug", "coke", "apple"}
    monkeypatch.setattr(orch, "KNOWN_OBJECT_NAMES", names)
    monkeypatch.setattr(orch, "KNOWN_OBJECT_PROMPTS", {n: n for n in names})
    return names


def test_reduces_unknown_attributed_query_to_known_subset(known_objects):
    from behavior_tree.GPSR.orchestrator import reduce_unknown_object_query
    assert reduce_unknown_object_query("red bowl") == "bowl"


def test_no_reduction_when_query_already_known(known_objects):
    from behavior_tree.GPSR.orchestrator import reduce_unknown_object_query
    assert reduce_unknown_object_query("bowl") is None


def test_no_reduction_when_no_known_subset(known_objects):
    from behavior_tree.GPSR.orchestrator import reduce_unknown_object_query
    assert reduce_unknown_object_query("blue gizmo") is None


def test_no_reduction_for_empty_query(known_objects):
    from behavior_tree.GPSR.orchestrator import reduce_unknown_object_query
    assert reduce_unknown_object_query("") is None
    assert reduce_unknown_object_query(None) is None


def test_reduces_multiword_known_object_query(monkeypatch):
    # W-2 (round-4 review, MEDIUM): "instant_noodles" (underscore-joined,
    # matching KNOWN_OBJECT_NAMES' storage convention) must be reachable
    # from a space-worded, attributed query -- not just a bare single-word
    # known name.
    from behavior_tree.GPSR import orchestrator as orch
    from behavior_tree.GPSR.orchestrator import reduce_unknown_object_query

    names = {"bowl", "instant_noodles"}
    monkeypatch.setattr(orch, "KNOWN_OBJECT_NAMES", names)
    monkeypatch.setattr(orch, "KNOWN_OBJECT_PROMPTS", {})
    assert reduce_unknown_object_query("spicy instant noodles") == "instant noodles"


def test_reduces_pluralized_known_object_query(known_objects):
    # W-3 (round-4 review, LOW/MEDIUM): a pluralized attributed query
    # ("red bowls") must still reduce, tolerant of the QUERY's own
    # pluralization, not just an exact-cased known name.
    from behavior_tree.GPSR.orchestrator import reduce_unknown_object_query
    assert reduce_unknown_object_query("red bowls") == "bowl"


def test_multiword_known_object_query_named_exactly_is_not_reduced(monkeypatch):
    from behavior_tree.GPSR import orchestrator as orch
    from behavior_tree.GPSR.orchestrator import reduce_unknown_object_query

    names = {"bowl", "instant_noodles"}
    monkeypatch.setattr(orch, "KNOWN_OBJECT_NAMES", names)
    monkeypatch.setattr(orch, "KNOWN_OBJECT_PROMPTS", {})
    assert reduce_unknown_object_query("instant noodles") is None


# ---------------------------------------------------------------------------
# BtNode_ReduceObjectQuery (one-shot guard node)
# ---------------------------------------------------------------------------

def _make_node():
    from behavior_tree.GPSR.small_trees import BtNode_ReduceObjectQuery
    node = BtNode_ReduceObjectQuery.__new__(BtNode_ReduceObjectQuery)
    node.feedback_message = ""
    node._reduced = False
    node._client = mock.Mock()
    return node


def test_reduces_and_writes_prompt_once(known_objects):
    import py_trees as pytree
    from behavior_tree.GPSR.small_trees import bb_keys

    node = _make_node()
    node._client.get.return_value = "red bowl"

    status = node.update()

    assert status == pytree.common.Status.SUCCESS
    node._client.set.assert_called_once_with(
        bb_keys.TARGET_OBJECT_PROMPT, "bowl", overwrite=True
    )
    assert node.feedback_message == 'reduced query "red bowl" -> "bowl"'


def test_second_call_fails_dead_already_reduced(known_objects):
    import py_trees as pytree

    node = _make_node()
    node._client.get.return_value = "red bowl"
    first = node.update()
    assert first == pytree.common.Status.SUCCESS
    node._client.set.reset_mock()

    second = node.update()

    assert second == pytree.common.Status.FAILURE
    node._client.set.assert_not_called()


def test_no_reduction_possible_fails(known_objects):
    import py_trees as pytree

    node = _make_node()
    node._client.get.return_value = "blue gizmo"

    status = node.update()

    assert status == pytree.common.Status.FAILURE
    node._client.set.assert_not_called()


def test_missing_prompt_key_fails(known_objects):
    import py_trees as pytree

    node = _make_node()
    node._client.get.side_effect = KeyError("gpsr/target_object_prompt")

    status = node.update()

    assert status == pytree.common.Status.FAILURE


# ---------------------------------------------------------------------------
# create_find_object structural wiring: strict-then-reduced Selector
# ---------------------------------------------------------------------------

def test_create_find_object_has_reduced_retry_branch():
    from behavior_tree.GPSR.small_trees import create_find_object

    tree = create_find_object()
    names = [getattr(n, "name", "") for n in _walk(tree)]
    sweeps = [n for n in names if "pantilt sweep" in n]
    # Strict sweep + reduced-retry sweep, both still classified find_object_sweep
    assert len(sweeps) == 2
    assert any("reduce object query" in n for n in names)


def _walk(node):
    yield node
    for child in getattr(node, "children", []) or []:
        yield from _walk(child)
