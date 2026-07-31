"""Tests for the restaurant Phase-1 waving scan:

* ``BtNode_QueueWavingCandidates`` pose-proximity dedup + on-screen count
  summary + fail-on-nothing-new.
* ``createScanForUpToNCustomers`` structure: skip gate, bounded retry, and the
  per-detection count announce.

The queue node is pure blackboard logic, so we tick it directly with lightweight
stand-in poses (anything exposing ``.point.x/.y`` -- matches ``PointStamped``).
"""
from __future__ import annotations

import types

import py_trees
import pytest

from behavior_tree.Restaurant.config import (
    KEY_ACTIVE_CUSTOMER_ID,
    KEY_CUSTOMER_QUEUE,
    KEY_WAVING_DETECT_SUMMARY,
    KEY_WAVING_PERSON_PICTURES,
    KEY_WAVING_PERSON_POSES,
)
from behavior_tree.Restaurant.state_nodes import BtNode_QueueWavingCandidates


def _pose(x: float, y: float):
    """A minimal PointStamped stand-in that ``_pose_xy`` can read."""
    return types.SimpleNamespace(point=types.SimpleNamespace(x=float(x), y=float(y)))


def _set_bb(**values):
    clear_fn = getattr(py_trees.blackboard.Blackboard, "clear", None)
    if callable(clear_fn):
        clear_fn()
    bb = py_trees.blackboard.Client(name="test_writer")
    for key in values:
        bb.register_key(
            key=key,
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key),
        )
    for key, value in values.items():
        setattr(bb, key, value)


def _get_bb(*keys):
    bb = py_trees.blackboard.Client(name="test_reader")
    for key in keys:
        bb.register_key(
            key=key,
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key),
        )
    return {key: getattr(bb, key) for key in keys}


def _make_queue_node():
    return BtNode_QueueWavingCandidates(
        name="queue waving",
        all_poses_key=KEY_WAVING_PERSON_POSES,
        all_pictures_key=KEY_WAVING_PERSON_PICTURES,
        queue_key=KEY_CUSTOMER_QUEUE,
        active_id_key=KEY_ACTIVE_CUSTOMER_ID,
        max_candidates=2,
        dedup_radius_m=0.3,
        summary_key=KEY_WAVING_DETECT_SUMMARY,
    )


def _tick_once(node):
    tree = py_trees.trees.BehaviourTree(node)
    tree.tick()
    return node.status


# --------------------------------------------------------------------------- #
# BtNode_QueueWavingCandidates
# --------------------------------------------------------------------------- #
def test_two_distinct_poses_are_both_queued_with_count_summary():
    _set_bb(**{
        KEY_WAVING_PERSON_POSES: [_pose(1.0, 0.0), _pose(3.0, 0.0)],
        KEY_WAVING_PERSON_PICTURES: [],
        KEY_CUSTOMER_QUEUE: [],
        KEY_ACTIVE_CUSTOMER_ID: None,
        KEY_WAVING_DETECT_SUMMARY: "",
    })
    assert _tick_once(_make_queue_node()) == py_trees.common.Status.SUCCESS
    bb = _get_bb(KEY_CUSTOMER_QUEUE, KEY_WAVING_DETECT_SUMMARY)
    assert len(bb[KEY_CUSTOMER_QUEUE]) == 2
    assert bb[KEY_WAVING_DETECT_SUMMARY] == "I detected 2 waving customers."


def test_pose_within_dedup_radius_of_queued_caller_is_skipped():
    # One caller already queued at (1.0, 0.0). New detections: a near-duplicate
    # (0.2 m away, inside 0.3 m) and a genuinely new one at (5.0, 0.0).
    existing = [{"id": 1, "pose": _pose(1.0, 0.0), "timestamp": 1.0,
                 "confidence": 1.0, "status": "queued"}]
    _set_bb(**{
        KEY_WAVING_PERSON_POSES: [_pose(1.2, 0.0), _pose(5.0, 0.0)],
        KEY_WAVING_PERSON_PICTURES: [],
        KEY_CUSTOMER_QUEUE: existing,
        KEY_ACTIVE_CUSTOMER_ID: None,
        KEY_WAVING_DETECT_SUMMARY: "",
    })
    assert _tick_once(_make_queue_node()) == py_trees.common.Status.SUCCESS
    queue = _get_bb(KEY_CUSTOMER_QUEUE)[KEY_CUSTOMER_QUEUE]
    xs = sorted(item["pose"].point.x for item in queue)
    assert xs == [1.0, 5.0]  # near-duplicate dropped, distinct one added


def test_all_duplicate_detections_fail_with_nothing_new():
    existing = [{"id": 1, "pose": _pose(1.0, 0.0), "timestamp": 1.0,
                 "confidence": 1.0, "status": "queued"}]
    _set_bb(**{
        KEY_WAVING_PERSON_POSES: [_pose(1.1, 0.1)],  # within 0.3 m of the caller
        KEY_WAVING_PERSON_PICTURES: [],
        KEY_CUSTOMER_QUEUE: existing,
        KEY_ACTIVE_CUSTOMER_ID: None,
        KEY_WAVING_DETECT_SUMMARY: "",
    })
    assert _tick_once(_make_queue_node()) == py_trees.common.Status.FAILURE
    assert len(_get_bb(KEY_CUSTOMER_QUEUE)[KEY_CUSTOMER_QUEUE]) == 1  # unchanged


def test_dedup_also_covers_already_served_callers():
    # A served caller (status="done") still occupies the spot; a re-detection
    # there must not be re-queued.
    served = [{"id": 1, "pose": _pose(2.0, 2.0), "timestamp": 1.0,
               "confidence": 1.0, "status": "done"}]
    _set_bb(**{
        KEY_WAVING_PERSON_POSES: [_pose(2.1, 2.0)],
        KEY_WAVING_PERSON_PICTURES: [],
        KEY_CUSTOMER_QUEUE: served,
        KEY_ACTIVE_CUSTOMER_ID: None,
        KEY_WAVING_DETECT_SUMMARY: "",
    })
    assert _tick_once(_make_queue_node()) == py_trees.common.Status.FAILURE


def test_single_detection_summary_is_singular():
    _set_bb(**{
        KEY_WAVING_PERSON_POSES: [_pose(1.0, 0.0)],
        KEY_WAVING_PERSON_PICTURES: [],
        KEY_CUSTOMER_QUEUE: [],
        KEY_ACTIVE_CUSTOMER_ID: None,
        KEY_WAVING_DETECT_SUMMARY: "",
    })
    assert _tick_once(_make_queue_node()) == py_trees.common.Status.SUCCESS
    assert _get_bb(KEY_WAVING_DETECT_SUMMARY)[KEY_WAVING_DETECT_SUMMARY] == (
        "I detected 1 waving customer."
    )


def test_empty_detection_list_fails():
    _set_bb(**{
        KEY_WAVING_PERSON_POSES: [],
        KEY_WAVING_PERSON_PICTURES: [],
        KEY_CUSTOMER_QUEUE: [],
        KEY_ACTIVE_CUSTOMER_ID: None,
        KEY_WAVING_DETECT_SUMMARY: "",
    })
    assert _tick_once(_make_queue_node()) == py_trees.common.Status.FAILURE


# --------------------------------------------------------------------------- #
# createScanForUpToNCustomers structure
# --------------------------------------------------------------------------- #
@pytest.fixture(scope="module")
def scan_tree():
    from behavior_tree.Restaurant.restaurants import createScanForUpToNCustomers
    return createScanForUpToNCustomers(
        [(0.0, 35.0), (30.0, 35.0), (-30.0, 35.0)], n_gate=2, max_sweeps=4
    )


def test_scan_has_skip_gate_for_already_queued_caller(scan_tree):
    from behavior_tree.Restaurant.state_nodes import BtNode_QueueHasQueued
    gate = next((n for n in scan_tree.iterate()
                 if n.name == "Already have a queued caller?"), None)
    assert isinstance(gate, BtNode_QueueHasQueued)
    assert gate.n_gate == 1


def test_scan_retry_bounds_sweeps_at_max_sweeps(scan_tree):
    retries = [n for n in scan_tree.iterate()
               if isinstance(n, py_trees.decorators.Retry)]
    assert len(retries) == 1
    assert retries[0].num_failures == 4


def test_scan_announces_detection_count_from_summary_key(scan_tree):
    announces = [n for n in scan_tree.iterate()
                 if getattr(n, "bb_source", None) == KEY_WAVING_DETECT_SUMMARY]
    # one per scan position
    assert len(announces) == 3


def test_scan_per_position_gate_uses_n_gate(scan_tree):
    from behavior_tree.Restaurant.state_nodes import BtNode_QueueHasQueued
    per_pos = [n for n in scan_tree.iterate()
               if isinstance(n, BtNode_QueueHasQueued)
               and n.name == "Enough customers queued already?"]
    assert per_pos and all(g.n_gate == 2 for g in per_pos)


# --------------------------------------------------------------------------- #
# Referee-view announcement (first = full spiel, subsequent = short reminder)
# --------------------------------------------------------------------------- #
def test_referee_guard_checkifempty_semantics():
    """The CheckIfEmpty guard drives first-vs-subsequent: falsy flag -> FAILURE
    (fall through to full spiel), truthy flag -> SUCCESS (reminder wins)."""
    from behavior_tree.nodes.BaseBehaviors import BtNode_CheckIfEmpty
    from behavior_tree.Restaurant.config import KEY_REFEREE_ANNOUNCED

    _set_bb(**{KEY_REFEREE_ANNOUNCED: False})
    falsy = BtNode_CheckIfEmpty(name="g_false", bb_source=KEY_REFEREE_ANNOUNCED)
    falsy.setup(node=object())
    assert _tick_once(falsy) == py_trees.common.Status.FAILURE

    _set_bb(**{KEY_REFEREE_ANNOUNCED: True})
    truthy = BtNode_CheckIfEmpty(name="g_true", bb_source=KEY_REFEREE_ANNOUNCED)
    truthy.setup(node=object())
    assert _tick_once(truthy) == py_trees.common.Status.SUCCESS


def test_referee_factory_has_guard_lines_and_latch():
    """Factory subtree contains the flag guard, all four spiel lines, the short
    reminder, and a latch write of True to the flag key."""
    from behavior_tree.Restaurant.restaurants import (
        _create_referee_view_announcement,
        REFEREE_VIEW_LINES,
        REFEREE_VIEW_REMINDER,
    )
    from behavior_tree.Restaurant.config import KEY_REFEREE_ANNOUNCED
    from behavior_tree.nodes.BaseBehaviors import (
        BtNode_CheckIfEmpty,
        BtNode_WriteToBlackboard,
    )

    nodes = list(_create_referee_view_announcement().iterate())

    guards = [n for n in nodes
              if isinstance(n, BtNode_CheckIfEmpty)
              and n.bb_source == KEY_REFEREE_ANNOUNCED]
    assert len(guards) == 1

    latches = [n for n in nodes
               if isinstance(n, BtNode_WriteToBlackboard)
               and n.bb_key == KEY_REFEREE_ANNOUNCED and n.object is True]
    assert len(latches) == 1

    msgs = [getattr(n, "given_msg", None) for n in nodes]
    for line in REFEREE_VIEW_LINES:
        assert line in msgs
    assert REFEREE_VIEW_REMINDER in msgs


def test_referee_reminder_branch_is_first():
    """The guarded reminder branch must be the Selector's FIRST child; the
    unguarded full spiel SECOND. (Reversed, the spiel would fire every time.)"""
    from behavior_tree.Restaurant.restaurants import _create_referee_view_announcement
    from behavior_tree.nodes.BaseBehaviors import BtNode_CheckIfEmpty

    selector = _create_referee_view_announcement().children[0]
    first_branch, second_branch = selector.children[0], selector.children[1]
    assert any(isinstance(n, BtNode_CheckIfEmpty) for n in first_branch.iterate())
    assert not any(isinstance(n, BtNode_CheckIfEmpty) for n in second_branch.iterate())


def test_scan_has_referee_spiel_once_per_position(scan_tree):
    """One referee-view factory per scan position: the first spiel line and the
    reminder each appear once per position (3 in the fixture)."""
    from behavior_tree.Restaurant.restaurants import (
        REFEREE_VIEW_LINES,
        REFEREE_VIEW_REMINDER,
    )
    msgs = [getattr(n, "given_msg", None) for n in scan_tree.iterate()]
    assert msgs.count(REFEREE_VIEW_LINES[0]) == 3
    assert msgs.count(REFEREE_VIEW_REMINDER) == 3


def test_collect_phase_inits_referee_flag_once_first():
    """createCollectOrdersPhaseItems initializes the latch to False exactly once,
    as the phase's first child (before the order-collect retries)."""
    from behavior_tree.Restaurant.order_intake_items import createCollectOrdersPhaseItems
    from behavior_tree.Restaurant.config import KEY_REFEREE_ANNOUNCED
    from behavior_tree.nodes.BaseBehaviors import BtNode_WriteToBlackboard

    phase = createCollectOrdersPhaseItems()
    inits = [n for n in phase.iterate()
             if isinstance(n, BtNode_WriteToBlackboard)
             and n.bb_key == KEY_REFEREE_ANNOUNCED and n.object is False]
    assert len(inits) == 1
    assert phase.children[0] is inits[0]
