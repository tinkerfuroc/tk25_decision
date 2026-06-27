import os

os.environ["BT_MOCK_MODE"] = "true"  # noqa: E402

import py_trees  # noqa: E402
import pytest  # noqa: E402

from behavior_tree.PickAndPlace.custom_nodes import (  # noqa: E402
    BtNode_BuildInventory, BtNode_PopWorkItem,
)
from behavior_tree.PickAndPlace import config as c  # noqa: E402


@pytest.fixture(autouse=True)
def _clear_blackboard():
    clear_fn = getattr(py_trees.blackboard.Blackboard, "clear", None)
    if callable(clear_fn):
        clear_fn()
    yield
    if callable(clear_fn):
        clear_fn()


def _write(key, value):
    cl = py_trees.blackboard.Client(name="w")
    cl.register_key(key="k", access=py_trees.common.Access.WRITE,
                    remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key))
    cl.k = value


def _read(key):
    cl = py_trees.blackboard.Client(name="r")
    cl.register_key(key="k", access=py_trees.common.Access.READ,
                    remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key))
    return cl.k


def test_build_inventory_seeds_when_empty_and_sorts_cabinet():
    # Empty upstream + explicit mock_seed: bowl (wash), pringles/apple/cereal (cabinet).
    node = BtNode_BuildInventory(
        name="build", source_pose_key=c.KEY_POSE_TABLE,
        mock_seed=["pringles", "apple", "bowl", "cereal"],
    )
    assert node.update() == py_trees.common.Status.SUCCESS
    queue = _read(c.KEY_WORK_QUEUE)
    classes = [it["destination"] for it in queue]
    # non-cabinet (wash_staging) first, then cabinet items sorted by reference_label.
    assert classes[0] == "wash_staging"
    cabinet_refs = [it["reference_label"] for it in queue if it["destination"] == "cabinet"]
    assert cabinet_refs == sorted(cabinet_refs) == ["apple", "cereal", "pringles"]
    # source_pose_key threaded onto every item.
    assert all(it["source_pose_key"] == c.KEY_POSE_TABLE for it in queue)


def test_pop_drains_then_fails_on_empty_without_raising():
    _write(c.KEY_WORK_QUEUE, [
        {"label": "bowl", "segment": None, "destination": "wash_staging",
         "reference_label": "", "source_pose_key": c.KEY_POSE_TABLE},
        {"label": "pringles", "segment": None, "destination": "cabinet",
         "reference_label": "pringles", "source_pose_key": c.KEY_POSE_TABLE},
    ])
    pop = BtNode_PopWorkItem(name="pop", place_policy="vlm")
    assert pop.update() == py_trees.common.Status.SUCCESS
    assert len(_read(c.KEY_WORK_QUEUE)) == 1
    assert pop.update() == py_trees.common.Status.SUCCESS
    assert len(_read(c.KEY_WORK_QUEUE)) == 0
    # empty -> FAILURE (loop terminator) and must NOT raise.
    assert pop.update() == py_trees.common.Status.FAILURE


def test_pop_writes_source_distinct_from_target():
    _write(c.KEY_WORK_QUEUE, [
        {"label": "bowl", "segment": None, "destination": "wash_staging",
         "reference_label": "", "source_pose_key": c.KEY_POSE_TABLE},
    ])
    BtNode_PopWorkItem(name="pop").update()
    source = _read(c.KEY_ACTIVE_SOURCE_POSE)
    target = _read(c.KEY_ACTIVE_TARGET_POSE)
    assert source is c.POSE_TABLE
    assert target is c.POSE_WASH_STAGING
    assert source is not target


def test_pop_placement_mode_vlm_vs_hardcoded():
    seed = [
        {"label": "bowl", "segment": None, "destination": "wash_staging",
         "reference_label": "", "source_pose_key": c.KEY_POSE_TABLE},
        {"label": "pringles", "segment": None, "destination": "cabinet",
         "reference_label": "pringles", "source_pose_key": c.KEY_POSE_TABLE},
    ]
    # vlm: wash -> FREE_SPACE, cabinet -> NEAR_SIMILAR.
    _write(c.KEY_WORK_QUEUE, list(seed))
    pop = BtNode_PopWorkItem(name="pop_vlm", place_policy="vlm")
    pop.update()
    assert _read("pp_active_placement_mode") == c.PLACEMENT_MODE_FREE_SPACE
    pop.update()
    assert _read("pp_active_placement_mode") == c.PLACEMENT_MODE_NEAR_SIMILAR
    assert _read("pp_active_reference_label") == "pringles"
    # hardcoded: both -> FIXED_POINT with the routing's hardcoded point.
    _write(c.KEY_WORK_QUEUE, list(seed))
    pop2 = BtNode_PopWorkItem(name="pop_hc", place_policy="hardcoded")
    pop2.update()
    assert _read("pp_active_placement_mode") == c.PLACEMENT_MODE_FIXED_POINT
    assert _read(c.KEY_ACTIVE_TARGET_POINT) is c.POINT_WASH_STAGING
    pop2.update()
    assert _read("pp_active_placement_mode") == c.PLACEMENT_MODE_FIXED_POINT
    assert _read(c.KEY_ACTIVE_TARGET_POINT) is c.POINT_CABINET_DEFAULT


def test_pop_queue_access_is_read_and_write():
    # Functional READ+WRITE: the node reads the queue and writes the shortened
    # list back through the same key without a fresh writer.
    _write(c.KEY_WORK_QUEUE, [
        {"label": "bowl", "segment": None, "destination": "wash_staging",
         "reference_label": "", "source_pose_key": c.KEY_POSE_TABLE},
    ])
    pop = BtNode_PopWorkItem(name="pop")
    pop.update()
    assert _read(c.KEY_WORK_QUEUE) == []


def test_record_event_appends_to_score_trace():
    from behavior_tree.PickAndPlace.custom_nodes import record_event
    record_event(None, phase="table", item="bowl", action="scan_and_place",
                 outcome="success", points_est=40)
    trace = _read(c.KEY_SCORE_TRACE)
    assert isinstance(trace, dict)
    assert trace["events"][-1] == {
        "phase": "table", "item": "bowl", "action": "scan_and_place",
        "outcome": "success", "points_est": 40,
    }
    assert "visited_phases" in trace and "place_policy" in trace
