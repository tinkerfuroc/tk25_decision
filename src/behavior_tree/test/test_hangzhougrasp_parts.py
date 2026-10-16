"""Independently runnable hangzhougrasp parts."""

import dataclasses

import py_trees
import pytest
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from behavior_tree.hangzhougrasp import config as cfg
from behavior_tree.hangzhougrasp import hangzhougrasp as task
from behavior_tree.hangzhougrasp.parts import PARTS, createPart
from behavior_tree.hangzhougrasp.pose_index import PoseIndex
from behavior_tree.hangzhougrasp.route import build_route

BB = py_trees.blackboard.Blackboard


@pytest.fixture(autouse=True)
def clear_blackboard():
    BB.clear()
    yield
    BB.clear()


def _pose(x, y):
    p = PoseStamped(header=Header(frame_id="map"))
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.w = 1.0
    return p


POSES = {
    "bedroom_table_0_0": _pose(1, 2), "bedroom_table_0_1": _pose(1, 3),
    "bedroom_table_1_0": _pose(4, 3),
    "bedroom_ground_0_0": _pose(2, 2),
    "kitchen_table_0_0": _pose(5, 2), "kitchen_ground_0_0": _pose(6, 2),
    "point_area": _pose(3, 4),
}


@pytest.fixture
def config():
    conf = cfg.load_config(enter_room="bedroom", poses=POSES)
    rooms = ["bedroom", "kitchen"]
    return dataclasses.replace(
        conf, rooms=rooms, route=build_route(PoseIndex(POSES), rooms))


ARGS = dict(room="bedroom", table=0, at="bedroom_table_0_0",
            next_pose="kitchen_table_0_0")


def _build(config, name, **kw):
    args = dict(ARGS)
    args.update(kw)
    return createPart(name, config, enable_grasp=True, **args)


def _classes(root, cls):
    return [n for n in root.iterate() if isinstance(n, cls)]


def _writes(root):
    writer = root.children[0]
    return [c.name for c in writer.children]


def test_parts_listed():
    assert set(PARTS) == {"scan-announce", "scan-grasp", "visit-table", "deliver",
                          "drop-elsewhere", "ground", "room", "full"}


@pytest.mark.parametrize("name", sorted(PARTS))
def test_each_part_builds_alone(config, name):
    assert isinstance(_build(config, name), py_trees.behaviour.Behaviour)


def test_scan_announce_part_needs_no_manipulation_nodes(config):
    root = createPart("scan-announce", config, enable_grasp=False)
    for cls in (task.BtNode_MoveArmSingle, task.BtNode_Grasp,
                task.BtNode_GripperAction):
        assert not _classes(root, cls)
    assert _classes(root, task.BtNode_ScanForYolo26)


def test_scan_grasp_part_contains_grasp_and_cleanup(config):
    root = _build(config, "scan-grasp")
    assert _classes(root, task.BtNode_Grasp)
    assert any(n.name == "Retract arm on failure" for n in root.iterate())


def test_visit_table_part_only_that_table(config):
    root = _build(config, "visit-table", table=0, at=None)
    gotos = [n.name for n in _classes(root, task.BtNode_GotoAction)]
    assert gotos == ["Go to bedroom_table_0_0", "Go to bedroom_table_0_1"]
    assert not _classes(root, task.BtNode_GripperAction)


def test_part_writes_only_poses_it_uses(config):
    root = createPart("scan-announce", config, enable_grasp=False,
                      at="bedroom_table_0_1")
    assert _writes(root) == ["Write bedroom_table_0_1"]
    root = createPart("scan-announce", config, enable_grasp=False)
    assert _writes(root) == []


def test_deliver_writes_destination_and_arm_only(config):
    root = _build(config, "deliver", at=None)
    assert sorted(_writes(root)) == [
        "Write arm navigating", "Write point_area"]


def test_missing_arguments_fail(config):
    for name, kw in [("visit-table", dict(room=None)),
                     ("visit-table", dict(table=None)),
                     ("ground", dict(room=None)), ("room", dict(room=None)),
                     ("drop-elsewhere", dict(next_pose=None))]:
        with pytest.raises(ValueError, match="requires"):
            _build(config, name, **kw)


def test_unknown_part_fails(config):
    with pytest.raises(ValueError, match="unknown part"):
        _build(config, "nope")


def test_unknown_room_or_table_fails(config):
    with pytest.raises(ValueError, match="room"):
        _build(config, "room", room="attic")
    with pytest.raises(ValueError, match="table"):
        _build(config, "visit-table", table=9)
    with pytest.raises(ValueError, match="pose"):
        _build(config, "scan-announce", at="nowhere")
    with pytest.raises(ValueError, match="pose"):
        _build(config, "drop-elsewhere", next_pose="nowhere")


@pytest.mark.parametrize("name", ["scan-grasp", "visit-table", "deliver",
                                  "drop-elsewhere"])
def test_grasp_part_without_enable_grasp_fails(config, name):
    with pytest.raises(ValueError, match="arm"):
        createPart(name, config, enable_grasp=False, **ARGS)


@pytest.mark.parametrize("name", sorted(PARTS))
def test_part_reads_only_constants_it_writes(config, name):
    """Independence: every pose/arm constant a part reads, its own writer writes.

    Scan, grasp-bundle and announcement keys are written at runtime by nodes
    inside the same creator; the cross-part inputs are the constants.
    """
    BB.clear()
    _build(config, name)
    checked = 0
    for key, md in BB.metadata.items():
        if key.lstrip("/").startswith(("hg_pose_", "hg_arm_")) and md.read:
            checked += 1
            assert md.write, f"{name}: {key} read but not written by the part"
    assert checked or name == "scan-announce"
