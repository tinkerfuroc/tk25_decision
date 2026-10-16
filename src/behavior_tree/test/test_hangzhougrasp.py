"""HangzhouGrasp composition, bounded retries and grasp payload contracts."""

from collections import Counter
import json
import math

import py_trees
import pytest
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from tinker_vision_msgs_26.msg import Object
from tinker_vision_msgs_26.srv import ObjectDetectionGeneralist

from behavior_tree.hangzhougrasp import hangzhougrasp as task
from behavior_tree.hangzhougrasp import config as cfg
from behavior_tree.hangzhougrasp.custom_nodes import BtNode_SelectGraspTarget

Status = py_trees.common.Status
BB = py_trees.blackboard.Blackboard


@pytest.fixture(autouse=True)
def clear_blackboard():
    BB.clear()
    yield
    BB.clear()


def _pose(x, y):
    from geometry_msgs.msg import PoseStamped
    p = PoseStamped(header=Header(frame_id="map"))
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.w = 1.0
    return p


ROUTE_POSES = {
    "bedroom_table_0_0": _pose(1, 2), "bedroom_table_0_1": _pose(1, 3),
    "bedroom_ground_0_0": _pose(2, 2),
    "kitchen_table_0_0": _pose(5, 2), "kitchen_ground_0_0": _pose(6, 2),
    "point_area": _pose(3, 4),
}


def _config(poses=None, rooms=("bedroom", "kitchen"), enter="bedroom"):
    import behavior_tree.hangzhougrasp.config as c
    conf = c.load_config(enter_room=enter, poses=poses or ROUTE_POSES)
    return conf


@pytest.fixture
def config(monkeypatch):
    conf = _config()
    return _with_rooms(conf, ["bedroom", "kitchen"], ROUTE_POSES)


def _with_rooms(conf, rooms, poses):
    import dataclasses
    from behavior_tree.hangzhougrasp.pose_index import PoseIndex
    from behavior_tree.hangzhougrasp.route import build_route
    return dataclasses.replace(
        conf, rooms=list(rooms), route=build_route(PoseIndex(poses), rooms))


def result(labels=("bottle", "cup")):
    rgb = Image(height=2, width=2, encoding="bgr8", step=6, data=bytes([1] * 12))
    depth = Image(height=2, width=2, encoding="16UC1", step=4, data=bytes([1] * 8))
    return ObjectDetectionGeneralist.Response(
        status=0, header=Header(frame_id="camera_optical_frame"),
        objects=[Object(cls=label) for label in labels],
        segments=[Image(height=2, width=2, encoding="8UC1", step=2,
                        data=bytes([255 - i] * 4)) for i, _ in enumerate(labels)],
        rgb_image=rgb, depth_image=depth, detection_source="yolo",
    )



def _nodes(root, cls):
    return [n for n in root.iterate() if isinstance(n, cls)]


def _gotos(root):
    return [n.name for n in root.iterate()
            if isinstance(n, task.BtNode_GotoAction)]


def test_route_order_in_tree(config):
    root = task.createHangzhouGrasp(config=config, enable_grasp=True)
    gotos = _gotos(root)
    expected = ["bedroom_table_0_0", "bedroom_table_0_1", "bedroom_ground_0_0",
                "kitchen_table_0_0", "kitchen_ground_0_0"]
    visits = [g for g in gotos if g.startswith(("Go to bedroom", "Go to kitchen"))]
    first = [v for v in visits if any(v.endswith(e) for e in expected)]
    order = []
    for v in first:
        if v[6:] not in order:
            order.append(v[6:])
    assert order == expected


def _by_name(root, name):
    return next(n for n in root.iterate() if n.name == name)


def test_enter_room_uses_deliver_other_room_uses_drop(config):
    root = task.createHangzhouGrasp(config=config, enable_grasp=True)
    bedroom = _by_name(root, "Room bedroom")
    kitchen = _by_name(root, "Room kitchen")
    assert any(n.name == "Deliver object" for n in bedroom.iterate())
    assert not any(n.name == "Drop object elsewhere" for n in bedroom.iterate())
    assert any(n.name == "Drop object elsewhere" for n in kitchen.iterate())


@pytest.fixture
def goto_keys(monkeypatch):
    keys = {}
    monkeypatch.setattr(
        task, "_gotoRetry",
        lambda name, key, config: keys.setdefault(name, []).append(key) or
        py_trees.behaviours.Success(name))
    return keys


def test_next_pose_is_next_stop_first_viewpoint(config, goto_keys):
    root = task.createHangzhouGrasp(config=config, enable_grasp=True)
    drop = _by_name(_by_name(root, "Room kitchen"), "Drop object elsewhere")
    assert drop is not None
    # bedroom (enter room) delivers to the destination, kitchen drops at next pose
    assert goto_keys["Go to drop pose"] == [
        "hg_pose_point_area", "hg_pose_kitchen_ground_0_0"]


def test_last_table_stop_drops_at_destination(goto_keys):
    poses = {"bedroom_ground_0_0": _pose(2, 2), "bedroom_table_0_0": _pose(1, 2),
             "kitchen_table_0_0": _pose(5, 2), "point_area": _pose(3, 4)}
    conf = _with_rooms(_config(poses), ["bedroom", "kitchen"], poses)
    root = task.createHangzhouGrasp(config=conf, enable_grasp=True)
    kitchen = _by_name(root, "Room kitchen")
    assert _by_name(kitchen, "Deliver object")
    assert not any(n.name == "Drop object elsewhere" for n in kitchen.iterate())
    assert goto_keys["Go to drop pose"][-1] == "hg_pose_point_area"


def test_room_with_only_ground_poses():
    poses = {"bedroom_ground_0_0": _pose(2, 2), "point_area": _pose(3, 4)}
    conf = _with_rooms(_config(poses), ["bedroom"], poses)
    for grasp in (False, True):
        assert task.createHangzhouGrasp(config=conf, enable_grasp=grasp)


def test_room_with_only_tables():
    poses = {"bedroom_table_0_0": _pose(2, 2), "point_area": _pose(3, 4)}
    conf = _with_rooms(_config(poses), ["bedroom"], poses)
    for grasp in (False, True):
        assert task.createHangzhouGrasp(config=conf, enable_grasp=grasp)


def test_no_grasp_builds_no_manipulation_nodes(config):
    root = task.createHangzhouGrasp(config=config, enable_grasp=False)
    banned = (task.BtNode_MoveArmSingle, task.BtNode_Grasp,
              task.BtNode_GripperAction, task.BtNode_SetHolding)
    assert not _nodes(root, banned)
    assert len(_nodes(root, task.BtNode_ScanForYolo26)) == 5
    assert not any("Deliver" in n.name or "Drop" in n.name for n in root.iterate())


def test_ground_phase_has_no_grasp(config):
    root = task.createHangzhouGrasp(config=config, enable_grasp=True)
    for room in ("bedroom", "kitchen"):
        ground = _by_name(_by_name(root, f"Room {room}"), "Ground phase")
        assert not _nodes(ground, task.BtNode_Grasp)
        assert not _nodes(ground, task.BtNode_MoveArmSingle)
        assert len(_nodes(ground, task.BtNode_ScanForYolo26)) == 1


def test_constants_cover_route_and_destination(config):
    writer = task.createConstantWriter(config, enable_grasp=False)
    names = {n.name for n in writer.children}
    assert "Write point_area" in names and "Write kitchen_table_0_0" in names
    assert len(writer.children) == 6


def test_invalid_enable_grasp_rejected(config):
    with pytest.raises(TypeError):
        task.createHangzhouGrasp(config=config, enable_grasp=1)


def test_ground_miss_is_success_and_navigation_retries_bounded(
        config, monkeypatch):
    counts = Counter()

    class Leaf(py_trees.behaviour.Behaviour):
        def __init__(self, kind, status, **kw):
            super().__init__(kw["name"])
            self.status = status

        def update(self):
            counts[self.name] += 1
            return self.status

    monkeypatch.setattr(task, "BtNode_GotoAction",
                        lambda **kw: Leaf("nav", Status.SUCCESS, **kw))
    monkeypatch.setattr(task, "BtNode_ScanForYolo26",
                        lambda **kw: Leaf("scan", Status.FAILURE, **kw))
    root = task.createHangzhouGrasp(config=config, enable_grasp=False)
    tree = py_trees.trees.BehaviourTree(root)
    for _ in range(100):
        tree.tick()
        if root.status != Status.RUNNING:
            break
    assert root.status == Status.SUCCESS
    assert counts["Scan objects with YOLO26 SAM"] == 5 * config.scan_retry_limit

    counts.clear()
    monkeypatch.setattr(task, "BtNode_GotoAction",
                        lambda **kw: Leaf("nav", Status.FAILURE, **kw))
    root = task.createHangzhouGrasp(config=config, enable_grasp=False)
    tree = py_trees.trees.BehaviourTree(root)
    for _ in range(50):
        tree.tick()
        if root.status != Status.RUNNING:
            break
    assert root.status == Status.FAILURE
    assert counts["Go to bedroom_table_0_0"] == config.nav_retry_limit


def selector():
    return BtNode_SelectGraspTarget("Select", "scan", "selected", "label")


def test_nearest_mask_and_original_response_preserved():
    source = result()
    BB.set("scan", source)
    assert selector().update() == Status.SUCCESS
    selected = BB.get("selected")
    assert selected is not source
    assert [o.cls for o in selected.objects] == ["bottle"]
    assert selected.segments == [source.segments[0]]
    assert selected.rgb_image == source.rgb_image
    assert selected.depth_image == source.depth_image
    assert len(source.objects) == len(source.segments) == 2
    assert BB.get("label") == "bottle"


@pytest.mark.parametrize("bad", ["empty", "mask_count", "size", "no_rgb", "no_frame", "zero_mask"])
def test_invalid_grasp_bundle_clears_old_target(bad):
    node = selector()
    BB.set("scan", result())
    assert node.update() == Status.SUCCESS
    value = result()
    if bad == "empty":
        value.objects = []
    elif bad == "mask_count":
        value.segments = []
    elif bad == "size":
        value.depth_image.width = 3
    elif bad == "no_rgb":
        value.rgb_image.data = b""
    elif bad == "no_frame":
        value.header.frame_id = ""
    else:
        value.segments[0].data = bytes(4)
    BB.set("scan", value)
    assert node.update() == Status.FAILURE
    assert BB.get("selected") is None and BB.get("label") == ""


