"""HangzhouGrasp composition and bounded retries."""

from collections import Counter
import json
import math

import py_trees
import pytest

from pathlib import Path as _Path
TEST_CONFIG = str(_Path(__file__).with_name("hangzhougrasp_test_config.json"))
from std_msgs.msg import Header

from behavior_tree.hangzhougrasp import hangzhougrasp as task
from behavior_tree.hangzhougrasp import config as cfg

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
    conf = c.load_config(TEST_CONFIG, enter_room=enter, poses=poses or ROUTE_POSES)
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
    assert goto_keys["Go to destination point_area"] == ["hg_pose_point_area"]
    assert goto_keys["Go to drop pose kitchen_ground_0_0"] == [
        "hg_pose_kitchen_ground_0_0"]


def test_last_table_stop_drops_at_destination(goto_keys):
    poses = {"bedroom_ground_0_0": _pose(2, 2), "bedroom_table_0_0": _pose(1, 2),
             "kitchen_table_0_0": _pose(5, 2), "point_area": _pose(3, 4)}
    conf = _with_rooms(_config(poses), ["bedroom", "kitchen"], poses)
    root = task.createHangzhouGrasp(config=conf, enable_grasp=True)
    kitchen = _by_name(root, "Room kitchen")
    assert _by_name(kitchen, "Deliver object")
    assert not any(n.name == "Drop object elsewhere" for n in kitchen.iterate())
    assert goto_keys["Go to destination point_area"][-1] == "hg_pose_point_area"


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


def test_no_grasp_builds_no_pick_or_gripper_nodes(config):
    root = task.createHangzhouGrasp(config=config, enable_grasp=False)
    banned = (task.BtNode_Tk27Pick, task.BtNode_Tk27GripperOpen, task.BtNode_SetHolding)
    assert not _nodes(root, banned)
    # The only arm command in --no-grasp is the navigation stow.
    assert {n.pose_name for n in _nodes(root, task.BtNode_Tk27SavedPose)} == {"navigation"}
    assert len(_nodes(root, task.BtNode_ScanForYolo26)) == 5
    assert not any("Deliver" in n.name or "Drop" in n.name for n in root.iterate())


@pytest.mark.parametrize("grasp", [False, True])
def test_every_navigation_is_preceded_by_a_stow(config, grasp):
    root = task.createHangzhouGrasp(config=config, enable_grasp=grasp)
    nodes = list(root.iterate())
    stows = [i for i, n in enumerate(nodes) if isinstance(n, task.BtNode_Tk27SavedPose)]
    first_goto = next(i for i, n in enumerate(nodes) if isinstance(n, task.BtNode_GotoAction))
    entering = next(n for n in nodes if n.name == "Arm to nav before entering the arena")
    assert entering.pose_name == "navigation" and stows[0] < first_goto
    for goto in (n for n in nodes if isinstance(n, task.BtNode_GotoAction)):
        stow_then = goto.parent.parent  # Retry -> "Stow then ..." Sequence
        assert stow_then.name == f"Stow then {goto.name}"
        first = stow_then.children[0].children[0]
        assert isinstance(first, task.BtNode_Tk27SavedPose)
        assert first.pose_name == "navigation"


@pytest.mark.parametrize("grasp", [False, True])
def test_every_orbbec_scan_commands_its_phase_tilt(config, grasp):
    root = task.createHangzhouGrasp(config=config, enable_grasp=grasp)
    for room in ("bedroom", "kitchen"):
        room_node = _by_name(root, f"Room {room}")
        for phase, tilt in (("Tables phase", 0.0), ("Ground phase", 0.0)):
            node = _by_name(room_node, phase)
            heads = _nodes(node, task.BtNode_HeadTilt)
            orbbec = [s for s in _nodes(node, task.BtNode_ScanForYolo26) if s.use_orbbec]
            assert heads and len(heads) == len(orbbec)
            assert {h.y for h in heads} == {tilt}
            assert {h.x for h in heads} == {0.0}


def test_ground_phase_has_no_grasp(config):
    root = task.createHangzhouGrasp(config=config, enable_grasp=True)
    for room in ("bedroom", "kitchen"):
        ground = _by_name(_by_name(root, f"Room {room}"), "Ground phase")
        assert not _nodes(ground, task.BtNode_Tk27Pick)
        assert {n.pose_name for n in _nodes(ground, task.BtNode_Tk27SavedPose)} == {"navigation"}
        assert len(_nodes(ground, task.BtNode_ScanForYolo26)) == 1


def test_constants_cover_route_and_destination(config):
    writer = task.createConstantWriter(config)
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
    monkeypatch.setattr(task, "BtNode_HeadTilt",
                        lambda **kw: Leaf("head", Status.SUCCESS, **kw))
    monkeypatch.setattr(task, "BtNode_Tk27SavedPose",
                        lambda **kw: Leaf("arm", Status.SUCCESS, **kw))
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


def test_table_stop_then_ground_drop_elsewhere_targets_first_ground(goto_keys):
    poses = {"bedroom_table_0_0": _pose(1, 2), "kitchen_table_0_0": _pose(5, 2),
             "kitchen_ground_0_0": _pose(6, 2), "kitchen_ground_0_1": _pose(6, 3),
             "point_area": _pose(3, 4)}
    conf = _with_rooms(_config(poses), ["bedroom", "kitchen"], poses)
    root = task.createHangzhouGrasp(config=conf, enable_grasp=True)
    kitchen = _by_name(root, "Room kitchen")
    assert _by_name(kitchen, "Drop object elsewhere")
    assert goto_keys["Go to drop pose kitchen_ground_0_0"] == [
        "hg_pose_kitchen_ground_0_0"]
    assert "Go to drop pose kitchen_ground_0_1" not in goto_keys


def test_enable_grasp_is_required_keyword(config):
    stops = [(s, None) for s in config.route]
    with pytest.raises(TypeError):
        task.createTablesPhase(config, stops)
    with pytest.raises(TypeError):
        task.createRoom(config, "bedroom", stops)
    with pytest.raises(TypeError):
        task.createTablesPhase(config, stops, True)
