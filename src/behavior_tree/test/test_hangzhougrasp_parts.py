"""Independently runnable hangzhougrasp parts."""

import dataclasses

import py_trees
import pytest

from pathlib import Path as _Path
TEST_CONFIG = str(_Path(__file__).with_name("hangzhougrasp_test_config.json"))
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from behavior_tree.hangzhougrasp import config as cfg
from behavior_tree.hangzhougrasp import hangzhougrasp as task
from behavior_tree.hangzhougrasp.parts import PARTS, createPart
from behavior_tree.hangzhougrasp.pose_index import PoseIndex
from behavior_tree.hangzhougrasp.route import build_route
from behavior_tree.hangzhougrasp.tk27_nodes import (
    BtNode_Tk27GripperOpen, BtNode_Tk27Pick, BtNode_Tk27SavedPose,
)

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
    conf = cfg.load_config(TEST_CONFIG, enter_room="bedroom", poses=POSES)
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


TK27_LEAVES = (BtNode_Tk27SavedPose, BtNode_Tk27Pick, BtNode_Tk27GripperOpen)


def test_scan_announce_part_needs_no_manipulation_nodes(config):
    root = createPart("scan-announce", config, enable_grasp=False)
    for cls in TK27_LEAVES:
        assert not _classes(root, cls)
    assert _classes(root, task.BtNode_ScanForYolo26)


def test_scan_grasp_part_contains_grasp_and_cleanup(config):
    root = _build(config, "scan-grasp")
    assert _classes(root, BtNode_Tk27Pick)
    assert any(n.name == "Retract arm on failure" for n in root.iterate())
    assert any(n.name == "Scan objects with YOLO26 SAM" for n in root.iterate())


def test_visit_table_part_only_that_table(config):
    root = _build(config, "visit-table", table=0, at=None)
    gotos = [n.name for n in _classes(root, task.BtNode_GotoAction)]
    assert gotos == ["Go to bedroom_table_0_0", "Go to bedroom_table_0_1"]
    opens = [n.name for n in _classes(root, BtNode_Tk27GripperOpen)]
    # only the pre-pick opens; visit-table never releases
    assert opens == ["Open gripper before pick"] * 2


def _grasp_trees(config):
    trees = [_build(config, name) for name in sorted(PARTS)]
    trees.append(task.createHangzhouGrasp(enable_grasp=True, config=config))
    return trees


def test_no_legacy_manipulation_in_grasp_trees(config):
    from behavior_tree.TemplateNodes import Manipulation
    legacy = (Manipulation.BtNode_Grasp, Manipulation.BtNode_MoveArmSingle,
              Manipulation.BtNode_GripperAction)
    for root in _grasp_trees(config):
        for node in root.iterate():
            assert not isinstance(node, legacy), node.name
            assert getattr(node, "action_name", None) not in (
                "start_grasp", "joint_move_action"), node.name
    assert not hasattr(task, "BtNode_Grasp")
    assert not hasattr(task, "BtNode_MoveArmSingle")


def test_every_pick_preceded_by_open(config):
    picks = 0
    for root in _grasp_trees(config):
        for node in root.iterate():
            if isinstance(node, BtNode_Tk27Pick):
                picks += 1
                siblings = node.parent.children
                assert isinstance(node.parent, py_trees.composites.Sequence)
                previous = siblings[siblings.index(node) - 1]
                assert isinstance(previous, BtNode_Tk27GripperOpen)
                assert siblings.index(node) > 0
    assert picks > 0


def test_saved_poses_are_pick45_and_navigation(config):
    root = task.createHangzhouGrasp(enable_grasp=True, config=config)
    poses = {n.pose_name for n in _classes(root, BtNode_Tk27SavedPose)}
    assert poses == {"pick45", "navigation"}


def test_part_writes_only_poses_it_uses(config):
    root = createPart("scan-announce", config, enable_grasp=False,
                      at="bedroom_table_0_1")
    assert _writes(root) == ["Write bedroom_table_0_1"]
    root = createPart("scan-announce", config, enable_grasp=False)
    assert _writes(root) == []


def test_deliver_writes_destination_only(config):
    root = _build(config, "deliver", at=None)
    assert _writes(root) == ["Write point_area"]


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


def _first_index(root, pred):
    return next(i for i, n in enumerate(root.iterate()) if pred(n))


@pytest.mark.parametrize("name", ["room", "ground"])
@pytest.mark.parametrize("at", [None, "bedroom_table_0_1"])
def test_room_part_arms_to_nav_before_first_goto(config, at, name):
    on = createPart(name, config, enable_grasp=True, room="bedroom", at=at)
    arm = _first_index(on, lambda n: n.name == "Arm to nav before first navigation")
    goto = _first_index(on, lambda n: isinstance(n, task.BtNode_GotoAction))
    assert arm < goto
    assert first_arm_is_navigation(on)
    off = createPart(name, config, enable_grasp=False, room="bedroom", at=at)
    assert not any("Arm to nav before" in n.name for n in off.iterate())


@pytest.mark.parametrize("name", ["scan-grasp", "visit-table", "deliver"])
def test_grasp_parts_with_at_arm_before_first_goto(config, name):
    root = _build(config, name, at="bedroom_table_0_1")
    arm = _first_index(root, lambda n: isinstance(n, BtNode_Tk27SavedPose))
    goto = _first_index(root, lambda n: isinstance(n, task.BtNode_GotoAction))
    assert arm < goto
    assert first_arm_is_navigation(root)


def first_arm_is_navigation(root):
    first = next(n for n in root.iterate() if isinstance(n, BtNode_Tk27SavedPose))
    return first.pose_name == "navigation"


@pytest.mark.parametrize("name", ["room", "ground"])
def test_room_and_ground_build_without_grasp(config, name):
    root = createPart(name, config, enable_grasp=False, room="kitchen")
    for cls in TK27_LEAVES + (task.BtNode_SetHolding,):
        assert not _classes(root, cls)
    if name == "room":
        assert _writes(root) == ["Write kitchen_table_0_0",
                                 "Write kitchen_ground_0_0", "Write point_area"]
    else:
        assert _writes(root) == ["Write kitchen_ground_0_0"]


# Keys a part's own subtree writes while it runs; every other hg_ key a part
# reads must come from that part's constants writer.
RUNTIME_KEYS = {cfg.KEY_SCAN_RESULT, cfg.KEY_GRASP_SCAN_RESULT,
                cfg.KEY_TARGET_LABEL, cfg.KEY_ANNOUNCEMENT, cfg.KEY_HOLDING}
Access = py_trees.common.Access
Status = py_trees.common.Status


def _scan_response():
    from sensor_msgs.msg import Image
    from tinker_vision_msgs_26.msg import Object
    from tinker_vision_msgs_26.srv import ObjectDetectionGeneralist
    rgb = Image(height=2, width=2, encoding="bgr8", step=6, data=bytes([1] * 12))
    depth = Image(height=2, width=2, encoding="16UC1", step=4, data=bytes([1] * 8))
    return ObjectDetectionGeneralist.Response(
        status=0, header=Header(frame_id="cam"), objects=[Object(cls="cup")],
        segments=[Image(height=2, width=2, encoding="8UC1", step=2,
                        data=bytes([255] * 4))],
        rgb_image=rgb, depth_image=depth, detection_source="yolo")


MISSING = []


class _Stub(py_trees.behaviour.Behaviour):
    """Leaf stub: reads/writes the keys the real node would; logs missing reads."""
    reads, writes = (), ()

    def __init__(self, name, **kw):
        super().__init__(name)
        kw.pop('name', None)
        self.kw = kw
        self.bb = self.attach_blackboard_client(name=name)
        for key in self._reads():
            self.bb.register_key(key, access=Access.READ)
        for key in self._writes():
            self.bb.register_key(key, access=Access.WRITE)

    def _reads(self):
        return [k for k in self.reads if k]

    def _writes(self):
        return [k for k in self.writes if k]

    def update(self):
        for key in self._reads():
            try:
                self.bb.get(key)
            except KeyError:
                MISSING.append((self.name, key))
                return Status.FAILURE
        for key in self._writes():
            self.bb.set(key, self._value(key), overwrite=True)
        return Status.SUCCESS

    def _value(self, key):
        return _scan_response()


def _stub(reads=(), writes=()):
    """reads/writes: lists of kwarg names (or literal keys) of the real node."""
    class S(_Stub):
        def _reads(self):
            return [self.kw.get(k) for k in reads if self.kw.get(k)]

        def _writes(self):
            return [self.kw.get(k) for k in writes if self.kw.get(k)]

        def _value(self, key):
            return _scan_response()
    return S


def _goto(name, key, **kw):
    return _stub(reads=["key"])(name=name, key=key)


@pytest.mark.parametrize("name", sorted(PARTS))
def test_part_ticks_on_empty_blackboard_with_stubbed_leaves(config, monkeypatch, name):
    _tick_stubbed(config, monkeypatch, name)


def test_stub_tick_detects_a_missing_constant(config, monkeypatch):
    from behavior_tree.hangzhougrasp import parts
    monkeypatch.setattr(parts, "_writer", lambda *a, **k: py_trees.behaviours.Success(name="none"))
    with pytest.raises(AssertionError):
        _tick_stubbed(config, monkeypatch, "deliver")


def _tick_stubbed(config, monkeypatch, name):
    """Independence: build on an empty blackboard, tick with stubbed leaves;
    no leaf may miss a key because another part would have written it."""
    BB.clear()
    MISSING.clear()
    monkeypatch.setattr(task, "BtNode_GotoAction", _goto)
    monkeypatch.setattr(task, "BtNode_Tk27SavedPose", lambda **kw: _stub()(**kw))
    monkeypatch.setattr(task, "BtNode_ScanForYolo26", lambda **kw: _stub(
        reads=["bb_source"], writes=["bb_key"])(**kw))
    monkeypatch.setattr(task, "BtNode_AnnounceDetections", lambda **kw: _stub(
        reads=["bb_source"], writes=["bb_text_key"])(**kw))
    monkeypatch.setattr(task, "BtNode_Tk27Pick", lambda **kw: _stub(
        reads=["prompt_bb_key"])(**kw))
    monkeypatch.setattr(task, "BtNode_Tk27GripperOpen", lambda **kw: _stub()(**kw))
    monkeypatch.setattr(task, "BtNode_Announce", lambda **kw: _stub()(**kw))
    root = _build(config, name)
    # every hg_ key read is written inside the part: its writer or runtime keys
    for key, md in BB.metadata.items():
        bare = key.lstrip("/")
        if bare.startswith("hg_") and md.read and bare not in RUNTIME_KEYS:
            assert md.write, f"{name}: {key} read but not written by the part"
    tree = py_trees.trees.BehaviourTree(root)
    for _ in range(300):
        tree.tick()
        if root.status in (Status.SUCCESS, Status.FAILURE):
            break
    assert not MISSING, f"{name}: cross-part dependency {MISSING}"
    assert root.status == Status.SUCCESS
