import importlib
import sys
import types

import py_trees


class _SuccessNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, *args, **kwargs):
        super().__init__(name=name)

    def update(self):
        return py_trees.common.Status.SUCCESS


class _FailNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, *args, **kwargs):
        super().__init__(name=name)

    def update(self):
        return py_trees.common.Status.FAILURE


def _recording_node(events):
    class _RecordingNode(py_trees.behaviour.Behaviour):
        def __init__(self, name, *args, **kwargs):
            super().__init__(name=name)

        def update(self):
            events.append(self.name)
            return py_trees.common.Status.SUCCESS

    return _RecordingNode


def _install_stubs(monkeypatch):
    rclpy = types.ModuleType("rclpy")

    class _Time:
        def to_msg(self):
            return None

    rclpy.time = types.SimpleNamespace(Time=lambda: _Time())
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)

    geom = types.ModuleType("geometry_msgs.msg")
    geom.Point = type("Point", (), {"__init__": lambda self, x=0.0, y=0.0, z=0.0: None})
    geom.Quaternion = type(
        "Quaternion",
        (),
        {"__init__": lambda self, x=0.0, y=0.0, z=0.0, w=1.0: None},
    )
    geom.Pose = type("Pose", (), {"__init__": lambda self, position=None, orientation=None: None})
    geom.PoseStamped = type("PoseStamped", (), {"__init__": lambda self, header=None, pose=None: None})
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", geom)

    std = types.ModuleType("std_msgs.msg")
    std.Header = type("Header", (), {"__init__": lambda self, stamp=None, frame_id="": None})
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std)

    audio = types.ModuleType("behavior_tree.TemplateNodes.Audio")
    audio.BtNode_Announce = _SuccessNode
    audio.BtNode_GetConfirmation = _SuccessNode
    audio.BtNode_PhraseExtraction = _SuccessNode
    monkeypatch.setitem(sys.modules, "behavior_tree.TemplateNodes.Audio", audio)

    base = types.ModuleType("behavior_tree.TemplateNodes.BaseBehaviors")
    base.BtNode_WriteToBlackboard = _SuccessNode
    monkeypatch.setitem(sys.modules, "behavior_tree.TemplateNodes.BaseBehaviors", base)

    manip = types.ModuleType("behavior_tree.TemplateNodes.Manipulation")
    manip.BtNode_GripperAction = _SuccessNode
    manip.BtNode_MoveArmSingle = _SuccessNode
    monkeypatch.setitem(sys.modules, "behavior_tree.TemplateNodes.Manipulation", manip)

    nav = types.ModuleType("behavior_tree.TemplateNodes.Navigation")
    nav.BtNode_GotoAction = _SuccessNode
    monkeypatch.setitem(sys.modules, "behavior_tree.TemplateNodes.Navigation", nav)

    vision = types.ModuleType("behavior_tree.TemplateNodes.Vision")
    vision.BtNode_DoorDetection = _SuccessNode
    vision.BtNode_FeatureExtraction = _SuccessNode
    vision.BtNode_SeatRecommend = _SuccessNode
    vision.BtNode_TurnPanTilt = _SuccessNode
    monkeypatch.setitem(sys.modules, "behavior_tree.TemplateNodes.Vision", vision)

    receptionist_custom = types.ModuleType("behavior_tree.Receptionist.customNodes")
    receptionist_custom.BtNode_CombinePerson = _SuccessNode
    receptionist_custom.BtNode_Confirm = _SuccessNode
    receptionist_custom.BtNode_HeadTrackingAction = _SuccessNode
    receptionist_custom.BtNode_Introduce = _SuccessNode
    monkeypatch.setitem(sys.modules, "behavior_tree.Receptionist.customNodes", receptionist_custom)


def _import_hri(monkeypatch):
    _install_stubs(monkeypatch)
    sys.modules.pop("behavior_tree.HRI.hri", None)
    return importlib.import_module("behavior_tree.HRI.hri")


def _tick_tree(root, max_ticks=20):
    clear_fn = getattr(py_trees.blackboard.Blackboard, "clear", None)
    if callable(clear_fn):
        clear_fn()
    tree = py_trees.trees.BehaviourTree(root)
    for _ in range(max_ticks):
        tree.tick()
        if root.status != py_trees.common.Status.RUNNING:
            return root.status
    return root.status


def test_hri_task_full_success_with_mocked_nodes(monkeypatch):
    hri = _import_hri(monkeypatch)
    root = hri.createHRITask()
    assert _tick_tree(root) == py_trees.common.Status.SUCCESS


def test_hri_arrival_trigger_uses_mock_fallback_when_real_trigger_fails(monkeypatch):
    hri = _import_hri(monkeypatch)
    monkeypatch.setattr(hri, "BtNode_DoorDetection", _FailNode)
    root = hri.createArrivalTrigger()
    assert _tick_tree(root) == py_trees.common.Status.SUCCESS
    assert any(
        isinstance(child, hri.BtNode_MockArrivalTrigger) for child in root.children
    )


def test_two_way_introductions_run_both_directions(monkeypatch):
    hri = _import_hri(monkeypatch)
    events = []
    monkeypatch.setattr(hri, "BtNode_Introduce", _recording_node(events))

    root = hri.createTwoWayIntroduction()
    assert _tick_tree(root) == py_trees.common.Status.SUCCESS
    assert "Introduce guest1 to guest2" in events
    assert "Introduce guest2 to guest1" in events


def test_bag_flow_executes_grasp_follow_drop_sequence(monkeypatch):
    hri = _import_hri(monkeypatch)
    events = []
    recording = _recording_node(events)
    for name in [
        "BtNode_Announce",
        "BtNode_MoveArmSingle",
        "BtNode_GripperAction",
        "BtNode_GotoAction",
    ]:
        monkeypatch.setattr(hri, name, recording)

    root = hri.createBagFlow()
    assert _tick_tree(root) == py_trees.common.Status.SUCCESS
    order = [
        "Move arm to handover pose",
        "Open gripper for bag",
        "Move to drop area proxy",
        "Move arm to drop pose",
        "Open gripper to drop bag",
    ]
    positions = [events.index(name) for name in order]
    assert positions == sorted(positions)
