import os

os.environ.setdefault("BT_MOCK_MODE", "true")

import py_trees  # noqa: E402
from behavior_tree.Inspection import inspection as I  # noqa: E402


def _announce_messages(root):
    return [
        c.given_msg
        for c in root.children
        if c.__class__.__name__ == "BtNode_Announce"
    ]


def test_root_is_memory_sequence_named_inspection_root():
    root = I.createInspection()
    assert isinstance(root, py_trees.composites.Sequence)
    assert root.name == "Inspection Root"
    assert root.memory is True
    assert len(root.children) == 9


def test_child_order_and_types():
    kids = I.createInspection().children
    assert isinstance(kids[0], py_trees.composites.Parallel)              # constants
    assert isinstance(kids[1], py_trees.decorators.Retry)                 # arm nav-pose
    assert isinstance(kids[2], py_trees.decorators.Retry)                 # door detection
    assert kids[2].decorated.__class__.__name__ == "BtNode_DoorDetection"
    assert kids[3].__class__.__name__ == "BtNode_Announce"               # NEW
    assert kids[3].given_msg == "door open"
    assert isinstance(kids[4], py_trees.composites.Sequence)             # to inspection
    assert kids[5].__class__.__name__ == "BtNode_Announce"              # self-intro
    assert kids[5].given_msg == "Dear referees, I am Tinker."
    assert kids[6].__class__.__name__ == "BtNode_PressEnterToSucceed"   # wait Enter
    assert kids[7].__class__.__name__ == "BtNode_Announce"              # leaving
    assert kids[7].given_msg == "Heading to the exit."
    assert isinstance(kids[8], py_trees.composites.Sequence)            # to exit


def test_door_open_announced_immediately_after_detection():
    kids = I.createInspection().children
    assert kids[2].decorated.__class__.__name__ == "BtNode_DoorDetection"
    assert kids[3].__class__.__name__ == "BtNode_Announce"
    assert kids[3].given_msg == "door open"


def test_verbose_self_intro_removed():
    msgs = _announce_messages(I.createInspection())
    assert msgs == ["door open", "Dear referees, I am Tinker.", "Heading to the exit."]
    assert not any("at home service robot" in (m or "") for m in msgs)
