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
    assert len(root.children) == 10


def test_child_order_and_types():
    kids = I.createInspection().children
    assert isinstance(kids[0], py_trees.composites.Parallel)              # constants
    assert isinstance(kids[1], py_trees.decorators.Retry)                 # arm nav-pose
    assert isinstance(kids[2], py_trees.composites.Parallel)             # ready announce + pan-tilt
    assert kids[2].name == "Announce ready + aim pan-tilt"
    assert isinstance(kids[3], py_trees.decorators.Retry)                 # door detection
    assert kids[3].decorated.__class__.__name__ == "BtNode_DoorDetection"
    assert kids[4].__class__.__name__ == "BtNode_Announce"               # door open
    assert kids[4].given_msg == "door open"
    assert isinstance(kids[5], py_trees.composites.Sequence)             # to inspection
    assert kids[6].__class__.__name__ == "BtNode_Announce"              # self-intro
    assert kids[6].given_msg == "Dear referees, I am Tinker."
    assert kids[7].__class__.__name__ == "BtNode_PressEnterToSucceed"   # wait Enter
    assert kids[8].__class__.__name__ == "BtNode_Announce"              # leaving
    assert kids[8].given_msg == "Heading to the exit."
    assert isinstance(kids[9], py_trees.composites.Sequence)            # to exit


def test_ready_announce_and_pan_tilt_run_in_parallel():
    kids = I.createInspection().children
    ready = kids[2]
    assert isinstance(ready, py_trees.composites.Parallel)
    names = {c.__class__.__name__ for c in ready.children}
    assert "BtNode_Announce" in names
    assert "BtNode_TurnPanTilt" in names

    announce = next(c for c in ready.children if c.__class__.__name__ == "BtNode_Announce")
    assert announce.given_msg == "I am ready for inspection, please open the door"

    pan_tilt = next(c for c in ready.children if c.__class__.__name__ == "BtNode_TurnPanTilt")
    assert pan_tilt.x == 0.0   # pan degrees
    assert pan_tilt.y == 45.0  # tilt degrees


def test_ready_announce_precedes_door_detection():
    kids = I.createInspection().children
    # readiness parallel comes before the door-detection wait
    assert isinstance(kids[2], py_trees.composites.Parallel)
    assert isinstance(kids[3], py_trees.decorators.Retry)
    assert kids[3].decorated.__class__.__name__ == "BtNode_DoorDetection"


def test_door_open_announced_immediately_after_detection():
    kids = I.createInspection().children
    assert kids[3].decorated.__class__.__name__ == "BtNode_DoorDetection"
    assert kids[4].__class__.__name__ == "BtNode_Announce"
    assert kids[4].given_msg == "door open"


def test_verbose_self_intro_removed():
    # root-level announces (the readiness announce is nested in the parallel)
    msgs = _announce_messages(I.createInspection())
    assert msgs == ["door open", "Dear referees, I am Tinker.", "Heading to the exit."]
    assert not any("at home service robot" in (m or "") for m in msgs)
