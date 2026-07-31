import os

os.environ.setdefault("BT_MOCK_MODE", "true")

import py_trees  # noqa: E402
from behavior_tree.HRI.hri_2026 import createHRITask2026  # noqa: E402
from behavior_tree.HRI.hri_2026 import createBagFlowReal2026  # noqa: E402


def test_root_starts_with_operator_gate():
    root = createHRITask2026()
    assert isinstance(root, py_trees.composites.Sequence)
    assert len(root.children) == 16
    assert root.children[0].__class__.__name__ == "BtNode_PressEnterToSucceed"


def test_constant_writer_still_second():
    root = createHRITask2026()
    assert isinstance(root.children[1], py_trees.composites.Parallel)


def test_start_announcement_still_third():
    root = createHRITask2026()
    assert root.children[2].__class__.__name__ == "BtNode_Announce"
    assert root.children[2].given_msg == "HRI task started."


def test_look_at_host_tilt_is_35_degrees():
    root = createBagFlowReal2026()
    nodes = [b for b in root.iterate() if b.name == "Look at host"]
    assert len(nodes) == 1, "exactly one Look-at-host leaf in the bag flow"
    look = nodes[0]
    assert look.__class__.__name__ == "BtNode_TurnPanTilt"
    assert look.x == 0.0
    # 45 degrees up put the camera on the ceiling/crowd at follow start
    # (2026-07-02 phantom-point incident); 35 matches hri.py's "look up".
    assert look.y == 35.0


def test_host_seating_instruction_is_fourth():
    root = createHRITask2026()
    announce = root.children[3]
    assert announce.__class__.__name__ == "BtNode_Announce"
    assert announce.given_msg == "Dear host, please sit down on the sofa."


def test_real_bag_flow_wired_into_full_tree():
    root = createHRITask2026()
    assert root.children[13].name == "HRI bag flow (real follow, 2026)"


def test_production_follow_has_allowlisted_operator_stop_detector():
    root = createBagFlowReal2026()
    follow_gate = next(
        node for node in root.iterate()
        if node.name == "Follow stop detectors (s)"
    )
    assert isinstance(follow_gate, py_trees.composites.Parallel)
    assert isinstance(
        follow_gate.policy,
        py_trees.common.ParallelPolicy.SuccessOnOne,
    )
    assert any(
        node.__class__.__name__ == "BtNode_CheckFollowArrived"
        for node in follow_gate.children
    )
    assert any(
        node.__class__.__name__ == "BtNode_WaitKeyboardPress"
        and node.key == "s"
        for node in follow_gate.children
    )
