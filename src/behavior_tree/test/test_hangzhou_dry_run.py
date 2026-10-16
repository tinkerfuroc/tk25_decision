"""Checks specific to mixed real-service / motion-mock dry-runs."""
from behavior_tree.HomeServiceHangzhou.config import load_config, validate
from behavior_tree.HomeServiceHangzhou.dry_run import mock_config, build_tree
from behavior_tree.HomeServiceHangzhou.bringup import missing_requirements


def test_dry_run_keeps_calibration_placeholders_untouched():
    original = load_config()
    effective = mock_config(original)
    validate(effective)
    assert set(effective["switches"]) == set(original["switches"]) == {"living_room", "bedroom"}
    assert original["registration_pose"] is None
    assert original["arm_stow"] == "navigation"
    assert all(value is None for value in original["room_poses"].values())


def test_motion_clients_are_absent_but_real_perception_remains():
    from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction, BtNode_Approach
    from behavior_tree.HomeServiceHangzhou.ros_nodes import Tk27SavedPose
    from behavior_tree.TemplateNodes.Audio import BtNode_Announce
    from behavior_tree.HomeServiceHangzhou.ros_nodes import SettledPanTilt
    from behavior_tree.HomeServiceHangzhou.perception import DetectScene
    root = build_tree(load_config())
    nodes = list(root.iterate())
    assert not any(isinstance(n, (BtNode_GotoAction, BtNode_Approach, Tk27SavedPose))
                   for n in nodes)
    assert any("[MOCK]" in n.name for n in nodes)
    for kind in (BtNode_Announce, SettledPanTilt, DetectScene):
        assert any(isinstance(n, kind) for n in nodes)
    assert not any(getattr(n, "mock_mode", False) for n in nodes)


def test_preflight_requires_real_head_but_no_motion_servers():
    graph = {"services": {"/announce", "/behaviour_detection/detect", "/litter_detection/detect"},
             "actions": {"/listen_action", "/phrase_extraction_action", "/feature_extraction_service"},
             "camera": True, "head": False, "head_feedback": False}
    assert missing_requirements(graph, load_config()) == ["head", "head_feedback"]
    graph["head"] = True
    assert missing_requirements(graph, load_config()) == ["head_feedback"]
    graph["head_feedback"] = True
    assert missing_requirements(graph, load_config()) == []

