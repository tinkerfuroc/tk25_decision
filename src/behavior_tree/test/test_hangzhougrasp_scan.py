from types import SimpleNamespace

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_AnnounceDetections
from behavior_tree.TemplateNodes.Vision import BtNode_ScanForYolo26
from behavior_tree.hangzhougrasp.hangzhougrasp import createScanAnnounce, createGrasp
from behavior_tree.hangzhougrasp.tk27_nodes import BtNode_Tk27SavedPose


def _config():
    return SimpleNamespace(
        detection_service="object_detection_yolo26_sam", prompt="cup",
        label_map={"cup": "杯子"}, announce_service="announce",
        scan_retry_limit=2, arm_retry_limit=2, announce_retry_limit=2,
        grasp_retry_limit=2, saved_pose_action="/manipulation/move_to_saved_pose",
        arm_pose_table="pick45", arm_pose_navigating="navigation",
        pick_action="/manipulation/pick", pick_plan_effort="high",
        gripper_action="/xarm_gripper/gripper_action",
        gripper_open_position=0.0, gripper_open_tolerance=0.025,
        gripper_max_effort=50.0, head_pan_deg=0.0, table_scan_tilt_deg=20.0,
        ground_scan_tilt_deg=0.0, head_retry_limit=1,
    )


def _nodes(root, cls):
    return [n for n in root.iterate() if isinstance(n, cls)]


def test_scan_announce_uses_yolo26_sam_and_orbbec():
    scans = _nodes(createScanAnnounce(_config(), surface="table"), BtNode_ScanForYolo26)
    assert len(scans) == 1
    assert scans[0].service_name == "object_detection_yolo26_sam"
    assert scans[0].use_orbbec


def test_scan_announce_has_announce_node():
    root = createScanAnnounce(_config(), surface="table")
    assert len(_nodes(root, BtNode_AnnounceDetections)) == 1
    assert not _nodes(root, BtNode_Tk27SavedPose)


def test_scans_request_no_image_payload():
    for root in (createScanAnnounce(_config(), surface="table"), createGrasp(_config())):
        for scan in _nodes(root, BtNode_ScanForYolo26):
            assert not (scan.return_rgb_image or scan.return_depth_image
                        or scan.return_segments)


def test_grasp_scan_is_realsense_after_table_pose():
    root = createGrasp(_config())
    order = [n for n in root.iterate()
             if isinstance(n, (BtNode_Tk27SavedPose, BtNode_ScanForYolo26))]
    assert isinstance(order[0], BtNode_Tk27SavedPose)
    assert order[0].pose_name == "pick45"
    assert isinstance(order[1], BtNode_ScanForYolo26)
    assert not order[1].use_orbbec
    assert order[1].object == "cup"
    assert not _nodes(root, BtNode_AnnounceDetections)


# ---- a speech failure is logged, not a miss ----
import pytest

from behavior_tree.hangzhougrasp import hangzhougrasp as task

Status = py_trees.common.Status


def _leaf(status):
    behaviour = (py_trees.behaviours.Success if status == Status.SUCCESS
                 else py_trees.behaviours.Failure)
    return lambda **kw: behaviour(name=kw["name"])


def _tick(root):
    tree = py_trees.trees.BehaviourTree(root)
    for _ in range(50):
        tree.tick()
        if root.status in (Status.SUCCESS, Status.FAILURE):
            break
    return root.status


def test_announce_failure_is_logged_and_scan_still_succeeds(monkeypatch):
    monkeypatch.setattr(task, "BtNode_ScanForYolo26", _leaf(Status.SUCCESS))
    monkeypatch.setattr(task, "BtNode_AnnounceDetections", _leaf(Status.FAILURE))
    monkeypatch.setattr(task, "BtNode_HeadTilt", _leaf(Status.SUCCESS))
    root = createScanAnnounce(_config(), surface="table")
    assert _tick(root) == Status.SUCCESS
    logged = [n for n in root.iterate() if n.name == "Log announce failure"]
    assert len(logged) == 1 and logged[0].status == Status.SUCCESS


def test_scan_miss_still_fails_even_with_announce_fallback(monkeypatch):
    monkeypatch.setattr(task, "BtNode_ScanForYolo26", _leaf(Status.FAILURE))
    monkeypatch.setattr(task, "BtNode_AnnounceDetections", _leaf(Status.SUCCESS))
    monkeypatch.setattr(task, "BtNode_HeadTilt", _leaf(Status.SUCCESS))
    assert _tick(createScanAnnounce(_config(), surface="table")) == Status.FAILURE


def test_announce_success_does_not_log(monkeypatch):
    monkeypatch.setattr(task, "BtNode_ScanForYolo26", _leaf(Status.SUCCESS))
    monkeypatch.setattr(task, "BtNode_AnnounceDetections", _leaf(Status.SUCCESS))
    monkeypatch.setattr(task, "BtNode_HeadTilt", _leaf(Status.SUCCESS))
    root = createScanAnnounce(_config(), surface="table")
    assert _tick(root) == Status.SUCCESS
    logged = [n for n in root.iterate() if n.name == "Log announce failure"]
    assert logged[0].status == Status.INVALID
