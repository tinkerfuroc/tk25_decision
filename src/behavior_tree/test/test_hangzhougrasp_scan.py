from types import SimpleNamespace

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_AnnounceDetections
from behavior_tree.TemplateNodes.Manipulation import BtNode_MoveArmSingle
from behavior_tree.TemplateNodes.Vision import BtNode_ScanForYolo26
from behavior_tree.hangzhougrasp.hangzhougrasp import createScanAnnounce


def _config():
    return SimpleNamespace(
        detection_service="object_detection_yolo26_sam", prompt="cup",
        label_map={"cup": "杯子"}, announce_service="announce",
        scan_retry_limit=2, arm_retry_limit=2, announce_retry_limit=2,
        arm_action="arm_action",
    )


def _nodes(root, cls):
    return [n for n in root.iterate() if isinstance(n, cls)]


def test_scan_announce_uses_yolo26_sam_and_orbbec():
    scans = _nodes(createScanAnnounce(_config()), BtNode_ScanForYolo26)
    assert len(scans) == 1
    assert scans[0].service_name == "object_detection_yolo26_sam"
    assert scans[0].use_orbbec


def test_scan_announce_has_announce_node():
    root = createScanAnnounce(_config())
    assert len(_nodes(root, BtNode_AnnounceDetections)) == 1
    assert not _nodes(root, BtNode_MoveArmSingle)


def test_grasp_variant_requests_payload_and_moves_arm():
    root = createScanAnnounce(_config(), grasp=True)
    scan = _nodes(root, BtNode_ScanForYolo26)[0]
    assert scan.return_rgb_image and scan.return_depth_image
    assert scan.return_segments
    order = [n for n in root.iterate()
             if isinstance(n, (BtNode_MoveArmSingle, BtNode_ScanForYolo26))]
    assert isinstance(order[0], BtNode_MoveArmSingle)


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


@pytest.mark.parametrize("grasp", [False, True])
def test_announce_failure_is_logged_and_scan_still_succeeds(monkeypatch, grasp):
    monkeypatch.setattr(task, "BtNode_ScanForYolo26", _leaf(Status.SUCCESS))
    monkeypatch.setattr(task, "BtNode_AnnounceDetections", _leaf(Status.FAILURE))
    monkeypatch.setattr(task, "BtNode_MoveArmSingle", _leaf(Status.SUCCESS))
    root = createScanAnnounce(_config(), grasp=grasp)
    assert _tick(root) == Status.SUCCESS
    logged = [n for n in root.iterate() if n.name == "Log announce failure"]
    assert len(logged) == 1 and logged[0].status == Status.SUCCESS


def test_scan_miss_still_fails_even_with_announce_fallback(monkeypatch):
    monkeypatch.setattr(task, "BtNode_ScanForYolo26", _leaf(Status.FAILURE))
    monkeypatch.setattr(task, "BtNode_AnnounceDetections", _leaf(Status.SUCCESS))
    assert _tick(createScanAnnounce(_config())) == Status.FAILURE


def test_announce_success_does_not_log(monkeypatch):
    monkeypatch.setattr(task, "BtNode_ScanForYolo26", _leaf(Status.SUCCESS))
    monkeypatch.setattr(task, "BtNode_AnnounceDetections", _leaf(Status.SUCCESS))
    root = createScanAnnounce(_config())
    assert _tick(root) == Status.SUCCESS
    logged = [n for n in root.iterate() if n.name == "Log announce failure"]
    assert logged[0].status == Status.INVALID
