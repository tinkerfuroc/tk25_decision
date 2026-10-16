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
