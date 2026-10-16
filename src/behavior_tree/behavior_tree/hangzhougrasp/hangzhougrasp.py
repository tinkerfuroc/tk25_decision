"""Phase factories and mission composition, matching HRI/PickAndPlace style."""

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_AnnounceDetections
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArmSingle
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import BtNode_ScanForGeneralist

from .config import (
    KEY_POSE_SCAN, KEY_POSE_DESTINATION, KEY_ARM_NAVIGATING, KEY_ARM_TABLE,
    KEY_SCAN_RESULT, KEY_GRASP_SCAN_RESULT, KEY_GRASP_VISION_RES,
    KEY_TARGET_LABEL, KEY_GRASP_LABEL, KEY_ANNOUNCEMENT, load_config,
)
from .custom_nodes import BtNode_SelectGraspTarget


def createConstantWriter(config, *, enable_grasp=False):
    root = py_trees.composites.Parallel(
        name="Write HangzhouGrasp constants",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    writes = [
        ("Write scan pose", KEY_POSE_SCAN, config.pose_scan),
        ("Write destination pose", KEY_POSE_DESTINATION, config.pose_destination),
    ]
    if enable_grasp:
        writes.extend([
            ("Write arm navigating", KEY_ARM_NAVIGATING, config.arm_pos_navigating),
            ("Write arm table", KEY_ARM_TABLE, config.arm_pos_table),
        ])
    for name, key, value in writes:
        root.add_child(BtNode_WriteToBlackboard(
            name=name, bb_namespace="", bb_source=None, bb_key=key, object=value,
        ))
    return root


def _moveArmRetry(name, arm_pose_key, config):
    return py_trees.decorators.Retry(
        name=f"Retry {name}",
        child=BtNode_MoveArmSingle(
            name=name, action_name=config.arm_action,
            arm_pose_bb_key=arm_pose_key, add_octomap=False,
        ),
        num_failures=config.arm_retry_limit,
    )


def _gotoRetry(name, pose_key, config):
    return py_trees.decorators.Retry(
        name=f"Retry {name}",
        child=BtNode_GotoAction(
            name=name, key=pose_key, action_name=config.navigation_action,
        ),
        num_failures=config.nav_retry_limit,
    )


def _scanForYoloWorldRetry(name, config, *, bb_key, use_orbbec,
                       bb_source=None, prompt=None, grasp_payload=False):
    return py_trees.decorators.Retry(
        name=f"Retry {name}",
        child=BtNode_ScanForGeneralist(
            name=name, bb_source=bb_source, bb_key=bb_key, object=prompt,
            service_name=config.detection_service, use_orbbec=use_orbbec,
            use_vlm_sam_fallback=False, force_vlm_sam=False,
            transform_to_map=False, sort_closest=True, sort_highest=False,
            return_rgb_image=grasp_payload, return_depth_image=grasp_payload,
            return_segments=grasp_payload,
        ),
        num_failures=config.scan_retry_limit,
    )


def createNavigateToScan(config, *, enable_grasp=False):
    root = py_trees.composites.Sequence(name="Navigate to scan point", memory=True)
    if enable_grasp:
        root.add_child(_moveArmRetry(
            "Arm to nav before first navigation", KEY_ARM_NAVIGATING, config))
    root.add_child(_gotoRetry("Go to scan point", KEY_POSE_SCAN, config))
    return root


def createScanAndAnnounce(config, *, enable_grasp=False):
    root = py_trees.composites.Sequence(name="Scan and announce objects", memory=True)
    if enable_grasp:
        root.add_child(_moveArmRetry("Arm to table scan pose", KEY_ARM_TABLE, config))
    root.add_child(_scanForYoloWorldRetry(
        "Scan objects with YOLO-World", config, bb_key=KEY_SCAN_RESULT,
        use_orbbec=(not enable_grasp and config.scan_camera == "orbbec"),
        prompt=config.prompt, grasp_payload=enable_grasp,
    ))
    root.add_child(py_trees.decorators.Retry(
        name="Retry announce detected objects",
        child=BtNode_AnnounceDetections(
            name="Announce detected objects in Chinese", bb_source=KEY_SCAN_RESULT,
            label_map=config.label_map, bb_text_key=KEY_ANNOUNCEMENT,
            service_name=config.announce_service,
        ),
        num_failures=config.announce_retry_limit,
    ))
    return root


def createGrasp(config):
    """Select once, then reacquire the chosen class before each grasp attempt."""
    root = py_trees.composites.Sequence(name="Grasp nearest object", memory=True)
    root.add_child(BtNode_SelectGraspTarget(
        name="Select nearest detected object", bb_source=KEY_SCAN_RESULT,
        bb_key=KEY_GRASP_VISION_RES, label_key=KEY_TARGET_LABEL,
    ))
    attempt = py_trees.composites.Sequence(name="Reacquire and grasp", memory=True)
    attempt.add_child(_scanForYoloWorldRetry(
        "Reacquire selected label with RealSense", config,
        bb_key=KEY_GRASP_SCAN_RESULT, use_orbbec=False,
        bb_source=KEY_TARGET_LABEL, grasp_payload=True,
    ))
    attempt.add_child(BtNode_SelectGraspTarget(
        name="Prepare fresh single-object grasp bundle",
        bb_source=KEY_GRASP_SCAN_RESULT,
        bb_key=KEY_GRASP_VISION_RES, label_key=KEY_GRASP_LABEL,
    ))
    attempt.add_child(BtNode_Grasp(
        name="Grasp selected object", action_name=config.grasp_action,
        bb_key_vision_res=KEY_GRASP_VISION_RES,
        bb_key_object_label=KEY_GRASP_LABEL, use_mesh=True, stay=False,
    ))
    root.add_child(py_trees.decorators.Retry(
        name="Retry reacquire and grasp", child=attempt,
        num_failures=config.grasp_retry_limit,
    ))
    root.add_child(_moveArmRetry(
        "Retract arm with object", KEY_ARM_NAVIGATING, config))
    return root


def createScanGraspWithCleanup(config):
    """Selector cleanup like PickAndPlace, retaining failure after recovery."""
    attempt = py_trees.composites.Sequence(
        name="Scan announce and grasp", memory=True,
        children=[createScanAndAnnounce(config, enable_grasp=True), createGrasp(config)],
    )
    cleanup = py_trees.composites.Sequence(
        name="Retract on scan or grasp failure", memory=True,
        children=[
            _moveArmRetry("Retract arm on failure", KEY_ARM_NAVIGATING, config),
            py_trees.behaviours.Failure(name="Report scan or grasp failure"),
        ],
    )
    return py_trees.composites.Selector(
        name="Scan and grasp with failure cleanup", memory=True,
        children=[attempt, cleanup],
    )


def createNavigateToDestination(config):
    return py_trees.composites.Sequence(
        name="Navigate to destination", memory=True,
        children=[_gotoRetry("Go to destination", KEY_POSE_DESTINATION, config)],
    )


def createHangzhouGrasp(*, enable_grasp=False, config=None):
    """Build grasp nodes only when enable_grasp=True; one mission per run."""
    if type(enable_grasp) is not bool:
        raise TypeError("enable_grasp must be a boolean")
    config = config if config is not None else load_config()
    root = py_trees.composites.Sequence(name="HangzhouGrasp Root", memory=True)
    root.add_child(createConstantWriter(config, enable_grasp=enable_grasp))
    root.add_child(createNavigateToScan(config, enable_grasp=enable_grasp))
    if enable_grasp:
        root.add_child(createScanGraspWithCleanup(config))
    else:
        root.add_child(createScanAndAnnounce(config))
    root.add_child(createNavigateToDestination(config))
    return root

