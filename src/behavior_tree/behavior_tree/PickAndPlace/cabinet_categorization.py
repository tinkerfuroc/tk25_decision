from __future__ import annotations

"""Cabinet Categorization subtree.

Mission:
    1. Per-layer vision: scan each cabinet shelf with the generalist detector.
    2. TTS announce what was seen on each layer so the referee can verify.

This is a *survey* step — it does NOT pick anything up. It runs before grasping
or placing so the categorization can inform later destination decisions.
"""

from typing import List, Tuple

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Vision import (
    BtNode_ScanForGeneralist,
    BtNode_TurnPanTilt,
)
from behavior_tree.TemplateNodes.manipulation_new import BtNode_JointMoveAction

from .config import (
    ARM_ACTION_NAME,
    ARM_POS_NAVIGATING,
    KEY_ANNOUNCEMENT_MSG,
    KEY_ARM_NAVIGATING,
    KEY_SCAN_RESULTS_SHELF,
)
from .custom_nodes import BtNode_WriteFoundItems


DEFAULT_CABINET_PROMPT = (
    "bottle . cup . can . box . bag . food . drink . cereal . milk . "
    "snack . chips . cookies . bowl . plate"
)


# (label, pan_deg, tilt_deg). Tweak after Setup Days against the real cabinet.
DEFAULT_CABINET_LAYERS: List[Tuple[str, float, float]] = [
    ("top shelf", 0.0, 10.0),
    ("bottom shelf", 0.0, 30.0),
]


def createConstantWriter() -> py_trees.composites.Parallel:
    """Write the blackboard constants this subtree depends on.

    Scoped to cabinet-categorization only — does not duplicate
    pick_and_place.createConstantWriter. Safe to run standalone; the production
    mission writes the same keys upstream and the two entry points are not run
    concurrently.
    """
    root = py_trees.composites.Parallel(
        name="Write CabinetCategorization constants",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    writes = [
        ("Write arm navigating", KEY_ARM_NAVIGATING, ARM_POS_NAVIGATING),
    ]
    for name, key, value in writes:
        root.add_child(
            BtNode_WriteToBlackboard(
                name=name,
                bb_namespace="",
                bb_source=None,
                bb_key=key,
                object=value,
            )
        )
    return root


def _moveArmRetry(
    name: str, arm_pose_key: str, *, add_octomap: bool = False, retries: int = 2
):
    return py_trees.decorators.Retry(
        name=f"Retry {name}",
        child=BtNode_JointMoveAction(
            name=name,
            action_name=ARM_ACTION_NAME,
            arm_pose_bb_key=arm_pose_key,
        ),
        num_failures=retries,
    )


def _scanForGeneralistRetry(name: str, bb_key, object, use_orbbec=True, retries=3):
    return py_trees.decorators.Retry(
        name=f"Retry {name}",
        child=BtNode_ScanForGeneralist(
            name=name,
            bb_source=None,
            bb_key=bb_key,
            object=object,
            use_orbbec=use_orbbec,
            return_rgb_image=True,
            return_depth_image=True,
            force_vlm_sam=True,
            return_segments=True,
        ),
        num_failures=retries,
    )


def _scanAndAnnounceLayer(
    label: str,
    pan: float,
    tilt: float,
    prompt: str,
    scan_result_key: str,
    announcement_key: str,
):
    """Per-layer subtree: pan/tilt to layer → scan → format → announce."""
    layer_seq = py_trees.composites.Sequence(
        name=f"Scan and announce ({label})", memory=True
    )
    layer_seq.add_child(
        BtNode_TurnPanTilt(name=f"head to {label}", x=pan, y=tilt)
    )
    layer_seq.add_child(
        _scanForGeneralistRetry(
            name=f"scan {label}",
            bb_key=scan_result_key,
            object=prompt,
            use_orbbec=True,
        )
    )
    layer_seq.add_child(
        BtNode_WriteFoundItems(
            name=f"format {label} announcement",
            bb_key_vision_res=scan_result_key,
            bb_key_announcement=announcement_key,
            place_seen=f"on the {label}",
        )
    )
    layer_seq.add_child(
        BtNode_Announce(
            name=f"announce {label} items",
            bb_source=announcement_key,
        )
    )
    # If a single layer scan fails, don't kill the whole categorization.
    return py_trees.decorators.FailureIsSuccess(
        name=f"Best-effort {label} categorization", child=layer_seq
    )


def createCabinetCategorization(
    *,
    layers: List[Tuple[str, float, float]] = None,
    prompt: str = DEFAULT_CABINET_PROMPT,
    pre_scan_arm_pose_key: str = KEY_ARM_NAVIGATING,
    scan_result_key: str = KEY_SCAN_RESULTS_SHELF,
    announcement_key: str = KEY_ANNOUNCEMENT_MSG,
    move_arm_retries: int = 3,
):
    """Cabinet Categorization subtree.

    Args:
        layers: list of (label, pan_deg, tilt_deg) tuples — one per shelf to
            scan. Defaults to a 2-layer top/bottom split; override after Setup
            Days when the real shelf heights are known.
        prompt: open-vocab detection prompt covering plausible cabinet items.
        pre_scan_arm_pose_key: arm pose used to clear the orbbec view before
            scanning. Defaults to the stowed navigation pose.
        scan_result_key: blackboard key reused per layer for the vision bundle.
        announcement_key: blackboard key reused per layer for the announce text.

    Assumes the robot is already positioned in front of the cabinet. Each layer
    is best-effort — a single failed layer scan does not abort the rest.
    """
    layers = layers or DEFAULT_CABINET_LAYERS

    root = py_trees.composites.Sequence(name="Cabinet categorization", memory=True)

    # 0. 写入本子树所需的 blackboard 常量（standalone-safe）
    root.add_child(createConstantWriter())

    # Clear the arm from the orbbec FOV before scanning shelves.
    root.add_child(
        _moveArmRetry(
            name="move arm clear of cabinet view",
            arm_pose_key=pre_scan_arm_pose_key,
            retries=move_arm_retries,
        )
    )

    # 1+2. Per-layer scan + announce. Layers run sequentially with `memory=True`
    # because the pan/tilt motion must settle before the scan fires.
    for label, pan, tilt in layers:
        root.add_child(
            _scanAndAnnounceLayer(
                label=label,
                pan=pan,
                tilt=tilt,
                prompt=prompt,
                scan_result_key=scan_result_key,
                announcement_key=announcement_key,
            )
        )

    # Recenter the head so downstream subtrees don't inherit a tilted gaze.
    root.add_child(
        BtNode_TurnPanTilt(name="recenter head after categorization", x=0.0, y=0.0)
    )
    return root
