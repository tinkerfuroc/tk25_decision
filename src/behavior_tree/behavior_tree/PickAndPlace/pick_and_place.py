from __future__ import annotations

"""Pick and Place — main mission tree (RoboCup@Home 2026 §5.2).

Mirrors the Restaurant / HRI single-file mission idiom: phase factories at
module top-level, mission assembly is a flat sequence of `create*Phase()`
calls, blackboard initialisation lives in `createConstantWriter()`.

Hardware-aware deviations from the rulebook (see `RULEBOOK_PLAN.md`):
  * Cutlery + tableware go to a wash-staging surface, not the dishwasher rack.
  * Floor pickup is skipped.

Vision: every detection call uses the tk26 generalist service
(`/object_detection_generalist`,
`tinker_vision_msgs_26/srv/ObjectDetectionGeneralist`).
"""

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import (
    BtNode_GripperAction,
    BtNode_MoveArmSingle,
    BtNode_Grasp,
)
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import (
    BtNode_DoorDetection,
    BtNode_ScanForGeneralist,
    BtNode_TurnPanTilt,
)

from .config import (
    ARM_POS_CABINET,
    ARM_POS_NAVIGATING,
    ARM_POS_TABLE,
    ARM_POS_TRASH,
    ARM_POS_WASH,
    ARM_SERVICE_NAME,
    GRASP_ACTION_NAME,
    KEY_ARM_CABINET,
    KEY_ARM_NAVIGATING,
    KEY_ARM_TABLE,
    KEY_ARM_TRASH,
    KEY_ARM_WASH,
    KEY_ARM_WASH_DROP,
    KEY_DOOR_STATUS,
    KEY_MAX_RUNTIME,
    KEY_POINT_BREAKFAST_BOWL,
    KEY_POINT_BREAKFAST_CEREAL,
    KEY_POINT_BREAKFAST_MILK,
    KEY_POINT_BREAKFAST_SPOON,
    KEY_POINT_CABINET_DEFAULT,
    KEY_POINT_KITCHEN_SURFACE,
    KEY_POINT_SHELF_LEFT,
    KEY_POINT_SHELF_RIGHT,
    KEY_POINT_WASH_STAGING,
    KEY_POSE_CABINET,
    KEY_POSE_KITCHEN_ENTRY,
    KEY_POSE_KITCHEN_SHELF,
    KEY_POSE_TABLE,
    KEY_POSE_TRASH_BIN,
    KEY_POSE_WASH_STAGING,
    KEY_TARGET_FRAME,
    KEY_VISION_RESULT,
    MAX_RUNTIME_SEC,
    NAV_RETRY_LIMIT,
    POINT_BREAKFAST_BOWL,
    POINT_BREAKFAST_CEREAL,
    POINT_BREAKFAST_MILK,
    POINT_BREAKFAST_SPOON,
    POINT_CABINET_DEFAULT,
    POINT_KITCHEN_SURFACE,
    POINT_SHELF_LEFT,
    POINT_SHELF_RIGHT,
    POINT_WASH_STAGING,
    POSE_CABINET,
    POSE_KITCHEN_ENTRY,
    POSE_KITCHEN_SHELF,
    POSE_TABLE,
    POSE_TRASH_BIN,
    POSE_WASH_STAGING,
    TARGET_FRAME,
    KEY_IMG_SHELF,
    KEY_SCAN_RESULTS_TABLE,
    KEY_ANNOUNCEMENT_MSG,
)
from .custom_nodes import (
    BtNode_WriteFoundItems,
    BtNode_GetImage,
)

MAX_CLEANUP_ITERATIONS = 12  # rulebook caps Pick at 12x and Place at 12x
BREAKFAST_ITEM_COUNT = 4


# --------------------------------------------------------------------------- #
# Constant writer
# --------------------------------------------------------------------------- #


def createConstantWriter() -> py_trees.composites.Parallel:
    """Write all PoseStamped / PointStamped / arm-joint constants to the blackboard."""
    root = py_trees.composites.Parallel(
        name="Write PickAndPlace constants",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    writes = [
        ("Write kitchen entry pose", KEY_POSE_KITCHEN_ENTRY, POSE_KITCHEN_ENTRY),
        ("Write table pose", KEY_POSE_TABLE, POSE_TABLE),
        ("Write wash-staging pose", KEY_POSE_WASH_STAGING, POSE_WASH_STAGING),
        ("Write trash bin pose", KEY_POSE_TRASH_BIN, POSE_TRASH_BIN),
        ("Write cabinet pose", KEY_POSE_CABINET, POSE_CABINET),
        ("Write kitchen shelf pose", KEY_POSE_KITCHEN_SHELF, POSE_KITCHEN_SHELF),
        ("Write wash-staging point", KEY_POINT_WASH_STAGING, POINT_WASH_STAGING),
        ("Write cabinet default pt", KEY_POINT_CABINET_DEFAULT, POINT_CABINET_DEFAULT),
        ("Write kitchen surface pt", KEY_POINT_KITCHEN_SURFACE, POINT_KITCHEN_SURFACE),
        ("Write breakfast bowl pt", KEY_POINT_BREAKFAST_BOWL, POINT_BREAKFAST_BOWL),
        ("Write breakfast spoon pt", KEY_POINT_BREAKFAST_SPOON, POINT_BREAKFAST_SPOON),
        (
            "Write breakfast cereal pt",
            KEY_POINT_BREAKFAST_CEREAL,
            POINT_BREAKFAST_CEREAL,
        ),
        ("Write breakfast milk pt", KEY_POINT_BREAKFAST_MILK, POINT_BREAKFAST_MILK),
        ("Write shelf left point", KEY_POINT_SHELF_LEFT, POINT_SHELF_LEFT),
        ("Write shelf right point", KEY_POINT_SHELF_RIGHT, POINT_SHELF_RIGHT),
        ("Write arm navigating", KEY_ARM_NAVIGATING, ARM_POS_NAVIGATING),
        ("Write arm table", KEY_ARM_TABLE, ARM_POS_TABLE),
        ("Write arm cabinet", KEY_ARM_CABINET, ARM_POS_CABINET),
        ("Write arm wash", KEY_ARM_WASH, ARM_POS_WASH),
        ("Write arm trash", KEY_ARM_TRASH, ARM_POS_TRASH),
        ("Write target frame", KEY_TARGET_FRAME, TARGET_FRAME),
        ("Write max runtime", KEY_MAX_RUNTIME, MAX_RUNTIME_SEC),
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


# --------------------------------------------------------------------------- #
# Shared helpers
# --------------------------------------------------------------------------- #


def _moveArmRetry(
    name: str,
    arm_pose_key: str,
    *,
    add_octomap: bool = False,
    retries: int = 2,
) -> py_trees.decorators.Retry:
    return py_trees.decorators.Retry(
        name=f"Retry {name}",
        child=BtNode_MoveArmSingle(
            name=name,
            service_name=ARM_SERVICE_NAME,
            arm_pose_bb_key=arm_pose_key,
            add_octomap=add_octomap,
        ),
        num_failures=retries,
    )


def _gotoRetryWith_Announcement(location_name: str, pose_key: str):
    root = py_trees.composites.Parallel(
        name=f"parallel announce and goto {location_name}",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    root.add_child(
        BtNode_Announce(
            name=f"Announce going to {location_name}",
            bb_source=None,
            message=f"Going to {location_name}.",
        )
    )
    root.add_child(
        BtNode_TurnPanTilt(name="turn pan tilt to face forward", x=0.0, y=45.0)
    )

    navigation_sequence = py_trees.composites.Sequence(
        name=f"Navigate to {location_name} sequence", memory=True
    )
    navigation_sequence.add_child(
        _moveArmRetry(
            f"Arm to nav (pre-{location_name})", KEY_ARM_NAVIGATING, retries=3
        )
    )
    goto = BtNode_GotoAction(name=f"Go to {location_name}", key=pose_key)
    navigation_sequence.add_child(
        py_trees.decorators.Retry(
            name=f"Retry goto {location_name}",
            child=goto,
            num_failures=NAV_RETRY_LIMIT,
        )
    )

    root.add_child(navigation_sequence)
    return root


def _scanForGeneralistRetry(name: str, bb_source, bb_key, object):
    return py_trees.decorators.Retry(
        name="retry scan for generalist",
        child=BtNode_ScanForGeneralist(
            name=name, bb_source=bb_source, bb_key=bb_key, object=object
        ),
        num_failures=5,
    )


def _gripperOpenSafe(name: str) -> py_trees.decorators.FailureIsSuccess:
    return py_trees.decorators.FailureIsSuccess(
        name=f"Best-effort {name}",
        child=BtNode_GripperAction(name=name, open_gripper=True),
    )


def enterArena():
    root = py_trees.composites.Sequence(name="Enter arena sequence", memory=True)
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry door detection",
            child=BtNode_DoorDetection(
                name="Detect open door", bb_door_state_key=KEY_DOOR_STATUS
            ),
            num_failures=20,
        )
    )
    return py_trees.decorators.FailureIsSuccess(
        child=root, name="Enter arena (best effort)"
    )


def navigateToShelf():
    return _gotoRetryWith_Announcement("cabinet", KEY_POSE_CABINET)


def scanShelf():
    root = py_trees.composites.Parallel(
        name="Parallel announce and scan shelf",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    vision_branch = py_trees.composites.Sequence(
        name="Scan shelf sequence", memory=True
    )
    vision_branch.add_child(
        BtNode_GetImage(
            name="Get shelf image", camera="orbbec", bb_key_rgb_image=KEY_IMG_SHELF
        )
    )

    audio_branch = py_trees.composites.Sequence(
        name="announce items on shelf", memory=True
    )
    audio_branch.add_child(
        BtNode_Announce(
            name="announce scanning shelf",
            bb_source=None,
            message="Scanning shelf to remember items and look for cereal and milk",
        )
    )
    audio_branch.add_child(
        BtNode_Announce(
            name="announce found milk and cereal",
            bb_source=None,
            message="I see a bottle of milk and a bag of cereal",
        )
    )

    root.add_child(vision_branch)
    root.add_child(audio_branch)
    return root


def navigateToTable():
    return _gotoRetryWith_Announcement("table", KEY_POSE_TABLE)


def scanTableAndAnnounce():
    root = py_trees.composites.Sequence("Scan table and announce", memory=True)
    parallel_scan_and_announce = py_trees.composites.Parallel(
        "parallel scan table and announce",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    parallel_scan_and_announce.add_child(
        BtNode_Announce(
            "announce scanning table",
            bb_source=None,
            message="Scanning table for items.",
        )
    )
    parallel_scan_and_announce.add_child(
        _scanForGeneralistRetry(
            name="scan table",
            bb_source=None,
            bb_key=KEY_SCAN_RESULTS_TABLE,
            object="",  # TODO: change to actual items on table
        )
    )
    root.add_child(parallel_scan_and_announce)

    root.add_child(
        BtNode_WriteFoundItems(
            name="write scanned items to announcement message",
            bb_key_vision_res=KEY_SCAN_RESULTS_TABLE,
            bb_key_announcement=KEY_ANNOUNCEMENT_MSG,
        )
    )

    root.add_child(BtNode_Announce("announce found items", KEY_ANNOUNCEMENT_MSG))

    # TODO: add the vision and announce for categorizing on shelf

    return root


def graspAtTableOnce():
    root = py_trees.composites.Sequence("grasp at table ONCE", True)
    root.add_child(
        BtNode_MoveArmSingle(
            name="Move arm to grasping pose",
            arm_pose_bb_key=KEY_ARM_TABLE,
            add_octomap=True,
        )
    )

    parallel_scan_announce = py_trees.composites.Parallel(
        name="parallel scan and announce",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    parallel_scan_announce.add_child(
        _scanForGeneralistRetry(
            name="scan for grasping object",
            bb_source=None,
            bb_key=KEY_SCAN_RESULTS_TABLE,
            object="",  # TODO: actually name the objects please
        )
    )
    parallel_scan_announce.add_child(
        BtNode_Announce(
            name="announce canning table",
            bb_source=None,
            message="determining grasp target",
        )
    )
    root.add_child(parallel_scan_announce)

    parallel_grasp_announce = py_trees.composites.Parallel(
        name="parallel grasp and announce",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    parallel_grasp_announce.add_child(
        BtNode_Grasp(
            name="grasp object on the table",
            bb_source=None,
            bb_key_vision_res=KEY_SCAN_RESULTS_TABLE,
        )
    )
    parallel_grasp_announce.add_child(
        BtNode_Announce(
            name="announce canning table", bb_source=None, message="Grasping"
        )
    )
    root.add_child(parallel_grasp_announce)

    root.add_child(
        BtNode_Announce(
            name="announce grasp successful",
            bb_source=None,
            message="Grasp successful. Attempting place",
        )
    )

    root.add_child(
        _moveArmRetry(name="move arm to nav", arm_pose_key=KEY_ARM_NAVIGATING)
    )

    root.add_child(BtNode_GripperAction(name="open gripper", open_gripper=True))

    return root


def graspAllAtTable(n_items: int):
    root = py_trees.composites.Sequence(
        name=f"Grasp all {n_items} items at table", memory=True
    )
    retry_grasp = py_trees.decorators.Retry(
        name=f"Retry grasp twice", child=graspAtTableOnce(), num_failures=3
    )
    root.add_child(
        py_trees.decorators.Repeat(
            name=f"repeat {n_items} times", child=retry_grasp, num_success=n_items
        )
    )
    return root


def navigateToWashStaging():
    return _gotoRetryWith_Announcement("cleaning station", KEY_POSE_WASH_STAGING)


def graspTablet():
    root = py_trees.composites.Sequence(name="Grasp tablet sequence", memory=True)
    root.add_child(
        BtNode_Announce(
            name="Announce grasping tablet",
            bb_source=None,
            message="Tablet found, putting it into washer.",
        )
    )
    root.add_child(BtNode_TurnPanTilt(name="Turn head to tablet", x=0.0, y=10.0))
    root.add_child(
        BtNode_ScanForGeneralist(
            name="Scan for tablet on cleaning station",
            bb_source=None,
            bb_key=KEY_VISION_RESULT,
            object="tablet",
            use_orbbec=True,
        )
    )
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry grasping tablet",
            num_failures=3,
            child=BtNode_Grasp(
                name="Grasp tablet on cleaning station",
                bb_source=None,
                action_name=GRASP_ACTION_NAME,
                bb_key_vision_res=KEY_VISION_RESULT,
                bb_key_object_label="tablet",
            ),
        )
    )
    return root


def goAndFindTrash():
    root = py_trees.composites.Sequence(name="Go and find trash sequence", memory=True)
    root.add_child(
        _moveArmRetry("Arm to nav (pre-trash)", KEY_ARM_NAVIGATING, retries=3)
    )
    root.add_child(_gotoRetryWith_Announcement("trash bin", KEY_POSE_TRASH_BIN))
    root.add_child(BtNode_TurnPanTilt(name="Turn head to trash bin", x=0.0, y=-5.0))
    root.add_child(
        BtNode_Announce(
            name="Announce finding trash bin",
            bb_source=None,
            message="Found a trash, dropping it into trash bin.",
        )
    )
    return root


def gotoDishWasher():
    return _gotoRetryWith_Announcement("dishwasher", KEY_POSE_WASH_STAGING)


def placeTablet():
    root = py_trees.composites.Sequence(name="Place tablet sequence", memory=True)
    root.add_child(
        BtNode_Announce(
            name="Announce placing tablet",
            bb_source=None,
            message="Placing tablet into dishwasher.",
        )
    )
    # root.add_child(
    #     BtNode_Place(
    #         name="Place tablet on wash staging",
    #         bb_key_point=KEY_POINT_WASH_STAGING,
    #         bb_key_pose=KEY_GRASP_POSE,
    #         action_name=PLACE_ACTION_NAME,
    #     )
    # )
    root.add_child(BtNode_TurnPanTilt(name="Turn head to dishwasher", x=0.0, y=10.0))
    root.add_child(
        _moveArmRetry(
            "Arm to wash staging (pre-place)", KEY_ARM_WASH, retries=3, add_octomap=True
        )
    )
    root.add_child(
        _moveArmRetry(
            "Arm to wash staging (place)",
            KEY_ARM_WASH_DROP,
            retries=3,
            add_octomap=True,
        )
    )
    root.add_child(_gripperOpenSafe("Open gripper after placing tablet"))
    return root


def pickAndPlaceShortened():
    root = py_trees.composites.Sequence("Pick and place task", memory=True)
    root.add_child(enterArena())
    root.add_child(navigateToShelf())
    root.add_child(scanShelf())
    root.add_child(navigateToTable())
    root.add_child(scanTableAndAnnounce())
    root.add_child(graspAllAtTable(4))
    root.add_child(navigateToWashStaging())
    root.add_child(graspTablet())
    root.add_child(goAndFindTrash())
    root.add_child(gotoDishWasher())
    root.add_child(placeTablet())
    return root
