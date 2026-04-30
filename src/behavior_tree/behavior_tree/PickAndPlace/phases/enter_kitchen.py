from __future__ import annotations

"""Enter kitchen phase — wait for door, navigate to kitchen entry, then to table."""

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import BtNode_DoorDetection, BtNode_TurnPanTilt

from ..config import KEY_DOOR_STATUS, KEY_POSE_KITCHEN_ENTRY, KEY_POSE_TABLE, NAV_RETRY_LIMIT


def createEnterKitchen() -> py_trees.composites.Sequence:
    root = py_trees.composites.Sequence(name="Enter kitchen", memory=True)
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry door detection",
            child=BtNode_DoorDetection(
                name="Wait for door open", bb_door_state_key=KEY_DOOR_STATUS,
            ),
            num_failures=999,
        )
    )
    parallel = py_trees.composites.Parallel(
        name="Enter parallel", policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    parallel.add_child(
        BtNode_Announce(
            name="Announce entering kitchen", bb_source=None, message="Entering kitchen.",
        )
    )
    parallel.add_child(BtNode_TurnPanTilt(name="Tilt head forward", x=0.0, y=20.0))
    parallel.add_child(
        py_trees.decorators.Retry(
            name="Retry goto kitchen entry",
            child=BtNode_GotoAction(name="Go to kitchen entry", key=KEY_POSE_KITCHEN_ENTRY),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    root.add_child(parallel)
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry goto table",
            child=BtNode_GotoAction(name="Go to dining table", key=KEY_POSE_TABLE),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    return root