from __future__ import annotations

"""HRI behavior tree composition.

This module keeps focus on phase composition and node wiring.
Constants, precomputed poses, and key declarations live in `HRI/config.py`.
"""

import py_trees

from behavior_tree.TemplateNodes.Audio import (
    BtNode_Announce,
    BtNode_GetConfirmation,
    BtNode_PhraseExtraction,
)
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import BtNode_GripperAction, BtNode_MoveArmSingle
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import (
    BtNode_DoorDetection,
    BtNode_FeatureExtraction,
    BtNode_SeatRecommend,
    BtNode_TurnPanTilt,
)
from behavior_tree.Receptionist.customNodes import (
    BtNode_CombinePerson,
    BtNode_Confirm,
    BtNode_HeadTrackingAction,
    BtNode_Introduce,
)
from .config import (
    ARM_POS_DROP,
    ARM_POS_HANDOVER,
    ARM_POS_NAVIGATING,
    DRINKS,
    HOST_DRINK,
    HOST_NAME,
    KEY_ARM_DROP,
    KEY_ARM_HANDOVER,
    KEY_ARM_NAVIGATING,
    KEY_DOOR_POSE,
    KEY_DOOR_STATUS,
    KEY_GUEST1_DRINK,
    KEY_GUEST1_FEATURES,
    KEY_GUEST1_NAME,
    KEY_GUEST2_DRINK,
    KEY_GUEST2_FEATURES,
    KEY_GUEST2_NAME,
    KEY_PERSONS,
    KEY_SEAT_RECOMMENDATION,
    KEY_SOFA_POSE,
    NAMES,
    POSE_DOOR,
    POSE_SOFA,
)


class BtNode_MockArrivalTrigger(py_trees.behaviour.Behaviour):
    """Fallback when no reliable doorbell/arrival node is available."""

    def __init__(self, name: str = "Mock arrival trigger"):
        super().__init__(name=name)
        self._done = False

    def update(self) -> py_trees.common.Status:
        if self._done:
            return py_trees.common.Status.SUCCESS
        self._done = True
        self.feedback_message = "TODO: replace with doorbell/knock event node"
        return py_trees.common.Status.SUCCESS


class BtNode_MockSafetyCheck(py_trees.behaviour.Behaviour):
    """Fallback placeholder for missing handover/drop confirmation node types."""

    def __init__(self, name: str, todo: str):
        super().__init__(name=name)
        self.todo = todo

    def update(self) -> py_trees.common.Status:
        self.feedback_message = f"TODO mock path: {self.todo}"
        return py_trees.common.Status.SUCCESS


def createConstantWriter():
    """Initialize static HRI values on the blackboard."""
    root = py_trees.composites.Parallel(
        name="Write HRI constants",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Write door pose",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_DOOR_POSE,
            object=POSE_DOOR,
        )
    )
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Write sofa pose",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_SOFA_POSE,
            object=POSE_SOFA,
        )
    )
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Write arm nav pose",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_ARM_NAVIGATING,
            object=ARM_POS_NAVIGATING,
        )
    )
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Write arm handover pose",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_ARM_HANDOVER,
            object=ARM_POS_HANDOVER,
        )
    )
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Write arm drop pose",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_ARM_DROP,
            object=ARM_POS_DROP,
        )
    )
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Initialize persons",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_PERSONS,
            object=[],
        )
    )
    return root


def createArrivalTrigger():
    """Try real door-trigger first, then fallback to mock trigger."""
    root = py_trees.composites.Selector(name="Arrival trigger", memory=False)
    real_trigger = py_trees.composites.Sequence(name="Door detection trigger", memory=True)
    real_trigger.add_child(
        py_trees.decorators.Retry(
            name="Retry door detection",
            child=BtNode_DoorDetection(name="Detect open door", bb_door_state_key=KEY_DOOR_STATUS),
            num_failures=3,
        )
    )
    real_trigger.add_child(
        BtNode_Announce(
            name="Arrival detected announcement",
            bb_source=None,
            message="I see a guest at the door.",
        )
    )
    root.add_child(real_trigger)
    root.add_child(BtNode_MockArrivalTrigger())
    return root


def _create_get_info(field_name: str, storage_key: str, word_list: list[str]):
    """Prompt -> extract -> confirm loop for one spoken field."""
    root = py_trees.composites.Sequence(name=f"Get {field_name}", memory=True)
    loop = py_trees.composites.Sequence(name=f"Get+confirm {field_name}", memory=True)
    loop.add_child(
        BtNode_Announce(
            name=f"Prompt for {field_name}",
            bb_source=None,
            message=f"Please tell me your {field_name}.",
        )
    )
    loop.add_child(
        BtNode_PhraseExtraction(
            name=f"Extract {field_name}",
            bb_dest_key=storage_key,
            wordlist=word_list,
            timeout=7.0,
        )
    )
    loop.add_child(
        BtNode_Confirm(
            name=f"Confirm {field_name}",
            key_confirmed=storage_key,
            type=field_name,
        )
    )
    loop.add_child(BtNode_GetConfirmation(name=f"Get {field_name} confirmation", timeout=5.0))
    root.add_child(py_trees.decorators.Retry(name="Retry get+confirm", child=loop, num_failures=5))
    return root


def createGuestIntake(guest_idx: int):
    """Collect name/drink/features for one guest at the door."""
    if guest_idx == 1:
        name_key = KEY_GUEST1_NAME
        drink_key = KEY_GUEST1_DRINK
        feature_key = KEY_GUEST1_FEATURES
    else:
        name_key = KEY_GUEST2_NAME
        drink_key = KEY_GUEST2_DRINK
        feature_key = KEY_GUEST2_FEATURES

    root = py_trees.composites.Sequence(name=f"Guest {guest_idx} intake", memory=True)
    root.add_child(
        py_trees.decorators.Retry(
            name=f"Retry goto door for guest {guest_idx}",
            child=BtNode_GotoAction(name=f"Go to door for guest {guest_idx}", key=KEY_DOOR_POSE),
            num_failures=5,
        )
    )
    root.add_child(_create_get_info("name", name_key, NAMES))
    root.add_child(_create_get_info("favorite drink", drink_key, DRINKS))
    root.add_child(BtNode_FeatureExtraction(name=f"Extract guest {guest_idx} features", bb_dest_key=feature_key))
    root.add_child(
        BtNode_CombinePerson(
            name=f"Store guest {guest_idx} profile",
            key_dest=KEY_PERSONS,
            key_name=name_key,
            key_drink=drink_key,
            key_features=feature_key,
        )
    )
    return root


def _with_gaze_supervisor(name: str, main_child: py_trees.behaviour.Behaviour):
    """Wrap a subtree with non-blocking gaze helpers."""
    root = py_trees.composites.Parallel(
        name=name,
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected([main_child]),
    )
    root.add_child(main_child)
    root.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="Gaze follow fallback",
            child=BtNode_HeadTrackingAction(name="Follow speaker gaze", actionName="follow_head_action"),
        )
    )
    root.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="Nav direction gaze fallback",
            child=BtNode_TurnPanTilt(name="Look to navigation direction", x=0.0, y=35.0, speed=0.0),
        )
    )
    return root


def createEscortAndSeat(guest_idx: int):
    """Escort one guest and issue explicit seat recommendation."""
    escort = py_trees.composites.Sequence(name=f"Escort and seat guest {guest_idx}", memory=True)
    escort.add_child(
        BtNode_Announce(
            name=f"Invite guest {guest_idx} to follow",
            bb_source=None,
            message="Please follow me to your seat.",
        )
    )
    escort.add_child(
        py_trees.decorators.Retry(
            name=f"Retry goto sofa for guest {guest_idx}",
            child=BtNode_GotoAction(name=f"Go to sofa for guest {guest_idx}", key=KEY_SOFA_POSE),
            num_failures=5,
        )
    )
    seat_recommend = py_trees.composites.Sequence(name=f"Seat recommend guest {guest_idx}", memory=True)
    seat_recommend.add_child(
        BtNode_SeatRecommend(
            name=f"Seat recommendation for guest {guest_idx}",
            bb_dest_key=KEY_SEAT_RECOMMENDATION,
            bb_source_key=KEY_PERSONS,
        )
    )
    seat_recommend.add_child(
        BtNode_TurnPanTilt(name=f"Turn to seat direction guest {guest_idx}", x=45.0, y=25.0, speed=0.0)
    )
    seat_recommend.add_child(
        BtNode_Announce(
            name=f"Announce seat recommendation guest {guest_idx}",
            bb_source=KEY_SEAT_RECOMMENDATION,
            message="Please sit here.",
        )
    )
    escort.add_child(seat_recommend)
    return _with_gaze_supervisor(f"Gaze-supervised escort guest {guest_idx}", escort)


def createTwoWayIntroduction():
    """Introduce guest1<->guest2 with gaze-aware wrappers."""
    root = py_trees.composites.Sequence(name="Two-way introductions", memory=True)
    intro_1 = BtNode_Introduce(
        name="Introduce guest1 to guest2",
        key_person=KEY_PERSONS,
        target_id=1,
        introduced_id=0,
        describe_introduced=True,
    )
    intro_2 = BtNode_Introduce(
        name="Introduce guest2 to guest1",
        key_person=KEY_PERSONS,
        target_id=0,
        introduced_id=1,
        describe_introduced=True,
    )
    root.add_child(_with_gaze_supervisor("Gaze intro 1", intro_1))
    root.add_child(_with_gaze_supervisor("Gaze intro 2", intro_2))
    return root


def createBagFlow():
    """Handle bag handover, host-follow proxy, and bag drop sequence."""
    root = py_trees.composites.Sequence(name="Bag handover-follow-drop", memory=True)
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry arm to handover pose",
            child=BtNode_MoveArmSingle(
                name="Move arm to handover pose",
                service_name="arm_joint_service",
                arm_pose_bb_key=KEY_ARM_HANDOVER,
                add_octomap=False,
            ),
            num_failures=3,
        )
    )
    root.add_child(BtNode_GripperAction(name="Open gripper for bag", open_gripper=True))
    root.add_child(
        BtNode_Announce(
            name="Ask for bag handover",
            bb_source=None,
            message="Please place your bag in my gripper.",
        )
    )
    root.add_child(py_trees.timers.Timer(name="Wait for bag placement", duration=3.0))
    root.add_child(BtNode_GripperAction(name="Close gripper with bag", open_gripper=False))
    root.add_child(
        BtNode_Announce(
            name="Follow host announcement",
            bb_source=None,
            message="I'll follow you and carry the bag.",
        )
    )
    # TODO: replace this with a dedicated follow-host behavior once available.
    root.add_child(
        py_trees.decorators.Retry(
            name="Proxy follow host by navigation",
            child=BtNode_GotoAction(name="Move to drop area proxy", key=KEY_SOFA_POSE),
            num_failures=3,
        )
    )
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry arm to drop pose",
            child=BtNode_MoveArmSingle(
                name="Move arm to drop pose",
                service_name="arm_joint_service",
                arm_pose_bb_key=KEY_ARM_DROP,
                add_octomap=False,
            ),
            num_failures=3,
        )
    )
    root.add_child(BtNode_GripperAction(name="Open gripper to drop bag", open_gripper=True))
    root.add_child(BtNode_MockSafetyCheck(name="Drop confirmation detector TODO", todo="replace with bag-drop detector"))
    root.add_child(BtNode_GripperAction(name="Close gripper after drop", open_gripper=False))
    return root


def createHRITask():
    """Compose full HRI task in the expected competition order."""
    root = py_trees.composites.Sequence(name="HRI Task", memory=True)
    root.add_child(createConstantWriter())
    root.add_child(
        BtNode_Announce(
            name="HRI start announcement",
            bb_source=None,
            message=f"HRI task started. Host: {HOST_NAME}. Favorite drink: {HOST_DRINK}.",
        )
    )
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry arm nav pose at start",
            child=BtNode_MoveArmSingle(
                name="Move arm to navigation pose",
                service_name="arm_joint_service",
                arm_pose_bb_key=KEY_ARM_NAVIGATING,
                add_octomap=False,
            ),
            num_failures=3,
        )
    )
    root.add_child(createArrivalTrigger())
    root.add_child(createGuestIntake(1))
    root.add_child(createEscortAndSeat(1))
    root.add_child(createArrivalTrigger())
    root.add_child(createGuestIntake(2))
    root.add_child(createEscortAndSeat(2))
    root.add_child(createTwoWayIntroduction())
    root.add_child(createBagFlow())
    root.add_child(
        BtNode_Announce(
            name="HRI completion announcement",
            bb_source=None,
            message="HRI task complete.",
        )
    )
    return root
