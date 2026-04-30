from __future__ import annotations
from ast import main

"""HRI behavior tree composition.

This module keeps focus on phase composition and node wiring.
Constants, precomputed poses, and key declarations live in `HRI/config.py`.
"""

import py_trees

from behavior_tree.TemplateNodes.Audio import (
    BtNode_Announce,
    BtNode_GetConfirmationAction,
    BtNode_ListenAction,
    BtNode_PhraseExtractionAction,
)
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import BtNode_GripperAction, BtNode_MoveArmSingle
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import (
    BtNode_DoorDetection,
    BtNode_FeatureExtraction,
    BtNode_FeatureMatching,
    BtNode_MaintainEyeContact,
    BtNode_SeatRecommend,
    BtNode_TurnPanTilt,
    BtNode_TurnTo,
)
from behavior_tree.Receptionist.customNodes import (
    BtNode_CombinePerson,
    BtNode_Confirm,
    BtNode_Introduce,
)
from .config import (
    ARM_POS_DROP,
    ARM_POS_HANDOVER,
    ARM_POS_NAVIGATING,
    DRINKS,
    FOLLOW_CONFIG,
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
    KEY_HOST_DRINK,
    KEY_HOST_FEATURES,
    KEY_HOST_NAME,
    KEY_PERSONS,
    KEY_PERSON_CENTROIDS,
    KEY_SEAT_RECOMMENDATION,
    KEY_SOFA_POSE,
    NAMES,
    POSE_DOOR,
    POSE_SOFA,
)
from .follow import createFollowPerson


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
    # Host name + drink come from `constants.json` via config; seed them on
    # the blackboard so `BtNode_CombinePerson` can read them when the host
    # is scanned at task start.
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Write host name",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_HOST_NAME,
            object=HOST_NAME,
        )
    )
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Write host drink",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_HOST_DRINK,
            object=HOST_DRINK,
        )
    )
    return root


def createArrivalTrigger():
    """Try real door-trigger first, then fallback to mock trigger."""
    root = py_trees.composites.Selector(name="Arrival trigger", memory=False)
    parallel_going = py_trees.composites.Parallel(name="Parallel go to door and announce", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_going.add_child(
        py_trees.decorators.Retry(
            name=f"Retry goto door for guest",
            child=BtNode_GotoAction(name=f"Go to door for guest", key=KEY_DOOR_POSE),
            num_failures=5,
        )
    )
    parallel_going.add_child(BtNode_Announce(
        name="Arrival announcement",
        bb_source=None,
        message="Going to check the door.",
    ))
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
            message="I see a guest.",
        )
    )
    root.add_child(parallel_going)
    root.add_child(real_trigger)
    root.add_child(BtNode_MockArrivalTrigger())
    return root


def _create_get_info(field_name: str, storage_key: str, word_list: list[str]):
    """High-confidence-first capture with a last-resort confirmation fallback.

    Primary branch: up to 2 attempts of prompt → action-based extract. The
    action (`phrase_extraction_action`) only succeeds on server status=0,
    which means Whisper + Qwen ASR cross-check agreed on the same wordlist
    entry. The rulebook awards a 4×15 "no non-essential questions" bonus for
    accepting on that signal without a confirmation prompt.

    Fallback branch: if both primary attempts abort, re-prompt, capture
    the raw transcription via `BtNode_ListenAction`, then `BtNode_Confirm`
    speaks it back (`"Your <field> is <value>, correct?"`) and
    `BtNode_GetConfirmationAction` waits for yes/no. Preserves partial
    scoring in noisy environments at the cost of the no-confirmation
    bonus for this field only.
    """
    primary_loop = py_trees.composites.Sequence(
        name=f"Prompt+extract {field_name}",
        memory=True,
    )
    primary_loop.add_child(
        BtNode_Announce(
            name=f"Prompt for {field_name}",
            bb_source=None,
            message=f"Please tell me your {field_name}.",
        )
    )
    primary_loop.add_child(
        BtNode_PhraseExtractionAction(
            name=f"High-conf extract {field_name}",
            wordlist=word_list,
            bb_dest_key=storage_key,
            timeout=7.0,
        )
    )
    primary = py_trees.decorators.Retry(
        name=f"Retry high-conf {field_name}",
        child=primary_loop,
        num_failures=2,
    )

    fallback = py_trees.composites.Sequence(
        name=f"Last-resort confirm {field_name}",
        memory=True,
    )
    fallback.add_child(
        BtNode_Announce(
            name=f"Fallback prompt for {field_name}",
            bb_source=None,
            message=f"Let me try again. Please tell me your {field_name} clearly.",
        )
    )
    fallback.add_child(
        BtNode_ListenAction(
            name=f"Fallback listen {field_name}",
            bb_dest_key=storage_key,
            timeout=7.0,
        )
    )
    fallback.add_child(
        BtNode_Confirm(
            name=f"Confirm {field_name}",
            key_confirmed=storage_key,
            type=field_name,
        )
    )
    fallback.add_child(
        BtNode_GetConfirmationAction(
            name=f"Get {field_name} confirmation",
            timeout=5.0,
        )
    )

    root = py_trees.composites.Selector(
        name=f"Get {field_name}",
        memory=True,
    )
    root.add_child(primary)
    root.add_child(fallback)
    return root


def createScanHostFeatures():
    """Navigate to sofa, scan the seated host, register them at index 0 of KEY_PERSONS.

    Mirrors `Receptionist/receptionist_new.py:createScanHostFeatures`. Runs at
    task start before the first guest arrival so SeatRecommend / FeatureMatching
    treat the host as a known seated person. After this completes,
    `KEY_PERSONS == [host]`; subsequent guest intakes append to indices 1, 2.
    """
    root = py_trees.composites.Sequence(name="Scan host features", memory=True)

    paralllel_going = py_trees.composites.Parallel(
        name="Parallel go to sofa and announce",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    paralllel_going.add_child(BtNode_Announce(
        name="Host scan announcement",
        bb_source=None,
        message="Going to scan host",
    ))
    paralllel_going.add_child(
        py_trees.decorators.Retry(
            name="Retry goto sofa for host scan",
            child=BtNode_GotoAction(name="Go to sofa for host scan", key=KEY_SOFA_POSE),
            num_failures=5,
        )
    )
    root.add_child(paralllel_going)
    root.add_child(BtNode_TurnPanTilt(name="Look down at host", x=0.0, y=20.0, speed=0.0))
    parallel_scan = py_trees.composites.Parallel(
        name="Parallel host scan",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    parallel_scan.add_child(
        BtNode_Announce(
            name="Announce host scan",
            bb_source=None,
            message="Scanning host.",
        )
    )
    parallel_scan.add_child(
        py_trees.decorators.Retry(
            name="Retry host feature extraction",
            child=BtNode_FeatureExtraction(
                name="Extract host features",
                bb_dest_key=KEY_HOST_FEATURES,
            ),
            num_failures=3,
        )
    )
    root.add_child(parallel_scan)
    root.add_child(
        BtNode_CombinePerson(
            name="Register host profile",
            key_dest=KEY_PERSONS,
            key_name=KEY_HOST_NAME,
            key_drink=KEY_HOST_DRINK,
            key_features=KEY_HOST_FEATURES,
        )
    )
    root.add_child(
        BtNode_TurnPanTilt(name="Look forward after host scan", x=0.0, y=35.0, speed=0.0)
    )
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
    root.add_child(_create_get_info("name", name_key, NAMES))
    root.add_child(_create_get_info("favorite drink", drink_key, DRINKS))
    root.add_child(BtNode_TurnPanTilt(name="loop up", x=0.0, y=45.0))
    root.add_child(BtNode_Announce(name="announce stand in front of me", bb_source=None, message="Pleas stand directly in front of me so I can remember you."))
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
            child=BtNode_MaintainEyeContact(name="Follow speaker gaze"),
        )
    )

    root_ = py_trees.composites.Sequence("gaze follow with end correction", memory=True)
    root_.add_child(root)
    root_.add_child(BtNode_TurnPanTilt(name="Look to navigation direction", x=0.0, y=35.0, speed=0.0))
    return root_


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
    """Introduce guest1<->guest2 with target-guided gaze per intro."""
    root = py_trees.composites.Sequence(name="Two-way introductions", memory=True)

    # Locate both seated guests ahead of the intros so we can orient the head
    # at the right one for each direction. trim_last_person=False because
    # BOTH guests are seated by this point (unlike Receptionist's pattern).
    root.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="Scan seated guest centroids",
            child=py_trees.decorators.Retry(
                name="Retry feature matching seated guests",
                child=BtNode_FeatureMatching(
                    name="Match seated guest features",
                    bb_dest_key=KEY_PERSON_CENTROIDS,
                    bb_persons_key=KEY_PERSONS,
                    trim_last_person=False,
                ),
                num_failures=3,
            ),
        )
    )

    # KEY_PERSONS layout after host scan + both intakes: [host, guest1, guest2]
    # → guest1 is index 1, guest2 is index 2.
    intro1 = py_trees.composites.Sequence(name="Intro guest1 to guest2 with gaze", memory=True)
    intro1.add_child(
        BtNode_TurnPanTilt(name="Look at guest2", x=45.0, y=0.0, speed=0.0)
    )
    intro1.add_child(
        BtNode_Introduce(
            name="Introduce guest1 to guest2",
            key_person=KEY_PERSONS,
            target_id=2,
            introduced_id=1,
            describe_introduced=True,
        )
    )
    maintain_gaze1 = BtNode_MaintainEyeContact(name="Maintain eye contact during intros")
    parrallel_intro_gaze1 = py_trees.composites.Parallel(
        name="Parallel intro1 and gaze",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected([intro1]),
        children=[intro1, maintain_gaze1]
    )

    intro2 = py_trees.composites.Sequence(name="Intro guest2 to guest1 with gaze", memory=True)
    intro2.add_child(
        py_trees.decorators.FailureIsSuccess(
            name=f"Pre-orient at guest 1",
            child=BtNode_TurnTo(
                name=f"Look at guest1",
                bb_key_persons=KEY_PERSONS,
                bb_key_points=KEY_PERSON_CENTROIDS,
                target_id=1,
            ),
        )
    )
    intro2.add_child(BtNode_Introduce(
        name="Introduce guest2 to guest1",
        key_person=KEY_PERSONS,
        target_id=1,
        introduced_id=2,
        describe_introduced=True,
    ))
    maintain_gaze = BtNode_MaintainEyeContact(name="Maintain eye contact during intros")
    parrallel_intro_gaze = py_trees.composites.Parallel(
        name="Parallel intro2 and gaze",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected([intro2]),
        children=[intro2, maintain_gaze]
    )
    
    root.add_child(parrallel_intro_gaze1)
    root.add_child(parrallel_intro_gaze)

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
    root.add_child(createFollowPerson(FOLLOW_CONFIG))
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
    root.add_child(createScanHostFeatures())
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
