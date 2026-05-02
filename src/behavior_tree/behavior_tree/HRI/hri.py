from __future__ import annotations
from ast import main
import os

"""HRI behavior tree composition.

This module keeps focus on phase composition and node wiring.
Constants, precomputed poses, and key declarations live in `HRI/config.py`.
"""

from openai import audio
import py_trees

USE_NAV_ORIENTATION_ANGLE_SERVICE = True

image_path = os.environ.get("HOST_IMAGE_PATH", "/home/tinker/tk25_ws/img_host.jpg")
description_path = os.environ.get("HOST_DESC_PATH", "/home/tinker/tk25_ws/host.txt")
    

KEY_PANTILT_ORIENTATION="pantilt_orientation"
if USE_NAV_ORIENTATION_ANGLE_SERVICE:
    from behavior_tree.TemplateNodes.Navigation import BtNode_GetOrientationAngle

from behavior_tree.TemplateNodes.Audio import (
    BtNode_Announce,
    BtNode_GetConfirmationAction,
    BtNode_ListenAction,
    BtNode_PhraseExtractionAction,
)
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard, BtNode_WaitTicks
from behavior_tree.TemplateNodes.Manipulation import BtNode_GripperAction, BtNode_MoveArmSingle, BtNode_PointTo
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import (
    BtNode_DoorDetection,
    BtNode_FeatureExtraction,
    BtNode_FeatureMatching,
    BtNode_MaintainEyeContact,
    BtNode_SeatRecommendBbox,
    BtNode_TurnPanTilt,
    BtNode_TurnTo,
    BtNode_LoadPersonReference
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
    ARM_POS_POINT_TO,
    DRINKS,
    FOLLOW_CONFIG,
    HOST_DRINK,
    HOST_NAME,
    KEY_ARM_DROP,
    KEY_ARM_HANDOVER,
    KEY_ARM_NAVIGATING,
    KEY_ARM_POINT_TO,
    KEY_DOOR_POSE,
    KEY_DOOR_STATUS,
    KEY_GUEST1_DRINK,
    KEY_GUEST1_COMPARISON_IMAGE,
    KEY_GUEST1_FEATURES,
    KEY_GUEST1_NAME,
    KEY_GUEST2_COMPARISON_IMAGE,
    KEY_GUEST2_DRINK,
    KEY_GUEST2_FEATURES,
    KEY_GUEST2_NAME,
    KEY_HOST_DRINK,
    KEY_HOST_FEATURES,
    KEY_HOST_NAME,
    KEY_HOST_COMPARISON_IMAGE,
    KEY_HOST_IMAGE,
    KEY_PERSONS,
    KEY_PERSON_CENTROIDS,
    KEY_SEAT_BBOX,
    KEY_SEAT_POINT,
    KEY_SEAT_POINTS,
    KEY_SEAT_RECOMMENDATION,
    KEY_SOFA_POSE,
    NAMES,
    POSE_DOOR,
    POSE_SOFA,
    SEAT_CATALOG,
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


class BtNode_WrapPointAsList(py_trees.behaviour.Behaviour):
    """Read a PointStamped from one blackboard key, write a 1-element list to another.

    Adapts the single-point output of `BtNode_SeatRecommendBbox` to the
    `list[PointStamped]` contract that `BtNode_PointTo` / `BtNode_TurnTo`
    consume via `target_id` indexing.
    """

    def __init__(self, name: str, bb_source_key: str, bb_dest_key: str):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key="src",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_source_key),
        )
        self.blackboard.register_key(
            key="dst",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_dest_key),
        )

    def update(self) -> py_trees.common.Status:
        try:
            pt = self.blackboard.src
        except KeyError:
            self.feedback_message = "Source key not yet written"
            return py_trees.common.Status.FAILURE
        if pt is None:
            self.feedback_message = "No point on source key"
            return py_trees.common.Status.FAILURE
        self.blackboard.dst = [pt]
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
            name="Write arm point-to pose",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_ARM_POINT_TO,
            object=ARM_POS_POINT_TO,
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

def gazeAtSofa():
    root = py_trees.composites.Sequence(name="Gaze at sofa", memory=True)
    if USE_NAV_ORIENTATION_ANGLE_SERVICE:
        turn_pantilt_to_sofa = py_trees.composites.Sequence(
            name="Turn pan-tilt to sofa orientation", 
            memory=True
        )
        turn_pantilt_to_sofa.add_child(
            BtNode_GetOrientationAngle(
                name="get orientation angle to sofa",
                bb_dest_key=KEY_PANTILT_ORIENTATION
            )
        )
        turn_pantilt_to_sofa.add_child(
            BtNode_TurnPanTilt(
                name="Turn pan-tilt to sofa orientation",
                x=0.0,
                y=20.0,
                x_key=KEY_PANTILT_ORIENTATION
            )
        )
        root.add_child(turn_pantilt_to_sofa)
    else:
        root.add_child(BtNode_TurnPanTilt(name="Look at sofa", x=0.0, y=20.0))
    return root


def createArrivalTrigger():
    """Try real door-trigger first, then fallback to mock trigger."""
    root = py_trees.composites.Sequence(name="Arrival trigger", memory=True)
    # root.add_child(
    #     py_trees.decorators.Retry(
    #         name=f"Retry stow arm",
    #         child=BtNode_MoveArmSingle(
    #             name=f"Stow arm",
    #             service_name="arm_joint_service",
    #             arm_pose_bb_key=KEY_ARM_NAVIGATING,
    #             add_octomap=False,
    #         ),
    #         num_failures=3,
    #     )
    # )
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
    root.add_child(parallel_going)
    real_trigger = py_trees.composites.Sequence(name="Door detection trigger", memory=True)
    # real_trigger.add_child(
    #     py_trees.decorators.Retry(
    #         name="Retry door detection",
    #         child=BtNode_DoorDetection(name="Detect open door", bb_door_state_key=KEY_DOOR_STATUS),
    #         num_failures=3,
    #     )
    # )
    real_trigger.add_child(
        BtNode_Announce(
            name="Arrival detected announcement",
            bb_source=None,
            message="I see a guest. Please come in and speak to me after the beep sound",
        )
    )
    # real_trigger.add_child(
    #     BtNode_Announce(
    #         name="announce speak after beep sound",
    #         bb_source=None,
    #         message="Please come in and speak to me after the beep sound"
    #     )
    # )
    root.add_child(real_trigger)
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
        num_failures=10
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
        BtNode_PhraseExtractionAction(
            name=f"High-conf extract {field_name}",
            wordlist=word_list,
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
    root.add_child(py_trees.decorators.Retry(name="retry 3 times", child=fallback, num_failures=3))
    return root


def createWriteHostInfo():
    parallel_announce = py_trees.composites.Parallel(
        name="Parallel announce host scan info",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    parallel_announce.add_child(
        BtNode_Announce(
            name="Announce host scan info",
            bb_source=None,
            message="loading"
        )
    )
    root = py_trees.composites.Sequence(
        name="Write host info", memory=True
    )
    root.add_child(
        BtNode_LoadPersonReference(
            name="load host ref",
            image_path=image_path,
            description_path=description_path,
            bb_features_key=KEY_HOST_FEATURES,
            bb_image_key=KEY_HOST_IMAGE
        )
    )
    root.add_child(
        BtNode_CombinePerson(
            name="pack host",
            key_dest=KEY_PERSONS,
            key_name=KEY_HOST_NAME,
            key_drink=KEY_HOST_DRINK,
            key_features=KEY_HOST_FEATURES,
            key_image=KEY_HOST_IMAGE,
        )
    )
    root.add_child(
        BtNode_TurnPanTilt(
            name="look forward",
            x=0.0,
            y=45.0
        )
    )
    root.add_child(
        BtNode_GripperAction(
            "close gripper before starting HRI",
            open_gripper=False
        )
    )
    parallel_announce.add_child(root)
    return parallel_announce

def createScanHostFeatures():
    """Navigate to sofa, scan the seated host, register them at index 0 of KEY_PERSONS.

    Mirrors `Receptionist/receptionist_new.py:createScanHostFeatures`. Runs at
    task start before the first guest arrival so SeatRecommend / FeatureMatching
    treat the host as a known seated person. After this completes,
    `KEY_PERSONS == [host]`; subsequent guest intakes append to indices 1, 2.
    """
    root = py_trees.composites.Sequence(name="Scan host features", memory=True)

    parallel_going = py_trees.composites.Parallel(
        name="Parallel go to sofa and announce",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    parallel_going.add_child(BtNode_Announce(
        name="Host scan announcement",
        bb_source=None,
        message="Going to scan host",
    ))
    parallel_going.add_child(
        py_trees.decorators.Retry(
            name="Retry goto sofa for host scan",
            child=BtNode_GotoAction(name="Go to sofa for host scan", key=KEY_SOFA_POSE),
            num_failures=5,
        )
    )
    parallel_going = py_trees.composites.Parallel(
        name="Parallel host scan",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    root.add_child(parallel_going)
    root.add_child(gazeAtSofa())

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
                bb_image_key=KEY_HOST_COMPARISON_IMAGE,
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
            key_image=KEY_HOST_COMPARISON_IMAGE,
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
        image_key = KEY_GUEST1_COMPARISON_IMAGE
    else:
        name_key = KEY_GUEST2_NAME
        drink_key = KEY_GUEST2_DRINK
        feature_key = KEY_GUEST2_FEATURES
        image_key = KEY_GUEST2_COMPARISON_IMAGE

    root = py_trees.composites.Sequence(name=f"Guest {guest_idx} intake", memory=True)
    
    info_intake = py_trees.composites.Sequence(
        name=f"Guest {guest_idx} info intake",
        memory=True,
    )
    info_intake.add_child(_create_get_info("name", name_key, NAMES))
    info_intake.add_child(_create_get_info("favorite drink", drink_key, DRINKS))

    maintain_eye_contact = BtNode_MaintainEyeContact(
        name=f"Maintain eye contact during guest {guest_idx} info intake",
        track_centermost=True
    )

    info_intake_with_eye_contact = py_trees.composites.Parallel(
        name=f"Guest {guest_idx} info intake with eye contact",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected([info_intake]),
        children=[info_intake, maintain_eye_contact]
    )

    root.add_child(info_intake_with_eye_contact)

    root.add_child(BtNode_TurnPanTilt(name="look up", x=0.0, y=45.0))
    root.add_child(
        BtNode_Announce(
            name="announce stand in front of me", 
            bb_source=None, 
            message="Please stand about two meters in front of me so I can remember you. Thank you"
        )
    )
    root.add_child(BtNode_Announce(
        name="announce thank you for standing in front me",
        bb_source=None,
        message="Please look at me and hold still for a moment."
    ))
    root.add_child(
        py_trees.decorators.Retry(
            name=f"Retry guest {guest_idx} feature extraction",
            child=BtNode_FeatureExtraction(
                name=f"Extract guest {guest_idx} features",
                bb_dest_key=feature_key,
                bb_image_key=image_key,
            ),
            num_failures=3,
        )
    )
    root.add_child(
        BtNode_CombinePerson(
            name=f"Store guest {guest_idx} profile",
            key_dest=KEY_PERSONS,
            key_name=name_key,
            key_drink=drink_key,
            key_features=feature_key,
            key_image=image_key,
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
    escort.add_child(gazeAtSofa())

    vision_branch = py_trees.composites.Sequence(
        name=f"Scan seated personnel guest {guest_idx}", memory=True
    )
    vision_branch.add_child(
        BtNode_SeatRecommendBbox(
            name=f"Seat recommend (bbox) guest {guest_idx}",
            bb_recommendation_key=KEY_SEAT_RECOMMENDATION,
            bb_bbox_key=KEY_SEAT_BBOX,
            bb_point_key=KEY_SEAT_POINT,
            bb_source_key=KEY_PERSONS,
            target_frame="base_link",
            known_seats=SEAT_CATALOG,
        )
    )

    turn_and_maintain_gaze_branch = py_trees.composites.Sequence(
        name=f"Turn and maintain gaze guest {guest_idx}", memory=True
    )

    turn_and_maintain_gaze_branch.add_child(
        BtNode_WaitTicks(
            name="wait a moment for orbbec to finish settling",
            ticks=30
        )
    )

    turn_and_maintain_gaze_branch.add_child(
        BtNode_TurnPanTilt(name=f"Look at guest", x=90.0, y=45.0, speed=0.0)
    )
    turn_and_maintain_gaze_branch.add_child(
        BtNode_MaintainEyeContact(
            name=f"Maintain eye contact guest {guest_idx}",
            track_centermost=True,
        )
    )

    audio_branch = py_trees.composites.Sequence(
        name=f"Sofa arrival announcements guest {guest_idx}", memory=True
    )
    audio_branch.add_child(
        BtNode_Announce(
            name=f"Arrived at sofa guest {guest_idx}",
            bb_source=None,
            message="I have arrived at the sofa. Scanning seated personnel. Please cross behind me and stand to my right.",
        )
    )
    audio_branch.add_child(
        BtNode_Announce(
            name=f"Stand on right guest {guest_idx}",
            bb_source=None,
            message="Trying to determine an empty seat for you. Thank you for your patience.",
        )
    )
    audio_branch.add_child(
        BtNode_Announce(
            name="announce still waiting",
            bb_source=None,
            message="Still trying to determine"
        )
    )

    _vision_branch = py_trees.decorators.FailureIsSuccess(
            name=f"Vision scan (audio still completes) guest {guest_idx}",
            child=vision_branch,
        )

    scan_and_announce = py_trees.composites.Parallel(
        name=f"Scan + announce sofa guest {guest_idx}",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected([_vision_branch, audio_branch]),
        children=[
            _vision_branch,
            audio_branch,
            turn_and_maintain_gaze_branch
        ]
    )

    seat_recommend_primary = py_trees.composites.Sequence(
        name=f"Seat recommend guest {guest_idx}", memory=True
    )
    seat_recommend_primary.add_child(BtNode_TurnPanTilt(name=f"Look at guest", x=90.0, y=45.0, speed=0.0))
    seat_recommend_primary.add_child(scan_and_announce)
    seat_recommend_primary.add_child(
        BtNode_WrapPointAsList(
            name=f"Wrap seat point as list guest {guest_idx}",
            bb_source_key=KEY_SEAT_POINT,
            bb_dest_key=KEY_SEAT_POINTS,
        )
    )
    recommend_audio = py_trees.composites.Sequence(name=f"Announce recommendation with eye contact guest {guest_idx}", memory=True)
    recommend_audio.add_child(
        py_trees.decorators.FailureIsSuccess(
            name=f"Arm point-to seat (best-effort) guest {guest_idx}",
            child=py_trees.decorators.Retry(
                name=f"Retry arm point-to seat guest {guest_idx}",
                child=BtNode_PointTo(
                    name=f"Point arm to seat guest {guest_idx}",
                    bb_key_persons=KEY_PERSONS,
                    bb_key_points=KEY_SEAT_POINTS,
                    bb_key_init_pose=KEY_ARM_POINT_TO,
                    target_id=0,
                ),
                num_failures=3,
            ),
        )
    )
    recommend_audio.add_child(
        BtNode_Announce(
            name=f"Announce seat recommendation guest {guest_idx}",
            bb_source=None,
            message="Please sit here.",
        )
    )
    # Centermost gaze: at this seat-recommend site KEY_PERSON_CENTROIDS is
    # not populated for the active guest — the upstream vision_branch runs
    # BtNode_SeatRecommendBbox (seat geometry), not BtNode_FeatureMatching
    # (guest centroids). track_centermost=True makes the server lock onto
    # whoever is most image-centered, which is correct because the
    # BtNode_TurnPanTilt at (90, 45) just before this node centered the
    # standing guest in the camera. Bystanders standing closer to the
    # robot but off to the side don't steal the lock.
    track_person = BtNode_MaintainEyeContact(
        name=f"Maintain eye contact guest {guest_idx}",
        track_centermost=True,
    )
    recommend_with_eye_contact = py_trees.composites.Parallel(
        name=f"Recommend seat with eye contact guest {guest_idx}",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected([recommend_audio]),
        children=[recommend_audio, track_person],
    )
    seat_recommend_primary.add_child(recommend_with_eye_contact)

    # Fallback: when primary fails (no seat point or service unavailable),
    # the guest still gets a verbal cue so the task continues.
    seat_recommend_fallback = py_trees.composites.Sequence(
        name=f"Seat recommend fallback guest {guest_idx}", memory=True)
    seat_recommend_fallback.add_child(BtNode_TurnPanTilt(name=f"Look at guest", x=90.0, y=45.0, speed=0.0))
    seat_recommend_fallback.add_child(BtNode_Announce(
        name=f"Fallback seat announcement guest {guest_idx}",
        bb_source=None,
        message=(
            "I am sorry, I cannot identify a specific seat for you. "
            "Please take any available seat near the others."
        ),
    ))
    seat_recommend = py_trees.composites.Selector(
        name=f"Seat recommendation (best-effort) guest {guest_idx}",
        memory=True,
    )
    seat_recommend.add_child(seat_recommend_primary)
    seat_recommend.add_child(seat_recommend_fallback)

    escort.add_child(seat_recommend)
    # escort.add_child(BtNode_Announce(
    #     name=f"Complete escort announcement guest {guest_idx}",
    #     bb_source=None,
    #     message="Please make yourself comfortable while sitting down"
    # ))

    # Restore head to navigation tilt before next phase. We deliberately do
    # NOT wrap escort in `_with_gaze_supervisor`: MaintainEyeContact's
    # continuous pan/tilt control fights the explicit head-down command
    # required to scan the sofa, defeating the seat-recommend perception.
    escort.add_child(
        BtNode_TurnPanTilt(
            name=f"Look forward after seat guest {guest_idx}",
            x=0.0,
            y=35.0,
            speed=0.0,
        )
    )

    escort_with_pantilt_fallback = py_trees.composites.Sequence(
        name="escort with fall back placing pan tilt in correct position",
        memory=True,
        children=[
            py_trees.decorators.FailureIsSuccess("failure is success", child=escort),
            BtNode_TurnPanTilt(
                name=f"Ensure head forward after seat guest {guest_idx}",
                x=0.0,
                y=35.0,
            )
        ]
    )

    return escort_with_pantilt_fallback


def createTwoWayIntroduction():
    """Introduce guest1<->guest2 with arm-point + gaze-lock per direction."""
    root = py_trees.composites.Sequence(name="Two-way introductions", memory=True)

    # Locate both seated guests ahead of the intros so we can point the arm
    # at the right one and orient the head at the other. trim_last_person=False
    # because BOTH guests are seated by this point (unlike Receptionist's
    # pattern, where the active guest is still standing).
    root.add_child(BtNode_Announce(
        name=f"Complete escort announcement",
        bb_source=None,
        message="Please sit down and make yourself comfortable."
    ))

    root.add_child(gazeAtSofa())
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
    #
    # Sequencing rule: BtNode_TurnTo first (publishes one absolute pan-tilt
    # command at the centroid), THEN BtNode_MaintainEyeContact in parallel
    # with the speech announce. Running them concurrently the other way
    # round lets the eye-contact loop overwrite the explicit aim before the
    # head ever reaches it — see the same caveat at createEscortAndSeat.
    intro1 = py_trees.composites.Sequence(name="Intro guest1 to guest2 with gaze", memory=True)
    turn_pantilt_and_arm1 = py_trees.composites.Parallel(
        name="Intro1 arm+gaze aim",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )

    turn_pantilt_and_arm1.add_child(
        BtNode_TurnTo(name="Look at guest2", bb_key_persons=KEY_PERSONS, bb_key_points=KEY_PERSON_CENTROIDS, target_id=2)
    )
    turn_pantilt_and_arm1.add_child(
        BtNode_PointTo(
            name="Point arm at guest1",
            bb_key_persons=KEY_PERSONS,
            bb_key_points=KEY_PERSON_CENTROIDS,
            bb_key_init_pose=KEY_ARM_POINT_TO,
            target_id=1,
        )
    )
    # Build the announce + gaze parallel; SuccessOnSelected wants node
    # objects so instantiate Introduce first and re-use the reference.
    introduce1 = BtNode_Introduce(
        name="Introduce guest1 to guest2",
        key_person=KEY_PERSONS,
        target_id=2,
        introduced_id=1,
        describe_introduced=True,
    )
    announce_with_gaze1 = py_trees.composites.Parallel(
        name="Announce intro1 + maintain gaze",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected([introduce1]),
        children=[
            introduce1,
            # Targeted-lock: gaze stays on guest2 (the introducee) even if a
            # closer bystander appears. KEY_PERSON_CENTROIDS is populated by
            # the BtNode_FeatureMatching above (line ~653, trim_last_person=False).
            # BtNode_MaintainEyeContact(
            #     name="Maintain eye contact during intro1",
            #     target_id=2,
            #     bb_key_persons=KEY_PERSONS,
            #     bb_key_points=KEY_PERSON_CENTROIDS,
            #     track_centermost=True
            # ),
        ],
    )
    intro1.add_child(turn_pantilt_and_arm1)
    intro1.add_child(announce_with_gaze1)

    intro2 = py_trees.composites.Sequence(name="Intro guest2 to guest1 with gaze", memory=True)
    turn_pantilt_and_arm2 = py_trees.composites.Parallel(
        name="Intro2 arm+gaze aim",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )

    turn_pantilt_and_arm2.add_child(
        BtNode_TurnTo(name="Look at guest1", bb_key_persons=KEY_PERSONS, bb_key_points=KEY_PERSON_CENTROIDS, target_id=1)
    )
    turn_pantilt_and_arm2.add_child(
        BtNode_PointTo(
            name="Point arm at guest2",
            bb_key_persons=KEY_PERSONS,
            bb_key_points=KEY_PERSON_CENTROIDS,
            bb_key_init_pose=KEY_ARM_POINT_TO,
            target_id=2,
        )
    )
    intro2.add_child(turn_pantilt_and_arm2)
    introduce2 = BtNode_Introduce(
        name="Introduce guest2 to guest1",
        key_person=KEY_PERSONS,
        target_id=1,
        introduced_id=2,
        describe_introduced=True,
    )
    announce_with_gaze2 = py_trees.composites.Parallel(
        name="Announce intro2 + maintain gaze",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected([introduce2]),
        children=[
            introduce2,
            # Targeted-lock onto guest1 for the symmetric direction.
            # BtNode_MaintainEyeContact(
            #     name="Maintain eye contact during intro2",
            #     target_id=1,
            #     bb_key_persons=KEY_PERSONS,
            #     bb_key_points=KEY_PERSON_CENTROIDS,
            #     track_centermost=True
            # ),
        ],
    )
    intro2.add_child(announce_with_gaze2)

    root.add_child(intro1)
    root.add_child(intro2)

    return root


def createBagFlow():
    """Handle bag handover, host-follow proxy, and bag drop sequence."""
    root = py_trees.composites.Sequence(name="Bag handover-follow-drop", memory=True)
    
    parallel_movearm_announce = py_trees.composites.Parallel(
        name="Parallel move arm to handover and announce",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    parallel_movearm_announce.add_child(
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
    parallel_movearm_announce.add_child(BtNode_Announce(
        name="Announce come over",
        bb_source=None,
        message="introductions finished. please come over"
    ))

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
    root.add_child(BtNode_Announce(
        name="announce attempting follow",
        bb_source=None,
        message="Attempting to follow"
    ))
    root.add_child(BtNode_Announce(
        name="announce attempting follow",
        bb_source=None,
        message="Attempting to follow"
    ))
    root.add_child(BtNode_Announce(
        name="announce attempting follow",
        bb_source=None,
        message="Attempting to follow"
    ))
    root.add_child(BtNode_Announce(
        name="announce attempting follow",
        bb_source=None,
        message="Attempting to follow"
    ))
    root.add_child(BtNode_Announce(
        name="announce follow failed",
        bb_source=None,
        message="Follow failed"
    ))
    # root.add_child(createFollowPerson(FOLLOW_CONFIG))
    # root.add_child(
    #     py_trees.decorators.Retry(
    #         name="Retry arm to drop pose",
    #         child=BtNode_MoveArmSingle(
    #             name="Move arm to drop pose",
    #             service_name="arm_joint_service",
    #             arm_pose_bb_key=KEY_ARM_DROP,
    #             add_octomap=False,
    #         ),
    #         num_failures=3,
    #     )
    # )
    # root.add_child(BtNode_GripperAction(name="Open gripper to drop bag", open_gripper=True))
    # root.add_child(BtNode_MockSafetyCheck(name="Drop confirmation detector TODO", todo="replace with bag-drop detector"))
    # root.add_child(BtNode_GripperAction(name="Close gripper after drop", open_gripper=False))
    # root.add_child(BtNode_MoveArmSingle(name="move arm back to navigation pose", service_name="arm_joint_service", arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False))
    return root


def arrivalAndIntake(guest_id: int):
    pass


def createHRITask():
    """Compose full HRI task in the expected competition order."""
    root = py_trees.composites.Sequence(name="HRI Task", memory=True)
    root.add_child(createConstantWriter())
    root.add_child(
        BtNode_Announce(
            name="HRI start announcement",
            bb_source=None,
            message=f"HRI task started.",
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
            num_failures=2,
        )
    )
    # root.add_child(createScanHostFeatures())
    root.add_child(createWriteHostInfo())
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
