from __future__ import annotations

import json
import math
from pathlib import Path

import py_trees
try:
    import rclpy
except ModuleNotFoundError:  # pragma: no cover - exercised in non-ROS unit tests
    rclpy = None
try:
    from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
    from std_msgs.msg import Header
except ModuleNotFoundError:  # pragma: no cover - exercised in non-ROS unit tests
    class Point:  # pylint: disable=too-few-public-methods
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x
            self.y = y
            self.z = z

    class Quaternion:  # pylint: disable=too-few-public-methods
        def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
            self.x = x
            self.y = y
            self.z = z
            self.w = w

    class Pose:  # pylint: disable=too-few-public-methods
        def __init__(self, position=None, orientation=None):
            self.position = position or Point()
            self.orientation = orientation or Quaternion()

    class Header:  # pylint: disable=too-few-public-methods
        def __init__(self, stamp=None, frame_id=""):
            self.stamp = stamp
            self.frame_id = frame_id

    class PoseStamped:  # pylint: disable=too-few-public-methods
        def __init__(self, header=None, pose=None):
            self.header = header or Header()
            self.pose = pose or Pose()

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


def _load_constants():
    candidates = [
        Path(__file__).with_name("constants.json"),
        Path(__file__).resolve().parents[1] / "Receptionist" / "constants.json",
    ]
    for path in candidates:
        if path.exists():
            with path.open("r", encoding="utf-8") as file:
                return json.load(file)
    raise FileNotFoundError("Unable to find HRI/Receptionist constants.json")


constants = _load_constants()


def _pose_reader(pose_dict):
    stamp = None
    if rclpy is not None:
        stamp = rclpy.time.Time().to_msg()
    return PoseStamped(
        header=Header(stamp=stamp, frame_id="map"),
        pose=Pose(
            position=Point(
                x=pose_dict["point"]["x"],
                y=pose_dict["point"]["y"],
                z=0.0,
            ),
            orientation=Quaternion(
                x=pose_dict["orientation"]["x"],
                y=pose_dict["orientation"]["y"],
                z=pose_dict["orientation"]["z"],
                w=pose_dict["orientation"]["w"],
            ),
        ),
    )


def _arm_pose_reader(arm_pose_list):
    return [x / 180 * math.pi for x in arm_pose_list]


POSE_DOOR = _pose_reader(constants["pose_door"])
POSE_SOFA = _pose_reader(constants["pose_sofa"])
ARM_POS_NAVIGATING = _arm_pose_reader(constants["arm_pos_navigating"])
ARM_POS_HANDOVER = _arm_pose_reader(constants.get("arm_pos_handover", constants["arm_pos_point_to"]))
ARM_POS_DROP = _arm_pose_reader(constants.get("arm_pos_drop", constants["arm_pos_navigating"]))

HOST_NAME = constants.get("host_name", "Host")
HOST_DRINK = constants.get("host_drink", "water")
DRINKS = list(constants.get("drinks", ["water", "cola", "juice", "milk", "coffee"]))
NAMES = list(constants.get("names", ["Alex", "Sam", "Maria", "John"]))

KEY_DOOR_POSE = "hri_door_pose"
KEY_SOFA_POSE = "hri_sofa_pose"
KEY_DOOR_STATUS = "hri_door_status"
KEY_ARM_NAVIGATING = "hri_arm_navigating"
KEY_ARM_HANDOVER = "hri_arm_handover"
KEY_ARM_DROP = "hri_arm_drop"
KEY_PERSONS = "hri_persons"
KEY_SEAT_RECOMMENDATION = "hri_seat_recommendation"

KEY_GUEST1_NAME = "hri_guest1_name"
KEY_GUEST1_DRINK = "hri_guest1_drink"
KEY_GUEST1_FEATURES = "hri_guest1_features"

KEY_GUEST2_NAME = "hri_guest2_name"
KEY_GUEST2_DRINK = "hri_guest2_drink"
KEY_GUEST2_FEATURES = "hri_guest2_features"


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
            message="I detected a guest at the door.",
        )
    )
    root.add_child(real_trigger)
    root.add_child(BtNode_MockArrivalTrigger())
    return root


def _create_get_info(field_name: str, storage_key: str, word_list: list[str]):
    root = py_trees.composites.Sequence(name=f"Get {field_name}", memory=True)
    loop = py_trees.composites.Sequence(name=f"Get+confirm {field_name}", memory=True)
    loop.add_child(
        BtNode_Announce(
            name=f"Prompt for {field_name}",
            bb_source=None,
            message=f"Please tell me your {field_name} after the beep.",
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
    escort = py_trees.composites.Sequence(name=f"Escort and seat guest {guest_idx}", memory=True)
    escort.add_child(
        BtNode_Announce(
            name=f"Invite guest {guest_idx} to follow",
            bb_source=None,
            message="Please follow me to the seating area.",
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
            message="Please take this seat.",
        )
    )
    escort.add_child(seat_recommend)
    return _with_gaze_supervisor(f"Gaze-supervised escort guest {guest_idx}", escort)


def createTwoWayIntroduction():
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
    root = py_trees.composites.Sequence(name="Bag handover-follow-drop", memory=True)
    root.add_child(
        BtNode_Announce(
            name="Ask for bag handover",
            bb_source=None,
            message="Please hand me your bag.",
        )
    )
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
    root.add_child(BtNode_MockSafetyCheck(name="Safe handover detector TODO", todo="replace with handover-done detector"))
    root.add_child(BtNode_GripperAction(name="Close gripper with bag", open_gripper=False))
    root.add_child(
        BtNode_Announce(
            name="Follow host announcement",
            bb_source=None,
            message="I will follow you and carry the bag.",
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
    root = py_trees.composites.Sequence(name="HRI Task", memory=True)
    root.add_child(createConstantWriter())
    root.add_child(
        BtNode_Announce(
            name="HRI start announcement",
            bb_source=None,
            message=f"Starting HRI task. Host is {HOST_NAME} and host drink is {HOST_DRINK}.",
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
            message="HRI task completed.",
        )
    )
    return root
