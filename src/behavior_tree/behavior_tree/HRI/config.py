from __future__ import annotations

"""HRI configuration.

Centralizes:
- constant-file loading
- precomputed poses/arm joints
- blackboard key names

Keeping these in one module makes HRI task logic easier to read and maintain.
"""

import json
import math
from pathlib import Path

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


def _load_constants():
    """Load HRI constants.

    Priority:
    1. local `HRI/constants.json` (if provided later)
    2. fallback to `Receptionist/constants.json` (current source of truth)
    """
    candidates = [
        Path(__file__).with_name("constants.json"),
        Path(__file__).resolve().parents[1] / "Receptionist" / "constants.json",
    ]
    for path in candidates:
        if path.exists():
            with path.open("r", encoding="utf-8") as file:
                return json.load(file)
    raise FileNotFoundError("Unable to find HRI/Receptionist constants.json")


def _pose_reader(pose_dict):
    """Convert dict pose config into `PoseStamped` in map frame."""
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
    """Convert degrees from JSON into radians expected by arm services."""
    return [x / 180 * math.pi for x in arm_pose_list]


constants = _load_constants()

POSE_DOOR = _pose_reader(constants["pose_door"])
POSE_SOFA = _pose_reader(constants["pose_sofa"])
ARM_POS_NAVIGATING = _arm_pose_reader(constants["arm_pos_navigating"])
ARM_POS_HANDOVER = _arm_pose_reader(
    constants.get("arm_pos_handover", constants["arm_pos_point_to"])
)
ARM_POS_DROP = _arm_pose_reader(constants.get("arm_pos_drop", constants["arm_pos_navigating"]))
ARM_POS_POINT_TO = _arm_pose_reader(constants["arm_pos_point_to"])

HOST_NAME = constants.get("host_name", "Host")
HOST_DRINK = constants.get("host_drink", "water")
DRINKS = list(constants.get("drinks", ["water", "cola", "juice", "milk", "coffee"]))
NAMES = list(constants.get("names", ["Alex", "Sam", "Maria", "John"]))

# Navigation / perception keys
KEY_DOOR_POSE = "hri_door_pose"
KEY_SOFA_POSE = "hri_sofa_pose"
KEY_DOOR_STATUS = "hri_door_status"
KEY_ARM_NAVIGATING = "hri_arm_navigating"
KEY_ARM_HANDOVER = "hri_arm_handover"
KEY_ARM_DROP = "hri_arm_drop"
KEY_ARM_POINT_TO = "hri_arm_point_to"
KEY_PERSONS = "hri_persons"
KEY_PERSON_CENTROIDS = "hri_person_centroids"
KEY_SEAT_RECOMMENDATION = "hri_seat_recommendation"
KEY_SEAT_BBOX = "hri_seat_bbox"
KEY_SEAT_POINT = "hri_seat_point"
KEY_SEAT_POINTS = "hri_seat_points"

# Host-specific memory keys (host is scanned at task start while seated on sofa)
KEY_HOST_NAME = "hri_host_name"
KEY_HOST_DRINK = "hri_host_drink"
KEY_HOST_FEATURES = "hri_host_features"

# Guest-specific memory keys
KEY_GUEST1_NAME = "hri_guest1_name"
KEY_GUEST1_DRINK = "hri_guest1_drink"
KEY_GUEST1_FEATURES = "hri_guest1_features"
KEY_GUEST1_COMPARISON_IMAGE = "hri_guest1_comparison_image"

KEY_GUEST2_NAME = "hri_guest2_name"
KEY_GUEST2_DRINK = "hri_guest2_drink"
KEY_GUEST2_FEATURES = "hri_guest2_features"
KEY_GUEST2_COMPARISON_IMAGE = "hri_guest2_comparison_image"

# Follow-person subtree config — see `HRI/follow.py:createFollowPerson`.
FOLLOW_CONFIG = dict(constants.get("follow", {}))
FOLLOW_CONFIG.setdefault("lost_short_sec", 5.0)
FOLLOW_CONFIG.setdefault("lost_long_sec", 15.0)
FOLLOW_CONFIG.setdefault("n_nav_retry", 3)
FOLLOW_CONFIG.setdefault("n_restart", 2)
FOLLOW_CONFIG.setdefault("follow_timeout_sec", 2.0)
FOLLOW_CONFIG.setdefault("target_frame", "map")
FOLLOW_CONFIG.setdefault("target_point_topic", "/target_points")
FOLLOW_CONFIG.setdefault("track_action_name", "track_person")
FOLLOW_CONFIG.setdefault("follow_action_name", "tracking_server")
FOLLOW_CONFIG.setdefault(
    "announce_short", "Please stop and wait for Tinker to catch up."
)
FOLLOW_CONFIG.setdefault(
    "announce_long", "I've lost you. Please come back in front of Tinker."
)
