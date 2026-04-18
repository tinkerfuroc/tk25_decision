from __future__ import annotations

"""PickAndPlace configuration and shared blackboard keys."""

import json
import math
from pathlib import Path

try:
    import rclpy
except ModuleNotFoundError:  # pragma: no cover - exercised in non-ROS unit tests
    rclpy = None
try:
    from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Quaternion
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

    class Header:  # pylint: disable=too-few-public-methods
        def __init__(self, stamp=None, frame_id=""):
            self.stamp = stamp
            self.frame_id = frame_id

    class Pose:  # pylint: disable=too-few-public-methods
        def __init__(self, position=None, orientation=None):
            self.position = position or Point()
            self.orientation = orientation or Quaternion()

    class PoseStamped:  # pylint: disable=too-few-public-methods
        def __init__(self, header=None, pose=None):
            self.header = header or Header()
            self.pose = pose or Pose()

    class PointStamped:  # pylint: disable=too-few-public-methods
        def __init__(self, header=None, point=None):
            self.header = header or Header()
            self.point = point or Point()


def _default_constants():
    return {
        "pose_start": {
            "point": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "pose_table": {
            "point": {"x": 1.2, "y": 0.4, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "pose_floor_scan": {
            "point": {"x": 1.0, "y": -0.6, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "pose_trash_bin": {
            "point": {"x": -0.6, "y": 0.8, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "pose_clean_table_zone": {
            "point": {"x": 1.0, "y": 0.7, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "point_trash_bin": {"x": -0.6, "y": 0.8, "z": 0.75},
        "point_clean_table_zone": {"x": 1.0, "y": 0.7, "z": 0.75},
        "arm_pos_navigating": [-87.0, -58.8, -2.4, 8.0, 12.3, -68.5, -8.2],
        "table_trash_prompts": ["paper cup", "napkin", "bottle", "can"],
        "floor_trash_prompts": ["plastic bag", "paper", "trash"],
        "table_relocation_prompts": ["spoon", "bowl", "cereal box", "milk"],
        "endgame_breakfast_prompts": ["bowl", "spoon"],
        "max_runtime_sec": 390.0,
    }


def _load_constants():
    defaults = _default_constants()
    path = Path(__file__).with_name("constants.json")
    if not path.exists():
        return defaults
    with path.open("r", encoding="utf-8") as file:
        loaded = json.load(file)
    defaults.update(loaded)
    return defaults


def _pose_reader(pose_dict):
    stamp = rclpy.time.Time().to_msg() if rclpy is not None else None
    return PoseStamped(
        header=Header(stamp=stamp, frame_id="map"),
        pose=Pose(
            position=Point(
                x=pose_dict["point"]["x"],
                y=pose_dict["point"]["y"],
                z=pose_dict["point"].get("z", 0.0),
            ),
            orientation=Quaternion(
                x=pose_dict["orientation"]["x"],
                y=pose_dict["orientation"]["y"],
                z=pose_dict["orientation"]["z"],
                w=pose_dict["orientation"]["w"],
            ),
        ),
    )


def _point_reader(point_dict):
    stamp = rclpy.time.Time().to_msg() if rclpy is not None else None
    return PointStamped(
        header=Header(stamp=stamp, frame_id="map"),
        point=Point(
            x=point_dict["x"],
            y=point_dict["y"],
            z=point_dict.get("z", 0.0),
        ),
    )


constants = _load_constants()

POSE_START = _pose_reader(constants["pose_start"])
POSE_TABLE = _pose_reader(constants["pose_table"])
POSE_FLOOR_SCAN = _pose_reader(constants["pose_floor_scan"])
POSE_TRASH_BIN = _pose_reader(constants["pose_trash_bin"])
POSE_CLEAN_TABLE_ZONE = _pose_reader(constants["pose_clean_table_zone"])

POINT_TRASH_BIN = _point_reader(constants["point_trash_bin"])
POINT_CLEAN_TABLE_ZONE = _point_reader(constants["point_clean_table_zone"])

ARM_POS_NAVIGATING = [x / 180 * math.pi for x in constants["arm_pos_navigating"]]

TABLE_TRASH_PROMPTS = list(constants["table_trash_prompts"])
FLOOR_TRASH_PROMPTS = list(constants["floor_trash_prompts"])
TABLE_RELOCATION_PROMPTS = list(constants["table_relocation_prompts"])
ENDGAME_BREAKFAST_PROMPTS = list(constants["endgame_breakfast_prompts"])

MAX_RUNTIME_SEC = float(constants.get("max_runtime_sec", 390.0))
NAV_RETRY_LIMIT = int(constants.get("nav_retry_limit", 3))
SCAN_RETRY_LIMIT = int(constants.get("scan_retry_limit", 2))
GRASP_RETRY_LIMIT = int(constants.get("grasp_retry_limit", 2))
ENDGAME_MIN_REMAINING_SEC = float(constants.get("endgame_min_remaining_sec", 60.0))

KEY_POSE_START = "pp_pose_start"
KEY_POSE_TABLE = "pp_pose_table"
KEY_POSE_FLOOR_SCAN = "pp_pose_floor_scan"
KEY_POSE_TRASH_BIN = "pp_pose_trash_bin"
KEY_POSE_CLEAN_TABLE_ZONE = "pp_pose_clean_table_zone"

KEY_POINT_TRASH_BIN = "pp_point_trash_bin"
KEY_POINT_CLEAN_TABLE_ZONE = "pp_point_clean_table_zone"
KEY_ARM_NAV = "pp_arm_nav_pose"

KEY_INVENTORY_TABLE = "pp_inventory_table"
KEY_INVENTORY_FLOOR = "pp_inventory_floor"
KEY_WORK_QUEUE = "pp_work_queue"

KEY_ACTIVE_OBJECT = "pp_active_object"
KEY_ACTIVE_OBJECT_CLASS = "pp_active_object_class"
KEY_ACTIVE_SOURCE_POSE = "pp_active_source_pose"
KEY_ACTIVE_TARGET_POSE = "pp_active_target_pose"
KEY_ACTIVE_TARGET_POINT = "pp_active_target_point"

KEY_VISION_RESULT = "pp_vision_result"
KEY_SCORE_TRACE = "pp_score_trace"
KEY_PHASE_DEADLINE = "pp_phase_deadline"
KEY_MAX_RUNTIME = "pp_max_runtime_sec"
KEY_SUMMARY_MESSAGE = "pp_summary_message"
