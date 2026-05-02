from __future__ import annotations

"""PickAndPlace configuration and shared blackboard keys.

Hardware-aware design (see RULEBOOK_PLAN.md):
  - Robot cannot pick from the floor.
  - Robot cannot place into dishwasher rack accurately;
    cutlery + tableware go to a wash-staging surface instead.
"""

import json
import math
from pathlib import Path

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


def _make_header():
    # All-zero stamp; Nav2 substitutes "now" rather than rejecting on stale timestamps.
    return Header(frame_id="map")


def _pose_reader(pose_dict):
    return PoseStamped(
        header=_make_header(),
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
    return PointStamped(
        header=_make_header(),
        point=Point(
            x=point_dict["x"],
            y=point_dict["y"],
            z=point_dict.get("z", 0.0),
        ),
    )


def _load_constants():
    """Load HRI constants.

    Priority:
    1. local `HRI/constants.json` (if provided later)
    2. fallback to `PickAndPlace/constants.json` (current source of truth)
    """
    candidates = [
        Path(__file__).with_name("constants.json"),
        Path(__file__).resolve().parents[1] / "PickAndPlace" / "constants.json",
    ]
    for path in candidates:
        if path.exists():
            with path.open("r", encoding="utf-8") as file:
                return json.load(file)
    raise FileNotFoundError("Unable to find PickAndPlace constants.json")


constants = _load_constants()

# --- Materialize PoseStamped / PointStamped ---
POSE_KITCHEN_ENTRY = _pose_reader(constants["pose_kitchen_entry"])
# Single source pose for the cleanup loop; pose_table_2..6 stay in JSON as
# future hooks for a multi-table sweep.
POSE_TABLE = _pose_reader(constants["pose_table_1"])
POSE_WASH_STAGING = _pose_reader(constants["pose_wash_staging"])
POSE_TRASH_BIN = _pose_reader(constants["pose_trash_bin"])
POSE_CABINET = _pose_reader(constants["pose_cabinet"])
POSE_KITCHEN_SHELF = _pose_reader(constants["pose_kitchen_shelf"])
POSE_DISH_WASHER = _pose_reader(constants["pose_dish_washer"])

POINT_WASH_STAGING = _point_reader(constants["point_wash_staging"])
POINT_CABINET_DEFAULT = _point_reader(constants["point_cabinet_default"])
POINT_KITCHEN_SURFACE = _point_reader(constants["point_kitchen_surface"])
POINT_BREAKFAST_BOWL = _point_reader(constants["point_breakfast_bowl"])
POINT_BREAKFAST_SPOON = _point_reader(constants["point_breakfast_spoon"])
POINT_BREAKFAST_CEREAL = _point_reader(constants["point_breakfast_cereal"])
POINT_BREAKFAST_MILK = _point_reader(constants["point_breakfast_milk"])
POINT_SHELF_LEFT = _point_reader(constants["point_shelf_left"])
POINT_SHELF_RIGHT = _point_reader(constants["point_shelf_right"])

# --- Arm joint configs (radians) ---
# One arm pose per destination — set names mirror constants.json:
#   arm_pos_navigating: stowed for base motion
#   arm_pos_table:      look-down at the dining table for scan / re-acquire / breakfast place
#   arm_pos_cabinet:    cabinet shelf scan + place (orbbec octomap before this move)
#   arm_pos_wash:       wash-staging surface place (realsense env_points)
#   arm_pos_trash:      reach over the bin for drop
ARM_POS_NAVIGATING = [x / 180 * math.pi for x in constants["arm_pos_navigating"]]
ARM_POS_TABLE = [x / 180 * math.pi for x in constants["arm_pos_table"]]
ARM_POS_CABINET = [x / 180 * math.pi for x in constants["arm_pos_cabinet"]]
ARM_POS_WASH = [x / 180 * math.pi for x in constants["arm_pos_wash"]]
ARM_POS_TRASH = [x / 180 * math.pi for x in constants["arm_pos_trash"]]
ARM_POS_WASH_DROP = [x / 180 * math.pi for x in constants['arm_pos_wash_drop']]
ARM_POS_CLEANING_STATION = [x / 180 * math.pi for x in constants['arm_pos_cleaning_station']]

# --- Label sets ---
DESIGNATED_TRASH_LABELS = list(constants["designated_trash_labels"])
CUTLERY_LABELS = list(constants["cutlery_labels"])
TABLEWARE_LABELS = list(constants["tableware_labels"])
WASH_STAGING_LABELS = CUTLERY_LABELS + TABLEWARE_LABELS

# --- Prompts ---
TABLE_SCAN_PROMPT = str(constants["table_scan_prompt"])

# --- Numerics ---
N_LAYERS = int(constants["n_layers"])
MAX_RUNTIME_SEC = float(constants["max_runtime_sec"])
NAV_RETRY_LIMIT = int(constants["nav_retry_limit"])
SCAN_RETRY_LIMIT = int(constants["scan_retry_limit"])
GRASP_RETRY_LIMIT = int(constants["grasp_retry_limit"])

# --- Service / action names ---
ARM_SERVICE_NAME = "arm_joint_service"
GRASP_ACTION_NAME = "start_grasp"
PLACE_ACTION_NAME = "place_action"
TARGET_FRAME = "base_link"

# ============================================================
# Blackboard keys
# ============================================================

# Poses
KEY_POSE_KITCHEN_ENTRY = "pp_pose_kitchen_entry"
KEY_POSE_TABLE = "pp_pose_table"
KEY_POSE_WASH_STAGING = "pp_pose_wash_staging"
KEY_POSE_TRASH_BIN = "pp_pose_trash_bin"
KEY_POSE_CABINET = "pp_pose_cabinet"
KEY_POSE_KITCHEN_SHELF = "pp_pose_kitchen_shelf"
KEY_POSE_DISH_WASHER = "pp_pose_dish_washer"

# Points
KEY_POINT_WASH_STAGING = "pp_point_wash_staging"
KEY_POINT_CABINET_DEFAULT = "pp_point_cabinet_default"
KEY_POINT_KITCHEN_SURFACE = "pp_point_kitchen_surface"
KEY_POINT_BREAKFAST_BOWL = "pp_point_breakfast_bowl"
KEY_POINT_BREAKFAST_SPOON = "pp_point_breakfast_spoon"
KEY_POINT_BREAKFAST_CEREAL = "pp_point_breakfast_cereal"
KEY_POINT_BREAKFAST_MILK = "pp_point_breakfast_milk"
KEY_POINT_SHELF_LEFT = "pp_point_shelf_left"
KEY_POINT_SHELF_RIGHT = "pp_point_shelf_right"
# Dynamic placing point resolved per-item by /placing_location (VLM).
KEY_POINT_PLACING_DYNAMIC = "pp_point_placing_dynamic"

# Arm poses
KEY_ARM_NAVIGATING = "pp_arm_navigating"
KEY_ARM_TABLE = "pp_arm_table"
KEY_ARM_CABINET = "pp_arm_cabinet"
KEY_ARM_CLEANING_STATION = "pp_arm_cleaning_station"
KEY_ARM_WASH = "pp_arm_wash"
KEY_ARM_TRASH = "pp_arm_trash"
KEY_ARM_WASH_DROP = "pp_arm_wash_drop"

# Misc constants
KEY_TARGET_FRAME = "pp_target_frame"
KEY_MAX_RUNTIME = "pp_max_runtime_sec"

# Inventory / queue
KEY_INVENTORY_TABLE = "pp_inventory_table"
KEY_WORK_QUEUE = "pp_work_queue"
KEY_BREAKFAST_QUEUE = "pp_breakfast_queue"

# Active item
KEY_ACTIVE_OBJECT_CLASS = "pp_active_object_class"
KEY_ACTIVE_PROMPT = "pp_active_prompt"
KEY_ACTIVE_SOURCE_POSE = "pp_active_source_pose"
KEY_ACTIVE_TARGET_POSE = "pp_active_target_pose"
KEY_ACTIVE_TARGET_POINT = "pp_active_target_point"

# Per-pick scratch (FindObjTable + GraspWithPose + Place)
KEY_TABLE_IMG = "pp_table_img"
KEY_OBJ_SEG = "pp_obj_seg"
KEY_VISION_RESULT = "pp_vision_result"
KEY_GRASP_ANNOUNCEMENT = "pp_grasp_announcement"
KEY_GRASP_POSE = "pp_grasp_pose"
KEY_OBJECT_LABEL = "pp_object_label"
KEY_ENV_POINTS = "pp_env_points"
KEY_PLACE_REASON = "pp_place_reason"

# Bookkeeping
KEY_SCORE_TRACE = "pp_score_trace"
KEY_PHASE_DEADLINE = "pp_phase_deadline"
KEY_SUMMARY_MESSAGE = "pp_summary_message"
KEY_DOOR_STATUS = "pp_door_status"


# OTHER
KEY_IMG_SHELF = "pp_img_shelf"
KEY_SCAN_RESULTS_SHELF = "scan_result_shelf"
KEY_SCAN_RESULTS_TABLE = "scan_result_table"
KEY_ANNOUNCEMENT_MSG = "announcement_msg"


KEY_GRASP_VISION_RES = "grasp_vision_result"
