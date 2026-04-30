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


def _default_constants():
    return {
        # --- Poses (frame=map) ---
        "pose_kitchen_entry": {
            "point": {"x": 0.5, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "pose_table": {
            "point": {"x": 1.2, "y": 0.4, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "pose_wash_staging": {
            "point": {"x": 1.4, "y": -0.6, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "pose_trash_bin": {
            "point": {"x": -0.6, "y": 0.8, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "pose_cabinet": {
            "point": {"x": 1.4658832550048828, "y": -1.2834669351577759, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": -0.4766690590, "w": 0.8790828221},
        },
        "pose_kitchen_shelf": {
            "point": {"x": 0.9, "y": -1.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        # --- Points (frame=map) ---
        "point_wash_staging": {"x": 1.4, "y": -0.6, "z": 0.75},
        "point_trash_bin": {"x": -0.6, "y": 0.8, "z": 0.75},
        "point_cabinet_default": {"x": 1.5, "y": -1.3, "z": 0.85},
        "point_kitchen_surface": {"x": 0.9, "y": -1.0, "z": 0.75},
        # Breakfast layout on the dining table. Spoon adjacent to bowl (≤15 cm),
        # cereal adjacent to milk (≤15 cm), all pairs ≥5 cm apart per rulebook §8.
        "point_breakfast_bowl":   {"x": 1.20, "y": 0.50, "z": 0.75},
        "point_breakfast_spoon":  {"x": 1.20, "y": 0.62, "z": 0.75},
        "point_breakfast_cereal": {"x": 1.35, "y": 0.50, "z": 0.75},
        "point_breakfast_milk":   {"x": 1.35, "y": 0.62, "z": 0.75},
        "point_shelf_left":  {"x": 1.30, "y": -1.40, "z": 0.0},
        "point_shelf_right": {"x": 1.60, "y": -1.20, "z": 0.0},
        # --- Arm joint configs (degrees, converted to radians at module load) ---
        "arm_pos_navigating": [-87.0, -58.8, -2.4, 8.0, 12.3, -68.5, -8.2],
        "arm_pos_scan":       [0.0, -45.0, 0.0, 30.0, 0.0, -75.0, 0.0],
        "arm_pos_placing":    [0.0, -30.0, 0.0, 60.0, 0.0, -90.0, 0.0],
        "arm_pos_drop":       [0.0, -20.0, 0.0, 70.0, 0.0, -90.0, 0.0],
        # --- Label dictionaries ---
        # Designated trash category (announced on Setup Days; rulebook §12).
        "designated_trash_labels": ["paper cup"],
        "cutlery_labels": ["fork", "knife", "spoon"],
        # 'bowl' is tableware for table items only; the breakfast bowl is
        # disambiguated by source location in BtNode_BuildBreakfastQueue.
        "tableware_labels": ["plate", "mug", "cup", "bowl"],
        # --- Vision prompts (open-vocabulary) ---
        "table_scan_prompt": (
            "fork . knife . spoon . plate . mug . cup . bowl . "
            "paper cup . napkin . bottle . can . wrapper . "
            "snack bag . pringles . chocolate . ketchup . oats . tuna"
        ),
        # --- Numerics ---
        "n_layers": 2,
        "max_runtime_sec": 390.0,  # 7:00 - 30 s safety margin
        "nav_retry_limit": 3,
        "scan_retry_limit": 2,
        "grasp_retry_limit": 2,
    }


def _load_constants():
    defaults = _default_constants()
    path = Path(__file__).with_name("constants.json")
    try:
        with path.open("r", encoding="utf-8") as file:
            loaded = json.load(file)
    except FileNotFoundError:
        return defaults
    defaults.update(loaded)
    return defaults


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


constants = _load_constants()

# --- Materialize PoseStamped / PointStamped ---
POSE_KITCHEN_ENTRY  = _pose_reader(constants["pose_kitchen_entry"])
POSE_TABLE          = _pose_reader(constants["pose_table"])
POSE_WASH_STAGING   = _pose_reader(constants["pose_wash_staging"])
POSE_TRASH_BIN      = _pose_reader(constants["pose_trash_bin"])
POSE_CABINET        = _pose_reader(constants["pose_cabinet"])
POSE_KITCHEN_SHELF  = _pose_reader(constants["pose_kitchen_shelf"])

POINT_WASH_STAGING     = _point_reader(constants["point_wash_staging"])
POINT_TRASH_BIN        = _point_reader(constants["point_trash_bin"])
POINT_CABINET_DEFAULT  = _point_reader(constants["point_cabinet_default"])
POINT_KITCHEN_SURFACE  = _point_reader(constants["point_kitchen_surface"])
POINT_BREAKFAST_BOWL   = _point_reader(constants["point_breakfast_bowl"])
POINT_BREAKFAST_SPOON  = _point_reader(constants["point_breakfast_spoon"])
POINT_BREAKFAST_CEREAL = _point_reader(constants["point_breakfast_cereal"])
POINT_BREAKFAST_MILK   = _point_reader(constants["point_breakfast_milk"])
POINT_SHELF_LEFT       = _point_reader(constants["point_shelf_left"])
POINT_SHELF_RIGHT      = _point_reader(constants["point_shelf_right"])

# --- Arm joint configs (radians) ---
ARM_POS_NAVIGATING = [x / 180 * math.pi for x in constants["arm_pos_navigating"]]
ARM_POS_SCAN       = [x / 180 * math.pi for x in constants["arm_pos_scan"]]
ARM_POS_PLACING    = [x / 180 * math.pi for x in constants["arm_pos_placing"]]
ARM_POS_DROP       = [x / 180 * math.pi for x in constants["arm_pos_drop"]]

# --- Label sets ---
DESIGNATED_TRASH_LABELS = list(constants["designated_trash_labels"])
CUTLERY_LABELS          = list(constants["cutlery_labels"])
TABLEWARE_LABELS        = list(constants["tableware_labels"])
WASH_STAGING_LABELS     = CUTLERY_LABELS + TABLEWARE_LABELS

# --- Prompts ---
TABLE_SCAN_PROMPT        = str(constants["table_scan_prompt"])

# --- Numerics ---
N_LAYERS              = int(constants["n_layers"])
MAX_RUNTIME_SEC       = float(constants["max_runtime_sec"])
NAV_RETRY_LIMIT       = int(constants["nav_retry_limit"])
SCAN_RETRY_LIMIT      = int(constants["scan_retry_limit"])
GRASP_RETRY_LIMIT     = int(constants["grasp_retry_limit"])

# --- Service / action names ---
ARM_SERVICE_NAME    = "arm_joint_service"
GRASP_ACTION_NAME   = "start_grasp"
PLACE_ACTION_NAME   = "place_action"
TARGET_FRAME        = "base_link"

# ============================================================
# Blackboard keys
# ============================================================

# Poses
KEY_POSE_KITCHEN_ENTRY  = "pp_pose_kitchen_entry"
KEY_POSE_TABLE          = "pp_pose_table"
KEY_POSE_WASH_STAGING   = "pp_pose_wash_staging"
KEY_POSE_TRASH_BIN      = "pp_pose_trash_bin"
KEY_POSE_CABINET        = "pp_pose_cabinet"
KEY_POSE_KITCHEN_SHELF  = "pp_pose_kitchen_shelf"

# Points
KEY_POINT_WASH_STAGING     = "pp_point_wash_staging"
KEY_POINT_TRASH_BIN        = "pp_point_trash_bin"
KEY_POINT_CABINET_DEFAULT  = "pp_point_cabinet_default"
KEY_POINT_KITCHEN_SURFACE  = "pp_point_kitchen_surface"
KEY_POINT_BREAKFAST_BOWL   = "pp_point_breakfast_bowl"
KEY_POINT_BREAKFAST_SPOON  = "pp_point_breakfast_spoon"
KEY_POINT_BREAKFAST_CEREAL = "pp_point_breakfast_cereal"
KEY_POINT_BREAKFAST_MILK   = "pp_point_breakfast_milk"
KEY_POINT_SHELF_LEFT       = "pp_point_shelf_left"
KEY_POINT_SHELF_RIGHT      = "pp_point_shelf_right"

# Arm poses
KEY_ARM_NAVIGATING = "pp_arm_navigating"
KEY_ARM_SCAN       = "pp_arm_scan"
KEY_ARM_PLACING    = "pp_arm_placing"
KEY_ARM_DROP       = "pp_arm_drop"

# Misc constants
KEY_TARGET_FRAME = "pp_target_frame"
KEY_MAX_RUNTIME  = "pp_max_runtime_sec"

# Inventory / queue
KEY_INVENTORY_TABLE  = "pp_inventory_table"
KEY_WORK_QUEUE       = "pp_work_queue"
KEY_BREAKFAST_QUEUE  = "pp_breakfast_queue"

# Active item
KEY_ACTIVE_OBJECT_CLASS = "pp_active_object_class"
KEY_ACTIVE_PROMPT       = "pp_active_prompt"
KEY_ACTIVE_SOURCE_POSE  = "pp_active_source_pose"
KEY_ACTIVE_TARGET_POSE  = "pp_active_target_pose"
KEY_ACTIVE_TARGET_POINT = "pp_active_target_point"

# Per-pick scratch (FindObjTable + GraspWithPose + Place)
KEY_TABLE_IMG          = "pp_table_img"
KEY_OBJ_SEG            = "pp_obj_seg"
KEY_VISION_RESULT      = "pp_vision_result"
KEY_GRASP_ANNOUNCEMENT = "pp_grasp_announcement"
KEY_GRASP_POSE         = "pp_grasp_pose"
KEY_OBJECT_LABEL      = "pp_object_label"
KEY_ENV_POINTS         = "pp_env_points"
KEY_PLACE_REASON       = "pp_place_reason"

# Bookkeeping
KEY_SCORE_TRACE     = "pp_score_trace"
KEY_PHASE_DEADLINE  = "pp_phase_deadline"
KEY_SUMMARY_MESSAGE = "pp_summary_message"
KEY_DOOR_STATUS     = "pp_door_status"
