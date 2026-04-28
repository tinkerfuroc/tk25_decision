from __future__ import annotations

"""DoingLaundry configuration and shared blackboard keys.

Hardware-aware design (RoboCup@Home 2026 §5.4):
  * Tinker has a single xArm — bimanual operations skipped:
      - basket transport (DO_BASKET_TRANSPORT=False)  → forfeits +300 bonus
  * Robot may be unable to:
      - open the washing-machine door (DO_REQUEST_WASHER_HELP=True; ask operator)
      - grasp exactly one piece (avoid -100 multi-grab; request handover)
      - fold a T-shirt (try fold_action, fall back to operator-assisted fold)
"""

import json
import math
from pathlib import Path

try:
    from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Quaternion
    from std_msgs.msg import Header
except ModuleNotFoundError:  # pragma: no cover
    class Point:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x, self.y, self.z = x, y, z

    class Quaternion:
        def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
            self.x, self.y, self.z, self.w = x, y, z, w

    class Header:
        def __init__(self, stamp=None, frame_id=""):
            self.stamp, self.frame_id = stamp, frame_id

    class Pose:
        def __init__(self, position=None, orientation=None):
            self.position = position or Point()
            self.orientation = orientation or Quaternion()

    class PoseStamped:
        def __init__(self, header=None, pose=None):
            self.header = header or Header()
            self.pose = pose or Pose()

    class PointStamped:
        def __init__(self, header=None, point=None):
            self.header = header or Header()
            self.point = point or Point()


def _default_constants():
    return {
        "pose_arena_entry": {
            "point": {"x": 0.5, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "pose_laundry_area": {
            "point": {"x": 1.5, "y": -0.8, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "pose_washing_machine": {
            "point": {"x": 1.6, "y": -1.2, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "pose_basket": {
            "point": {"x": 1.4, "y": -0.6, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "pose_folding_table": {
            "point": {"x": 1.0, "y": 0.4, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },

        "point_table_fold_zone":   {"x": 1.00, "y": 0.40, "z": 0.75},
        "point_table_stack_zone":  {"x": 1.15, "y": 0.40, "z": 0.75},
        "point_basket_top":        {"x": 1.40, "y": -0.60, "z": 0.55},
        "point_washer_drum":       {"x": 1.60, "y": -1.20, "z": 0.55},

        "arm_pos_navigating":   [-87.0, -58.8, -2.4, 8.0, 12.3, -68.5, -8.2],
        "arm_pos_scan":         [0.0, -45.0, 0.0, 30.0, 0.0, -75.0, 0.0],
        "arm_pos_pick_basket":  [0.0, -25.0, 0.0, 65.0, 0.0, -85.0, 0.0],
        "arm_pos_pick_washer":  [-15.0, -10.0, 0.0, 80.0, 0.0, -85.0, 0.0],
        "arm_pos_placing":      [0.0, -30.0, 0.0, 60.0, 0.0, -90.0, 0.0],
        "arm_pos_fold_start":   [0.0, -40.0, 0.0, 55.0, 0.0, -80.0, 0.0],

        "fold_cycles": 3,

        "max_runtime_sec": 390.0,
        "nav_retry_limit": 3,

        # Hardware-aware flags
        "do_basket_transport": False,
        "do_request_washer_help": True,
        "do_pick_from_washer": True,
        "do_pick_from_basket": False,
        "do_request_single_piece_via_handover": True,
        "do_fold_picked_pieces": True,

        # Caps
        "max_extra_folds": 2,
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


def _pose_reader(d):
    return PoseStamped(
        header=_make_header(),
        pose=Pose(
            position=Point(x=d["point"]["x"], y=d["point"]["y"], z=d["point"].get("z", 0.0)),
            orientation=Quaternion(
                x=d["orientation"]["x"], y=d["orientation"]["y"],
                z=d["orientation"]["z"], w=d["orientation"]["w"],
            ),
        ),
    )


def _point_reader(d):
    return PointStamped(
        header=_make_header(),
        point=Point(x=d["x"], y=d["y"], z=d.get("z", 0.0)),
    )


constants = _load_constants()

POSE_ARENA_ENTRY     = _pose_reader(constants["pose_arena_entry"])
POSE_LAUNDRY_AREA    = _pose_reader(constants["pose_laundry_area"])
POSE_WASHING_MACHINE = _pose_reader(constants["pose_washing_machine"])
POSE_BASKET          = _pose_reader(constants["pose_basket"])
POSE_FOLDING_TABLE   = _pose_reader(constants["pose_folding_table"])

POINT_TABLE_FOLD_ZONE  = _point_reader(constants["point_table_fold_zone"])
POINT_TABLE_STACK_ZONE = _point_reader(constants["point_table_stack_zone"])
POINT_BASKET_TOP       = _point_reader(constants["point_basket_top"])
POINT_WASHER_DRUM      = _point_reader(constants["point_washer_drum"])

ARM_POS_NAVIGATING  = [x / 180 * math.pi for x in constants["arm_pos_navigating"]]
ARM_POS_SCAN        = [x / 180 * math.pi for x in constants["arm_pos_scan"]]
ARM_POS_PICK_BASKET = [x / 180 * math.pi for x in constants["arm_pos_pick_basket"]]
ARM_POS_PICK_WASHER = [x / 180 * math.pi for x in constants["arm_pos_pick_washer"]]
ARM_POS_PLACING     = [x / 180 * math.pi for x in constants["arm_pos_placing"]]
ARM_POS_FOLD_START  = [x / 180 * math.pi for x in constants["arm_pos_fold_start"]]

FOLD_CYCLES         = int(constants["fold_cycles"])
MAX_RUNTIME_SEC     = float(constants["max_runtime_sec"])
NAV_RETRY_LIMIT     = int(constants["nav_retry_limit"])

DO_BASKET_TRANSPORT                  = bool(constants["do_basket_transport"])
DO_REQUEST_WASHER_HELP               = bool(constants["do_request_washer_help"])
DO_PICK_FROM_WASHER                  = bool(constants["do_pick_from_washer"])
DO_PICK_FROM_BASKET                  = bool(constants["do_pick_from_basket"])
DO_REQUEST_SINGLE_PIECE_VIA_HANDOVER = bool(constants["do_request_single_piece_via_handover"])
DO_FOLD_PICKED_PIECES                = bool(constants["do_fold_picked_pieces"])

MAX_EXTRA_FOLDS = int(constants["max_extra_folds"])

ARM_SERVICE_NAME  = "arm_joint_service"
TARGET_FRAME      = "base_link"

# ============================================================
# Blackboard keys
# ============================================================

KEY_POSE_ARENA_ENTRY     = "dl_pose_arena_entry"
KEY_POSE_LAUNDRY_AREA    = "dl_pose_laundry_area"
KEY_POSE_WASHING_MACHINE = "dl_pose_washing_machine"
KEY_POSE_BASKET          = "dl_pose_basket"
KEY_POSE_FOLDING_TABLE   = "dl_pose_folding_table"

KEY_POINT_TABLE_FOLD_ZONE  = "dl_point_table_fold_zone"
KEY_POINT_TABLE_STACK_ZONE = "dl_point_table_stack_zone"
KEY_POINT_BASKET_TOP       = "dl_point_basket_top"
KEY_POINT_WASHER_DRUM      = "dl_point_washer_drum"

KEY_ARM_NAVIGATING  = "dl_arm_navigating"
KEY_ARM_SCAN        = "dl_arm_scan"
KEY_ARM_PICK_BASKET = "dl_arm_pick_basket"
KEY_ARM_PICK_WASHER = "dl_arm_pick_washer"
KEY_ARM_PLACING     = "dl_arm_placing"
KEY_ARM_FOLD_START  = "dl_arm_fold_start"

KEY_TARGET_FRAME = "dl_target_frame"
KEY_MAX_RUNTIME  = "dl_max_runtime_sec"

KEY_DOOR_STATUS     = "dl_door_status"
KEY_SCORE_TRACE     = "dl_score_trace"
KEY_PHASE_DEADLINE  = "dl_phase_deadline"
KEY_SUMMARY_MESSAGE = "dl_summary_message"
KEY_FOLD_COUNT      = "dl_fold_count"
KEY_STACK_COUNT     = "dl_stack_count"
