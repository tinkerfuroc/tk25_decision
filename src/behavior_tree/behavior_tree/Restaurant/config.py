from __future__ import annotations

"""Restaurant configuration.

Central place for:
- constants.json loading
- precomputed shared poses/arm joints
- blackboard key declarations for both standard and simplified strategies
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
    """Load Restaurant task constants from module-local JSON file."""
    constants_path = Path(__file__).with_name("constants.json")
    with constants_path.open("r", encoding="utf-8") as file:
        return json.load(file)


constants = _load_constants()

_stamp = rclpy.time.Time().to_msg() if rclpy is not None else None
POSE_KITCHEN_BAR = PoseStamped(
    header=Header(stamp=_stamp, frame_id="map"),
    pose=Pose(
        position=Point(
            x=constants["pose_kitchen_bar"]["point"]["x"],
            y=constants["pose_kitchen_bar"]["point"]["y"],
            z=0.0,
        ),
        orientation=Quaternion(
            x=constants["pose_kitchen_bar"]["orientation"]["x"],
            y=constants["pose_kitchen_bar"]["orientation"]["y"],
            z=constants["pose_kitchen_bar"]["orientation"]["z"],
            w=constants["pose_kitchen_bar"]["orientation"]["w"],
        ),
    ),
)

ARM_POS_NAVIGATING = [x / 180 * math.pi for x in constants["arm_pos_navigating"]]
ARM_POS_SERVING = [x / 180 * math.pi for x in constants["arm_pos_serving"]]

# Shared restaurant keys used by both strategies.
KEY_KITCHEN_BAR_POSE = "kitchen_bar_pose"
KEY_CUSTOMER_LOCATION = "customer_location"
KEY_CUSTOMER_ORDER = "customer_order"
KEY_TRAY_LOCATION = "tray_location"
KEY_ARM_NAVIGATING = "arm_navigating"
KEY_ARM_SERVING = "arm_serving"
KEY_ORDER_CHECKLIST = "order_checklist"
KEY_PICKUP_VERIFIED = "pickup_verified"
KEY_ACTIVE_CUSTOMER_ID = "active_customer_id"

# Original restaurant queue keys (`restaurants.py`).
KEY_CUSTOMER_QUEUE = "customer_queue"

# Simplified strategy batch keys (`restaurant_simplified.py`).
KEY_CUSTOMER_BATCH = "customer_batch"
KEY_CURRENT_BATCH_INDEX = "current_batch_index"
KEY_BATCH_SIZE_LIMIT = "batch_size_limit"
KEY_BATCH_ORDERS_SUMMARY = "batch_orders_summary"
KEY_BATCH_DETECTION_SUMMARY = "batch_detection_summary"

BATCH_SIZE_LIMIT = 3
