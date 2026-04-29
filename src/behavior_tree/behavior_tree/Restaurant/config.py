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

def _load_constants():
    """Load Restaurant task constants from module-local JSON file."""
    constants_path = Path(__file__).with_name("constants.json")
    with constants_path.open("r", encoding="utf-8") as file:
        return json.load(file)


constants = _load_constants()

ARM_POS_NAVIGATING = [x / 180 * math.pi for x in constants["arm_pos_navigating"]]
ARM_POS_SERVING = [x / 180 * math.pi for x in constants["arm_pos_serving"]]

# Shared restaurant keys used by both strategies.
KEY_KITCHEN_BAR_POSE = "kitchen_bar_pose"
KEY_CUSTOMER_LOCATION = "customer_location"
KEY_CUSTOMER_APPROACH_POSE = "customer_approach_pose"
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

# Waving-detection / per-item-delivery keys used by `restaurants.py` (ported from
# `restaurants_fake.py`).
KEY_WAVING_PERSON_POSES = "waving_person_poses"
KEY_WAVING_PERSON_PICTURES = "waving_person_pictures"
KEY_WAVING_CLOSEST_PERSON = "waving_closest_person"
KEY_ACTIVE_CUSTOMER_PICTURE = "active_customer_picture"
KEY_CURRENT_ITEMS = "current_items"
KEY_CURRENT_ITEM = "current_item"
KEY_CURRENT_ITEM_SUMMARY = "current_item_summary"

# Batched pipeline keys (production `restaurants.py`). `KEY_ORDER_LIST` mirrors the
# demo tree's schema (`list[{id, pose, picture_path, items, delivered_items}]`).
KEY_ORDER_LIST = "order_list"
KEY_BARMAN_TEXT = "barman_text"

# Radius (meters) for waving-person detection. Raise for larger venues.
DETECT_WAVING_THRESHOLD_M = 8.0
