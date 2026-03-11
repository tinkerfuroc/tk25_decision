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

from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_GetConfirmation
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import BtNode_MoveArmSingle
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.runtime import run_tree

from .custumNodes import (
    BtNode_CommunicateWithBarman,
    BtNode_ConfirmOrder,
    BtNode_DetectCallingCustomer,
    BtNode_DetectTray,
    BtNode_ScanForCallingCustomer,
    BtNode_ServeOrder,
    BtNode_TakeOrder,
)
from .state_nodes import (
    BtNode_AppendCustomerCandidate,
    BtNode_CloseActiveCustomer,
    BtNode_InitOrderChecklist,
    BtNode_MarkPickupVerified,
    BtNode_RequirePickupVerified,
    BtNode_SelectNextCustomer,
    BtNode_UpdateChecklistFlag,
)


def _load_constants():
    constants_path = Path(__file__).with_name("constants.json")
    with constants_path.open("r", encoding="utf-8") as file:
        return json.load(file)


constants = _load_constants()

_stamp = rclpy.time.Time().to_msg() if rclpy is not None else None
pose_kitchen_bar = PoseStamped(
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

KEY_KITCHEN_BAR_POSE = "kitchen_bar_pose"
KEY_CUSTOMER_LOCATION = "customer_location"
KEY_CUSTOMER_ORDER = "customer_order"
KEY_TRAY_LOCATION = "tray_location"
KEY_ARM_NAVIGATING = "arm_navigating"
KEY_ARM_SERVING = "arm_serving"
KEY_CUSTOMER_QUEUE = "customer_queue"
KEY_ACTIVE_CUSTOMER_ID = "active_customer_id"
KEY_ORDER_CHECKLIST = "order_checklist"
KEY_PICKUP_VERIFIED = "pickup_verified"


def createConstantWriter():
    root = py_trees.composites.Parallel(
        name="Write constants to blackboard",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Write kitchen bar location",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_KITCHEN_BAR_POSE,
            object=pose_kitchen_bar,
        )
    )
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Write arm navigating pose",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_ARM_NAVIGATING,
            object=ARM_POS_NAVIGATING,
        )
    )
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Write arm serving pose",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_ARM_SERVING,
            object=ARM_POS_SERVING,
        )
    )
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Initialize caller queue",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_CUSTOMER_QUEUE,
            object=[],
        )
    )
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Initialize active caller id",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_ACTIVE_CUSTOMER_ID,
            object=None,
        )
    )
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Initialize pickup verification",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_PICKUP_VERIFIED,
            object=False,
        )
    )
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Initialize order checklist",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_ORDER_CHECKLIST,
            object={},
        )
    )
    return root


def createDetectAndArbitrateCustomers():
    root = py_trees.composites.Sequence(name="Detect and arbitrate callers", memory=True)
    detect_selector = py_trees.composites.Selector(name="Detect callers", memory=False)
    detect_selector.add_child(
        BtNode_DetectCallingCustomer(
            name="Direct detect calling customer",
            bb_dest_key=KEY_CUSTOMER_LOCATION,
            timeout=5.0,
        )
    )
    detect_selector.add_child(
        BtNode_ScanForCallingCustomer(
            name="Scan for calling customer",
            bb_dest_key=KEY_CUSTOMER_LOCATION,
            timeout=30.0,
        )
    )

    # Try to enqueue up to two detections to support simultaneous-caller policy.
    collect = py_trees.composites.Sequence(name="Collect caller candidates", memory=True)
    collect.add_child(detect_selector)
    collect.add_child(
        BtNode_AppendCustomerCandidate(
            name="Queue caller candidate 1",
            queue_key=KEY_CUSTOMER_QUEUE,
            source_pose_key=KEY_CUSTOMER_LOCATION,
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
            confidence=1.0,
        )
    )
    collect.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="Optional second caller detection",
            child=_create_optional_second_caller(),
        )
    )
    root.add_child(collect)
    root.add_child(
        BtNode_SelectNextCustomer(
            name="Select next caller",
            queue_key=KEY_CUSTOMER_QUEUE,
            selected_pose_key=KEY_CUSTOMER_LOCATION,
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
        )
    )
    return root


def createApproachCustomer():
    root = py_trees.composites.Selector(name="Approach selected customer", memory=False)
    success_path = py_trees.composites.Sequence(name="Reach customer", memory=True)
    success_path.add_child(
        py_trees.decorators.Retry(
            name="retry goto customer",
            child=BtNode_GotoAction(name="Go to customer table", key=KEY_CUSTOMER_LOCATION),
            num_failures=3,
        )
    )
    success_path.add_child(
        BtNode_UpdateChecklistFlag(
            name="Mark reached",
            checklist_key=KEY_ORDER_CHECKLIST,
            flag="reached",
            value=True,
        )
    )

    partial_score_path = py_trees.composites.Sequence(
        name="Partial score on unreachable caller", memory=True
    )
    partial_score_path.add_child(
        BtNode_Announce(
            name="Announce detected-but-unreachable caller",
            bb_source=None,
            message="I saw a caller but couldn't reach the table.",
        )
    )
    partial_score_path.add_child(
        BtNode_UpdateChecklistFlag(
            name="Mark partial score",
            checklist_key=KEY_ORDER_CHECKLIST,
            flag="partial_score",
            value=True,
        )
    )
    partial_score_path.add_child(
        BtNode_CloseActiveCustomer(
            name="Close unreachable active caller",
            queue_key=KEY_CUSTOMER_QUEUE,
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
        )
    )
    root.add_child(success_path)
    root.add_child(partial_score_path)
    return root


def _create_optional_second_caller():
    root = py_trees.composites.Sequence(name="Second caller sequence", memory=True)
    root.add_child(
        BtNode_DetectCallingCustomer(
            name="Detect possible second caller",
            bb_dest_key=KEY_CUSTOMER_LOCATION,
            timeout=2.0,
        )
    )
    root.add_child(
        BtNode_AppendCustomerCandidate(
            name="Queue caller candidate 2",
            queue_key=KEY_CUSTOMER_QUEUE,
            source_pose_key=KEY_CUSTOMER_LOCATION,
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
            confidence=0.9,
        )
    )
    return root


def createTakeAndConfirmOrder():
    root = py_trees.composites.Sequence(name="Take and confirm order", memory=True)
    root.add_child(
        BtNode_Announce(
            name="Order engagement prompt",
            bb_source=None,
            message="Hi. What would you like to order?",
        )
    )
    order_loop = py_trees.composites.Sequence(name="Order taking loop", memory=True)
    order_loop.add_child(
        BtNode_TakeOrder(
            name="Take order",
            bb_dest_key=KEY_CUSTOMER_ORDER,
            timeout=constants["order_confirmation_timeout"],
        )
    )
    order_loop.add_child(
        BtNode_ConfirmOrder(name="Confirm order", bb_order_key=KEY_CUSTOMER_ORDER)
    )
    order_loop.add_child(BtNode_GetConfirmation(name="Get confirmation", timeout=5.0))
    root.add_child(
        py_trees.decorators.Retry(
            name="retry order taking",
            child=order_loop,
            num_failures=3,
        )
    )
    root.add_child(
        BtNode_UpdateChecklistFlag(
            name="Mark order confirmed",
            checklist_key=KEY_ORDER_CHECKLIST,
            flag="order_confirmed",
            value=True,
        )
    )
    return root


def createPlaceOrderWithBarman():
    root = py_trees.composites.Sequence(name="Place order with barman", memory=True)
    root.add_child(
        py_trees.decorators.Retry(
            name="retry goto kitchen bar",
            child=BtNode_GotoAction(name="Go to kitchen bar", key=KEY_KITCHEN_BAR_POSE),
            num_failures=3,
        )
    )
    root.add_child(
        BtNode_CommunicateWithBarman(
            name="Communicate with barman",
            bb_order_key=KEY_CUSTOMER_ORDER,
        )
    )
    return root


def createPickupVerification():
    root = py_trees.composites.Sequence(name="Pickup verification", memory=True)
    root.add_child(
        BtNode_Announce(
            name="Pickup prompt",
            bb_source=None,
            message="I'll pick up your order now.",
        )
    )
    root.add_child(
        BtNode_MarkPickupVerified(
            name="Mark pickup verified",
            checklist_key=KEY_ORDER_CHECKLIST,
            pickup_verified_key=KEY_PICKUP_VERIFIED,
        )
    )
    return root


def createOptionalTrayTransport():
    root = py_trees.composites.Selector(name="Optional tray transport", memory=False)
    tray_sequence = py_trees.composites.Sequence(name="Use tray transport", memory=True)
    tray_sequence.add_child(
        BtNode_DetectTray(
            name="Detect available tray",
            bb_dest_key=KEY_TRAY_LOCATION,
        )
    )
    tray_sequence.add_child(
        BtNode_Announce(
            name="Announce tray usage",
            bb_source=None,
            message="I'll use a tray for your order.",
        )
    )
    direct_transport = BtNode_Announce(
        name="Direct transport",
        bb_source=None,
        message="I couldn't find a tray, so I'll carry it directly.",
    )
    root.add_child(tray_sequence)
    root.add_child(direct_transport)
    return root


def createDeliverOrder():
    root = py_trees.composites.Sequence(name="Deliver order", memory=True)
    root.add_child(
        BtNode_RequirePickupVerified(
            name="Require pickup verified",
            pickup_verified_key=KEY_PICKUP_VERIFIED,
        )
    )
    root.add_child(createOptionalTrayTransport())
    root.add_child(
        py_trees.decorators.Retry(
            name="retry return to customer",
            child=BtNode_GotoAction(name="Return to customer table", key=KEY_CUSTOMER_LOCATION),
            num_failures=3,
        )
    )
    root.add_child(
        BtNode_ServeOrder(
            name="Serve order",
            bb_order_key=KEY_CUSTOMER_ORDER,
        )
    )
    root.add_child(
        BtNode_UpdateChecklistFlag(
            name="Mark served",
            checklist_key=KEY_ORDER_CHECKLIST,
            flag="served",
            value=True,
        )
    )
    root.add_child(
        BtNode_CloseActiveCustomer(
            name="Close served active caller",
            queue_key=KEY_CUSTOMER_QUEUE,
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
        )
    )
    return root


def createSingleOrderCycle():
    root = py_trees.composites.Sequence(name="Single order cycle", memory=True)
    root.add_child(
        BtNode_InitOrderChecklist(
            name="Initialize per-order checklist",
            checklist_key=KEY_ORDER_CHECKLIST,
            pickup_verified_key=KEY_PICKUP_VERIFIED,
        )
    )
    root.add_child(createDetectAndArbitrateCustomers())
    root.add_child(createApproachCustomer())
    root.add_child(createTakeAndConfirmOrder())
    root.add_child(createPlaceOrderWithBarman())
    root.add_child(createPickupVerification())
    root.add_child(createDeliverOrder())
    return root


def createRestaurantTask():
    root = py_trees.composites.Sequence(name="Restaurant Task", memory=True)
    root.add_child(createConstantWriter())
    root.add_child(
        py_trees.decorators.Retry(
            name="retry arm setup",
            child=BtNode_MoveArmSingle(
                name="Move arm to navigation pose",
                service_name="arm_joint_service",
                arm_pose_bb_key=KEY_ARM_NAVIGATING,
                add_octomap=False,
            ),
            num_failures=3,
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Start announcement",
            bb_source=None,
            message="Restaurant service started. I'm ready for orders.",
        )
    )

    cycles = py_trees.composites.Selector(name="Order cycles", memory=False)
    first = py_trees.composites.Sequence(name="First order cycle", memory=True)
    first.add_child(createSingleOrderCycle())
    second_optional = py_trees.composites.Selector(name="Optional second cycle", memory=False)
    second_optional.add_child(createSingleOrderCycle())
    second_optional.add_child(
        BtNode_Announce(
            name="No second customer",
            bb_source=None,
            message="No more customers detected. Service complete.",
        )
    )
    first.add_child(second_optional)
    cycles.add_child(first)
    cycles.add_child(
        BtNode_Announce(
            name="No customers fallback",
            bb_source=None,
            message="No callers yet. Waiting.",
        )
    )
    root.add_child(cycles)
    root.add_child(
        BtNode_Announce(
            name="Task completion",
            bb_source=None,
            message="Restaurant service complete. Thank you.",
        )
    )
    return root


def restaurant():
    run_tree(createRestaurantTask, period_ms=500.0, title="Restaurant")


if __name__ == "__main__":
    restaurant()
