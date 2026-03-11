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
    BtNode_AddDetectedCustomerToBatch,
    BtNode_AdvanceBatchIndex,
    BtNode_BuildBatchOrdersSummary,
    BtNode_CurrentBatchCustomerHasOrder,
    BtNode_CurrentBatchCustomerNeedsService,
    BtNode_HasUnservedBatchCustomers,
    BtNode_InitCustomerBatch,
    BtNode_MarkBatchCustomerServed,
    BtNode_ResetBatchIndex,
    BtNode_SelectBatchCustomerByIndex,
    BtNode_SetActiveBatchCustomerStatus,
    BtNode_StoreOrderForActiveBatchCustomer,
)

BATCH_SIZE_LIMIT = 3
MAX_TASK_RUNTIME_SEC = 14 * 60.0


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
KEY_ORDER_CHECKLIST = "order_checklist"
KEY_PICKUP_VERIFIED = "pickup_verified"

KEY_CUSTOMER_BATCH = "customer_batch"
KEY_CURRENT_BATCH_INDEX = "current_batch_index"
KEY_BATCH_SIZE_LIMIT = "batch_size_limit"
KEY_BATCH_ORDERS_SUMMARY = "batch_orders_summary"
KEY_BATCH_DETECTION_SUMMARY = "batch_detection_summary"
KEY_ACTIVE_CUSTOMER_ID = "active_customer_id"


class BtNode_BuildBatchDetectionSummary(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, *, batch_key: str, summary_key: str):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="batch",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", batch_key),
        )
        self.blackboard.register_key(
            key="summary",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", summary_key),
        )

    def update(self) -> py_trees.common.Status:
        batch = list(self.blackboard.batch or [])
        if not batch:
            self.blackboard.summary = "I did not detect any callers."
            return py_trees.common.Status.SUCCESS
        ids = ", ".join([str(item["id"]) for item in batch])
        count = len(batch)
        self.blackboard.summary = f"I detected {count} callers: {ids}."
        return py_trees.common.Status.SUCCESS


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


def _createDetectAndAddOne():
    root = py_trees.composites.Sequence(name="Detect and add one caller", memory=True)
    detect_selector = py_trees.composites.Selector(name="Detect caller", memory=False)
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
            timeout=20.0,
        )
    )
    root.add_child(detect_selector)
    root.add_child(
        BtNode_AddDetectedCustomerToBatch(
            name="Add detected caller to batch",
            batch_key=KEY_CUSTOMER_BATCH,
            source_pose_key=KEY_CUSTOMER_LOCATION,
            batch_size_limit_key=KEY_BATCH_SIZE_LIMIT,
        )
    )
    return root


def createPhaseDetectCustomers():
    root = py_trees.composites.Sequence(name="Phase A: detect callers", memory=True)
    root.add_child(
        BtNode_InitCustomerBatch(
            name="Initialize customer batch",
            batch_key=KEY_CUSTOMER_BATCH,
            batch_size_limit_key=KEY_BATCH_SIZE_LIMIT,
            current_index_key=KEY_CURRENT_BATCH_INDEX,
            default_limit=BATCH_SIZE_LIMIT,
            summary_key=KEY_BATCH_ORDERS_SUMMARY,
        )
    )

    detect_loop = py_trees.composites.Selector(name="Collect callers until stop", memory=False)
    detect_loop.add_child(
        py_trees.decorators.Repeat(
            name="Collect caller attempts",
            child=_createDetectAndAddOne(),
            num_success=BATCH_SIZE_LIMIT,
        )
    )
    detect_loop.add_child(py_trees.behaviours.Success(name="Stop caller collection"))
    root.add_child(detect_loop)

    root.add_child(
        BtNode_BuildBatchDetectionSummary(
            name="Build detection summary",
            batch_key=KEY_CUSTOMER_BATCH,
            summary_key=KEY_BATCH_DETECTION_SUMMARY,
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Announce detected callers",
            bb_source=KEY_BATCH_DETECTION_SUMMARY,
            message=None,
        )
    )
    return root


def _createTakeAndConfirmOrder():
    root = py_trees.composites.Sequence(name="Take and confirm order", memory=True)
    root.add_child(
        BtNode_Announce(
            name="Order prompt",
            bb_source=None,
            message="Hi. What would you like to order?",
        )
    )
    order_loop = py_trees.composites.Sequence(name="Order loop", memory=True)
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
            name="Retry order intake",
            child=order_loop,
            num_failures=3,
        )
    )
    return root


def _createCollectOneCustomerOrder():
    root = py_trees.composites.Sequence(name="Collect one customer order", memory=True)
    root.add_child(
        BtNode_SelectBatchCustomerByIndex(
            name="Select customer by index",
            batch_key=KEY_CUSTOMER_BATCH,
            current_index_key=KEY_CURRENT_BATCH_INDEX,
            customer_pose_key=KEY_CUSTOMER_LOCATION,
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
            customer_order_key=KEY_CUSTOMER_ORDER,
        )
    )

    per_customer = py_trees.composites.Selector(name="Order intake or mark failed", memory=False)
    success_path = py_trees.composites.Sequence(name="Successful order intake", memory=True)
    success_path.add_child(
        py_trees.decorators.Retry(
            name="Retry goto customer",
            child=BtNode_GotoAction(name="Go to customer table", key=KEY_CUSTOMER_LOCATION),
            num_failures=3,
        )
    )
    success_path.add_child(_createTakeAndConfirmOrder())
    success_path.add_child(
        BtNode_StoreOrderForActiveBatchCustomer(
            name="Store order for selected customer",
            batch_key=KEY_CUSTOMER_BATCH,
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
            customer_order_key=KEY_CUSTOMER_ORDER,
        )
    )
    failure_path = py_trees.composites.Sequence(name="Mark order failed", memory=True)
    failure_path.add_child(
        BtNode_Announce(
            name="Order collection failed",
            bb_source=None,
            message="I could not take this order. I will continue.",
        )
    )
    failure_path.add_child(
        BtNode_SetActiveBatchCustomerStatus(
            name="Set order failed status",
            batch_key=KEY_CUSTOMER_BATCH,
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
            status="order_failed",
        )
    )
    per_customer.add_child(success_path)
    per_customer.add_child(failure_path)
    root.add_child(per_customer)
    root.add_child(
        BtNode_AdvanceBatchIndex(
            name="Advance batch index after order attempt",
            current_index_key=KEY_CURRENT_BATCH_INDEX,
        )
    )
    return root


def createPhaseCollectOrders():
    root = py_trees.composites.Sequence(name="Phase B: collect all orders", memory=True)
    root.add_child(
        BtNode_ResetBatchIndex(
            name="Reset batch index before order phase",
            current_index_key=KEY_CURRENT_BATCH_INDEX,
        )
    )
    collect_loop = py_trees.composites.Selector(name="Collect orders loop", memory=False)
    collect_loop.add_child(
        py_trees.decorators.Repeat(
            name="Collect all customer orders",
            child=_createCollectOneCustomerOrder(),
            num_success=BATCH_SIZE_LIMIT,
        )
    )
    collect_loop.add_child(py_trees.behaviours.Success(name="Done collecting orders"))
    root.add_child(collect_loop)
    return root


def _createPlaceOneOrder():
    root = py_trees.composites.Sequence(name="Place one order with barman", memory=True)
    root.add_child(
        BtNode_SelectBatchCustomerByIndex(
            name="Select order candidate",
            batch_key=KEY_CUSTOMER_BATCH,
            current_index_key=KEY_CURRENT_BATCH_INDEX,
            customer_pose_key=KEY_CUSTOMER_LOCATION,
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
            customer_order_key=KEY_CUSTOMER_ORDER,
        )
    )
    maybe_place = py_trees.composites.Selector(name="Place or skip", memory=False)
    place = py_trees.composites.Sequence(name="Place selected order", memory=True)
    place.add_child(
        BtNode_CurrentBatchCustomerHasOrder(
            name="Check selected customer has order",
            batch_key=KEY_CUSTOMER_BATCH,
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
        )
    )
    place.add_child(
        BtNode_CommunicateWithBarman(
            name="Communicate selected order",
            bb_order_key=KEY_CUSTOMER_ORDER,
        )
    )
    maybe_place.add_child(place)
    maybe_place.add_child(py_trees.behaviours.Success(name="Skip customer with no order"))
    root.add_child(maybe_place)
    root.add_child(
        BtNode_AdvanceBatchIndex(
            name="Advance batch index after place attempt",
            current_index_key=KEY_CURRENT_BATCH_INDEX,
        )
    )
    return root


def createPhasePlaceAllOrders():
    root = py_trees.composites.Sequence(name="Phase C: place all orders", memory=True)
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry return to known start position",
            child=BtNode_GotoAction(
                name="Return to known bar/start position",
                key=KEY_KITCHEN_BAR_POSE,
            ),
            num_failures=3,
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Call for barman at known position",
            bb_source=None,
            message="Barman, please come here. I have customer orders.",
        )
    )
    summary_selector = py_trees.composites.Selector(name="Order summary handling", memory=False)
    has_orders = py_trees.composites.Sequence(name="Has orders", memory=True)
    has_orders.add_child(
        BtNode_BuildBatchOrdersSummary(
            name="Build batch order summary",
            batch_key=KEY_CUSTOMER_BATCH,
            summary_key=KEY_BATCH_ORDERS_SUMMARY,
        )
    )
    has_orders.add_child(
        BtNode_Announce(
            name="Announce collected orders",
            bb_source=KEY_BATCH_ORDERS_SUMMARY,
            message="Collected orders:",
        )
    )
    summary_selector.add_child(has_orders)
    summary_selector.add_child(
        BtNode_Announce(
            name="No valid orders announcement",
            bb_source=None,
            message="No valid orders were collected.",
        )
    )
    root.add_child(summary_selector)
    root.add_child(
        BtNode_ResetBatchIndex(
            name="Reset batch index before placing",
            current_index_key=KEY_CURRENT_BATCH_INDEX,
        )
    )
    place_loop = py_trees.composites.Selector(name="Place all orders loop", memory=False)
    place_loop.add_child(
        py_trees.decorators.Repeat(
            name="Place each collected order",
            child=_createPlaceOneOrder(),
            num_success=BATCH_SIZE_LIMIT,
        )
    )
    place_loop.add_child(py_trees.behaviours.Success(name="Done placing orders"))
    root.add_child(place_loop)
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
            message="I'll use a tray for this order.",
        )
    )
    root.add_child(tray_sequence)
    root.add_child(py_trees.behaviours.Success(name="Skip tray transport"))
    return root


def _createServeOneCustomer():
    root = py_trees.composites.Sequence(name="Serve one customer", memory=True)
    root.add_child(
        BtNode_SelectBatchCustomerByIndex(
            name="Select customer to serve",
            batch_key=KEY_CUSTOMER_BATCH,
            current_index_key=KEY_CURRENT_BATCH_INDEX,
            customer_pose_key=KEY_CUSTOMER_LOCATION,
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
            customer_order_key=KEY_CUSTOMER_ORDER,
        )
    )

    maybe_serve = py_trees.composites.Selector(name="Serve or skip", memory=False)
    serve_sequence = py_trees.composites.Sequence(name="Serve selected customer", memory=True)
    serve_sequence.add_child(
        BtNode_CurrentBatchCustomerNeedsService(
            name="Check selected customer needs service",
            batch_key=KEY_CUSTOMER_BATCH,
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
        )
    )
    serve_sequence.add_child(
        BtNode_Announce(
            name="Pickup verification hook",
            bb_source=None,
            message="Picking up this order now.",
        )
    )
    serve_sequence.add_child(createOptionalTrayTransport())
    serve_sequence.add_child(
        py_trees.decorators.Retry(
            name="Retry return to customer",
            child=BtNode_GotoAction(name="Return to customer table", key=KEY_CUSTOMER_LOCATION),
            num_failures=3,
        )
    )
    serve_sequence.add_child(
        BtNode_ServeOrder(
            name="Serve selected order",
            bb_order_key=KEY_CUSTOMER_ORDER,
        )
    )
    serve_sequence.add_child(
        BtNode_MarkBatchCustomerServed(
            name="Mark selected customer served",
            batch_key=KEY_CUSTOMER_BATCH,
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
        )
    )
    maybe_serve.add_child(serve_sequence)
    maybe_serve.add_child(py_trees.behaviours.Success(name="Skip non-service customer"))
    root.add_child(maybe_serve)
    root.add_child(
        BtNode_AdvanceBatchIndex(
            name="Advance batch index after serve attempt",
            current_index_key=KEY_CURRENT_BATCH_INDEX,
        )
    )
    return root


def createPhaseServeOrders():
    root = py_trees.composites.Selector(name="Phase D: serve orders", memory=False)

    serve_phase = py_trees.composites.Sequence(name="Serve all batched orders", memory=True)
    serve_phase.add_child(
        BtNode_HasUnservedBatchCustomers(
            name="Check unserved customers exist",
            batch_key=KEY_CUSTOMER_BATCH,
        )
    )
    serve_phase.add_child(
        BtNode_ResetBatchIndex(
            name="Reset batch index before serving",
            current_index_key=KEY_CURRENT_BATCH_INDEX,
        )
    )
    serve_loop = py_trees.composites.Selector(name="Serve loop", memory=False)
    serve_loop.add_child(
        py_trees.decorators.Repeat(
            name="Serve each batched customer",
            child=_createServeOneCustomer(),
            num_success=BATCH_SIZE_LIMIT,
        )
    )
    serve_loop.add_child(py_trees.behaviours.Success(name="Done serving"))
    serve_phase.add_child(serve_loop)
    root.add_child(serve_phase)
    root.add_child(
        BtNode_Announce(
            name="No pending deliveries",
            bb_source=None,
            message="No pending deliveries.",
        )
    )
    return root


def createRestaurantSimplifiedTask():
    phases = py_trees.composites.Sequence(name="Restaurant Simplified Phases", memory=True)
    phases.add_child(createConstantWriter())
    phases.add_child(
        py_trees.decorators.Retry(
            name="Retry arm setup",
            child=BtNode_MoveArmSingle(
                name="Move arm to navigation pose",
                service_name="arm_joint_service",
                arm_pose_bb_key=KEY_ARM_NAVIGATING,
                add_octomap=False,
            ),
            num_failures=3,
        )
    )
    phases.add_child(
        BtNode_Announce(
            name="Simplified start announcement",
            bb_source=None,
            message="Starting batch restaurant mode.",
        )
    )
    phases.add_child(createPhaseDetectCustomers())
    phases.add_child(createPhaseCollectOrders())
    phases.add_child(createPhasePlaceAllOrders())
    phases.add_child(createPhaseServeOrders())
    phases.add_child(
        BtNode_Announce(
            name="Simplified completion announcement",
            bb_source=None,
            message="Batch restaurant service complete.",
        )
    )

    root = py_trees.composites.Selector(name="Simplified with timeout guard", memory=False)
    root.add_child(
        py_trees.decorators.Timeout(
            name="Task timeout guard",
            child=phases,
            duration=MAX_TASK_RUNTIME_SEC,
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Timeout completion announcement",
            bb_source=None,
            message="Time limit reached. Ending with completed actions.",
        )
    )
    return root


def restaurant_simplified():
    run_tree(
        createRestaurantSimplifiedTask,
        period_ms=500.0,
        title="Restaurant Simplified",
    )


if __name__ == "__main__":
    restaurant_simplified()
