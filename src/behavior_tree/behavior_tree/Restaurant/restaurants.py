from __future__ import annotations

"""Standard Restaurant behavior tree composition.

This module defines the default order-cycle strategy. Shared constants and
blackboard keys are centralized in ``Restaurant/config.py``.

The tree combines two design vocabularies:

1. **Queue-arbitration upstream** (from PR #8) — ``BtNode_QueueWavingCandidates``
   feeds detected customers into a queue; ``BtNode_SelectNextQueuedCustomer``
   picks the oldest and exposes it as the active target. This handles the
   rulebook's "simultaneous callers" clause.

2. **Per-item delivery downstream** (ported from ``restaurants_fake.py`` /
   PR #7) — one order is split into items via ``BtNode_SplitOrderItems``;
   ``BtNode_PopNextItem`` drives a loop that makes one bar trip per item
   (the arm only grasps one object at a time), with a show-picture fallback
   on unreachable-customer failure.

Also folded in from PR #7: ``BtNode_ScanForWavingPerson`` as the primary
customer-detection path, ``BtNode_MaintainEyeContact`` ticks around order
intake and service to avoid the −80 eye-contact penalty, and
``BtNode_ShowImage`` for the partial-score fallback when a detected customer
cannot be reached.
"""

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_GetConfirmationAction
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import BtNode_GripperAction, BtNode_MoveArmSingle
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import (
    BtNode_MaintainEyeContact,
    BtNode_ScanForWavingPerson,
    BtNode_ShowImage,
)
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
from .config import (
    ARM_POS_NAVIGATING,
    ARM_POS_SERVING,
    DETECT_WAVING_THRESHOLD_M,
    KEY_ACTIVE_CUSTOMER_ID,
    KEY_ACTIVE_CUSTOMER_PICTURE,
    KEY_ARM_NAVIGATING,
    KEY_ARM_SERVING,
    KEY_CURRENT_ITEM,
    KEY_CURRENT_ITEM_SUMMARY,
    KEY_CURRENT_ITEMS,
    KEY_CUSTOMER_LOCATION,
    KEY_CUSTOMER_ORDER,
    KEY_CUSTOMER_QUEUE,
    KEY_KITCHEN_BAR_POSE,
    KEY_ORDER_CHECKLIST,
    KEY_PICKUP_VERIFIED,
    KEY_TRAY_LOCATION,
    KEY_WAVING_CLOSEST_PERSON,
    KEY_WAVING_PERSON_PICTURES,
    KEY_WAVING_PERSON_POSES,
    POSE_KITCHEN_BAR,
    constants,
)
from .state_nodes import (
    BtNode_AppendCustomerCandidate,
    BtNode_CloseActiveCustomer,
    BtNode_InitOrderChecklist,
    BtNode_MarkPickupVerified,
    BtNode_PopNextItem,
    BtNode_QueueWavingCandidates,
    BtNode_RequirePickupVerified,
    BtNode_SelectNextQueuedCustomer,
    BtNode_SplitOrderItems,
    BtNode_UpdateChecklistFlag,
)


def createConstantWriter():
    """Initialize static task constants and runtime state on blackboard."""
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
            object=POSE_KITCHEN_BAR,
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
            name="Initialize active caller picture",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_ACTIVE_CUSTOMER_PICTURE,
            object="",
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
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Initialize current items list",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_CURRENT_ITEMS,
            object=[],
        )
    )
    return root


def _create_waving_detection_pass():
    """Primary detection: tk26 waving-person service with per-person picture crops."""
    seq = py_trees.composites.Sequence(name="Waving detection pass", memory=True)
    seq.add_child(
        BtNode_ScanForWavingPerson(
            name="Scan for waving persons",
            bb_key_all_persons=KEY_WAVING_PERSON_POSES,
            bb_key_closest_person=KEY_WAVING_CLOSEST_PERSON,
            threshold_meters=DETECT_WAVING_THRESHOLD_M,
            target_frame="map",
            bb_key_pictures=KEY_WAVING_PERSON_PICTURES,
        )
    )
    seq.add_child(
        BtNode_QueueWavingCandidates(
            name="Queue waving candidates",
            all_poses_key=KEY_WAVING_PERSON_POSES,
            all_pictures_key=KEY_WAVING_PERSON_PICTURES,
            queue_key=KEY_CUSTOMER_QUEUE,
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
            max_candidates=2,
        )
    )
    return seq


def _create_legacy_detection_pass():
    """Fallback detection: legacy calling-customer YOLO path (no pictures)."""
    seq = py_trees.composites.Sequence(name="Legacy detection pass", memory=True)
    inner = py_trees.composites.Selector(name="Legacy detect selector", memory=False)
    inner.add_child(
        BtNode_DetectCallingCustomer(
            name="Direct detect calling customer",
            bb_dest_key=KEY_CUSTOMER_LOCATION,
            timeout=5.0,
        )
    )
    inner.add_child(
        BtNode_ScanForCallingCustomer(
            name="Scan for calling customer",
            bb_dest_key=KEY_CUSTOMER_LOCATION,
            timeout=30.0,
        )
    )
    seq.add_child(inner)
    seq.add_child(
        BtNode_AppendCustomerCandidate(
            name="Queue legacy caller candidate",
            queue_key=KEY_CUSTOMER_QUEUE,
            source_pose_key=KEY_CUSTOMER_LOCATION,
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
            confidence=0.7,
        )
    )
    return seq


def createDetectAndArbitrateCustomers():
    """Detect callers and select one active target using queue arbitration."""
    root = py_trees.composites.Sequence(name="Detect and arbitrate callers", memory=True)
    detect = py_trees.composites.Selector(name="Detection strategy", memory=False)
    detect.add_child(_create_waving_detection_pass())
    detect.add_child(_create_legacy_detection_pass())
    root.add_child(detect)
    root.add_child(
        BtNode_SelectNextQueuedCustomer(
            name="Select next caller",
            queue_key=KEY_CUSTOMER_QUEUE,
            selected_pose_key=KEY_CUSTOMER_LOCATION,
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
            picture_path_key=KEY_ACTIVE_CUSTOMER_PICTURE,
        )
    )
    return root


def createApproachCustomer():
    """Reach selected customer; fallback to show-picture partial-score path if unreachable."""
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
        BtNode_MaintainEyeContact(name="Eye-contact (on arrival)")
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
            message="I saw a caller but couldn't reach the table. Here is who I detected.",
        )
    )
    partial_score_path.add_child(
        BtNode_ShowImage(
            name="Show detected caller picture",
            bb_image_path_key=KEY_ACTIVE_CUSTOMER_PICTURE,
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


def createTakeAndConfirmOrder():
    """Run customer-facing order intake and confirmation loop."""
    root = py_trees.composites.Sequence(name="Take and confirm order", memory=True)
    root.add_child(BtNode_MaintainEyeContact(name="Eye-contact (order-taking)"))
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
    order_loop.add_child(BtNode_GetConfirmationAction(name="Get confirmation", timeout=5.0))
    root.add_child(
        py_trees.decorators.Retry(
            name="retry order taking",
            child=order_loop,
            num_failures=3,
        )
    )
    root.add_child(
        BtNode_SplitOrderItems(
            name="Split order into items",
            order_key=KEY_CUSTOMER_ORDER,
            items_key=KEY_CURRENT_ITEMS,
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
    """Navigate to bar and communicate one confirmed order to the barman."""
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


def createOptionalTrayTransport():
    """Optional tray branch with direct-carry fallback (single announcement, pre-loop)."""
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
        message="I couldn't find a tray, so I'll carry items individually.",
    )
    root.add_child(tray_sequence)
    root.add_child(direct_transport)
    return root


def createPickupVerification():
    """Receive one item from the barman at the kitchen bar (per-item barman round-trip).

    Called once per item by the delivery loop. Positions the arm, asks the
    barman to place the current item (``KEY_CURRENT_ITEM_SUMMARY``), gates on
    a spoken confirmation, then stows the arm for navigation.
    """
    root = py_trees.composites.Sequence(name="Pickup verification", memory=True)
    root.add_child(
        py_trees.decorators.Retry(
            name="retry goto kitchen bar",
            child=BtNode_GotoAction(name="Go to kitchen bar", key=KEY_KITCHEN_BAR_POSE),
            num_failures=3,
        )
    )
    root.add_child(
        BtNode_MoveArmSingle(
            name="Arm to serving pose (bar receive)",
            service_name="arm_joint_service",
            arm_pose_bb_key=KEY_ARM_SERVING,
            add_octomap=False,
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Ask barman to place item",
            bb_source=KEY_CURRENT_ITEM_SUMMARY,
            message="Please place the following item in my gripper:",
        )
    )
    root.add_child(BtNode_GripperAction(name="Open gripper (receive)", open_gripper=True))
    root.add_child(BtNode_GetConfirmationAction(name="Item placed?", timeout=60.0))
    root.add_child(BtNode_GripperAction(name="Close gripper (secure item)", open_gripper=False))
    root.add_child(
        BtNode_MoveArmSingle(
            name="Arm to navigating pose",
            service_name="arm_joint_service",
            arm_pose_bb_key=KEY_ARM_NAVIGATING,
            add_octomap=False,
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


def createDeliverOrder():
    """Deliver the current item to the active customer, with show-picture fallback."""
    root = py_trees.composites.Sequence(name="Deliver order", memory=True)
    root.add_child(
        BtNode_RequirePickupVerified(
            name="Require pickup verified",
            pickup_verified_key=KEY_PICKUP_VERIFIED,
        )
    )

    deliver_or_fallback = py_trees.composites.Selector(
        name="Deliver or fallback",
        memory=False,
    )

    normal = py_trees.composites.Sequence(name="Normal delivery", memory=True)
    normal.add_child(
        py_trees.decorators.Retry(
            name="retry return to customer",
            child=BtNode_GotoAction(name="Return to customer table", key=KEY_CUSTOMER_LOCATION),
            num_failures=3,
        )
    )
    normal.add_child(BtNode_MaintainEyeContact(name="Eye-contact (serving)"))
    normal.add_child(
        BtNode_MoveArmSingle(
            name="Arm to serving pose (at table)",
            service_name="arm_joint_service",
            arm_pose_bb_key=KEY_ARM_SERVING,
            add_octomap=False,
        )
    )
    normal.add_child(
        BtNode_Announce(
            name="Serve item",
            bb_source=KEY_CURRENT_ITEM_SUMMARY,
            message="Here is:",
        )
    )
    normal.add_child(
        BtNode_Announce(
            name="Countdown",
            bb_source=None,
            message="Three. Two. One.",
        )
    )
    normal.add_child(BtNode_GripperAction(name="Release item", open_gripper=True))
    normal.add_child(
        BtNode_MoveArmSingle(
            name="Arm to navigating pose",
            service_name="arm_joint_service",
            arm_pose_bb_key=KEY_ARM_NAVIGATING,
            add_octomap=False,
        )
    )
    normal.add_child(
        BtNode_ServeOrder(
            name="Serve announcement",
            bb_order_key=KEY_CUSTOMER_ORDER,
        )
    )

    fallback = py_trees.composites.Sequence(name="Show-picture fallback", memory=True)
    fallback.add_child(
        BtNode_Announce(
            name="Announce unreachable delivery",
            bb_source=None,
            message="I could not deliver to this customer. Showing who the order was for.",
        )
    )
    fallback.add_child(
        BtNode_ShowImage(
            name="Show customer picture (delivery fallback)",
            bb_image_path_key=KEY_ACTIVE_CUSTOMER_PICTURE,
        )
    )
    fallback.add_child(
        BtNode_UpdateChecklistFlag(
            name="Mark partial score (delivery)",
            checklist_key=KEY_ORDER_CHECKLIST,
            flag="partial_score",
            value=True,
        )
    )
    fallback.add_child(py_trees.behaviours.Success(name="Fallback SUCCESS"))

    deliver_or_fallback.add_child(normal)
    deliver_or_fallback.add_child(fallback)
    root.add_child(deliver_or_fallback)
    return root


def createPerItemLoop():
    """Iterate items in the active customer's order: one bar round-trip per item.

    Each iteration: ``PopNextItem → createPickupVerification → createDeliverOrder``.
    The item list is destructively drained; when ``PopNextItem`` returns FAILURE
    the ``Retry`` is exhausted and ``FailureIsSuccess`` converts the terminal
    FAILURE into SUCCESS for the parent sequence.
    """
    iter_body = py_trees.composites.Sequence(name="One item bar-to-table", memory=True)
    iter_body.add_child(
        BtNode_PopNextItem(
            name="Pop next item",
            items_key=KEY_CURRENT_ITEMS,
            current_item_key=KEY_CURRENT_ITEM,
            current_item_summary_key=KEY_CURRENT_ITEM_SUMMARY,
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
        )
    )
    iter_body.add_child(createPickupVerification())
    iter_body.add_child(createDeliverOrder())

    return py_trees.decorators.FailureIsSuccess(
        name="Deliver all items",
        child=py_trees.decorators.Retry(
            name="per-item loop",
            child=iter_body,
            num_failures=32,
        ),
    )


def createSingleOrderCycle():
    """End-to-end cycle for one customer order."""
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
    root.add_child(createOptionalTrayTransport())
    root.add_child(createPerItemLoop())
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


def createRestaurantTask():
    """Compose the default Restaurant task with optional second cycle."""
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
    """Runtime entry for the default Restaurant strategy."""
    run_tree(createRestaurantTask, period_ms=500.0, title="Restaurant")


if __name__ == "__main__":
    restaurant()
