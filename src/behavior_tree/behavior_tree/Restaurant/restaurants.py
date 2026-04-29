from __future__ import annotations

"""Standard Restaurant behavior tree composition.

Shape: **batched three-phase pipeline**, mirroring ``restaurants_fake.py``.

1. **Phase 1 — Collect orders.** ``Repeat(n=2)`` × ``Retry(n=2)`` around a
   single-order collection sequence. Queue-arbitration is preserved from PR #8
   for simultaneous-caller handling; ``BtNode_QueueHasQueued`` gates the scan
   so the detector runs only when the queue is dry (dedup across iterations).
2. **Phase 2 — Barman trip.** One trip only. Gated by
   ``BtNode_OrderListNotEmpty`` — if Phase 1 produced no orders, skip to a
   "nothing to place" announcement and move on.
3. **Phase 3 — Deliver all items.** Single per-item loop across all orders.
   Each iteration: bar round-trip (open gripper → barman places → close) then
   table delivery (with show-picture fallback for unreachable tables).

Shared constants and blackboard keys in ``Restaurant/config.py``. Helpers come
from two modules:
  - ``custumNodes.py`` — list-of-orders helpers (``BtNode_RecordOrder``,
    ``BtNode_FormatOrdersForBarman``, ``BtNode_IterateOrderItems``,
    ``BtNode_MarkItemDelivered``). Originally written for ``restaurants_fake.py``
    but key-name–parameterized, so reused here.
  - ``state_nodes.py`` — queue-arbitration helpers plus the three small guards
    this module introduces.
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
    BtNode_ConfirmOrder,
    BtNode_DetectCallingCustomer,
    BtNode_DetectTray,
    BtNode_FormatOrdersForBarman,
    BtNode_IterateOrderItems,
    BtNode_MarkItemDelivered,
    BtNode_RecordOrder,
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
    KEY_BARMAN_TEXT,
    KEY_CURRENT_ITEM,
    KEY_CURRENT_ITEM_SUMMARY,
    KEY_CUSTOMER_LOCATION,
    KEY_CUSTOMER_ORDER,
    KEY_CUSTOMER_QUEUE,
    KEY_KITCHEN_BAR_POSE,
    KEY_ORDER_CHECKLIST,
    KEY_ORDER_LIST,
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
    BtNode_OrderListNotEmpty,
    BtNode_QueueHasQueued,
    BtNode_QueueWavingCandidates,
    BtNode_RequireActiveCustomer,
    BtNode_RequirePickupVerified,
    BtNode_SelectNextQueuedCustomer,
    BtNode_UpdateChecklistFlag,
)


def createConstantWriter():
    """Initialize static task constants and runtime state on blackboard."""
    root = py_trees.composites.Parallel(
        name="Write constants to blackboard",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    for name, key, value in (
        ("Write kitchen bar location", KEY_KITCHEN_BAR_POSE, POSE_KITCHEN_BAR),
        ("Write arm navigating pose", KEY_ARM_NAVIGATING, ARM_POS_NAVIGATING),
        ("Write arm serving pose", KEY_ARM_SERVING, ARM_POS_SERVING),
        ("Initialize caller queue", KEY_CUSTOMER_QUEUE, []),
        ("Initialize active caller id", KEY_ACTIVE_CUSTOMER_ID, None),
        ("Initialize active caller picture", KEY_ACTIVE_CUSTOMER_PICTURE, ""),
        ("Initialize pickup verification", KEY_PICKUP_VERIFIED, False),
        ("Initialize order checklist", KEY_ORDER_CHECKLIST, {}),
        ("Initialize order list", KEY_ORDER_LIST, []),
        ("Initialize barman text", KEY_BARMAN_TEXT, ""),
    ):
        root.add_child(
            BtNode_WriteToBlackboard(
                name=name,
                bb_namespace="",
                bb_source=None,
                bb_key=key,
                object=value,
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
    inner = py_trees.composites.Selector(name="Legacy detect selector", memory=True)
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
    """Detect callers (only if queue dry) and select one active target via arbitration."""
    root = py_trees.composites.Sequence(name="Detect and arbitrate callers", memory=True)
    detect = py_trees.composites.Selector(name="Detection strategy", memory=True)
    detect.add_child(
        BtNode_QueueHasQueued(
            name="Queue has queued entries?",
            queue_key=KEY_CUSTOMER_QUEUE,
        )
    )
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
    root = py_trees.composites.Selector(name="Approach selected customer", memory=True)
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
        BtNode_UpdateChecklistFlag(
            name="Mark order confirmed",
            checklist_key=KEY_ORDER_CHECKLIST,
            flag="order_confirmed",
            value=True,
        )
    )

    maintain_eye_contact = BtNode_MaintainEyeContact(name="Maintain eye contact (order)", timeout=30.0)
    root_with_eye_contact = py_trees.composites.Parallel(
        name="Take order with eye contact",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected([root]),
        children=[maintain_eye_contact, root]
    )

    return root_with_eye_contact


def createOptionalTrayTransport():
    """Optional tray branch with direct-carry fallback (single announcement, pre-loop)."""
    root = py_trees.composites.Selector(name="Optional tray transport", memory=True)
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

    Called once per item by the Phase-3 delivery loop. Positions the arm, asks
    the barman to place the current item (``KEY_CURRENT_ITEM_SUMMARY``), gates
    on a spoken confirmation, then stows the arm for navigation.
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
        memory=True,
    )

    normal = py_trees.composites.Sequence(name="Normal delivery", memory=True)
    normal.add_child(
        py_trees.decorators.Retry(
            name="retry return to customer",
            child=BtNode_GotoAction(name="Return to customer table", key=KEY_CUSTOMER_LOCATION),
            num_failures=3,
        )
    )
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


def createCollectOneOrder():
    """One pass through the Phase-1 collect loop: detect → approach → order → record."""
    root = py_trees.composites.Sequence(name="Collect one order", memory=True)
    root.add_child(
        BtNode_InitOrderChecklist(
            name="Initialize per-order checklist",
            checklist_key=KEY_ORDER_CHECKLIST,
            pickup_verified_key=KEY_PICKUP_VERIFIED,
        )
    )
    root.add_child(createDetectAndArbitrateCustomers())
    root.add_child(createApproachCustomer())
    root.add_child(
        BtNode_RequireActiveCustomer(
            name="Require active customer (post-approach)",
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
        )
    )
    root.add_child(createTakeAndConfirmOrder())
    root.add_child(
        BtNode_RecordOrder(
            name="Record order to order list",
            bb_key_order_list=KEY_ORDER_LIST,
            bb_key_cur_id=KEY_ACTIVE_CUSTOMER_ID,
            bb_key_cur_pose=KEY_CUSTOMER_LOCATION,
            bb_key_cur_picture=KEY_ACTIVE_CUSTOMER_PICTURE,
            bb_key_cur_order_items=KEY_CUSTOMER_ORDER,
        )
    )
    root.add_child(
        BtNode_CloseActiveCustomer(
            name="Close active caller (order recorded)",
            queue_key=KEY_CUSTOMER_QUEUE,
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
        )
    )
    return root


def createCollectOrdersPhase():
    """Phase 1: two orders, one retry per order."""
    root = py_trees.composites.Sequence(name="Collect orders (2x)", memory=True)
    for i in range(2):
        root.add_child(
            py_trees.decorators.Retry(
                name=f"retry collect order {i + 1}",
                child=createCollectOneOrder(),
                num_failures=2,
            )
        )
    return root


def createBarmanPhase():
    """Phase 2: one bar trip for all collected orders. Gated on non-empty order list."""
    root = py_trees.composites.Selector(name="Barman trip (gated)", memory=True)

    actual = py_trees.composites.Sequence(name="Actual barman trip", memory=True)
    actual.add_child(
        BtNode_OrderListNotEmpty(
            name="Order list not empty?",
            order_list_key=KEY_ORDER_LIST,
        )
    )
    actual.add_child(
        py_trees.decorators.Retry(
            name="retry goto kitchen bar",
            child=BtNode_GotoAction(name="Go to kitchen bar", key=KEY_KITCHEN_BAR_POSE),
            num_failures=3,
        )
    )
    actual.add_child(
        BtNode_FormatOrdersForBarman(
            name="Format orders for barman",
            bb_key_order_list=KEY_ORDER_LIST,
            bb_key_barman_text=KEY_BARMAN_TEXT,
        )
    )
    actual.add_child(
        BtNode_Announce(
            name="Announce orders to barman",
            bb_source=KEY_BARMAN_TEXT,
        )
    )
    actual.add_child(
        BtNode_GetConfirmationAction(
            name="Barman ready?",
            timeout=120.0,  # rulebook allows up to 2 min to instruct the barman
        )
    )

    skip = BtNode_Announce(
        name="Skip barman (no orders)",
        bb_source=None,
        message="No customers served. Skipping barman.",
    )

    root.add_child(actual)
    root.add_child(skip)
    return root


def createDeliverAllItemsPhase():
    """Phase 3: drain KEY_ORDER_LIST one item at a time, each with one bar round-trip."""
    iter_body = py_trees.composites.Sequence(name="One item bar-to-table", memory=True)
    iter_body.add_child(
        BtNode_IterateOrderItems(
            name="Next undelivered item",
            bb_key_order_list=KEY_ORDER_LIST,
            bb_key_cur_item=KEY_CURRENT_ITEM,
            bb_key_cur_order_id=KEY_ACTIVE_CUSTOMER_ID,
            bb_key_cur_order_pose=KEY_CUSTOMER_LOCATION,
            bb_key_cur_order_picture=KEY_ACTIVE_CUSTOMER_PICTURE,
            bb_key_cur_order_summary=KEY_CURRENT_ITEM_SUMMARY,
        )
    )
    iter_body.add_child(createPickupVerification())
    iter_body.add_child(createDeliverOrder())
    iter_body.add_child(
        BtNode_MarkItemDelivered(
            name="Mark item delivered",
            bb_key_order_list=KEY_ORDER_LIST,
            bb_key_cur_order_id=KEY_ACTIVE_CUSTOMER_ID,
            bb_key_cur_item=KEY_CURRENT_ITEM,
        )
    )

    return py_trees.decorators.FailureIsSuccess(
        name="Deliver all items",
        child=py_trees.decorators.Retry(
            name="deliver-all-items",
            child=iter_body,
            num_failures=32,
        ),
    )


def createRestaurantTask():
    """Compose the batched three-phase Restaurant task."""
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

    root.add_child(createCollectOrdersPhase())
    root.add_child(BtNode_Announce(
        name="Phase 1 complete announcement",
        bb_source=None,
        message="Order collection phase complete. I'll now proceed to the barman."
    ))
    root.add_child(createOptionalTrayTransport())
    root.add_child(createBarmanPhase())
    root.add_child(createDeliverAllItemsPhase())

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
    run_tree(createRestaurantTask, period_ms=250.0, title="Restaurant")


if __name__ == "__main__":
    restaurant()
