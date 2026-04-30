import py_trees
import py_trees_ros
import rclpy
import json
import math
import os

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_CaptureCurrentPose, BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import (
    BtNode_Announce,
    BtNode_GetConfirmationAction,
    BtNode_ListenAction,
    BtNode_PhraseExtractionAction,
)
from behavior_tree.TemplateNodes.Vision import (
    BtNode_ScanForWavingPerson,
    BtNode_MaintainEyeContact,
    BtNode_ShowImage,
)
from behavior_tree.TemplateNodes.Manipulation import (
    BtNode_MoveArmSingle,
    BtNode_GripperAction,
)
from behavior_tree.visualization import create_post_tick_visualizer
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

from .custumNodes import (
    BtNode_SelectNextCustomer,
    BtNode_RecordOrder,
    BtNode_FormatOrdersForBarman,
    BtNode_IterateOrderItems,
    BtNode_MarkItemDelivered,
)


try:
    # Known hardcoded path bug — leave as-is (see .claude/rules/behavior-tree.md "Hardcoded paths in task scripts").
    with open("/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/Restaurant/constants.json", "r") as file:
        constants = json.load(file)
except FileNotFoundError:
    print("ERROR: constants.json not found!")
    raise


ARM_POS_NAVIGATING = [x / 180 * math.pi for x in constants["arm_pos_navigating"]]
ARM_POS_SERVING = [x / 180 * math.pi for x in constants["arm_pos_serving"]]
STANDARD_OBJECTS = constants.get("standard_objects", [])

# Module-local blackboard keys.
KEY_KITCHEN_BAR_POSE = "kitchen_bar_pose"
KEY_ARM_NAVIGATING = "arm_navigating"
KEY_ARM_SERVING = "arm_serving"

KEY_ALL_PERSON_POSES = "restaurant_all_person_poses"
KEY_ALL_PERSON_PICTURES = "restaurant_all_person_pictures"
KEY_CLOSEST_PERSON = "restaurant_closest_person"

KEY_CUR_ID = "restaurant_cur_id"
KEY_CUR_POSE = "restaurant_cur_pose"
KEY_CUR_PICTURE = "restaurant_cur_picture"
KEY_CUR_ORDER_ITEMS = "restaurant_cur_order_items"

KEY_ORDER_LIST = "restaurant_order_list"
KEY_BARMAN_TEXT = "restaurant_barman_text"

KEY_CUR_ITEM = "restaurant_cur_item"
KEY_CUR_ORDER_ID = "restaurant_cur_order_id"
KEY_CUR_ORDER_POSE = "restaurant_cur_order_pose"
KEY_CUR_ORDER_PICTURE = "restaurant_cur_order_picture"
KEY_CUR_ORDER_SUMMARY = "restaurant_cur_order_summary"

# Detection knobs.
DETECT_WAVING_THRESHOLD_M = 8.0
ORDER_WORDLIST = STANDARD_OBJECTS


def createConstantWriter():
    root = py_trees.composites.Parallel(
        name="Write constants to blackboard",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    # Barman/anchor pose = robot's pose at task start (operator placement),
    # not a hardcoded map coordinate.
    root.add_child(py_trees.decorators.Retry(
        name="Retry capture task start pose",
        child=BtNode_CaptureCurrentPose(
            name="Capture task start pose as kitchen bar",
            bb_key=KEY_KITCHEN_BAR_POSE,
        ),
        num_failures=3,
    ))
    root.add_child(BtNode_WriteToBlackboard(
        name="Write arm navigating pose",
        bb_namespace="", bb_source=None,
        bb_key=KEY_ARM_NAVIGATING, object=ARM_POS_NAVIGATING,
    ))
    root.add_child(BtNode_WriteToBlackboard(
        name="Write arm serving pose",
        bb_namespace="", bb_source=None,
        bb_key=KEY_ARM_SERVING, object=ARM_POS_SERVING,
    ))
    root.add_child(BtNode_WriteToBlackboard(
        name="Seed empty order list",
        bb_namespace="", bb_source=None,
        bb_key=KEY_ORDER_LIST, object=[],
    ))
    return root


def createCollectOneOrder():
    """
    One customer-order collection pass:
      scan → select → goto → eye-contact → listen → confirm → record.

    If any step fails, the outer Retry decorator re-ticks this whole subtree.
    SelectNextCustomer skips already-assigned ids across ticks.
    """
    seq = py_trees.composites.Sequence(name="Collect one order", memory=True)

    seq.add_child(BtNode_ScanForWavingPerson(
        name="Scan for waving customer",
        bb_key_all_persons=KEY_ALL_PERSON_POSES,
        bb_key_closest_person=KEY_CLOSEST_PERSON,
        bb_key_pictures=KEY_ALL_PERSON_PICTURES,
        threshold_meters=DETECT_WAVING_THRESHOLD_M,
        target_frame="map",
    ))
    seq.add_child(BtNode_SelectNextCustomer(
        name="Select next customer",
        bb_key_all_poses=KEY_ALL_PERSON_POSES,
        bb_key_all_pictures=KEY_ALL_PERSON_PICTURES,
        bb_key_cur_id=KEY_CUR_ID,
        bb_key_cur_pose=KEY_CUR_POSE,
        bb_key_cur_picture=KEY_CUR_PICTURE,
    ))
    seq.add_child(BtNode_GotoAction(
        name="Go to customer table",
        key=KEY_CUR_POSE,
    ))
    seq.add_child(BtNode_MaintainEyeContact(name="Eye-contact (order-taking)"))
    seq.add_child(BtNode_Announce(
        name="Greet customer",
        bb_source=None,
        message="Hello, may I take your order?",
    ))
    seq.add_child(BtNode_ListenAction(
        name="Listen for order",
        bb_dest_key=KEY_CUR_ORDER_ITEMS,
        timeout=10.0,
    ))
    # Keep a phrase-extraction pass so downstream item iteration has something structured.
    # If PhraseExtraction returns a single string, IterateOrderItems will treat it as one item.
    seq.add_child(BtNode_PhraseExtractionAction(
        name="Extract order phrase",
        wordlist=ORDER_WORDLIST,
        bb_dest_key=KEY_CUR_ORDER_ITEMS,
        timeout=5.0,
    ))
    seq.add_child(BtNode_Announce(
        name="Confirm order prompt",
        bb_source=KEY_CUR_ORDER_ITEMS,
        message="You ordered:",
    ))
    seq.add_child(BtNode_Announce(
        name="Ask for confirmation",
        bb_source=None,
        message="Is that correct?",
    ))
    seq.add_child(BtNode_GetConfirmationAction(
        name="Get order confirmation",
        timeout=10.0,
    ))
    seq.add_child(BtNode_RecordOrder(
        name="Record order",
        bb_key_order_list=KEY_ORDER_LIST,
        bb_key_cur_id=KEY_CUR_ID,
        bb_key_cur_pose=KEY_CUR_POSE,
        bb_key_cur_picture=KEY_CUR_PICTURE,
        bb_key_cur_order_items=KEY_CUR_ORDER_ITEMS,
    ))
    return seq


def createCollectOrdersPhase():
    """Two orders; if the first attempt at an order fails, retry once."""
    phase = py_trees.composites.Sequence(name="Collect orders (2x)", memory=True)
    for i in range(2):
        phase.add_child(py_trees.decorators.Retry(
            name=f"retry collect order {i + 1}",
            child=createCollectOneOrder(),
            num_failures=2,
        ))
    return phase


def createBarmanPhase():
    """Single trip to the bar; announce all orders; wait for barman ack."""
    seq = py_trees.composites.Sequence(name="Communicate orders to barman", memory=True)
    seq.add_child(py_trees.decorators.Retry(
        name="retry goto kitchen bar",
        child=BtNode_GotoAction(
            name="Go to kitchen bar",
            key=KEY_KITCHEN_BAR_POSE,
        ),
        num_failures=3,
    ))
    seq.add_child(BtNode_FormatOrdersForBarman(
        name="Format orders for barman",
        bb_key_order_list=KEY_ORDER_LIST,
        bb_key_barman_text=KEY_BARMAN_TEXT,
    ))
    seq.add_child(BtNode_Announce(
        name="Announce orders to barman",
        bb_source=KEY_BARMAN_TEXT,
    ))
    seq.add_child(BtNode_GetConfirmationAction(
        name="Barman ready?",
        timeout=120.0,  # rulebook: up to 2 min barman interaction per order
    ))
    return seq


def createDeliverOneItem():
    """
    For the current item (set by IterateOrderItems):
      1. goto bar, receive item from barman
      2. try deliver to table (eye-contact, place, announce)
      3. on failure, show picture + announce — partial-points fallback
      4. mark item delivered so the loop advances
    """
    seq = py_trees.composites.Sequence(name="Deliver one item", memory=True)

    seq.add_child(py_trees.decorators.Retry(
        name="retry goto bar for item",
        child=BtNode_GotoAction(
            name="Go to bar for item",
            key=KEY_KITCHEN_BAR_POSE,
        ),
        num_failures=3,
    ))
    seq.add_child(BtNode_MoveArmSingle(
        name="Arm to serving pose (at bar)",
        service_name="arm_joint_service",
        arm_pose_bb_key=KEY_ARM_SERVING,
        add_octomap=False,
    ))
    seq.add_child(BtNode_Announce(
        name="Ask barman to place item",
        bb_source=KEY_CUR_ORDER_SUMMARY,
        message="Please place the following item in my gripper:",
    ))
    seq.add_child(BtNode_GripperAction(name="Open gripper", open_gripper=True))
    seq.add_child(BtNode_GetConfirmationAction(
        name="Item placed?",
        timeout=60.0,
    ))
    seq.add_child(BtNode_GripperAction(name="Close gripper", open_gripper=False))
    seq.add_child(BtNode_MoveArmSingle(
        name="Arm to navigating pose",
        service_name="arm_joint_service",
        arm_pose_bb_key=KEY_ARM_NAVIGATING,
        add_octomap=False,
    ))

    deliver_or_fallback = py_trees.composites.Selector(
        name="Deliver or show-picture fallback",
        memory=False,
    )

    deliver = py_trees.composites.Sequence(name="Normal delivery", memory=True)
    deliver.add_child(BtNode_GotoAction(
        name="Go to customer table",
        key=KEY_CUR_ORDER_POSE,
    ))
    deliver.add_child(BtNode_MaintainEyeContact(name="Eye-contact (serving)"))
    deliver.add_child(BtNode_MoveArmSingle(
        name="Arm to serving pose (at table)",
        service_name="arm_joint_service",
        arm_pose_bb_key=KEY_ARM_SERVING,
        add_octomap=False,
    ))
    deliver.add_child(BtNode_Announce(
        name="Serve item",
        bb_source=KEY_CUR_ORDER_SUMMARY,
        message="Here is your:",
    ))
    deliver.add_child(BtNode_Announce(
        name="Countdown",
        bb_source=None,
        message="Three. Two. One.",
    ))
    deliver.add_child(BtNode_GripperAction(name="Release item", open_gripper=True))
    deliver.add_child(BtNode_MoveArmSingle(
        name="Arm to navigating pose",
        service_name="arm_joint_service",
        arm_pose_bb_key=KEY_ARM_NAVIGATING,
        add_octomap=False,
    ))
    deliver_or_fallback.add_child(deliver)

    fallback = py_trees.composites.Sequence(name="Show-picture fallback", memory=True)
    fallback.add_child(BtNode_Announce(
        name="Announce inability",
        bb_source=None,
        message="I could not reach this customer. Here is who the order was for.",
    ))
    fallback.add_child(BtNode_ShowImage(
        name="Show customer picture",
        bb_image_path_key=KEY_CUR_ORDER_PICTURE,
    ))
    fallback.add_child(py_trees.behaviours.Success(name="Fallback SUCCESS"))
    deliver_or_fallback.add_child(fallback)

    seq.add_child(deliver_or_fallback)
    seq.add_child(BtNode_MarkItemDelivered(
        name="Mark item delivered",
        bb_key_order_list=KEY_ORDER_LIST,
        bb_key_cur_order_id=KEY_CUR_ORDER_ID,
        bb_key_cur_item=KEY_CUR_ITEM,
    ))
    return seq


def createDeliveryPhase():
    """
    Loop: pick next undelivered item → deliver → repeat until IterateOrderItems FAILURE.
    `FailureIsSuccess` converts the terminal FAILURE (no more items) into SUCCESS so the
    parent sequence continues.
    """
    body = py_trees.composites.Sequence(name="Iterate + deliver", memory=True)
    body.add_child(BtNode_IterateOrderItems(
        name="Next undelivered item",
        bb_key_order_list=KEY_ORDER_LIST,
        bb_key_cur_item=KEY_CUR_ITEM,
        bb_key_cur_order_id=KEY_CUR_ORDER_ID,
        bb_key_cur_order_pose=KEY_CUR_ORDER_POSE,
        bb_key_cur_order_picture=KEY_CUR_ORDER_PICTURE,
        bb_key_cur_order_summary=KEY_CUR_ORDER_SUMMARY,
    ))
    body.add_child(createDeliverOneItem())

    # Wrap in a Retry with a large ceiling so the sequence keeps restarting (i.e. loops)
    # until IterateOrderItems returns FAILURE (no more items). FailureIsSuccess then
    # converts that terminal FAILURE into SUCCESS for the parent.
    loop = py_trees.decorators.FailureIsSuccess(
        name="Delivery loop wrapper",
        child=py_trees.decorators.Retry(
            name="deliver-all-items",
            child=body,
            num_failures=32,
        ),
    )
    return loop


def createRestaurantTask():
    root = py_trees.composites.Sequence(name="Restaurant Task", memory=True)
    root.add_child(createConstantWriter())
    root.add_child(BtNode_Announce(
        name="Kickoff announcement",
        bb_source=None,
        message="Restaurant service starting.",
    ))
    root.add_child(createCollectOrdersPhase())
    root.add_child(createBarmanPhase())
    root.add_child(createDeliveryPhase())
    root.add_child(BtNode_Announce(
        name="Completion announcement",
        bb_source=None,
        message="Restaurant task complete.",
    ))
    root.add_child(py_trees.behaviours.Running(name="idle-at-end"))
    return root


def main():
    rclpy.init(args=None)
    root = createRestaurantTask()

    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node_name="restaurant_node", timeout=15)
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="Restaurant Demo",
        print_blackboard=True,
    )

    tree.tick_tock(period_ms=100.0, post_tick_handler=print_tree)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(tree.node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        shutdown_visualizer()
        tree.shutdown()
        rclpy.try_shutdown()
