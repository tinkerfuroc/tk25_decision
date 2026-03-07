import py_trees
import py_trees_ros
import rclpy
import json
import math
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_GetConfirmation
from behavior_tree.TemplateNodes.Vision import BtNode_ScanFor
from behavior_tree.TemplateNodes.Manipulation import BtNode_MoveArmSingle, BtNode_Grasp, BtNode_Place
from .custumNodes import (
    BtNode_DetectCallingCustomer, 
    BtNode_TakeOrder, 
    BtNode_ConfirmOrder,
    BtNode_CommunicateWithBarman,
    BtNode_DetectTray,
    BtNode_ServeOrder,
    BtNode_ScanForCallingCustomer
)

try:
    with open("/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/Restaurant/constants.json", "r") as file:
        constants = json.load(file)
except FileNotFoundError:
    print("ERROR: constants.json not found!")
    raise FileNotFoundError

pose_kitchen_bar = PoseStamped(
    header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
    pose=Pose(
        position=Point(
            x=constants["pose_kitchen_bar"]["point"]["x"],
            y=constants["pose_kitchen_bar"]["point"]["y"],
            z=0.0
        ),
        orientation=Quaternion(
            x=constants["pose_kitchen_bar"]["orientation"]["x"],
            y=constants["pose_kitchen_bar"]["orientation"]["y"],
            z=constants["pose_kitchen_bar"]["orientation"]["z"],
            w=constants["pose_kitchen_bar"]["orientation"]["w"]
        )
    )
)

ARM_POS_NAVIGATING = [x / 180 * math.pi for x in constants["arm_pos_navigating"]]
ARM_POS_SERVING = [x / 180 * math.pi for x in constants["arm_pos_serving"]]

KEY_KITCHEN_BAR_POSE = "kitchen_bar_pose"
KEY_CUSTOMER_LOCATION = "customer_location"
KEY_CUSTOMER_ORDER = "customer_order"
KEY_TRAY_LOCATION = "tray_location"
KEY_ARM_NAVIGATING = "arm_navigating"
KEY_ARM_SERVING = "arm_serving"
KEY_ORDER_OBJECTS = "order_objects"

def createConstantWriter():
    root = py_trees.composites.Parallel(
        name="Write constants to blackboard", 
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    
    root.add_child(BtNode_WriteToBlackboard(
        name="Write kitchen bar location", 
        bb_namespace="", 
        bb_source=None, 
        bb_key=KEY_KITCHEN_BAR_POSE, 
        object=pose_kitchen_bar
    ))
    
    root.add_child(BtNode_WriteToBlackboard(
        name="Write arm navigating pose", 
        bb_namespace="", 
        bb_source=None, 
        bb_key=KEY_ARM_NAVIGATING, 
        object=ARM_POS_NAVIGATING
    ))
    
    root.add_child(BtNode_WriteToBlackboard(
        name="Write arm serving pose", 
        bb_namespace="", 
        bb_source=None, 
        bb_key=KEY_ARM_SERVING, 
        object=ARM_POS_SERVING
    ))
    
    return root

def createDetectAndReachCustomer():
    root = py_trees.composites.Sequence(name="Detect and reach customer", memory=True)
    
    detection_selector = py_trees.composites.Selector(name="Customer detection strategy", memory=False)
    
    direct_detection = BtNode_DetectCallingCustomer(
        name="Direct detect calling customer",
        bb_dest_key=KEY_CUSTOMER_LOCATION,
        timeout=5.0
    )
    
    scan_detection = BtNode_ScanForCallingCustomer(
        name="Scan for calling customer",
        bb_dest_key=KEY_CUSTOMER_LOCATION,
        timeout=30.0
    )
    
    detection_selector.add_child(direct_detection)
    detection_selector.add_child(scan_detection)
    
    root.add_child(detection_selector)
    
    root.add_child(py_trees.decorators.Retry(
        name="retry goto customer",
        child=BtNode_GotoAction(
            name="Go to customer table",
            key=KEY_CUSTOMER_LOCATION
        ),
        num_failures=3
    ))
    
    return root

def createTakeAndConfirmOrder():
    root = py_trees.composites.Sequence(name="Take and confirm order", memory=True)
    
    root.add_child(BtNode_Announce(
        name="Greet customer",
        bb_source=None,
        message="Hello! I'm Tinker, your service robot. What would you like to order today?"
    ))
    
    order_loop = py_trees.composites.Sequence(name="Order taking loop", memory=True)
    order_loop.add_child(BtNode_TakeOrder(
        name="Take order",
        bb_dest_key=KEY_CUSTOMER_ORDER,
        timeout=constants["order_confirmation_timeout"]
    ))
    order_loop.add_child(BtNode_ConfirmOrder(
        name="Confirm order",
        bb_order_key=KEY_CUSTOMER_ORDER
    ))
    order_loop.add_child(BtNode_GetConfirmation(
        name="Get confirmation",
        timeout=5.0
    ))
    
    root.add_child(py_trees.decorators.Retry(
        name="retry order taking",
        child=order_loop,
        num_failures=3
    ))
    
    return root

def createPlaceOrderWithBarman():
    root = py_trees.composites.Sequence(name="Place order with barman", memory=True)
    
    root.add_child(py_trees.decorators.Retry(
        name="retry goto kitchen bar",
        child=BtNode_GotoAction(
            name="Go to kitchen bar",
            key=KEY_KITCHEN_BAR_POSE
        ),
        num_failures=3
    ))
    
    root.add_child(BtNode_CommunicateWithBarman(
        name="Communicate with barman",
        bb_order_key=KEY_CUSTOMER_ORDER
    ))
    
    root.add_child(BtNode_Announce(
        name="Wait for order",
        bb_source=None,
        message="Please prepare the order. I will wait here."
    ))
    
    return root

def createOptionalTrayTransport():
    root = py_trees.composites.Selector(name="Optional tray transport", memory=False)
    
    tray_sequence = py_trees.composites.Sequence(name="Use tray transport", memory=True)
    tray_sequence.add_child(BtNode_DetectTray(
        name="Detect available tray",
        bb_dest_key=KEY_TRAY_LOCATION
    ))
    tray_sequence.add_child(BtNode_Announce(
        name="Announce tray usage",
        bb_source=None,
        message="I will use a tray to transport your order."
    ))
    
    root.add_child(tray_sequence)
    
    root.add_child(BtNode_Announce(
        name="Direct transport",
        bb_source=None,
        message="I will transport the items directly."
    ))
    
    return root

def createDeliverOrder():
    root = py_trees.composites.Sequence(name="Deliver order", memory=True)
    
    root.add_child(createOptionalTrayTransport())
    
    root.add_child(py_trees.decorators.Retry(
        name="retry return to customer",
        child=BtNode_GotoAction(
            name="Return to customer table",
            key=KEY_CUSTOMER_LOCATION
        ),
        num_failures=3
    ))
    
    root.add_child(BtNode_ServeOrder(
        name="Serve order",
        bb_order_key=KEY_CUSTOMER_ORDER
    ))
    
    return root

def createSingleOrderCycle():
    root = py_trees.composites.Sequence(name="Single order cycle", memory=True)
    
    root.add_child(createDetectAndReachCustomer())
    root.add_child(createTakeAndConfirmOrder())
    root.add_child(createPlaceOrderWithBarman())
    root.add_child(createDeliverOrder())
    
    return root

def createRestaurantTask():
    root = py_trees.composites.Sequence(name="Restaurant Task", memory=True)
    
    root.add_child(createConstantWriter())
    
    root.add_child(py_trees.decorators.Retry(
        name="retry arm setup",
        child=BtNode_MoveArmSingle(
            name="Move arm to navigation pose",
            service_name="arm_joint_service",
            arm_pose_bb_key=KEY_ARM_NAVIGATING,
            add_octomap=False
        ),
        num_failures=3
    ))
    
    root.add_child(BtNode_Announce(
        name="Start announcement",
        bb_source=None,
        message="Restaurant service starting. I am ready to take your orders."
    ))
    
    order_cycles = py_trees.composites.Selector(name="Multiple order cycles", memory=False)
    
    first_order = py_trees.composites.Sequence(name="First order", memory=True)
    first_order.add_child(createSingleOrderCycle())
    
    second_order_selector = py_trees.composites.Selector(name="Optional second order", memory=False)
    second_order = py_trees.composites.Sequence(name="Second order", memory=True)
    second_order.add_child(BtNode_Announce(
        name="Ready for next order",
        bb_source=None,
        message="I am ready to take another order if needed."
    ))
    second_order.add_child(createSingleOrderCycle())
    
    second_order_selector.add_child(second_order)
    second_order_selector.add_child(BtNode_Announce(
        name="No more orders",
        bb_source=None,
        message="No more customers detected. Service completed."
    ))
    
    first_order.add_child(second_order_selector)
    order_cycles.add_child(first_order)
    
    root.add_child(order_cycles)
    
    root.add_child(BtNode_Announce(
        name="Task completion",
        bb_source=None,
        message="Restaurant service task completed successfully. Thank you!"
    ))
    
    return root

def restaurant():
    rclpy.init(args=None)
    
    root = createRestaurantTask()
    
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node_name="restaurant_node", timeout=15)
    
    def print_tree(tree):
        print(py_trees.display.unicode_tree(root=tree.root, show_status=True))
        print(py_trees.display.unicode_blackboard())
    
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    
    tree.tick_tock(period_ms=500.0, post_tick_handler=print_tree)
    
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == "__main__":
    restaurant()