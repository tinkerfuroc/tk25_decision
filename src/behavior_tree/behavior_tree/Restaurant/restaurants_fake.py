import py_trees
import py_trees_ros
import rclpy
import json
import math
import os
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WaitKeyboardPress, BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_GetConfirmation, BtNode_Listen
from behavior_tree.TemplateNodes.Vision import BtNode_ScanFor
from behavior_tree.TemplateNodes.Manipulation import BtNode_MoveArmSingle, BtNode_Grasp, BtNode_Place, BtNode_GripperAction
from .custumNodes import (
    BtNode_DetectCallingCustomer, 
    BtNode_TakeOrder, 
    BtNode_ConfirmOrder,
    BtNode_CommunicateWithBarman,
    BtNode_DetectTray,
    BtNode_ServeOrder,
    BtNode_ScanForCallingCustomer
)

# Debug flag - set to False to skip keyboard waits
DEBUG_KEYPRESS = os.environ.get('DEBUG_KEYPRESS', 'false').lower() in ('true', '1', 'yes')

try:
    # with open("/home/cindy/Documents/tk25_ws/tk25_decision/src/behavior_tree/behavior_tree/Restaurant/constants.json", "r") as file:
    with open("/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/Restaurant/constants.json", "r") as file:
        constants = json.load(file)
except FileNotFoundError:
    print("ERROR: constants.json not found!")
    raise FileNotFoundError

orders = [{
                "order": "cola and chips",
                "objects": "cola and chips",
                "on_tinker" : "cola"
            }, 
          {
              "order": "water and can of juice",
              "objects": "water and can of juice",
              "on_tinker" : "water"
          }]

def pose_reader(pose_dict):
    return PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=pose_dict["point"]["x"], y=pose_dict["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=pose_dict["orientation"]["x"], 
                                                            y=pose_dict["orientation"]["y"], 
                                                            z=pose_dict["orientation"]["z"], 
                                                            w=pose_dict["orientation"]["w"]))
                        )


pose_kitchen_bar = pose_reader(constants["pose_kitchen_bar"])

KEY_CUSTOMER_ORDER = "customer_order"

ARM_POS_NAVIGATING = [x / 180 * math.pi for x in constants["arm_pos_navigating"]]
ARM_POS_SERVING = [x / 180 * math.pi for x in constants["arm_pos_serving"]]

KEY_KITCHEN_BAR_POSE = "kitchen_bar_pose"
KEY_ARM_NAVIGATING = "arm_navigating"
KEY_ARM_SERVING = "arm_serving"

KEY_CUSTOMER_LOCATION = "customer_location"
KEY_TRAY_LOCATION = "tray_location"

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

def createTakeAndConfirmOrder(order=''):
    root = py_trees.composites.Sequence(name="Take and confirm order", memory=True)
    
    root.add_child(BtNode_Announce(
        name="Hello! I'm Tinker, your service robot. What would you like to order today?",
        bb_source=None,
        message="Hello! I'm Tinker, your service robot. What would you like to order today?"
    ))
    
    root.add_child(BtNode_Listen('listen', KEY_CUSTOMER_ORDER, timeout=5.0))
    root.add_child(BtNode_Announce(
        name=f"You ordered: {order}. Is that correct?",
        bb_source=None,
        message=f"You ordered: {order}. Is that correct?"
    ))
    root.add_child(BtNode_GetConfirmation(
        name="Get order confirmation",
        timeout=5.0
    ))
    root.add_child(BtNode_Announce(
        name="Thank you for your order!",
        bb_source=None,
        message="Thank you for your order!"
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
        name="Please prepare the order. I will wait here.",
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
        name="I will use a tray to transport your order.",
        bb_source=None,
        message="I will use a tray to transport your order."
    ))
    
    root.add_child(tray_sequence)
    
    root.add_child(BtNode_Announce(
        name="I will transport the items directly.",
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

def createSingleOrderCycleFor2ndCall(order:str):
    root = py_trees.composites.Sequence(name="Single order cycle", memory=True)
    root.add_child(BtNode_Announce(
        name="I am ready to take orders.",
        bb_source=None,
        message="I am ready to take orders."
    ))

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

    root.add_child(BtNode_GotoAction("Going to scan for waving customers", key=KEY_CUSTOMER_LOCATION))

    root.add_child(BtNode_Announce("Scanning for calling customers.", bb_source=None, message="Scanning for calling customers."))

    # root.add_child(py_trees.timers.Timer(name="search for calling/waving", duration=3.0)) #fake wait for customer calling/waving
    root.add_child(BtNode_Announce(
        name="Identified calling customer,planning to take orders",
        bb_source=None,
        message="Identified calling customer,planning to take orders"
    ))
    
    # Navigate to customer
    if DEBUG_KEYPRESS:
        root.add_child(BtNode_WaitKeyboardPress("approach customer", 's'))
    else:
        root.add_child(py_trees.decorators.Retry(
            name="retry goto customer",
            child=BtNode_GotoAction(
                name="Go to customer table",
                key=KEY_CUSTOMER_LOCATION
            ),
            num_failures=3
        ))

    root.add_child(createTakeAndConfirmOrder(order['order']))

    # Navigate to barman
    if DEBUG_KEYPRESS:
        root.add_child(BtNode_WaitKeyboardPress("approach barman", 's'))
    else:
        root.add_child(py_trees.decorators.Retry(
            name="retry goto kitchen bar",
            child=BtNode_GotoAction(
                name="Go to kitchen bar",
                key=KEY_KITCHEN_BAR_POSE
            ),
            num_failures=3
        ))

    root.add_child(BtNode_Announce(name=f"Hello, I need to place an order: {order['objects']}. Please prepare these items.", bb_source=None, message=f"Hello, I need to place an order: {order['objects']}. Please prepare these items."))
    root.add_child(BtNode_MoveArmSingle("Move arm to serving pose",
                                        service_name="arm_joint_service",
                                        arm_pose_bb_key=KEY_ARM_SERVING,
                                        add_octomap=False))
    root.add_child(BtNode_GripperAction(name="Open gripper to grasp the order", open_gripper=True))

    root.add_child(BtNode_Announce(
        name="Please place the order in my claw and the rest in cans on my right side once the order is prepared.",
        bb_source=None,
        message="Please place the order in my claw and the rest in cans on my right side once the order is prepared."
    ))
    
    # Wait for order preparation
    if DEBUG_KEYPRESS:
        root.add_child(BtNode_WaitKeyboardPress("wait for order", 's'))
    else:
        # Use a timer to simulate order preparation time (e.g., 5 seconds)
        root.add_child(py_trees.timers.Timer(name="Wait for order preparation", duration=5.0))
    
    root.add_child(BtNode_GripperAction(name="Close gripper", open_gripper=False))
    root.add_child(BtNode_Announce(
        name="Order secured,planning to deliver the order to customer.",
        bb_source=None,
        message="Order secured,planning to deliver the order to customer."
    ))

    # Navigate back to customer
    if DEBUG_KEYPRESS:
        root.add_child(BtNode_WaitKeyboardPress("navigating to customer", 's'))
    else:
        root.add_child(py_trees.decorators.Retry(
            name="retry return to customer",
            child=BtNode_GotoAction(
                name="Return to customer table",
                key=KEY_CUSTOMER_LOCATION
            ),
            num_failures=3
        ))

    root.add_child(BtNode_MoveArmSingle("move arm to serve order", service_name="arm_joint_service", arm_pose_bb_key=KEY_ARM_SERVING))
    root.add_child(BtNode_Announce("announce serving order", bb_source=None, message="Dear customer, here is your order, please take it"))
    root.add_child(py_trees.timers.Timer(name="wait for customer to take order", duration=2.0)) #fake wait for customer to take the order
    root.add_child(BtNode_GripperAction(name="Open gripper", open_gripper=True))
    root.add_child(BtNode_Announce("announce rest of order is in cans", bb_source=None, message=f"The {order['on_tinker']} is in cans on my right side. Please take it."))
    root.add_child(py_trees.timers.Timer(name="wait for customer to take order", duration=2.0)) #fake wait for customer to take the order
    root.add_child(BtNode_Announce("announce order completion", bb_source=None, message="Your order has been completed!"))
    root.add_child(BtNode_MoveArmSingle("Move arm to navigation pose", service_name="arm_joint_service", arm_pose_bb_key=KEY_ARM_NAVIGATING))

    return root


def createRestaurantTask():
    root = py_trees.composites.Sequence(name="Restaurant Task", memory=True)
    
    root.add_child(createConstantWriter())
    root.add_child(BtNode_Announce(
        name="Restaurant service starting.",
        bb_source=None,
        message="Restaurant service starting."
    ))

    root.add_child(createSingleOrderCycleFor2ndCall(orders[0]))
    root.add_child(createSingleOrderCycleFor2ndCall(orders[1]))
    # root.add_child(createSingleOrderCycleFor2ndCall(orders[2]))

    return root

def main():
    rclpy.init(args=None)
    
    # Print debug status
    print("="*70)
    if DEBUG_KEYPRESS:
        print("🔧 DEBUG_KEYPRESS=True: Manual keyboard control enabled")
        print("   Press 's' at each wait point to continue")
    else:
        print("🚀 DEBUG_KEYPRESS=False: Automatic execution (no keyboard waits)")
        print("   Set DEBUG_KEYPRESS=true to enable manual control")
    print("="*70)
    
    root = createRestaurantTask()
    
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node_name="restaurant_node", timeout=15)
    
    def print_tree(tree):
        print(py_trees.display.unicode_tree(root=tree.root, show_status=True))
        print(py_trees.display.unicode_blackboard())
    
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    
    tree.tick_tock(period_ms=500.0, post_tick_handler=print_tree)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(tree.node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()