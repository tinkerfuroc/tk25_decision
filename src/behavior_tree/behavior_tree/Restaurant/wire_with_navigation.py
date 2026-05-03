from __future__ import annotations

import math

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
import py_trees
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.Vision import BtNode_TurnPanTilt
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Quaternion, Pose
from std_msgs.msg import Header

from .custumNodes import BtNode_DetectCallingCustomer

import rclpy
import py_trees_ros
import py_trees
from behavior_tree.TemplateNodes.Audio import (
    BtNode_Announce,
    BtNode_PhraseExtractionAction,
)


CUSTOMER_ORDER_DRINK1="customer_order_drink"
CUSTOMER_ORDER_FOOD1="customer_order_food"
CUSTOMER_ORDER_DRINK2="customer_order_drink"
CUSTOMER_ORDER_FOOD2="customer_order_food"

def _create_get_info(field_name: str, storage_key: str, word_list: list[str]):
    """High-confidence-first capture with a last-resort confirmation fallback.

    Primary branch: up to 2 attempts of prompt → action-based extract. The
    action (`phrase_extraction_action`) only succeeds on server status=0,
    which means Whisper + Qwen ASR cross-check agreed on the same wordlist
    entry. The rulebook awards a 4×15 "no non-essential questions" bonus for
    accepting on that signal without a confirmation prompt.

    Fallback branch: if both primary attempts abort, re-prompt, capture
    the raw transcription via `BtNode_ListenAction`, then `BtNode_Confirm`
    speaks it back (`"Your <field> is <value>, correct?"`) and
    `BtNode_GetConfirmationAction` waits for yes/no. Preserves partial
    scoring in noisy environments at the cost of the no-confirmation
    bonus for this field only.
    """
    primary_loop = py_trees.composites.Sequence(
        name=f"Prompt+extract {field_name}",
        memory=True,
    )
    primary_loop.add_child(
        BtNode_Announce(
            name=f"Prompt for {field_name}",
            bb_source=None,
            message=f"What is your {field_name} order?.",
        )
    )
    primary_loop.add_child(
        BtNode_PhraseExtractionAction(
            name=f"High-conf extract {field_name}",
            wordlist=word_list,
            bb_dest_key=storage_key,
            timeout=7.0,
        )
    )
    primary = py_trees.decorators.Retry(
        name=f"Retry high-conf {field_name}",
        child=primary_loop,
        num_failures=10
    )

    fallback = py_trees.composites.Sequence(
        name=f"Last-resort confirm {field_name}",
        memory=True,
    )
    fallback.add_child(
        BtNode_Announce(
            name=f"Fallback prompt for {field_name}",
            bb_source=None,
            message=f"Let me try again. Please tell me your {field_name} clearly.",
        )
    )
    fallback.add_child(
        BtNode_PhraseExtractionAction(
            name=f"High-conf extract {field_name}",
            wordlist=word_list,
            bb_dest_key=storage_key,
            timeout=7.0,
        )
    )

    root = py_trees.composites.Selector(
        name=f"Get {field_name}",
        memory=True,
    )
    root.add_child(primary)
    root.add_child(py_trees.decorators.Retry(name="retry 3 times", child=fallback, num_failures=4))
    return root

def get_order(drink_order_key, food_order_key):
    root = py_trees.composites.Sequence(
        name="Get customer order",
        memory=True
    )

    root.add_child(
        BtNode_Announce(
            "announce waiting door bell",
            bb_source=None,
            message="Hi customer, please speak to me after the beep sound"
        )
    )

    root.add_child(
        _create_get_info(
            field_name="drink",
            storage_key=drink_order_key,
            word_list=['cola', 'water', 'sprite', 'orange', 'milk']
        )
    )

    root.add_child(
        BtNode_Announce(
            "repeat order",
            bb_source=drink_order_key,
            message="Got order of "
        )
    )

    root.add_child(
        _create_get_info(
            field_name="food",
            storage_key=food_order_key,
            word_list=['chip', 'chips', 'biscuit', 'cookie', 'bread', 'lays']
        )
    )

    root.add_child(
        BtNode_Announce(
            "repeat order",
            bb_source=food_order_key,
            message="Got order of "
        )
    )

    return root


def main():
    rclpy.init()
    root = get_order()
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node_name="test_scan", timeout=15)

    def _print(t):
        print(py_trees.display.unicode_tree(root=t.root, show_status=True))

    tree.tick_tock(period_ms=500.0, post_tick_handler=_print)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
    


###########################
KEY_DETECT_WAVING_RESULT = "detect_waving_result"
KEY_CUSTOMER_CENTROIDS = "customer_centroids"
PAN_ANGLES = (-180.0, -120.0, -60.0, 0.0, 60.0, 120.0)
TILT_DEG = 30.0
SETTLE_SEC = 0.5

class BtNode_PackWavingCustomers(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name,
        bb_key_detect_waving_results,
        bb_key_customer_centroids,
        centroid_distance_gate:float=0.3,
    ):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="waving_results",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_detect_waving_results),
        )
        self.blackboard.register_key(
            key="customer_centroids",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_customer_centroids),
        )
        self.centroid_distance_gate = centroid_distance_gate
    
    def update(self):
        if not self.blackboard.exists("waving_results"):
            self.feedback_message = "waving results does not exist on black board"
            return py_trees.common.Status.FAILURE
            
        if not self.blackboard.exists("customer_centroids"):
            self.blackboard.customer_centroids = []
        
        # should be PointStamped[]
        detected_centroids = self.blackboard.waving_results
        for centroid in detected_centroids:
            centroid_header = getattr(centroid, "header", None)
            centroid_point = getattr(centroid, "point", None)
            should_skip = any(
                math.hypot(
                    centroid.point.x - existing_centroid.point.x,
                    centroid.point.y - existing_centroid.point.y,
                ) < self.centroid_distance_gate
                for existing_centroid in self.blackboard.customer_centroids
            )
            self.logger.warning(
                f"Detected waving centroid ({'skipped' if should_skip else 'appended'}): "
                f"header={centroid_header}, point={centroid_point}"
            )
            if should_skip:
                continue
            self.blackboard.customer_centroids.append(centroid)
        
        return py_trees.common.Status.SUCCESS

class BtNode_GateBlackBoardList(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name,
        bb_key_list,
        n_gate=2
    ):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="list_to_be_gated",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_list),
        )
        self.n_gate = n_gate
    
    def update(self):
        if not self.blackboard.exists("list_to_be_gated"):
            self.feedback_message = "list to be gated is not on blackbarod"
            return py_trees.common.Status.FAILURE
        
        list_to_be_gated = self.blackboard.list_to_be_gated

        if not isinstance(list_to_be_gated, list):
            self.feedback_message = "list ot be gated is not instance of LIST"
            return py_trees.common.Status.FAILURE
        
        if len(list_to_be_gated) >= self.n_gate:
            self.feedback_message = f"list length is {len(list_to_be_gated)} which is greater or equal to {self.n_gate}"
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
              

def scan_once(pan:float, tilt:float=35.0, target_frame="base_link"):
    root = py_trees.composites.Sequence(
        name=f"scan at {pan} x for waving customers",
        memory=True
    )
    root.add_child(
        BtNode_TurnPanTilt(
            name=f"turn pan tilt towards pan {pan}",
            x=pan,
            y=tilt
        )
    )
    root.add_child(py_trees.timers.Timer(name="wait for pan tilt to settle", duration=1.0))
    root.add_child(
        BtNode_DetectCallingCustomer(
            name=f"detect calling customer vision at {pan}",
            target_frame=target_frame,
            bb_dest_key=KEY_DETECT_WAVING_RESULT
        )
    )
    root.add_child(
        BtNode_PackWavingCustomers(
            name="pack detect waving results",
            bb_key_detect_waving_results=KEY_DETECT_WAVING_RESULT,
            bb_key_customer_centroids=KEY_CUSTOMER_CENTROIDS
        )
    )
    return root


def scanAllPositions(target_frame="map", n_gate=1):
    root = py_trees.composites.Sequence(
        name="turn pan tilt to look in six positions for calling customer",
        memory=True
    )

    for pan in [ 
        # -60.0, 
        0.0,
        60.0,
        120.0
        -120.0,
        -180.0,
        0.0,
        60.0,
        120.0
        ]:
        pre_gate = py_trees.composites.Selector(
            f"gate and then scan for x {pan}",
            memory=True
        )
        pre_gate.add_child(
            BtNode_GateBlackBoardList(
                name="gate",
                bb_key_list=KEY_CUSTOMER_CENTROIDS,
                n_gate=n_gate
            )
        )
        pre_gate.add_child(scan_once(pan, target_frame=target_frame))
        root.add_child(pre_gate)

    return root

############################
from behavior_tree.TemplateNodes.Navigation import BtNode_Approach

class BtNode_ExtractOnePoint(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name,
        bb_key_customer_centroids,
        bb_key_extracted_centroid
    ):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="customer_centroids",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_customer_centroids),
        )
        self.blackboard.register_key(
            key="extracted_centroid",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_extracted_centroid),
        )
    
    def update(self):
        if not self.blackboard.exists("customer_centroids"):
            self.feedback_message = "customer_centroids does not exist on black board"
            return py_trees.common.Status.FAILURE

        self.blackboard.extracted_centroid = self.blackboard.customer_centroids[0]
        self.blackboard.customer_centroids = self.blackboard.customer_centroids[1:]
        return py_trees.common.Status.SUCCESS

KEY_CUSTOMER_LOCATION = "customer_location"

def navigateToCustomer():
    root=py_trees.composites.Sequence(
        "navigate to customer",
        True
    )

    root.add_child(
        BtNode_Announce(
            name="announce start navigation",
            bb_source=None,
            message="Navigating to calling customer",
        )
    )

    root.add_child(
        BtNode_ExtractOnePoint(
            name="extract one customer location",
            bb_key_customer_centroids=KEY_CUSTOMER_CENTROIDS,
            bb_key_extracted_centroid=KEY_CUSTOMER_LOCATION
        )
    )

    root.add_child(
        py_trees.decorators.Retry(
            name="Retry approach",
            child=BtNode_Approach(
                name="Approach detected person",
                bb_target_key=KEY_CUSTOMER_LOCATION,
            ),
            num_failures=3
        )
    )

    return root    


POSE_BARMAN = PoseStamped(
    header=Header(frame_id="map"),
    pose=Pose(
        position=Point(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0)
    )
)
KEY_POSE_BARMAN = "pose_barman"

# def writeBarmanPoseToBlackboard():
#     return BtNode_WriteToBlackboard(
#         name="write barman pose to blackboard",
#         bb_key=KEY_POSE_BARMAN,
#         value=POSE_BARMAN
#     )

def announceAllOrders():
    root = py_trees.composites.Sequence(
        name="give all orders to barman",
        memory=True
    )

    root.add_child(
        BtNode_Announce(
            "repeat order",
            bb_source= CUSTOMER_ORDER_DRINK1,
            message="customer one ordered drink of "
        )
    )

    root.add_child(
        BtNode_Announce(
            "repeat order",
            bb_source=CUSTOMER_ORDER_FOOD2,
            message="and food of "
        )
    )

    root.add_child(
        BtNode_Announce(
            "repeat order",
            bb_source=CUSTOMER_ORDER_DRINK2,
            message="customer two orderd drink of "
        )
    )

    root.add_child(
        BtNode_Announce(
            "repeat order",
            bb_source=CUSTOMER_ORDER_FOOD2,
            message="and food of "
        )
    )


    return root


def with_navigation():
    root=py_trees.composites.Sequence(
        "scan for waving person and navigate to them",
        True
    )
    # root.add_child(writeBarmanPoseToBlackboard())
    root.add_child(
        BtNode_Announce(
            name="announce start navigation",
            bb_source=None,
            message="Detecting waving customer",
        )
    )
    root.add_child(scanAllPositions(n_gate=2))
    root.add_child(navigateToCustomer())
    root.add_child(get_order(
        drink_order_key=CUSTOMER_ORDER_DRINK1,
        food_order_key=CUSTOMER_ORDER_FOOD1
    ))

    root.add_child(navigateToCustomer())
    root.add_child(get_order(
        drink_order_key=CUSTOMER_ORDER_DRINK2,
        food_order_key=CUSTOMER_ORDER_FOOD2
    ))




    return root

def main():
    rclpy.init()
    root = with_navigation()
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node_name="test_approach", timeout=15)

    def _print(t):
        print(py_trees.display.unicode_tree(root=t.root, show_status=True))

    tree.tick_tock(period_ms=500.0, post_tick_handler=_print)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
    

