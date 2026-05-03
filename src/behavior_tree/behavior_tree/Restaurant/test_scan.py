from __future__ import annotations

import math

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.Vision import BtNode_TurnPanTilt
from geometry_msgs.msg import PointStamped, Point
from regex import P

from .custumNodes import BtNode_DetectCallingCustomer

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


def scanAllPositions(target_frame="base_link"):
    root = py_trees.composites.Sequence(
        name="turn pan tilt to look in six positions for calling customer",
        memory=True
    )

    for pan in [ 
        -60.0, 
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
                n_gate=2
            )
        )
        pre_gate.add_child(scan_once(pan, target_frame=target_frame))
        root.add_child(pre_gate)

    return root


def main():
    rclpy.init()
    root = scanAllPositions()
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
