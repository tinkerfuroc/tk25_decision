import py_trees

import py_trees_ros
from behavior_tree.TemplateNodes.Navigation import BtNode_GetOrientationAngle
from behavior_tree.TemplateNodes.Vision import BtNode_TurnPanTilt
from behavior_tree.TemplateNodes.BaseBehaviors import ServiceHandler
from tinker_nav_msgs.srv import OrientationAngle
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
import rclpy

KEY_PANTILT_ORIENTATION = "pantilt_orientation"
KEY_APPROACH_TARGET = "approach_target"


def gazeatCustomer():
    root = py_trees.composites.Sequence(name="Gaze at customer", memory=True)
    turn_pantilt_to_customer = py_trees.composites.Sequence(
        name="Turn pan-tilt to customer orientation", memory=True
    )
    turn_pantilt_to_customer.add_child(
        BtNode_GetOrientationAngle(
            name="Get orientation angle to customer",
            target_point=KEY_APPROACH_TARGET,
            bb_dest_key=KEY_PANTILT_ORIENTATION,
        )
    )
    turn_pantilt_to_customer.add_child(
        BtNode_TurnPanTilt(
            name="Turn pan-tilt to customer orientation",
            x=0.0,
            y=20.0,
            x_key=KEY_PANTILT_ORIENTATION,
        )
    )
    root.add_child(turn_pantilt_to_customer)
    return root


# class BtNode_getOrientationAngleToCustomer(ServiceHandler):
#     def __init__(self, name, bb_target_key, bb_dest_key):
#         super().__init__(
#             service_name=name,
#             service_type=OrientationAngle,
#             request_builder=self._build_request,
#             response_callback=self._handle_response,
#         )
#         self.bb_target_key = bb_target_key
#         self.bb_dest_key = bb_dest_key

#     def _build_request(self, blackboard):
#         point = blackboard.get(self.bb_target_key)
#         req = OrientationAngle.Request()
#         req.point = point
#         req.timeout = 5.0
#         req.max_try = 1
#         return req

#     def _handle_response(self, response, blackboard):
#         if response is None:
#             self.get_logger().warning(f"{self.name}: No response received from service")
#             return False
#         if not response.success:
#             self.get_logger().warning(
#                 f"{self.name}: Service call failed with message: {response.message}"
#             )
#             return False
#         angle = response.angle
#         self.get_logger().info(
#             f"{self.name}: Received orientation angle: {angle} radians"
#         )
#         blackboard.set(self.bb_dest_key, angle)
#         return True


class BtTestOrientation(py_trees.behaviour.Behaviour):
    def __init__(self, name="TestOrientation"):
        self.bb = py_trees.blackboard.Blackboard()
        super().__init__(name)

    def initialise(self):
        test_point = PointStamped(
            point=Point(x=8.42, y=1.55, z=0.0),
            header=Header(frame_id="map", stamp=rclpy.time.Time().to_msg()),
        )
        self.bb.set(KEY_APPROACH_TARGET, test_point)

        return super().initialise()

    def update(self):
        return py_trees.common.Status.SUCCESS


def main(args=None):
    rclpy.init(args=args)
    root = py_trees.composites.Sequence(name="TestOrientation", memory=True)
    bb = py_trees.blackboard.Blackboard()
    bb.set(KEY_PANTILT_ORIENTATION, 0.0)
    root.add_child(BtTestOrientation(name="Set test point on blackboard"))
    # root.add_child(BtNode_PackWavingCustomers(name="Pack waving customers"))
    root.add_child(
        BtNode_GetOrientationAngle(
            name="Get orientation angle to customer",
            target_point=KEY_APPROACH_TARGET,
            bb_dest_key=KEY_PANTILT_ORIENTATION,
        )
    )
    root.add_child(
        BtNode_TurnPanTilt(
            name="Turn pan-tilt to customer orientation",
            x=0.0,
            y=20.0,
            x_key=KEY_PANTILT_ORIENTATION,
        )
    )
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
