from typing import Any
import py_trees
import rclpy
import tf2_geometry_msgs
from .BaseBehaviors import ServiceHandler
from .ActionBase import ActionHandler

from geometry_msgs.msg import PointStamped, PoseStamped

from behavior_tree.messages import Goto, GotoGrasp, ComputeGrasp
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException

import action_msgs.msg as action_msgs  # GoalStatus

import time


class BtNode_GotoAction(ActionHandler):
    def __init__(self, name: str, key: str, action_name: str = "navigate_to_pose", wait_for_server_timeout_sec: float = -3):
        super().__init__(name, NavigateToPose, action_name, key, wait_for_server_timeout_sec)
    
    def send_goal(self):
        try:
            goal = NavigateToPose.Goal()
            goal.pose = self.blackboard.goal
            self.send_goal_request(goal)
            self.feedback_message = "sent goal request"
        except KeyError:
            pass  # self.send_goal_future will be None, check on that
    
    def process_result(self):
        # for navigation only, where the action can return all sorts of results while it's at it
        if self.result_status == action_msgs.GoalStatus.STATUS_ABORTED:  # noqa
            self.feedback_message = "action aborted"
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS
    
    def feedback_callback(self, msg: Any):
        """
        Default generator for feedback messages from the action server. This will
        update the behaviour's feedback message with a stringified version of the
        incoming feedback message.

        Args:
            msg: incoming feedback message (e.g. move_base_msgs.action.MoveBaseFeedback)
        """
        feedback = msg.feedback
        self.last_feedback_time = time.time()
        self.feedback_timeout = 1000
        self.action_status = 0
        self.process_feedback(feedback)  


class BtNode_Goto(ServiceHandler):
    def __init__(self, 
                name: str,
                bb_source: str,
                service_name : str = "goto",
                target : PoseStamped = None
                ):
        """
        executed when creating tree diagram, therefor very minimal
        Args:
            name: the name of the pytree node
            bb_source: path to the key in blackboard containing a geometry_msgs/PoseStamped object of the pos of trash can
            service_name: name of the service of type tinker_decision_msgs/Drop     
        """
        super(BtNode_Goto, self).__init__(name, service_name, Goto)
        self.bb_source = bb_source
        self.bb_read_client = None
        self.target = target
        if target is not None:
            assert isinstance(self.target, PoseStamped)
        

    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)

        if self.target is None:
            self.bb_read_client = self.attach_blackboard_client(name="Goto Read")
            self.bb_read_client.register_key(self.bb_source, access=py_trees.common.Access.READ)

            # debugger info (shown with DebugVisitor)
            self.logger.debug(f"Setup Goto, reading from {self.bb_source}")
        else:
            self.logger.debug(f"Setup Goto from fixed input {self.target}")

    def initialise(self) -> None:
        """
        Called when the node is visited
        """

        if self.target is None:
            try:
                self.target = self.bb_read_client.get(self.bb_source)
                assert isinstance(self.target, PoseStamped)
            except Exception as e:
                self.feedback_message = f"Goto reading target pose failed"
                raise e

        self.logger.debug(f"Initialized Goto for pose {self.target}")

        request = Goto.Request()
        request.target = self.target
        # setup things that needs to be cleared
        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized Goto"

    def update(self):
        self.logger.debug(f"Update Goto")
        if self.response.done():
            if self.response.result().status == 0:
                self.feedback_message = f"Goto Successful"
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = f"Goto failed with status {self.response.result().status}: {self.response.result().error_msg}"
                return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = "Still navigating to target pose..."
            return py_trees.common.Status.RUNNING


class BtNode_GotoGrasp(ServiceHandler):
    def __init__(self, 
                name: str,
                bb_source: str,
                service_name : str = "go_to_grasp",
                target : PointStamped = None
                ):
        """
        executed when creating tree diagram, therefor very minimal
        Args:
            name: the name of the pytree node
            bb_source: path to the key in blackboard containing a geometry_msgs/PoseStamped object of the pos of trash can
            service_name: name of the service of type tinker_decision_msgs/Drop     
        """
        super(BtNode_GotoGrasp, self).__init__(name, service_name, GotoGrasp)
        self.bb_source = bb_source
        self.bb_read_client = None
        self.target = target
        self.read = True
        if target is not None:
            assert isinstance(self.target, PointStamped)
            self.read = False


    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)

        if self.read:
            self.bb_read_client = self.attach_blackboard_client(name="GotoGrasp Read")
            self.bb_read_client.register_key(self.bb_source, access=py_trees.common.Access.READ)

            # debugger info (shown with DebugVisitor)
            self.logger.debug(f"Setup GotoGrasp, reading from {self.bb_source}")
        else:
            self.logger.debug(f"Setup GotoGrasp from fixed point {self.target}")

    def initialise(self) -> None:
        """
        Called when the node is visited
        """
        if self.read:
            try:
                self.target = self.bb_read_client.get(self.bb_source)
                assert isinstance(self.target, PointStamped)
                self.read = True
            except Exception as e:
                self.feedback_message = f"GotoGrasp reading target point failed"
                raise e

        self.logger.debug(f"Initialized GotoGrasp for target point {self.target}")

        request = GotoGrasp.Request()
        request.target = self.target
        # setup things that needs to be cleared
        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized GotoGrasp, read set to {self.read}"

    def update(self):
        self.logger.debug(f"Update GotoGrasp")
        if self.response.done():
            self.feedback_message = f"GotoGrasp returned with status {self.response.result().status}"
            return py_trees.common.Status.SUCCESS
            # if self.response.result().status == 0:
            #     self.feedback_message = f"GotoGrasp Successful"
            #     return py_trees.common.Status.SUCCESS
            # else:
            #     self.feedback_message = f"GotoGrasp failed with status {self.response.result().status}: {self.response.result().error_msg}"
            #     return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = f"Still navigating to grasping pose {self.target} from {self.bb_source} (read set to {self.read})"
            return py_trees.common.Status.RUNNING


class BtNode_CalcGraspPose(ServiceHandler):
    def __init__(self, 
                name: str,
                bb_source: str,
                bb_dest:str,
                service_name : str = "compute_grasp_pos",
                target : PointStamped = None
                ):
        """
        executed when creating tree diagram, therefor very minimal
        Args:
            name: the name of the pytree node
            bb_source: path to the key in blackboard containing a geometry_msgs/PoseStamped object of the pos of trash can
            service_name: name of the service of type tinker_decision_msgs/Drop     
        """
        super(BtNode_CalcGraspPose, self).__init__(name, service_name, ComputeGrasp)
        self.bb_source = bb_source
        self.bb_read_client = None
        self.bb_dest = bb_dest
        self.target = target
        self.read = True
        if target is not None:
            assert isinstance(self.target, PointStamped)
            self.read = False


    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)

        self.bb_write_client = self.attach_blackboard_client(name="CalcGraspPose Write")
        self.bb_write_client.register_key(self.bb_dest, access=py_trees.common.Access.WRITE)

        if self.read:
            self.bb_read_client = self.attach_blackboard_client(name="CalcGraspPose Read")
            self.bb_read_client.register_key(self.bb_source, access=py_trees.common.Access.READ)

            # debugger info (shown with DebugVisitor)
            self.logger.debug(f"Setup CalcGraspPose, reading from {self.bb_source}")
        else:
            self.logger.debug(f"Setup CalcGraspPose from fixed point {self.target}")
        
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self.node)

    def initialise(self) -> None:
        """
        Called when the node is visited
        """
        if self.read:
            try:
                self.target = self.bb_read_client.get(self.bb_source)
                assert isinstance(self.target, PointStamped)
                self.read = True
            except Exception as e:
                self.feedback_message = f"CalcGraspPose reading target point failed"
                raise e

        self.logger.debug(f"Initialized CalcGraspPose for target point {self.target}")

        try:
            transform = self._tf_buffer.lookup_transform(
                                target_frame="map",
                                source_frame=self.target.header.frame_id[1 if self.target.header.frame_id[0] == '/' else 0:],
                                time=rclpy.time.Time()
                            )
        except LookupException as e:
            assert False, "Failed to lookup transform"
            
        map_point = tf2_geometry_msgs.do_transform_point(self.target, transform)

        request = ComputeGrasp.Request()
        request.target = map_point
        # setup things that needs to be cleared
        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized CalcGraspPose, read set to {self.read}"

    def update(self):
        self.logger.debug(f"Update CalcGraspPose")
        if self.response.done():
            self.feedback_message = f"CalcGraspPose finished"
            goal_pose = self.response.result().target
            self.bb_write_client.set(self.bb_dest, goal_pose, overwrite=True)

            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"Still calculating grasping pose {self.target} from {self.bb_source} (read set to {self.read})"
            return py_trees.common.Status.RUNNING
