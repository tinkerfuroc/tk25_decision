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


class BtNode_GotoAction(ActionHandler):
    def __init__(self, name: str, key: str, action_name: str = "'/navigate_to_pose'", wait_for_server_timeout_sec: float = -3):
        super().__init__(name, NavigateToPose, action_name, key, wait_for_server_timeout_sec)


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
        super(BtNode_GotoGrasp, self).__init__(name, service_name, ComputeGrasp)
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

        transform = self._tf_buffer.lookup_transform(
                            target_frame="map",
                            source_frame=request.target.header.frame_id[1 if request.target.header.frame_id[0] == '/' else 0:],
                            time=rclpy.time.Time()
                        )
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
