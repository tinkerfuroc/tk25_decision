from typing import Any
import py_trees as pytree

# from tinker_decision_msgs.srv import Grasp, Drop
# from tinker_decision_msgs.srv import ObjectDetection
from geometry_msgs.msg import PointStamped
from behavior_tree.messages import Grasp, ObjectDetection, Drop, ArmJointService
from control_msgs.action import GripperCommand
from py_trees.common import Status
from behavior_tree.Constants import SCAN_POSES

from .BaseBehaviors import ServiceHandler
from .ActionBase import ActionHandler
import math


class BtNode_Grasp(ServiceHandler):
    """
    Node for grasping an object with a specific prompt
    """
    def __init__(self, 
                 name: str,
                 bb_source: str,
                 service_name : str = "grasp",
                 ):
        """
        executed when creating tree diagram, therefor very minimal

        Args:
            name: name of the node (to be displayed in the tree)
            bb_source: blackboard key to a str prompt
            service_name: name of the service running Grasp
            prompt: optional, if given, skips reading from blackboard
        """
        super(BtNode_Grasp, self).__init__(name, service_name, Grasp)
        self.bb_source = bb_source
        self.bb_read_client = None


    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)

        self.bb_read_client = self.attach_blackboard_client(name="Grasp Read")
        self.bb_read_client.register_key(self.bb_source, access=pytree.common.Access.READ)

        # debugger info (shown with DebugVisitor)
        self.logger.debug(f"Setup Grasp, reading from {self.bb_source}")
        

    def initialise(self) -> None:
        """
        Called when the node is visited
        """
        try:
            vision_result = self.bb_read_client.get(self.bb_source)
            assert isinstance(vision_result, ObjectDetection.Response)
        except Exception as e:
            self.feedback_message = f"Grasp reading object name failed"
            raise e

        request = Grasp.Request()
        request.header = vision_result.header
        request.rgb_image = vision_result.rgb_image
        request.depth_image = vision_result.depth_image
        request.segments = vision_result.segments
        # setup things that needs to be cleared
        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized Grasp"

    def update(self):
        self.logger.debug(f"Update Grasp")
        if self.response.done():
            if self.response.result().success:
                self.feedback_message = f"Grasp Successful"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Grasp failed with status {self.response.result().stage}: {self.response.result().error_msg}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still grasping..."
            return pytree.common.Status.RUNNING


class BtNode_Drop(ServiceHandler):
    def __init__(self, 
                 name: str,
                 bb_source: str,
                 service_name : str = "drop",
                 bin_point : PointStamped = None
                 ):
        """
        executed when creating tree diagram, therefor very minimal
        Args:
            name: the name of the pytree node
            bb_source: path to the key in blackboard containing a geometry_msgs/PointStamped object of the pos of trash can
            service_name: name of the service of type tinker_decision_msgs/Drop     
        """
        super(BtNode_Drop, self).__init__(name, service_name, Drop)
        self.bb_source = bb_source
        self.bb_read_client = None
        self.bin_point = bin_point

        if self.bin_point is not None:
            assert isinstance(self.bin_point, PointStamped)


    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)

        if self.bin_point is None:
            self.bb_read_client = self.attach_blackboard_client(name="Drop Read")
            self.bb_read_client.register_key(self.bb_source, access=pytree.common.Access.READ)

            # debugger info (shown with DebugVisitor)
            self.logger.debug(f"Setup Drop, reading from {self.bb_source}")

    def initialise(self) -> None:
        """
        Called when the node is visited
        """
        if self.bin_point is None:
            try:
                self.bin_point = self.bb_read_client.get(self.bb_source)
                assert isinstance(self.bin_point, PointStamped)
            except Exception as e:
                self.feedback_message = f"Drop reading object name failed"
                raise e

        self.logger.debug(f"Initialized Drop for bin point {self.bin_point}")

        request = Drop.Request()
        request.bin_point = self.bin_point
        # setup things that needs to be cleared
        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized Drop"

    def update(self):
        self.logger.debug(f"Update Drop")
        if self.response.done():
            if self.response.result().status == 0:
                self.feedback_message = f"Drop Successful"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Drop failed with status {self.response.result().status}: {self.response.result().error_msg}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still dropping object..."
            return pytree.common.Status.RUNNING


class BtNode_MoveArm(ServiceHandler):
    def __init__(self, name: str, 
                 service_name: str, 
                #  arm_joint_pose: list[float]
                 arm_pose_bb_key
                 ):
        super().__init__(name, service_name, ArmJointService)
        self.arm_pose_bb_key = arm_pose_bb_key
        self.arm_joint_pose = None
    
    def setup(self, **kwargs):
        ServiceHandler.setup(self, **kwargs)

        self.bb_write_client = self.attach_blackboard_client(name="MoveArm Read")
        self.bb_write_client.register_key(self.arm_pose_bb_key, access=pytree.common.Access.WRITE)

        # debugger info (shown with DebugVisitor)
        self.logger.debug(f"Setup MoveArm, reading from {self.arm_pose_bb_key}")

    def initialise(self):
        try:
            self.arm_pose_idx = self.bb_write_client.get(self.arm_pose_bb_key)
            assert isinstance(self.arm_pose_idx, int)
            # if SCAN_POSES is None:
            #     arm_joint_pose_d = SCAN_POSES_D[self.arm_pose_idx % len(SCAN_POSES)]
            #     self.arm_joint_pose = [x / 180 * math.pi for x in arm_joint_pose_d]
            # else:
            #     self.arm_joint_pose_d = None
            self.arm_joint_pose = SCAN_POSES[self.arm_pose_idx % len(SCAN_POSES)]

        except Exception as e:
            self.feedback_message = f"MoveArm reading object name failed"
            raise e

        request = ArmJointService.Request()
        
        request.joint0 = self.arm_joint_pose[0]
        request.joint1 = self.arm_joint_pose[1]
        request.joint2 = self.arm_joint_pose[2]
        request.joint3 = self.arm_joint_pose[3]
        request.joint4 = self.arm_joint_pose[4]
        request.joint5 = self.arm_joint_pose[5]
        request.joint6 = self.arm_joint_pose[6]

        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized move arm joint for joints {self.arm_joint_pose}"
    
    def update(self) -> Status:
        self.logger.debug(f"Update move arm joint")
        if self.response.done():
            # increase counter
            self.bb_write_client.set(self.arm_pose_bb_key, self.arm_pose_idx + 1, overwrite=True)

            if self.response.result().success:
                self.feedback_message = f"Move arm Successful"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Move arm failed"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = f"Still moving arm to {self.arm_joint_pose}..."
            return pytree.common.Status.RUNNING


class BtNode_MoveArmSingle(ServiceHandler):
    def __init__(self, name: str, 
                 service_name: str, 
                #  arm_joint_pose: list[float]
                 arm_pose_bb_key
                 ):
        super().__init__(name, service_name, ArmJointService)
        self.arm_pose_bb_key = arm_pose_bb_key
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="arm_joint_pose",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", arm_pose_bb_key)
        )
    
    def setup(self, **kwargs):
        ServiceHandler.setup(self, **kwargs)

        # debugger info (shown with DebugVisitor)
        self.logger.debug(f"Setup MoveArm, reading from {self.arm_pose_bb_key}")

    def initialise(self):

        request = ArmJointService.Request()
        
        request.joint0 = self.blackboard.arm_joint_pose[0]
        request.joint1 = self.blackboard.arm_joint_pose[1]
        request.joint2 = self.blackboard.arm_joint_pose[2]
        request.joint3 = self.blackboard.arm_joint_pose[3]
        request.joint4 = self.blackboard.arm_joint_pose[4]
        request.joint5 = self.blackboard.arm_joint_pose[5]
        request.joint6 = self.blackboard.arm_joint_pose[6]

        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized move arm joint for joints {self.blackboard.arm_joint_pose}"
    
    def update(self) -> Status:
        self.logger.debug(f"Update move arm joint")
        if self.response.done():
            if self.response.result().success:
                self.feedback_message = f"Move arm Successful"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Move arm failed"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = f"Still moving arm to {self.blackboard.arm_joint_pose}..."
            return pytree.common.Status.RUNNING


class BtNode_GripperAction(ActionHandler):
    def __init__(self, 
                 name: str, 
                 open_gripper: bool, 
                 action_name: str = '/xarm_gripper/gripper_action', 
                 wait_for_server_timeout_sec: float = -3,
                 ):
        super().__init__(name, GripperCommand, action_name, None, wait_for_server_timeout_sec)
        if open_gripper:
            self.goal = 0.0
        else:
            self.goal = 0.85
    
    def send_goal(self):
        try:
            goal = GripperCommand.Goal()
            goal.command.position = self.goal
            goal.command.max_effort = 10.0
            self.send_goal_request(goal)
            self.feedback_message = f"Sent gripper goal {self.goal}"
        except Exception as e:
            self.feedback_message = f"Failed to send gripper goal {self.goal}"
            pass
    
    def process_result(self):
        if self.result.position < 0.05:
            self.feedback_message = f"Gripper action successful"
            return pytree.common.Status.SUCCESS
        else:
            self.feedback_message = f"Gripper action failed"
            return pytree.common.Status.FAILURE    
