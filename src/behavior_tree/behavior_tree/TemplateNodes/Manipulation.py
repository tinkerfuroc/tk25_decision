from typing import Any
import py_trees as pytree

# from tinker_decision_msgs.srv import Grasp, Drop
# from tinker_decision_msgs.srv import ObjectDetection
from geometry_msgs.msg import PointStamped, Pose, Point
from behavior_tree.messages import Grasp, ObjectDetection, Drop, ArmJointService, Place, PointTo, JointMove, CartesianMove
from control_msgs.action import GripperCommand
from py_trees.common import Status
from behavior_tree.Constants import SCAN_POSES
import action_msgs.msg as action_msgs


from .BaseBehaviors import ServiceHandler
from .ActionBase import ActionHandler
import math

class BtNode_CartesianMove(ActionHandler):
    def __init__(self,
                 name: str,
                 bb_key_pointcloud: str,
                 bb_key_point: str,
                 action_name="cartisian_move_action"
                 ):
        super().__init__(name, CartesianMove, action_name)
        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key(
            key="pointcloud",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_key_pointcloud)
        )
        self.blackboard.register_key(
            key="point",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_key_point)
        )
    def send_goal(self):
        try:
            point = self.blackboard.point
            if isinstance(point, PointStamped):
                point = point.point
            elif not isinstance(point, Point):
                self.feedback_message = "ERROR: invalid point!"
                return pytree.common.Status.FAILURE
            goal = CartesianMove.Goal()
            goal.env_points = self.blackboard.pointcloud
            goal.target_pose = point
            self.send_goal_request(goal)
        except Exception as e:
            self.feedback_message = "ERROR: invalid point!"
            return pytree.common.Status.FAILURE
    def process_result(self):
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = f"CartesianMove feedback received with status: {self.result_status}"
            self.logger.debug(f"CartesianMove feedback received with status: {self.result_status}")
            return pytree.common.Status.FAILURE
        else:
            result = self.result_message.result
            if result.success:
                self.feedback_message = f"CartesianMove feedback received with success: {result.success}"
                self.logger.debug(f"CartesianMove feedback received with success")
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"CartesianMove feedback received with success: {result.success} and error message {result.error_msg}"
                self.logger.debug(f"CartesianMove feedback received with success: {result.success} and error message {result.error_msg}")
                return pytree.common.Status.FAILURE
    def feedback_callback(self, msg):
        return super().feedback_callback(msg)

class BtNode_MoveArmJointPC(ActionHandler):
    def __init__(self,
                 name: str,
                 bb_key_pointcloud: str,
                 bb_key_arm_pose: str,
                 action_name : str = "move_arm_joint_pc"):
        super().__init__(name, PointTo, action_name, None, wait_for_server_timeout_sec=-3)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="pointcloud",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_key_pointcloud)
        )
        self.blackboard.register_key(
            key="arm_pose",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_key_arm_pose)
        )
    def send_goal(self):
        try:
            goal = JointMove.Goal()
            goal.env_points = self.blackboard.pointcloud
            arm_pose = self.blackboard.arm_pose
            goal.joint0 = arm_pose[0]
            goal.joint1 = arm_pose[1]
            goal.joint2 = arm_pose[2]
            goal.joint3 = arm_pose[3]
            goal.joint4 = arm_pose[4]
            goal.joint5 = arm_pose[5]
            goal.joint6 = arm_pose[6]
            self.send_goal_request(goal)
            self.feedback_message = f"Sent move arm joint pc goal with pointcloud and arm pose"
            self.logger.debug(f"Sent move arm joint pc goal with pointcloud and arm pose")
        except Exception as e:
            self.feedback_message = f"Failed to send move arm joint pc goal; error: {e}"
            self.logger.error(f"Failed to send move arm joint pc goal; error: {e}")
            return pytree.common.Status.FAILURE
    def process_result(self):
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = f"MoveArmJointPC feedback received with status: {self.result_status}"
            self.logger.debug(f"MoveArmJointPC feedback received with status: {self.result_status}")
            return pytree.common.Status.FAILURE
        else:
            result = self.result_message.result
            if result.success:
                self.feedback_message = f"MoveArmJointPC feedback received with success: {result.success}"
                self.logger.debug(f"MoveArmJointPC feedback received with success")
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"MoveArmJointPC feedback received with success: {result.success} and error message {result.error_msg}"
                self.logger.debug(f"MoveArmJointPC feedback received with success: {result.success} and error message {result.error_msg}")
                return pytree.common.Status.FAILURE
    def feedback_callback(self, msg):
        return super().feedback_callback(msg)


class BtNode_Grasp(ActionHandler):
    """
    Node for grasping an object with a specific prompt
    """
    def __init__(self, 
                 name: str,
                 bb_source: str,
                 action_name : str = "grasp",
                 ):
        """
        executed when creating tree diagram, therefor very minimal

        Args:
            name: name of the node (to be displayed in the tree)
            bb_source: blackboard key to a str prompt
            service_name: name of the service running Grasp
            prompt: optional, if given, skips reading from blackboard
        """
        super(BtNode_Grasp, self).__init__(name, Grasp, action_name, bb_source, wait_for_server_timeout_sec=-3)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="vision_result",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_source)
        )
    
    def send_goal(self):
        try:
            goal = Grasp.Goal()
            goal.header = self.blackboard.vision_result.header
            goal.rgb_image = self.blackboard.vision_result.rgb_image
            goal.depth_image = self.blackboard.vision_result.depth_image
            goal.segments = self.blackboard.vision_result.segments
            self.send_goal_request(goal)
            self.feedback_message = f"Sent grasp goal with header {goal.header} and segments {len(goal.segments)}"
        except Exception as e:
            self.feedback_message = f"Failed to send grasp goal; error: {e}"
            self.logger.error(f"Failed to send grasp goal; error: {e}")
            return pytree.common.Status.FAILURE

    def process_result(self):
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = f"Grasp feedback received with status: {self.result_status}"
            self.logger.debug(f"Grasp feedback received with status: {self.result_status}")
            return pytree.common.Status.FAILURE
        else:
            result = self.result_message.result
            if result.success:
                self.feedback_message = f"Grasp feedback received with success: {result.success}"
                self.logger.debug(f"Grasp feedback received with success")
                return pytree.common.Status.SUCCESS
            else: 
                self.feedback_message = f"Grasp feedback received with stage: {result.stage} and error message {result.error_msg}"
                self.logger.debug(f"Grasp feedback received with stage: {result.stage} and error message {result.error_msg}")
                return pytree.common.Status.FAILURE

    def feedback_callback(self, msg: Any):
        return super().feedback_callback(msg)


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
        # self.bb_read_client = None
        self.bin_point = bin_point

        if self.bin_point is None:
            self.blackboard = self.attach_blackboard_client(name=self.name)
            self.blackboard.register_key(
                key = "drop_point",
                access=pytree.common.Access.READ,
                remap_to=pytree.blackboard.Blackboard.absolute_name("/", self.bb_source)
            )
        else:
            self.blackboard = None
            assert isinstance(self.bin_point, PointStamped)


    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)

        if self.bin_point is None:
            # self.bb_read_client = self.attach_blackboard_client(name="Drop Read")
            # self.bb_read_client.register_key(self.bb_source, access=pytree.common.Access.READ)

            # debugger info (shown with DebugVisitor)
            self.logger.debug(f"Setup Drop, reading from {self.bb_source}")

    def initialise(self) -> None:
        """
        Called when the node is visited
        """
        if self.bin_point is None:
            try:
                self.bin_point = self.blackboard.drop_point
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

class BtNode_Place(ActionHandler):
    def __init__(self, 
                 name: str,
                 bb_key_point: str,
                 bb_key_pose: str,
                 bb_key_env_points: str,
                 action_name : str = "place_action",
                 ):
        super(BtNode_Place, self).__init__(name, Place, action_name, None, wait_for_server_timeout_sec=-3)
        self.bb_key_point = bb_key_point
        self.bb_key_pose = bb_key_pose

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "target_point",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", self.bb_key_point)
        )
        self.blackboard.register_key(
            key = "grasp_pose",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", self.bb_key_pose)
        )
        self.blackboard.register_key(
            key="env_points",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_key_env_points)
        )

    def send_goal(self):
        try:
            goal = Place.Goal()
            goal.target_point = self.blackboard.target_point
            goal.grasp_pose = self.blackboard.grasp_pose
            goal.env_points = self.blackboard.env_points
            self.send_goal_request(goal)
            self.feedback_message = f"Sent place goal with target point {self.blackboard.target_point} and grasp pose {self.blackboard.grasp_pose}"
            self.logger.debug(f"Sent place goal with target point {self.blackboard.target_point} and grasp pose {self.blackboard.grasp_pose}")
        except Exception as e:
            self.feedback_message = f"Failed to send place goal with target point {self.blackboard.target_point} and grasp pose {self.blackboard.grasp_pose}; error: {e}"
            self.logger.error(f"Failed to send place goal with target point {self.blackboard.target_point} and grasp pose {self.blackboard.grasp_pose}; error: {e}")
            return pytree.common.Status.FAILURE
    
    def process_result(self):
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = f"Place feedback received with status: {self.result_status}"
            self.logger.debug(f"Place feedback received with status: {self.result_status}")
            return pytree.common.Status.FAILURE
        else:
            result = self.result_message.result
            if result.success:
                self.feedback_message = f"Place feedback received with success: {result.success}"
                self.logger.debug(f"Place feedback received with success")
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Place feedback received with success: {result.success} and error message {result.error_msg}"
                self.logger.debug(f"Place feedback received with success: {result.success} and error message {result.error_msg}")
                return pytree.common.Status.FAILURE

    def feedback_callback(self, msg: Any):
        feedback = msg.feedback
        if feedback.status != 0:
            self.feedback_message = f"ERROR:  {feedback.status} - {feedback.message}"
            self.logger.error(f"Place feedback received with error: {feedback.status} - {feedback.message}")
        else:
            self.feedback_message = f"INFO:  {feedback.status} - {feedback.message}"
            self.logger.debug(f"Place feedback received with info: {feedback.status} - {feedback.message}")


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
        request.add_octomap = False

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
                 arm_pose_bb_key: str,
                 add_octomap: bool = False
                 ):
        super().__init__(name, service_name, ArmJointService)
        self.arm_pose_bb_key = arm_pose_bb_key
        self.add_octomap = add_octomap
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
        request.add_octomap = self.add_octomap

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
            self.goal = 0.8
    
    def send_goal(self):
        try:
            goal = GripperCommand.Goal()
            goal.command.position = self.goal
            goal.command.max_effort = 10.0
            self.send_goal_request(goal)
            self.feedback_message = f"Sent gripper goal {self.goal}"
        except Exception as e:
            self.feedback_message = f"Failed to send gripper goal {self.goal}; error: {e}"
            pass
    
    def feedback_callback(self, msg):
        pass
    
    def process_result(self):
        return pytree.common.Status.SUCCESS
        # if self.result_message.result.position:
        #     self.feedback_message = f"Gripper action successful"
        #     return pytree.common.Status.SUCCESS
        # else:
        #     self.feedback_message = f"Gripper action failed"
        #     return pytree.common.Status.FAILURE    


class BtNode_PointTo(ServiceHandler):
    def __init__(self, name: str,
                 bb_key_persons: str,
                 bb_key_points: str,
                 bb_key_init_pose: str,
                 target_id: int = 0,
                 service_name: str = "point_to_service"
                 ):
        super().__init__(name, service_name, ArmJointService)
        # self.bb_key_point = bb_key_point
        self.bb_key_persons = bb_key_persons
        self.bb_key_points = bb_key_points
        self.bb_keY_init_pose = bb_key_init_pose
        self.target_id = target_id
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="persons",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_key_persons)
        )
        self.blackboard.register_key(
            key="points",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_key_points)
        )
        self.blackboard.register_key(
            key="arm_joint_pose",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_key_init_pose)
        )
    
    def setup(self, **kwargs):
        ServiceHandler.setup(self, **kwargs)

        # debugger info (shown with DebugVisitor)
        self.logger.debug(f"Setup PointTo, reading from {self.bb_key_persons}")

    def initialise(self):
        if len(self.blackboard.persons) <= self.target_id:
            self.feedback_message = f"Failed to initialize point_to"
            self.response = None
        else:
            # request = PointTo.Request()
            point = self.blackboard.points[self.target_id]
            # self.response = self.client.call_async(request)

            request = ArmJointService.Request()
            
            request.joint0 = self.blackboard.arm_joint_pose[0]
            request.joint1 = self.blackboard.arm_joint_pose[1]
            request.joint2 = self.blackboard.arm_joint_pose[2]
            request.joint3 = self.blackboard.arm_joint_pose[3]
            request.joint4 = self.blackboard.arm_joint_pose[4]
            request.joint5 = self.blackboard.arm_joint_pose[5]
            request.joint6 = self.blackboard.arm_joint_pose[6]
            request.joint0 = math.atan2(point.point.y, point.point.x)
            self.angle = math.atan2(point.point.y, point.point.x)
            request.add_octomap = False

            self.response = self.client.call_async(request)

            self.feedback_message = f"Initialized point to for joints {self.angle}"
    
    def update(self) -> Status:
        self.logger.debug(f"Update point to")
        if self.response is None:
            self.feedback_message = f"Point To failed for joints {self.angle}"
            return pytree.common.Status.FAILURE

        if self.response.done():
            if self.response.result().success:
                self.feedback_message = f"Point To Successful"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Point To failed"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = f"Still pointing...."
            return pytree.common.Status.RUNNING
