# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Manipulation Nodes Module
# =========================
#
# This module provides behavior tree nodes for robot arm manipulation.
# All nodes inherit from either ServiceHandler or ActionHandler and include
# built-in mock mode support with teleop interaction.
#
# Classes
# -------
# BtNode_CartesianMove
#     Moves the arm using Cartesian (end-effector) control with point cloud.
# BtNode_Grasp
#     Grasps an object using vision-guided grasping.
# BtNode_Drop
#     Drops an object at a specified location (trash bin).
# BtNode_Place
#     Places an object at a target location.
# BtNode_MoveArm
#     Moves arm through predefined scan poses (iterative).
# BtNode_MoveArmSingle
#     Moves arm to a single predefined pose.
# BtNode_GripperAction
#     Opens or closes the gripper.
# BtNode_PointTo
#     Points the arm towards a specific person.
#
# Mock Mode
# ---------
# Manipulation nodes support TELEOP mode in mock mode, allowing keyboard-based
# arm control for testing and development without real hardware.
#

from typing import Any, Optional
import py_trees as pytree

# from tinker_decision_msgs.srv import Grasp, Drop
# from tinker_decision_msgs.srv import ObjectDetection
from geometry_msgs.msg import PointStamped, Pose, Point
from behavior_tree.messages import (
    Grasp,
    ObjectDetection,
    Drop,
    Place,
    JointMove,
    CartesianMove,
    GripperCommand,
    Fold,
    ScanAndPlace,
)
from py_trees.common import Status
from behavior_tree.Constants import SCAN_POSES
import action_msgs.msg as action_msgs


from .BaseBehaviors import ServiceHandler
from .ActionBase import ActionHandler
from .pointing_math import compute_point_to_pan
import math


class BtNode_CartesianMove(ActionHandler):
    """
    Moves the arm using Cartesian (end-effector) control with point cloud.

    This node performs Cartesian path planning using a point cloud for
    collision avoidance. It moves the end-effector to a target point
    while avoiding obstacles in the environment.
    """

    def __init__(
        self,
        name: str,
        bb_key_pointcloud: str,
        bb_key_point: str,
        action_name="cartesian_move_action",
    ):
        super().__init__(name, CartesianMove, action_name, None)
        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key(
            key="pointcloud",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_key_pointcloud),
        )
        self.blackboard.register_key(
            key="point",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_key_point),
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
            self.feedback_message = (
                f"CartesianMove feedback received with status: {self.result_status}"
            )
            self.logger.debug(
                f"CartesianMove feedback received with status: {self.result_status}"
            )
            return pytree.common.Status.FAILURE
        else:
            result = self.result_message.result
            if result.success:
                self.feedback_message = (
                    f"CartesianMove feedback received with success: {result.success}"
                )
                self.logger.debug(f"CartesianMove feedback received with success")
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"CartesianMove feedback received with success: {result.success} and error message {result.error_msg}"
                self.logger.debug(
                    f"CartesianMove feedback received with success: {result.success} and error message {result.error_msg}"
                )
                return pytree.common.Status.FAILURE

    def feedback_callback(self, msg):
        return super().feedback_callback(msg)


class BtNode_Grasp(ActionHandler):
    """
    Node for grasping an object with a specific prompt
    """

    def __init__(
        self,
        name: str,
        bb_source: Optional[str] = None,
        action_name: str = "start_grasp",
        bb_key_vision_res: Optional[str] = None,
        bb_key_object_label: Optional[str] = None,
    ):
        """
        executed when creating tree diagram, therefor very minimal

        Args:
            name: name of the node (to be displayed in the tree)
            bb_source: blackboard key to a str prompt
            action_name: name of the action running Grasp
            bb_key_object_label: blackboard key to a str object label
                (e.g. "plate", "bowl"); if None, falls back to
                vision_result.objects[0].cls, then empty string
        """
        super(BtNode_Grasp, self).__init__(
            name, Grasp, action_name, bb_source, wait_for_server_timeout_sec=-3
        )
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self._bb_key_vision_res = bb_key_vision_res
        if bb_key_vision_res is not None:
            self.blackboard.register_key(
                key="vision_result",
                access=pytree.common.Access.READ,
                remap_to=pytree.blackboard.Blackboard.absolute_name(
                    "/", bb_key_vision_res
                ),
            )
        self._bb_key_object_label = bb_key_object_label
        if bb_key_object_label is not None:
            self.blackboard.register_key(
                key="object_label",
                access=pytree.common.Access.READ,
                remap_to=pytree.blackboard.Blackboard.absolute_name(
                    "/", bb_key_object_label
                ),
            )

    def send_goal(self):
        # Handle mock mode — when vision is mocked, vision_result is a
        # MockMessage with no .header/.rgb_image/.segments, so skip real goal
        # construction and use the base ActionHandler mock path (mirrors the
        # sibling manip nodes, e.g. BtNode_MoveArmSingle ~line 540).
        if self.mock_mode:
            self.feedback_message = "MOCK: Grasp succeeded"
            super().send_goal()
            return
        try:
            goal = Grasp.Goal()
            goal.header = self.blackboard.vision_result.header
            goal.rgb_image = self.blackboard.vision_result.rgb_image
            goal.depth_image = self.blackboard.vision_result.depth_image
            goal.segments = self.blackboard.vision_result.segments
            # Resolve object_label: explicit bb key > vision_result.objects[0].cls > ""
            if self._bb_key_object_label is not None:
                goal.object_label = str(self.blackboard.object_label or "")
            else:
                try:
                    goal.object_label = str(
                        self.blackboard.vision_result.objects[0].cls
                    )
                except (AttributeError, IndexError):
                    goal.object_label = ""
            self.send_goal_request(goal)
            self.feedback_message = f"Sent grasp goal with header {goal.header}, segments {len(goal.segments)}, label '{goal.object_label}'"
        except Exception as e:
            self.feedback_message = f"Failed to send grasp goal; error: {e}"
            self.logger.error(f"Failed to send grasp goal; error: {e}")
            return pytree.common.Status.FAILURE

    def process_result(self):
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            result = self.result_message.result
            self.feedback_message = f"Grasp failed with status: {self.result_status}, stage: {result.stage}, error: {result.error_msg}"
            self.logger.debug(
                f"Grasp failed with status: {self.result_status}, stage: {result.stage}, error: {result.error_msg}"
            )
            return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Grasp succeeded"
            self.logger.debug("Grasp succeeded")
            return pytree.common.Status.SUCCESS

    def feedback_callback(self, msg: Any):
        return super().feedback_callback(msg)


class BtNode_Drop(ServiceHandler):
    """
    Drops an object at a specified location (trash bin).

    This node moves the arm to a drop position and releases the gripper
    to drop an object. The drop location can be read from the blackboard
    or specified directly.
    """

    def __init__(
        self,
        name: str,
        bb_source: str,
        service_name: str = "start_drop",
        bin_point: PointStamped = None,
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
                key="drop_point",
                access=pytree.common.Access.READ,
                remap_to=pytree.blackboard.Blackboard.absolute_name(
                    "/", self.bb_source
                ),
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
        super().initialise()

        # Handle mock mode
        if self.mock_mode:
            self.feedback_message = "MOCK: Dropped object successfully"
            print(f"📦 MOCK DROP: Object dropped")
            return

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
        self.response = self.call_service_async(request)

        self.feedback_message = f"Initialized Drop"

    def update(self):
        # Handle mock mode
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()

        if self.response is None:
            return pytree.common.Status.FAILURE

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


class BtNode_FoldClothing(ActionHandler):
    """
    Folds a piece of clothing on a target point.

    Sends a `FoldClothing` action goal carrying:
      - target_point: PointStamped where the cloth currently sits / should be folded
      - object_label: cloth class hint (e.g. "shirt", "towel"), routed via blackboard
      - env_points: latest environment PointCloud2 for collision avoidance
      - fold_cycles: number of fold passes the action should perform
    """

    def __init__(
        self,
        name: str,
        action_name: str = "fold_action",
    ):
        super().__init__(
            name, Fold, action_name, None, wait_for_server_timeout_sec=-3
        )


    def send_goal(self):
        try:
            goal = Fold.Goal()
            self.send_goal_request(goal)
        except Exception as e:
            self.feedback_message = f"Failed to send fold goal; error: {e}"
            self.logger.error(f"Failed to send fold goal; error: {e}")
            return pytree.common.Status.FAILURE

    def process_result(self):
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            err = getattr(getattr(self, "result_message", None), "result", None)
            self.feedback_message = (
                f"Fold failed with status: {self.result_status}"
            )
            return pytree.common.Status.FAILURE
        self.feedback_message = "Fold succeeded"
        return pytree.common.Status.SUCCESS

    def feedback_callback(self, msg: Any):
        return super().feedback_callback(msg)


class BtNode_Place(ActionHandler):
    """
    Places an object at a target location.

    This node performs a place action using vision guidance and collision
    avoidance. It takes the target point, current grasp pose, and
    environment point cloud to plan and execute the place motion.
    """

    def __init__(
        self,
        name: str,
        bb_key_point: str,
        bb_key_pose: str,
        bb_key_env_points: str,
        action_name: str = "place_action",
    ):
        super(BtNode_Place, self).__init__(
            name, Place, action_name, None, wait_for_server_timeout_sec=-3
        )
        self.bb_key_point = bb_key_point
        self.bb_key_pose = bb_key_pose

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="target_point",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", self.bb_key_point),
        )
        self.blackboard.register_key(
            key="grasp_pose",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", self.bb_key_pose),
        )
        self.blackboard.register_key(
            key="env_points",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_key_env_points),
        )

    def send_goal(self):
        try:
            goal = Place.Goal()
            goal.target_point = self.blackboard.target_point
            goal.orientation = self.blackboard.grasp_pose
            goal.env_points = self.blackboard.env_points
            self.send_goal_request(goal)
            self.feedback_message = f"Sent place goal with target point {self.blackboard.target_point} and grasp pose {self.blackboard.grasp_pose}"
            self.logger.debug(
                f"Sent place goal with target point {self.blackboard.target_point} and grasp pose {self.blackboard.grasp_pose}"
            )
        except Exception as e:
            self.feedback_message = f"Failed to send place goal with target point {self.blackboard.target_point} and grasp pose {self.blackboard.grasp_pose}; error: {e}"
            self.logger.error(
                f"Failed to send place goal with target point {self.blackboard.target_point} and grasp pose {self.blackboard.grasp_pose}; error: {e}"
            )
            return pytree.common.Status.FAILURE

    def process_result(self):
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            result = self.result_message.result
            self.feedback_message = f"Place failed with status: {self.result_status}, error: {result.error_msg}"
            self.logger.debug(
                f"Place failed with status: {self.result_status}, error: {result.error_msg}"
            )
            return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Place succeeded"
            self.logger.debug("Place succeeded")
            return pytree.common.Status.SUCCESS

    def feedback_callback(self, msg: Any):
        feedback = msg.feedback
        if feedback.status != 0:
            self.feedback_message = f"ERROR:  {feedback.status} - {feedback.message}"
            self.logger.error(
                f"Place feedback received with error: {feedback.status} - {feedback.message}"
            )
        else:
            self.feedback_message = f"INFO:  {feedback.status} - {feedback.message}"
            self.logger.debug(
                f"Place feedback received with info: {feedback.status} - {feedback.message}"
            )


class BtNode_MoveArm(ActionHandler):
    """
    Moves arm through predefined scan poses (iterative).

    This node moves the arm to a sequence of predefined poses for scanning
    operations. It reads an index from the blackboard and moves to the
    corresponding pose, incrementing the index for the next iteration.

    Migrated from the ``arm_joint_service`` service (ArmJointService) to the
    ``joint_move_action`` action (JointMove). The interfaces are field
    equivalent (joint0..joint6 + add_octomap; result.success). The action name
    is selected via the ``action_name`` kwarg (default ``"joint_move_action"``).
    The legacy ``service_name`` kwarg is retained as a deprecated alias; a
    passed value of ``"arm_joint_service"`` is transparently remapped to
    ``"joint_move_action"`` for back-compat.
    """

    def __init__(
        self,
        name: str,
        #  arm_joint_pose: list[float]
        arm_pose_bb_key,
        action_name: str = "joint_move_action",
        add_octomap: bool = False,
        *,
        service_name: Optional[str] = None,  # deprecated alias for action_name
    ):
        # Back-compat: the node was migrated from the ``arm_joint_service``
        # service to the ``joint_move_action`` action. The legacy
        # ``service_name=`` kwarg and the old ``"arm_joint_service"`` value are
        # still accepted and transparently mapped to ``action_name``.
        if service_name is not None:
            action_name = service_name
        if action_name == "arm_joint_service":
            action_name = "joint_move_action"
        super().__init__(
            name, JointMove, action_name, None, wait_for_server_timeout_sec=-3
        )
        self.arm_pose_bb_key = arm_pose_bb_key
        self.add_octomap = add_octomap
        self.arm_joint_pose = None
        self.arm_pose_idx = 0

    def setup(self, **kwargs):
        ActionHandler.setup(self, **kwargs)

        self.bb_write_client = self.attach_blackboard_client(name="MoveArm Read")
        self.bb_write_client.register_key(
            self.arm_pose_bb_key, access=pytree.common.Access.WRITE
        )

        # debugger info (shown with DebugVisitor)
        self.logger.debug(f"Setup MoveArm, reading from {self.arm_pose_bb_key}")

    def send_goal(self):
        try:
            self.arm_pose_idx = self.bb_write_client.get(self.arm_pose_bb_key)
            assert isinstance(self.arm_pose_idx, int)
            self.arm_joint_pose = SCAN_POSES[self.arm_pose_idx % len(SCAN_POSES)]
        except Exception as e:
            self.feedback_message = f"MoveArm reading pose index failed"
            raise e

        # Advance the index so the next visit scans the next pose. Done here
        # (rather than in the old update() after the service completed) because
        # ActionHandler.update() is the base poller and isn't overridden.
        self.bb_write_client.set(
            self.arm_pose_bb_key, self.arm_pose_idx + 1, overwrite=True
        )

        # Handle mock mode
        if self.mock_mode:
            self.feedback_message = f"MOCK: Moved arm to pose {self.arm_joint_pose}"
            print(f"🦾 MOCK MOVE ARM JOINT: Pose {self.arm_pose_idx}")
            super().send_goal()
            return

        try:
            goal = JointMove.Goal()
            goal.joint0 = self.arm_joint_pose[0]
            goal.joint1 = self.arm_joint_pose[1]
            goal.joint2 = self.arm_joint_pose[2]
            goal.joint3 = self.arm_joint_pose[3]
            goal.joint4 = self.arm_joint_pose[4]
            goal.joint5 = self.arm_joint_pose[5]
            goal.joint6 = self.arm_joint_pose[6]
            goal.add_octomap = self.add_octomap
            self.send_goal_request(goal)
            self.feedback_message = (
                f"Sent move arm joint goal for joints {self.arm_joint_pose}"
            )
        except Exception as e:
            self.feedback_message = f"Failed to send move arm joint goal; error: {e}"
            self.logger.error(f"Failed to send move arm joint goal; error: {e}")

    def process_result(self):
        # JointMove.Result has ONLY `success` (no status, no error_msg).
        if (
            self.result_status == action_msgs.GoalStatus.STATUS_SUCCEEDED
            and self.result_message.result.success
        ):
            self.feedback_message = "Move arm Successful"
            return pytree.common.Status.SUCCESS
        self.feedback_message = (
            f"Move arm failed with status: {self.result_status}"
        )
        return pytree.common.Status.FAILURE

    def feedback_callback(self, msg):
        # JointMove feedback is EMPTY; override the base (which reads
        # delay_limit/status/stage) to a no-op.
        pass


class BtNode_MoveArmSingle(ActionHandler):
    """
    Moves arm to a single predefined pose.

    This node moves the arm to a joint configuration read from the
    blackboard. Unlike BtNode_MoveArm, it does not iterate through
    multiple poses.

    Migrated from the ``arm_joint_service`` service (ArmJointService) to the
    ``joint_move_action`` action (JointMove). The interfaces are field
    equivalent (joint0..joint6 + add_octomap; result.success). The action name
    is selected via the ``action_name`` kwarg (default ``"joint_move_action"``).
    The legacy ``service_name`` kwarg is retained as a deprecated alias; a
    passed value of ``"arm_joint_service"`` is transparently remapped to
    ``"joint_move_action"``.
    """

    def __init__(
        self,
        name: str,
        arm_pose_bb_key: str,
        action_name: str = "joint_move_action",
        #  arm_joint_pose: list[float]
        add_octomap: bool = False,
        *,
        service_name: Optional[str] = None,  # deprecated alias for action_name
    ):
        # Back-compat: the node was migrated from the ``arm_joint_service``
        # service to the ``joint_move_action`` action. The legacy
        # ``service_name=`` kwarg and the old ``"arm_joint_service"`` value are
        # still accepted and transparently mapped to ``action_name``.
        if service_name is not None:
            action_name = service_name
        if action_name == "arm_joint_service":
            action_name = "joint_move_action"
        super().__init__(
            name, JointMove, action_name, None, wait_for_server_timeout_sec=-3
        )
        self.arm_pose_bb_key = arm_pose_bb_key
        self.add_octomap = add_octomap
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="arm_joint_pose",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", arm_pose_bb_key),
        )

    def setup(self, **kwargs):
        ActionHandler.setup(self, **kwargs)

        # debugger info (shown with DebugVisitor)
        self.logger.debug(f"Setup MoveArm, reading from {self.arm_pose_bb_key}")

    def send_goal(self):
        # Handle mock mode
        if self.mock_mode:
            print(f"🤖 MOCK: Moving arm to position")
            self.feedback_message = "MOCK: Arm movement simulated"
            super().send_goal()
            return

        try:
            arm_joint_pose = self.blackboard.arm_joint_pose
            goal = JointMove.Goal()
            goal.joint0 = arm_joint_pose[0]
            goal.joint1 = arm_joint_pose[1]
            goal.joint2 = arm_joint_pose[2]
            goal.joint3 = arm_joint_pose[3]
            goal.joint4 = arm_joint_pose[4]
            goal.joint5 = arm_joint_pose[5]
            goal.joint6 = arm_joint_pose[6]
            goal.add_octomap = self.add_octomap
            self.send_goal_request(goal)
            self.feedback_message = (
                f"Sent move arm joint goal for joints {arm_joint_pose}"
            )
        except Exception as e:
            self.feedback_message = f"Failed to send move arm joint goal; error: {e}"
            self.logger.error(f"Failed to send move arm joint goal; error: {e}")

    def process_result(self):
        # JointMove.Result has ONLY `success` (no status, no error_msg).
        if (
            self.result_status == action_msgs.GoalStatus.STATUS_SUCCEEDED
            and self.result_message.result.success
        ):
            self.feedback_message = "Move arm Successful"
            return pytree.common.Status.SUCCESS
        self.feedback_message = (
            f"Move arm failed with status: {self.result_status}"
        )
        return pytree.common.Status.FAILURE

    def feedback_callback(self, msg):
        # JointMove feedback is EMPTY; override the base (which reads
        # delay_limit/status/stage) to a no-op.
        pass


class BtNode_GripperAction(ActionHandler):
    """
    Opens or closes the gripper.

    This node controls the gripper to either open or close based on the
    open_gripper parameter. It uses the standard ROS2 gripper action
    interface.
    """

    def __init__(
        self,
        name: str,
        open_gripper: bool,
        action_name: str = "/xarm_gripper/gripper_action",
        wait_for_server_timeout_sec: float = -3,
    ):
        super().__init__(
            name, GripperCommand, action_name, None, wait_for_server_timeout_sec
        )
        if open_gripper:
            self.goal = 0.0
        else:
            self.goal = 0.78

    def send_goal(self):
        try:
            goal = GripperCommand.Goal()
            goal.command.position = self.goal
            goal.command.max_effort = 10.0
            self.send_goal_request(goal)
            self.feedback_message = f"Sent gripper goal {self.goal}"
        except Exception as e:
            self.feedback_message = (
                f"Failed to send gripper goal {self.goal}; error: {e}"
            )
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


class BtNode_PointTo(ActionHandler):
    """
    Points the arm towards a specific person.

    This node moves the arm to point at a target person identified by ID.
    It reads the target's position from the blackboard and computes an
    appropriate arm configuration for pointing.

    Migrated from the ``arm_joint_service`` service (ArmJointService) to the
    ``joint_move_action`` action (JointMove). The interfaces are field
    equivalent (joint0..joint6 + add_octomap; result.success). The action name
    is selected via the ``action_name`` kwarg (default ``"joint_move_action"``).
    The legacy ``service_name`` kwarg is retained as a deprecated alias; a
    passed value of ``"arm_joint_service"`` is transparently remapped to
    ``"joint_move_action"``.

    ``pan_bias`` (radians, default ``0.0``) corrects a constant rotation
    between the target point's source frame and the arm joint0 frame before the
    range check. It is subtracted from the raw ``atan2(y, x)`` bearing and the
    result is wrapped to ``(-pi, pi]``. The ``seat_recommend_bbox_service``
    centroid is rotated ``pi`` about the base Z axis relative to ``base_link``,
    so seat-pointing call sites pass ``pan_bias=math.pi`` to keep the commanded
    pan inside the reachable ``[-pi/2, pi/2]`` window. With the default ``0.0``
    the joint0 math is byte-for-byte identical to the legacy behaviour, so
    person-pointing callers (Receptionist, Inspection, HRI introductions) are
    unaffected.
    """

    def __init__(
        self,
        name: str,
        bb_key_persons: str,
        bb_key_points: str,
        bb_key_init_pose: str,
        target_id: int = 0,
        action_name: str = "joint_move_action",
        *,
        service_name: Optional[str] = None,  # deprecated alias for action_name
        pan_bias: float = 0.0,
    ):
        # Back-compat: the node was migrated from the ``arm_joint_service``
        # service to the ``joint_move_action`` action. The legacy
        # ``service_name=`` kwarg and the old ``"arm_joint_service"`` value are
        # still accepted and transparently mapped to ``action_name``.
        if service_name is not None:
            action_name = service_name
        if action_name == "arm_joint_service":
            action_name = "joint_move_action"
        super().__init__(
            name, JointMove, action_name, None, wait_for_server_timeout_sec=-3
        )
        self.bb_key_persons = bb_key_persons
        self.bb_key_points = bb_key_points
        self.target_id = target_id
        # Constant rotation (radians) between the point's source frame and the
        # arm joint0 frame, subtracted from the raw bearing. Default 0.0 keeps
        # legacy behaviour; the seat-recommend centroid is pi-rotated about base
        # Z, so seat-pointing call sites pass pan_bias=math.pi (see PointTo docs
        # and TemplateNodes/pointing_math.compute_point_to_pan).
        self.pan_bias = pan_bias
        # Default value so the no-goal failure branch can safely format
        # `self.angle` even when send_goal skipped the happy-path assignment
        # (e.g. missing or short points list).
        self.angle = 0.0
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="persons",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_key_persons),
        )
        self.blackboard.register_key(
            key="points",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_key_points),
        )
        self.blackboard.register_key(
            key="arm_joint_pose",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_key_init_pose),
        )

    def setup(self, **kwargs):
        ActionHandler.setup(self, **kwargs)

        # debugger info (shown with DebugVisitor)
        self.logger.debug(f"Setup PointTo, reading from {self.bb_key_persons}")

    def send_goal(self):
        try:
            persons = self.blackboard.persons
        except KeyError:
            persons = None
        try:
            points = self.blackboard.points
        except KeyError:
            points = None

        if (
            persons is None
            or len(persons) <= self.target_id
            or points is None
            or len(points) <= self.target_id
        ):
            # Feature matching did not produce a PointStamped for this target.
            # Leave send_goal_future as None so the base update() fast-fails
            # cleanly; FailureIsSuccess wrappers in the BT can absorb it.
            self.feedback_message = (
                f"PointTo skipped: persons={'None' if persons is None else len(persons)}, "
                f"points={'None' if points is None else len(points)}, target_id={self.target_id}"
            )
            return

        # Handle mock mode
        if self.mock_mode:
            self.feedback_message = f"MOCK: Pointed to target {self.target_id}"
            print(f"👉 MOCK POINT TO: Target {self.target_id}")
            super().send_goal()
            return

        point = points[self.target_id]

        try:
            goal = JointMove.Goal()
            goal.joint0 = compute_point_to_pan(
                point.point.x, point.point.y, self.pan_bias
            )
            self.node.get_logger().info(
                f"Calculated joint0 angle {goal.joint0} (pan_bias={self.pan_bias}) to point at target {self.target_id} with coordinates ({point.point.x}, {point.point.y})"
            )
            if goal.joint0 < -math.pi / 2 or goal.joint0 > math.pi / 2:
                self.node.get_logger().warning(
                    f"Calculated joint0 angle {goal.joint0} is out of expected range [-pi/2, pi/2]"
                )
            goal.joint1 = self.blackboard.arm_joint_pose[1]
            goal.joint2 = self.blackboard.arm_joint_pose[2]
            goal.joint3 = self.blackboard.arm_joint_pose[3]
            goal.joint4 = self.blackboard.arm_joint_pose[4]
            goal.joint5 = self.blackboard.arm_joint_pose[5]
            goal.joint6 = self.blackboard.arm_joint_pose[6]
            goal.add_octomap = False

            self.angle = goal.joint0
            self.send_goal_request(goal)
            self.feedback_message = f"Sent point to goal for joints {self.angle}"
        except Exception as e:
            self.feedback_message = f"Failed to send point to goal; error: {e}"
            self.logger.error(f"Failed to send point to goal; error: {e}")

    def process_result(self):
        # JointMove.Result has ONLY `success` (no status, no error_msg).
        if (
            self.result_status == action_msgs.GoalStatus.STATUS_SUCCEEDED
            and self.result_message.result.success
        ):
            self.feedback_message = "Point To Successful"
            return pytree.common.Status.SUCCESS
        self.feedback_message = (
            f"Point To failed for joints {self.angle} with status: {self.result_status}"
        )
        return pytree.common.Status.FAILURE

    def feedback_callback(self, msg):
        # JointMove feedback is EMPTY; override the base (which reads
        # delay_limit/status/stage) to a no-op.
        pass


# Deferred to the bottom of the module (not with the top imports) on purpose:
# behavior_tree.PickAndPlace.__init__ eagerly imports pick_and_place, which
# imports BtNode_GripperAction/BtNode_MoveArmSingle/BtNode_Grasp from THIS
# module. Importing the PickAndPlace package while Manipulation is still
# partially initialised (i.e. from the top) would deadlock that cycle. By the
# time execution reaches here every class pick_and_place needs is defined, so
# the triggered package import resolves cleanly (config.py itself is stdlib-only).
from behavior_tree.PickAndPlace.config import (  # noqa: E402
    SCAN_AND_PLACE_ACTION_NAME,
    KEY_OBJECT_LABEL,
    KEY_ACTIVE_TARGET_POINT,
    PLACEMENT_MODE_FREE_SPACE,
)


class BtNode_ScanAndPlace(ActionHandler):
    """Scan a destination surface and place the held object.

    Wraps arm_api/scan_and_place_server (action `scan_and_place_action`). The
    server owns *where-on-the-surface* placement; the BT owns *which surface*
    (nav + arm scan pose + mode). Mode + target are read from the blackboard,
    set by BtNode_PopWorkItem per the tree's place_policy:
        0 FREE_SPACE | 1 NEAR_SIMILAR(reference_label) | 2 FIXED_POINT(fixed_target)
    Auto-mocked under the `manipulation` subsystem (registered in
    mock_config.json); under IMMEDIATE it auto-succeeds with no ROS client.
    """

    def __init__(
        self,
        name: str,
        bb_item_description: str = KEY_OBJECT_LABEL,
        bb_placement_mode: str = "pp_active_placement_mode",
        bb_reference_label: str = "pp_active_reference_label",
        bb_margin: str = "pp_active_margin",
        bb_orientation: str = "pp_active_orientation",
        bb_fixed_target: str = KEY_ACTIVE_TARGET_POINT,
        bb_scan_pose: str = "pp_active_scan_pose",
        bb_skip_scan_move: str = "pp_active_skip_scan",
        bb_dry_run: str = "pp_active_dry_run",
        bb_out_placed_at: str = "pp_active_placed_at",
        bb_out_status: str = "pp_active_place_status",
        action_name: str = SCAN_AND_PLACE_ACTION_NAME,
    ):
        super().__init__(
            name, ScanAndPlace, action_name, None, wait_for_server_timeout_sec=-3
        )
        self.blackboard = self.attach_blackboard_client(name=self.name)
        # local-attr -> blackboard-key for every READ input.
        self._reads = {
            "item_description": bb_item_description,
            "placement_mode": bb_placement_mode,
            "reference_label": bb_reference_label,
            "margin": bb_margin,
            "orientation": bb_orientation,
            "fixed_target": bb_fixed_target,
            "scan_pose": bb_scan_pose,
            "skip_scan_move": bb_skip_scan_move,
            "dry_run": bb_dry_run,
        }
        for local, key in self._reads.items():
            self.blackboard.register_key(
                key=local,
                access=pytree.common.Access.READ,
                remap_to=pytree.blackboard.Blackboard.absolute_name("/", key),
            )
        self.blackboard.register_key(
            key="placed_at",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_out_placed_at),
        )
        # status/reason output (spec §6): {'status': int, 'reason': str}.
        self.blackboard.register_key(
            key="place_status",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_out_status),
        )

    def setup(self, **kwargs):
        ActionHandler.setup(self, **kwargs)
        self.logger.debug(f"Setup ScanAndPlace on {self.action_name}")

    def _read(self, local, default):
        try:
            value = getattr(self.blackboard, local)
        except Exception:
            return default
        return default if value is None else value

    def send_goal(self):
        # Mock mode: defer to the base (no goal assembly, no blackboard reads).
        if self.mock_mode:
            return super().send_goal()
        try:
            goal = ScanAndPlace.Goal()
            goal.item_description = str(self._read("item_description", ""))
            goal.placement_mode = int(self._read("placement_mode", PLACEMENT_MODE_FREE_SPACE))
            goal.reference_label = str(self._read("reference_label", ""))
            goal.margin_m = float(self._read("margin", 0.0))
            goal.max_candidates = 0
            orientation = self._read("orientation", None)
            if orientation is not None:
                goal.orientation = orientation
            fixed_target = self._read("fixed_target", None)
            if fixed_target is not None:
                goal.fixed_target = fixed_target
            scan_pose = self._read("scan_pose", None)
            if scan_pose:
                goal.scan_pose_deg = [float(x) for x in scan_pose]
            goal.skip_scan_move = bool(self._read("skip_scan_move", False))
            goal.dry_run = bool(self._read("dry_run", False))
            self.send_goal_request(goal)
            self.feedback_message = f"ScanAndPlace goal sent (mode {goal.placement_mode})"
        except Exception as e:
            self.feedback_message = f"Failed to send ScanAndPlace goal: {e}"
            self.logger.error(self.feedback_message)
            return pytree.common.Status.FAILURE

    def process_result(self):
        # Lazy import breaks the custom_nodes <-> Manipulation import cycle
        # (custom_nodes imports BtNode_Grasp from this module).
        from behavior_tree.PickAndPlace.custom_nodes import record_event

        item = self._read("item_description", "")
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            err, st = "", -1
            try:
                err = self.result_message.result.error_msg
                st = self.result_message.result.status
            except Exception:
                pass
            try:
                self.blackboard.place_status = {"status": st, "reason": err}
            except Exception:
                pass
            record_event(self.blackboard, phase="", item=item,
                         action="scan_and_place", outcome="failure", points_est=0)
            self.feedback_message = f"ScanAndPlace failed: status={self.result_status} err={err}"
            return pytree.common.Status.FAILURE
        result = self.result_message.result
        try:
            self.blackboard.placed_at = result.placed_at
        except Exception:
            pass
        try:
            self.blackboard.place_status = {"status": result.status, "reason": result.error_msg}
        except Exception:
            pass
        # Scoring is owned by the BT route leaf (_RecordEventLeaf in
        # pick_and_place_rulebook). Do NOT record_event here on success, or the
        # placement double-counts on the real robot.
        self.feedback_message = "ScanAndPlace succeeded"
        return pytree.common.Status.SUCCESS

    def feedback_callback(self, msg: Any):
        # ScanAndPlace.Feedback is stage-only (no delay_limit/status), so the
        # base feedback_callback (which reads delay_limit/status) must NOT run.
        import time
        self.action_stage = msg.feedback.stage
        self.last_feedback_time = time.time()
