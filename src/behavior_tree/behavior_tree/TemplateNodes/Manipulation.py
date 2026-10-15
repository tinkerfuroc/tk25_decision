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
# BtNode_FoldClothingDn
#     Folds a garment via the YOLO-driven "dn" server (fold_dn_action).
#
# Mock Mode
# ---------
# Manipulation nodes support TELEOP mode in mock mode, allowing keyboard-based
# arm control for testing and development without real hardware.
#

from typing import Any, Optional
import copy
import time
import os
import py_trees as pytree

# from tinker_decision_msgs.srv import Grasp, Drop
# from tinker_decision_msgs.srv import ObjectDetection
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, Point
from behavior_tree.messages import (
    Pick,
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
from .FoldClothingAction import BtNode_FoldClothingAction
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


class _Tk27Task(ActionHandler):
    """Shared minimal-result and progress handling for TK27 business actions."""

    def process_result(self):
        result = self.result_message.result
        succeeded = (
            self.result_status == action_msgs.GoalStatus.STATUS_SUCCEEDED
            and result.success
            and result.status == 0
        )
        self.feedback_message = (
            f"{self.action_name}: ros_status={self.result_status}, "
            f"status={result.status}, success={result.success}, "
            f"detail={result.error_msg}"
        )
        return Status.SUCCESS if succeeded else Status.FAILURE

    def feedback_callback(self, msg):
        progress = msg.feedback.progress
        self.last_feedback_time = time.time()
        self.action_stage = progress.stage
        self.feedback_message = (
            f"stage={progress.stage}, elapsed={progress.elapsed_s:.2f}s: "
            f"{progress.detail}"
        )


class BtNode_Grasp(_Tk27Task):
    """Execute TK27 Pick; vision capture, closure and lift belong to the server."""

    def __init__(
        self,
        name: str,
        bb_source: Optional[str] = None,
        action_name: str = "/manipulation/pick",
        bb_key_vision_res: Optional[str] = None,
        bb_key_object_label: Optional[str] = None,
        *,
        prompt: str = "",
        plan_effort: str = "",
    ):
        timeout_ticks = 0
        raw_timeout = os.environ.get("GPSR_GRASP_ACTION_TIMEOUT_S")
        if raw_timeout:
            try:
                timeout_ticks = max(1, round(float(raw_timeout) / 0.5))
            except (TypeError, ValueError):
                pass
        super().__init__(
            name,
            Pick,
            action_name,
            None,
            wait_for_server_timeout_sec=-3,
            action_timeout_ticks=timeout_ticks,
        )
        self.prompt = prompt
        self.plan_effort = plan_effort
        self._bb_source = bb_source
        self._bb_key_vision_res = bb_key_vision_res
        self._bb_key_object_label = bb_key_object_label
        self.blackboard = self.attach_blackboard_client(name=self.name)
        for local, key in (
            ("prompt", bb_source),
            ("object_label", bb_key_object_label),
            ("vision_result", bb_key_vision_res),
        ):
            if key is not None:
                self.blackboard.register_key(
                    key=local,
                    access=pytree.common.Access.READ,
                    remap_to=pytree.blackboard.Blackboard.absolute_name("/", key),
                )

    def send_goal(self):
        if self.mock_mode:
            return super().send_goal()
        try:
            # Existing discovery trees may supply a label from an observation.
            # Only its text is reused; TK27 obtains a fresh capture itself.
            prompt = self.prompt
            if self._bb_source is not None:
                prompt = self.blackboard.prompt
            elif self._bb_key_object_label is not None:
                prompt = self.blackboard.object_label
            elif not prompt and self._bb_key_vision_res is not None:
                prompt = self.blackboard.vision_result.objects[0].cls
            if not isinstance(prompt, str) or not prompt.strip():
                raise ValueError("Pick requires a nonempty text prompt")
            if self.plan_effort not in ("", "auto", "low", "medium", "high"):
                raise ValueError("Unsupported plan_effort")
            goal = Pick.Goal()
            goal.prompt = prompt.strip()
            goal.plan_effort = self.plan_effort
            self.send_goal_request(goal)
            self.feedback_message = f"Sent Pick prompt: {goal.prompt}"
        except Exception as exc:
            self.feedback_message = f"Failed to send Pick: {exc}"
            self.logger.error(self.feedback_message)
            return Status.FAILURE


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


# class BtNode_FoldClothing(ActionHandler):
#     """
#     Folds a piece of clothing on a target point.

#     Sends a `FoldClothing` action goal carrying:
#       - target_point: PointStamped where the cloth currently sits / should be folded
#       - object_label: cloth class hint (e.g. "shirt", "towel"), routed via blackboard
#       - env_points: latest environment PointCloud2 for collision avoidance
#       - fold_cycles: number of fold passes the action should perform
#     """

#     def __init__(
#         self,
#         name: str,
#         action_name: str = "fold_action",
#     ):
#         super().__init__(name, Fold, action_name, None, wait_for_server_timeout_sec=-3)

#     def send_goal(self):
#         try:
#             goal = Fold.Goal()
#             self.send_goal_request(goal)
#         except Exception as e:
#             self.feedback_message = f"Failed to send fold goal; error: {e}"
#             self.logger.error(f"Failed to send fold goal; error: {e}")
#             return pytree.common.Status.FAILURE

#     def process_result(self):
#         if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
#             err = getattr(getattr(self, "result_message", None), "result", None)
#             self.feedback_message = f"Fold failed with status: {self.result_status}"
#             return pytree.common.Status.FAILURE
#         self.feedback_message = "Fold succeeded"
#         return pytree.common.Status.SUCCESS

#     def feedback_callback(self, msg: Any):
#         return super().feedback_callback(msg)


class BtNode_FoldClothingDn(BtNode_FoldClothingAction):
    """Fold a garment via the YOLO-driven "dn" server (``fold_dn_action``).

    Replaces the retired ``BtNode_FoldClothing`` above (which sent an empty
    ``Fold.Goal()`` to the old garment-agnostic ``fold_action``). The dn server
    (``arm_api/fold_clothing_dn.py``) speaks the SAME ``tinker_arm_msgs/FoldClothing``
    contract as ``fold_clothing_action``, so this node reuses
    ``BtNode_FoldClothingAction`` wholesale — feedback handling (the
    non-conformant feedback override), result mapping, and garment-label
    resolution are all inherited; only the default ``action_name`` differs.

    The dn server runs fixed metric-offset folds, so it IGNORES
    ``garment_label`` / ``bottom_fold_mode`` (kept here for contract parity and to
    let trees swap nodes freely); it HONORS ``return_to_scan``.

    Mock mode: registered as ``BtNode_FoldClothingDn`` under the ``manipulation``
    subsystem in ``mock_config.json`` (mock is keyed by class name, so a subclass
    needs its own entry).
    """

    def __init__(
        self,
        name: str,
        bb_key_garment_label: Optional[str] = None,
        garment_label: str = "",
        bottom_fold_mode: int = 0,
        return_to_scan: bool = True,
        action_name: str = "fold_dn_action",
        wait_for_server_timeout_sec: float = -3.0,
    ):
        super().__init__(
            name=name,
            bb_key_garment_label=bb_key_garment_label,
            garment_label=garment_label,
            bottom_fold_mode=bottom_fold_mode,
            return_to_scan=return_to_scan,
            action_name=action_name,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
        )


class BtNode_Place(_Tk27Task):
    """Convert the tree's framed TCP point and orientation into TK27 Place."""

    def __init__(
        self,
        name: str,
        bb_key_point: str,
        bb_key_pose: str,
        bb_key_env_points: str,
        action_name: str = "/manipulation/place",
        *,
        retain_grasp_orientation: Optional[bool] = None,
    ):
        super().__init__(name, Place, action_name, None, wait_for_server_timeout_sec=-3)
        self.bb_key_point = bb_key_point
        self.bb_key_pose = bb_key_pose
        self.retain_grasp_orientation = retain_grasp_orientation
        self.blackboard = self.attach_blackboard_client(name=self.name)
        for local, key in (
            ("target_point", bb_key_point),
            ("grasp_pose", bb_key_pose),
            ("env_points", bb_key_env_points),
        ):
            self.blackboard.register_key(
                key=local,
                access=pytree.common.Access.READ,
                remap_to=pytree.blackboard.Blackboard.absolute_name("/", key),
            )

    def send_goal(self):
        if self.mock_mode:
            return super().send_goal()
        try:
            point = self.blackboard.target_point
            if not isinstance(point, PointStamped) or not point.header.frame_id:
                raise ValueError("Place requires a framed TCP PointStamped")
            goal = Place.Goal()
            goal.tcp_pose.header = copy.deepcopy(point.header)
            goal.tcp_pose.pose.position = copy.deepcopy(point.point)
            orientation = None
            if self.retain_grasp_orientation is not True:
                pose = self.blackboard.grasp_pose
                if pose is not None:
                    if isinstance(pose, PoseStamped):
                        if pose.header.frame_id != point.header.frame_id:
                            raise ValueError(
                                "Place point and orientation frames differ"
                            )
                        pose = pose.pose
                    if not isinstance(pose, Pose):
                        raise ValueError(
                            "Place orientation must be Pose or PoseStamped"
                        )
                    q = pose.orientation
                    values = (q.x, q.y, q.z, q.w)
                    if not all(math.isfinite(v) for v in values):
                        raise ValueError("Place orientation must be finite")
                    if any(values):
                        if abs(sum(v * v for v in values) - 1.0) > 1e-3:
                            raise ValueError(
                                "Place orientation must be a unit quaternion"
                            )
                        orientation = q
            retain = self.retain_grasp_orientation
            if retain is None:
                retain = orientation is None
            if not retain and orientation is None:
                raise ValueError("Explicit Place orientation is missing")
            goal.retain_grasp_orientation = retain
            if retain:
                # The server replaces this valid placeholder with its cached Pick orientation.
                goal.tcp_pose.pose.orientation.w = 1.0
            else:
                goal.tcp_pose.pose.orientation = copy.deepcopy(orientation)
            goal.env_points = self.blackboard.env_points
            self.send_goal_request(goal)
            self.feedback_message = f"Sent Place TCP pose in {point.header.frame_id}"
        except Exception as exc:
            self.feedback_message = f"Failed to send Place: {exc}"
            self.logger.error(self.feedback_message)
            return Status.FAILURE


def _joint_move_goal(pose):
    # Existing decision poses describe xArm7. Named dictionaries support other
    # configured groups without imposing this legacy ordering on the server.
    if isinstance(pose, dict):
        names, values = list(pose), list(pose.values())
    else:
        names, values = [f"joint{i}" for i in range(1, 8)], list(pose)
    if (
        len(names) != len(values)
        or not names
        or any(
            not isinstance(value, (int, float)) or not math.isfinite(value)
            for value in values
        )
    ):
        raise ValueError("Expected a complete finite joint target in radians")
    goal = JointMove.Goal()
    goal.joint_names = names
    goal.positions = [float(value) for value in values]
    return goal


class _Tk27JointTask(_Tk27Task):
    def feedback_callback(self, msg):
        self.last_feedback_time = time.time()
        self.action_stage = msg.feedback.stage
        self.feedback_message = f"stage={msg.feedback.stage}: {msg.feedback.detail}"


class BtNode_MoveArm(_Tk27JointTask):
    """
    Moves arm through predefined scan poses (iterative).

    This node moves the arm to a sequence of predefined poses for scanning
    operations. It reads an index from the blackboard and moves to the
    corresponding pose, incrementing the index for the next iteration.

    Uses TK27 named joint targets with observed collision checking.
    """

    def __init__(
        self,
        name: str,
        #  arm_joint_pose: list[float]
        arm_pose_bb_key,
        action_name: str = "/manipulation/joint_move",
        add_octomap: bool = False,
        *,
        service_name: Optional[str] = None,  # deprecated alias for action_name
    ):
        # Legacy tree keyword; collision checks are always enabled.
        if service_name is not None:
            action_name = service_name
        if action_name == "arm_joint_service":
            action_name = "/manipulation/joint_move"
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
            goal = _joint_move_goal(self.arm_joint_pose)
            self.send_goal_request(goal)
            self.feedback_message = (
                f"Sent move arm joint goal for joints {self.arm_joint_pose}"
            )
        except Exception as e:
            self.feedback_message = f"Failed to send move arm joint goal; error: {e}"
            self.logger.error(f"Failed to send move arm joint goal; error: {e}")


class BtNode_MoveArmSingle(_Tk27JointTask):
    """
    Moves arm to a single predefined pose.

    This node moves the arm to a joint configuration read from the
    blackboard. Unlike BtNode_MoveArm, it does not iterate through
    multiple poses.

    Uses TK27 named joint targets with observed collision checking.
    """

    def __init__(
        self,
        name: str,
        arm_pose_bb_key: str,
        action_name: str = "/manipulation/joint_move",
        #  arm_joint_pose: list[float]
        add_octomap: bool = False,
        *,
        service_name: Optional[str] = None,  # deprecated alias for action_name
    ):
        # Legacy tree keyword; collision checks are always enabled.
        if service_name is not None:
            action_name = service_name
        if action_name == "arm_joint_service":
            action_name = "/manipulation/joint_move"
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
            goal = _joint_move_goal(arm_joint_pose)
            self.send_goal_request(goal)
            self.feedback_message = (
                f"Sent move arm joint goal for joints {arm_joint_pose}"
            )
        except Exception as e:
            self.feedback_message = f"Failed to send move arm joint goal; error: {e}"
            self.logger.error(f"Failed to send move arm joint goal; error: {e}")


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
        action_timeout_sec: float = 45.0,
    ):
        super().__init__(
            name, GripperCommand, action_name, None, wait_for_server_timeout_sec
        )
        if action_timeout_sec <= 0.0:
            raise ValueError("gripper action timeout must be positive")
        self.action_timeout_sec = float(action_timeout_sec)
        if open_gripper:
            self.goal = 0.0
        else:
            self.goal = 0.78

    def initialise(self):
        """Start a gripper goal with a bounded client-side wait.

        The gripper action does not expose the legacy feedback fields consumed
        by :class:`ActionHandler`, so ``feedback_callback`` below intentionally
        does not refresh ``feedback_timeout``.  Set the timeout after the base
        initialisation resets its per-goal state; this bounds an accepted goal
        whose result response is lost without changing mock-mode behaviour.
        """
        super().initialise()
        self.feedback_timeout = self.action_timeout_sec

    def goal_timeout(self):
        """Cancel a gripper goal that did not produce a result in time."""
        self.feedback_message = (
            f"gripper action timed out after {self.action_timeout_sec:.1f}s"
        )
        self.send_cancel_request()
        return pytree.common.Status.FAILURE

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
        if self.result_status == action_msgs.GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "Gripper action successful"
            return pytree.common.Status.SUCCESS
        status = self.status_strings.get(self.result_status, self.result_status)
        self.feedback_message = f"Gripper action failed with status {status}"
        return pytree.common.Status.FAILURE


class BtNode_PointTo(_Tk27JointTask):
    """
    Points the arm towards a specific person.

    This node moves the arm to point at a target person identified by ID.
    It reads the target's position from the blackboard and computes an
    appropriate arm configuration for pointing.

    Uses TK27 named joint targets with observed collision checking.
    """

    def __init__(
        self,
        name: str,
        bb_key_persons: str,
        bb_key_points: str,
        bb_key_init_pose: str,
        target_id: int = 0,
        action_name: str = "/manipulation/joint_move",
        *,
        service_name: Optional[str] = None,  # deprecated alias for action_name
        pan_bias: float = 0.0,
    ):
        # Legacy tree keyword; collision checks are always enabled.
        if service_name is not None:
            action_name = service_name
        if action_name == "arm_joint_service":
            action_name = "/manipulation/joint_move"
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
            pan = compute_point_to_pan(point.point.x, point.point.y, self.pan_bias)
            positions = list(self.blackboard.arm_joint_pose)
            positions[0] = pan
            goal = _joint_move_goal(positions)
            self.angle = pan
            self.send_goal_request(goal)
            self.feedback_message = f"Sent point to goal for joints {self.angle}"
        except Exception as e:
            self.feedback_message = f"Failed to send point to goal; error: {e}"
            self.logger.error(f"Failed to send point to goal; error: {e}")


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
            goal.placement_mode = int(
                self._read("placement_mode", PLACEMENT_MODE_FREE_SPACE)
            )
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
            self.feedback_message = (
                f"ScanAndPlace goal sent (mode {goal.placement_mode})"
            )
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
            record_event(
                self.blackboard,
                phase="",
                item=item,
                action="scan_and_place",
                outcome="failure",
                points_est=0,
            )
            self.feedback_message = (
                f"ScanAndPlace failed: status={self.result_status} err={err}"
            )
            return pytree.common.Status.FAILURE
        result = self.result_message.result
        try:
            self.blackboard.placed_at = result.placed_at
        except Exception:
            pass
        try:
            self.blackboard.place_status = {
                "status": result.status,
                "reason": result.error_msg,
            }
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


class BtNode_JointMoveAction(BtNode_MoveArmSingle):
    """Retain the existing tree class name for the shared joint-target adapter."""
