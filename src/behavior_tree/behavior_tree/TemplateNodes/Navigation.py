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
# Navigation Nodes Module
# =======================
#
# This module provides behavior tree nodes for robot navigation operations.
# All nodes inherit from either ServiceHandler or ActionHandler and include
# built-in mock mode support.
#
# Classes
# -------
# BtNode_GotoAction
#     Action-based navigation to a target pose using Nav2.
# BtNode_ConvertGraspPose
#     Converts a PointStamped to a grasp-compatible pose.
# BtNode_GoToLuggage
#     Navigates to a luggage position and returns the target pose.
# BtNode_GetOrientationAngle
#     Calls orientation_angle_service and writes the returned angle (rad) to a blackboard key.
#
# Mock Mode
# ---------
# All navigation nodes support mock mode via the mock_config.json settings.
# In mock mode, they can:
# - Wait for keyboard press before succeeding (KEYPRESS mode)
# - Auto-complete immediately (IMMEDIATE mode)
# - Use teleoperation-style control (TELEOP mode - for arm-related navigation)
#
# Usage
# -----
# >>> # Navigate to a pose stored in blackboard
# >>> nav_node = BtNode_GotoAction(
# ...     name="Go to kitchen",
# ...     key="target_pose"  # Blackboard key with PoseStamped
# ... )
#

from typing import Any
import py_trees
import rclpy
import tf2_geometry_msgs
from .BaseBehaviors import ServiceHandler
from .ActionBase import ActionHandler
from behavior_tree.config import is_node_mocked

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Quaternion

# from behavior_tree.messages import Goto, GotoGrasp, ComputeGrasp
from nav_msgs.msg import Odometry
from behavior_tree.messages import NavigateToPose, SetLuggagePose, ComputeGrasp, OrientationAngle
from behavior_tree.messages import GoToApproach
# from behavior_tree.messages import FindApproachPose
import math

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException

import action_msgs.msg as action_msgs  # GoalStatus

import time


class BtNode_GotoAction(ActionHandler):
    """
    Action-based navigation to a target pose using Nav2.

    This node uses ROS2 Nav2 action server to navigate the robot base to a
    target pose. The target can be either a PoseStamped or PointStamped
    stored on the blackboard.

    Attributes:
        key: Blackboard key containing the navigation goal.
    """
    def __init__(self, name: str, key: str, action_name: str = "navigate_to_pose", wait_for_server_timeout_sec: float = -3, action_timeout_ticks=0):
        super().__init__(name, NavigateToPose, action_name, key, wait_for_server_timeout_sec, action_timeout_ticks)
    
    def initialise(self):
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self.node)
        return super().initialise()

    def setup(self, **kwargs):
        self.tf_buffer = None
        return super().setup(**kwargs)

    def send_goal(self):
        # Handle mock mode - must check before doing any real action
        if self.mock_mode:
            self.feedback_message = "MOCK: Navigation goal sent (mock mode)"
            # Create a mock send_goal_future that appears done
            class MockFuture:
                def done(self):
                    return True
            self.send_goal_future = MockFuture()
            return
        
        transform = None
        # if self.tf_buffer is None:
        #     self.tf_buffer = Buffer()
        #     self.tf_listener = TransformListener(self.tf_buffer, self.node)
        # try:
        #     time_now = self.node.get_clock().now()
            
        #     transform = self.tf_buffer.lookup_transform(
        #                     target_frame='map',
        #                     source_frame=self.blackboard.goal.header.frame_id,
        #                     time=time_now
        #                 )
        # except:
        #     self.node.get_logger().warn(f'Failed to lookup transform from {self.blackboard.goal.header.frame_id} to "map" at (now = {time_now}).')
        #     transform = None
        try:
            goal = NavigateToPose.Goal()
            if not self.blackboard.exists("goal"):
                self.feedback_message = "No goal found in blackboard"
                self.node.get_logger().warn("No goal found in blackboard, setting to identity pose")
                return
            if isinstance(self.blackboard.goal, PoseStamped):
                if self.blackboard.goal.header.frame_id != 'map' and transform is not None:
                    # assert False
                    goal.pose.pose = tf2_geometry_msgs.do_transform_pose(self.blackboard.goal.pose, transform)
                    goal.pose.header.frame_id = 'map'
                    self.blackboard.goal = goal.pose.pose
                else:
                    goal.pose = self.blackboard.goal
            elif isinstance(self.blackboard.goal, PointStamped):
                # assert False
                goal.pose = PoseStamped(header=self.blackboard.goal.header, pose=Pose(position=self.blackboard.goal.point, orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)))
                if transform is not None:
                    goal.pose.pose = tf2_geometry_msgs.do_transform_pose(goal.pose.pose, transform)
                    goal.pose.header.frame_id = 'map'
            else:
                print("Unknown goal type for navigation")
                assert False
            if goal.pose.header.frame_id == '':
                self.feedback_message = "Goal pose header frame_id is empty. INVALID!"
                self.node.get_logger().warn("Goal pose header frame_id is empty. INVALID!")
                return
            self.send_goal_request(goal)
            self.goal = goal
            try:
                self.feedback_message = f"sent goal request for pose {goal.pose.pose.position.x:2f}, {goal.pose.pose.position.y:2f}, {goal.pose.pose.position.z:2f}"
            except Exception as e:
                self.feedback_message = "sent goal request"
            print('goal = ', str(self.goal))
        except KeyError:
            self.node.get_logger().warn("Send goal failed!")
    
    def process_result(self):
        # for navigation only, where the action can return all sorts of results while it's at it
        if self.result_status == action_msgs.GoalStatus.STATUS_ABORTED:  # noqa
            self.feedback_message = "action aborted"
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS
    
    def feedback_callback(self, msg):
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


# class BtNode_FindApproachPose(ServiceHandler):
#     """
#     Compute a free, reachable, target-facing approach pose offset from a target point.

#     Reads a PoseStamped or PointStamped from the blackboard at ``bb_target_key``
#     (only the position/point is used; orientation is ignored — the planner
#     computes orientation to face the target). Calls the ``find_approach_pose``
#     service (``tinker_nav_msgs/srv/FindApproachPose``) and writes the resulting
#     PoseStamped to ``bb_approach_pose_key``. The downstream ``BtNode_GotoAction``
#     can then drive to that pose.
    # """
    # Mock mode: synthesizes a PoseStamped 0.7 m offset in the target frame's -x
    # direction and writes it directly. Skips the service call entirely so the
    # behavior tree can be tested without nav2 running.

    # Reachability is opt-in (``check_reachability=False`` by default). Nav2 will
    # refuse unreachable goals at NavigateToPose time anyway, and pre-validation
    # via ``ComputePathToPose`` adds up to ``top_k_reachability *
    # reachability_timeout_sec`` (≈4 s) per call. Set ``check_reachability=True``
    # explicitly when fast-failing on unreachable poses matters more than latency.
    # """

    # def __init__(self,
    #              name: str,
    #              bb_target_key: str,
    #              bb_approach_pose_key: str,
    #              desired_distance: float = 0.7,
    #              min_distance: float = 0.45,
    #              max_distance: float = 1.2,
    #              num_angles: int = 16,
    #              check_reachability: bool = False,
    #              preferred_yaw_rad: float = float('nan'),
    #              facing_yaw_offset_rad: float = 0.0,
    #              service_name: str = "find_approach_pose"):
    #     super().__init__(name, service_name, FindApproachPose)
    #     self.bb_target_key = bb_target_key
    #     self.bb_approach_pose_key = bb_approach_pose_key
    #     self.desired_distance = float(desired_distance)
    #     self.min_distance = float(min_distance)
    #     self.max_distance = float(max_distance)
    #     self.num_angles = int(num_angles)
    #     self.check_reachability = bool(check_reachability)
    #     self.preferred_yaw_rad = float(preferred_yaw_rad)
    #     self.facing_yaw_offset_rad = float(facing_yaw_offset_rad)

#     def setup(self, **kwargs):
#         ServiceHandler.setup(self, **kwargs)
#         self.bb_read_client = self.attach_blackboard_client(name=f"FindApproachPoseRead")
#         self.bb_write_client = self.attach_blackboard_client(name=f"FindApproachPoseWrite")
#         self.bb_read_client.register_key(
#             "source",
#             access=py_trees.common.Access.READ,
#             remap_to=py_trees.blackboard.Blackboard.absolute_name("/", self.bb_target_key)
#         )
#         self.bb_write_client.register_key(
#             "dest",
#             access=py_trees.common.Access.WRITE,
#             remap_to=py_trees.blackboard.Blackboard.absolute_name("/", self.bb_approach_pose_key)
#         )
#         self.logger.debug(f"Setup BtNode_FindApproachPose")

#     def initialise(self) -> None:
#         super().initialise()
#         if self.mock_mode:
#             target = None
#             try:
#                 target = self.bb_read_client.source
#             except Exception:
#                 target = None
#             self.bb_write_client.dest = self._mock_offset_pose(target)
#             self.feedback_message = "MOCK: synthesized approach pose"
#             print(f"📍 MOCK FIND APPROACH POSE: synthesized offset")
#             return

#         try:
#             source = self.bb_read_client.source
#         except Exception as e:
#             self.feedback_message = f"Failed to read target from '{self.bb_target_key}': {e}"
#             self.response = None
#             return

#         target_point = self._coerce_to_point_stamped(source)
#         if target_point is None:
#             self.feedback_message = f"Unsupported target type: {type(source).__name__}"
#             self.response = None
#             return

#         request = FindApproachPose.Request()
#         request.target = target_point
#         request.desired_distance = float(self.desired_distance)
#         request.min_distance = float(self.min_distance)
#         request.max_distance = float(self.max_distance)
#         request.num_angles = int(self.num_angles)
#         request.check_reachability = bool(self.check_reachability)
#         request.preferred_yaw_rad = float(self.preferred_yaw_rad)
#         request.facing_yaw_offset_rad = float(self.facing_yaw_offset_rad)
#         request.timeout_sec = 0.0
#         # robot_pose_override empty header signals "use TF"
#         self.response = self.call_service_async(request)
#         self.feedback_message = (
#             f"Requested approach pose for target ({target_point.point.x:.2f}, "
#             f"{target_point.point.y:.2f}) in {target_point.header.frame_id}"
#         )

#     def update(self):
#         if self.mock_mode:
#             return self.wait_for_keypress_in_mock()

#         if self.response is None:
#             return py_trees.common.Status.FAILURE
#         if not self.response.done():
#             self.feedback_message = "Computing approach pose..."
#             return py_trees.common.Status.RUNNING

#         result = self.response.result()
#         if result is None:
#             self.feedback_message = "Service call returned None"
#             return py_trees.common.Status.FAILURE
#         if result.status == 0:
#             self.bb_write_client.dest = result.pose
#             self.feedback_message = (
#                 f"Approach pose at d={result.chosen_distance:.2f}, "
#                 f"reachable={result.reachable}"
#             )
#             return py_trees.common.Status.SUCCESS
#         self.feedback_message = f"FindApproachPose failed: status={result.status} {result.errormsg}"
#         return py_trees.common.Status.FAILURE

#     @staticmethod
#     def _coerce_to_point_stamped(source):
#         """Accept PointStamped, PoseStamped, or anything with `.point`/`.pose.position`."""
#         if isinstance(source, PointStamped):
#             return source
#         if isinstance(source, PoseStamped):
#             ps = PointStamped()
#             ps.header = source.header
#             ps.point.x = source.pose.position.x
#             ps.point.y = source.pose.position.y
#             ps.point.z = source.pose.position.z
#             return ps
#         return None

#     @staticmethod
#     def _mock_offset_pose(target) -> 'PoseStamped':
#         """Synthetic 0.7 m offset along -x in target frame, identity orientation, target frame preserved."""
#         out = PoseStamped()
#         if target is None:
#             out.header.frame_id = "map"
#             out.pose.orientation.w = 1.0
#             return out
#         # Pull (x, y, z) + frame_id from either type
#         if hasattr(target, "point"):
#             out.header = target.header
#             out.pose.position.x = float(target.point.x) - 0.7
#             out.pose.position.y = float(target.point.y)
#             out.pose.position.z = float(target.point.z)
#         elif hasattr(target, "pose"):
#             out.header = target.header
#             out.pose.position.x = float(target.pose.position.x) - 0.7
#             out.pose.position.y = float(target.pose.position.y)
#             out.pose.position.z = float(target.pose.position.z)
#         else:
#             out.header.frame_id = "map"
#         out.pose.orientation.w = 1.0
#         return out

class BtNode_Approach(ActionHandler):
    """Drive the robot to a target-facing approach pose via the
    ``go_to_approach`` action server (``tinker_nav_msgs/action/GoToApproach``).

    Reads ``bb_target_key`` (PointStamped or PoseStamped — PoseStamped is
    coerced to PointStamped) and delegates the full
    "candidate → reachability → nav → on-fail recompute → retry" flow to the
    action server. The server runs two attempts internally:

    1. Pure geometric candidate (``target − desired·unit(target − robot)``)
       sent straight to ``/navigate_to_pose``.
    2. On nav abort/stall, recompute candidates against the *current* global
       costmap, gate via reachability, retry with the first reachable.

    Distance / angle kwargs default to 0 so the server applies its own
    defaults (desired=0.7 m, min=0.45 m, max=1.2 m, num_angles=16, timeout=60 s).

    Mock mode: skip navigation entirely. ``send_goal`` short-circuits with a
    fake done future so the BT advances without contacting any action server.
    """

    def __init__(self,
                 name: str,
                 bb_target_key: str,
                 desired_distance: float = 0.0,
                 min_distance: float = 0.0,
                 max_distance: float = 0.0,
                 num_angles: int = 0,
                 preferred_yaw_rad: float = float("nan"),
                 facing_yaw_offset_rad: float = 0.0,
                 timeout_sec: float = 0.0,
                 debug: bool = False,
                 action_name: str = "go_to_approach",
                 wait_for_server_timeout_sec: float = -3.0,
                 action_timeout_ticks: int = 0):
        super().__init__(name, GoToApproach, action_name, bb_target_key,
                         wait_for_server_timeout_sec, action_timeout_ticks)
        self.desired_distance = float(desired_distance)
        self.min_distance = float(min_distance)
        self.max_distance = float(max_distance)
        self.num_angles = int(num_angles)
        self.preferred_yaw_rad = float(preferred_yaw_rad)
        self.facing_yaw_offset_rad = float(facing_yaw_offset_rad)
        self.timeout_sec = float(timeout_sec)
        self.debug = bool(debug)

    def send_goal(self):
        # Mock policy: skip the navigation process entirely.
        if self.mock_mode:
            self.feedback_message = "MOCK: GoToApproach skipped (mock mode)"
            class MockFuture:
                def done(self):
                    return True
            self.send_goal_future = MockFuture()
            return

        if not self.blackboard.exists("goal"):
            self.feedback_message = "No target found in blackboard"
            self.node.get_logger().warn(
                f"{self.name}: blackboard target key not set"
            )
            return

        target = self._coerce_to_point_stamped(self.blackboard.goal)
        if target is None:
            self.feedback_message = (
                f"{self.name}: blackboard target unsupported type "
                f"{type(self.blackboard.goal).__name__}"
            )
            self.node.get_logger().warn(self.feedback_message)
            return
        if target.header.frame_id == "":
            self.feedback_message = (
                f"{self.name}: target frame_id empty, cannot send GoToApproach"
            )
            self.node.get_logger().warn(self.feedback_message)
            return

        goal = GoToApproach.Goal()
        goal.target = target
        goal.desired_distance = self.desired_distance
        goal.min_distance = self.min_distance
        goal.max_distance = self.max_distance
        goal.num_angles = self.num_angles
        goal.preferred_yaw_rad = self.preferred_yaw_rad
        goal.facing_yaw_offset_rad = self.facing_yaw_offset_rad
        goal.timeout_sec = self.timeout_sec
        goal.debug = self.debug

        self.send_goal_request(goal)
        self.goal = goal
        self.feedback_message = (
            f"sent GoToApproach target=({target.point.x:.2f}, "
            f"{target.point.y:.2f}) frame='{target.header.frame_id}'"
        )

    def process_result(self):
        # GoToApproach reports semantic failures (UNREACHABLE, NAV_FAILED, ...)
        # as a succeeded action with non-zero result.status, so inspect the
        # payload rather than relying on GoalStatus alone.
        res = self.result_message.result if hasattr(self.result_message, "result") \
            else self.result_message
        if res is None:
            self.feedback_message = "GoToApproach returned no result"
            return py_trees.common.Status.FAILURE
        status = int(getattr(res, "status", -1))
        if status == 0:
            self.feedback_message = (
                f"approached: phase={res.phase_at_exit}, "
                f"nav_attempts={res.nav_attempts}, d={res.chosen_distance:.2f}"
            )
            return py_trees.common.Status.SUCCESS
        self.feedback_message = (
            f"GoToApproach FAILED status={status} "
            f"phase={getattr(res, 'phase_at_exit', '')} "
            f"msg='{getattr(res, 'errormsg', '')}'"
        )
        return py_trees.common.Status.FAILURE

    def process_feedback(self, feedback):
        try:
            phase = getattr(feedback, "phase", "")
            d = getattr(feedback, "distance_remaining", float("nan"))
            self.feedback_message = f"GoToApproach phase={phase} d={d:.2f}"
        except Exception:
            pass

    def feedback_callback(self, msg):
        # GoToApproach.Feedback has no `delay_limit` field, so the base
        # ActionHandler.feedback_callback would AttributeError. Mirror the
        # BtNode_GotoAction override: stamp the time, set a generous
        # feedback_timeout so the action isn't killed for "no progress",
        # and route the payload to process_feedback.
        feedback = msg.feedback
        self.last_feedback_time = time.time()
        self.feedback_timeout = 1000
        self.action_status = 0
        self.process_feedback(feedback)

    @staticmethod
    def _coerce_to_point_stamped(source):
        """Accept PointStamped or PoseStamped; return PointStamped or None."""
        if isinstance(source, PointStamped):
            return source
        if isinstance(source, PoseStamped):
            ps = PointStamped()
            ps.header = source.header
            ps.point.x = float(source.pose.position.x)
            ps.point.y = float(source.pose.position.y)
            ps.point.z = float(source.pose.position.z)
            return ps
        return None


class BtNode_CaptureCurrentPose(py_trees.behaviour.Behaviour):
    """Look up the robot's current pose via TF and write it to a blackboard key.

    Generic primitive for "remember where I am right now". Restaurant uses
    this to snapshot the operator-placed start pose as the barman/anchor
    position, instead of hardcoding map coordinates that go stale across
    maps and competition setups.

    Mock mode: writes a sentinel ``PoseStamped(frame_id="map", w=1)`` so any
    downstream ``BtNode_GotoAction`` does not crash on an empty frame_id.
    """

    def __init__(self,
                 name: str,
                 bb_key: str,
                 source_frame: str = "base_link",
                 target_frame: str = "map",
                 tf_timeout_sec: float = 1.0):
        super().__init__(name=name)
        self.bb_key = bb_key
        self.source_frame = source_frame
        self.target_frame = target_frame
        self.tf_timeout_sec = float(tf_timeout_sec)
        self.mock_mode = is_node_mocked(self.__class__.__name__)
        self.node = None
        self.tf_buffer = None
        self.tf_listener = None
        self.bb_write = None
        self.bb_write = self.attach_blackboard_client(name="CaptureCurrentPoseWrite")
        self.bb_write.register_key(
            "dest",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", self.bb_key)
        )
        self.time_now = None
        self.wait_ticks = 0
    
    def initialise(self):
        self.time_now = self.node.get_clock().now()
        self.wait_ticks = 0
        return super().initialise()

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        if self.node is not None and not self.mock_mode:
            self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10))
            self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.logger.debug(f"Setup BtNode_CaptureCurrentPose -> {self.bb_key}")
        return super().setup(**kwargs)

    def update(self):
        self.wait_ticks += 1
        if self.mock_mode:
            ps = PoseStamped()
            ps.header.frame_id = self.target_frame
            ps.pose.orientation.w = 1.0
            self.bb_write.dest = ps
            self.feedback_message = f"MOCK: synthetic origin pose at {self.target_frame}"
            print(f"📌 MOCK CAPTURE POSE: wrote synthetic origin to '{self.bb_key}'")
            return py_trees.common.Status.SUCCESS

        if self.tf_buffer is None:
            self.feedback_message = "TF buffer not initialised (setup not called?)"
            return py_trees.common.Status.FAILURE

        try:
            # rclpy.time.Time() (zero stamp) = "latest available transform".
            # Passing now() risks ExtrapolationException when amcl/robot_state_publisher
            # lag the wall clock by a few ms.
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec),
            )
        except Exception as e:
            self.feedback_message = f"TF lookup '{self.target_frame}' <- '{self.source_frame}' failed: {e}"
            if self.wait_ticks < 10:
                return py_trees.common.Status.RUNNING
            else:
                return py_trees.common.Status.FAILURE

        ps = PoseStamped()
        ps.header.frame_id = self.target_frame
        ps.header.stamp = self.time_now.to_msg()
        ps.pose.position.x = tf.transform.translation.x
        ps.pose.position.y = tf.transform.translation.y
        ps.pose.position.z = tf.transform.translation.z
        ps.pose.orientation = tf.transform.rotation
        self.bb_write.dest = ps
        self.feedback_message = (
            f"Captured pose ({ps.pose.position.x:.2f}, {ps.pose.position.y:.2f}) "
            f"in '{self.target_frame}' -> '{self.bb_key}'"
        )
        return py_trees.common.Status.SUCCESS


class BtNode_ConvertGraspPose(ServiceHandler):
    """
    Converts a PointStamped to a grasp-compatible pose.

    This node takes a 3D point from the blackboard and calls a service to
    compute an appropriate arm pose for grasping. The resulting pose is
    stored back on the blackboard.
    """
    def __init__(self, name: str, bb_source_key: str, bb_dest_key: str, service_name: str = "compute_grasp_pos"):
        super().__init__(name, service_name, ComputeGrasp)
        self.bb_source_key = bb_source_key
        self.bb_dest_key = bb_dest_key

    def setup(self, **kwargs):
        ServiceHandler.setup(self, **kwargs)
        self.bb_read_client = self.attach_blackboard_client(name=f"ConvertGraspPoseRead")
        self.bb_write_client = self.attach_blackboard_client(name=f"ConvertGraspPoseWrite")
        self.bb_read_client.register_key(
            "source",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", self.bb_source_key)
        )
        self.bb_write_client.register_key(
            "dest",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", self.bb_dest_key)
        )
        self.logger.debug(f"Setup ConvertGraspPose")

    def initialise(self) -> None:
        super().initialise()

        # Handle mock mode
        if self.mock_mode:
            from geometry_msgs.msg import PoseStamped
            mock_pose = PoseStamped()
            self.bb_write_client.dest = mock_pose
            self.feedback_message = "MOCK: Converted grasp pose"
            print(f"🎯 MOCK CONVERT GRASP POSE: Conversion complete")
            return
            
        try:
            self.source_point = self.bb_read_client.source
            assert isinstance(self.source_point, PointStamped)
        except Exception as e:
            self.feedback_message = f"Failed to read PointStamped from key '{self.bb_source_key}': {e}"
            raise e

        request = ComputeGrasp.Request()
        request.target = self.source_point
        self.response = self.call_service_async(request)
        self.feedback_message = f"Initialized ConvertGraspPose for point in {self.bb_source_key}"

    def update(self):
        # Handle mock mode
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
            
        if self.response is None:
            return py_trees.common.Status.FAILURE
            
        self.logger.debug(f"Update ConvertGraspPose")
        if self.response.done():
            result = self.response.result()
            if result:
                self.feedback_message = "Successfully converted grasp pose"
                self.bb_write_client.dest = result.target
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = "Service call to compute_grasp_pos failed"
                return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = "Still computing grasp pose..."
            return py_trees.common.Status.RUNNING


class BtNode_GoToLuggage(ServiceHandler):
    """
    Navigates to a luggage position and returns the target pose.

    This node takes a point representing luggage location, calls a service
    to compute an appropriate navigation pose for approaching the luggage,
    and stores the result on the blackboard.
    """
    def __init__(self, name: str, bb_src_key: str, bb_target_key, service_name: str = "set_luggage_pose"):
        super().__init__(name, service_name, SetLuggagePose)

        self.bb_src_key = bb_src_key
        self.bb_target_key = bb_target_key
    
    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)
        self.bb_read_client = self.attach_blackboard_client(name=f"GoToLuggageRead")
        self.bb_write_client = self.attach_blackboard_client(name=f"GoToLuggageWrite")
        self.bb_write_client.register_key("target",
                                         access=py_trees.common.Access.WRITE,
                                         remap_to=py_trees.blackboard.Blackboard.absolute_name("/", self.bb_target_key))
        self.bb_read_client.register_key("source",
                                         access=py_trees.common.Access.READ,
                                         remap_to=py_trees.blackboard.Blackboard.absolute_name("/", self.bb_src_key))

    def initialise(self) -> None:
        """
        Called when the node is visited
        """
        super().initialise()

        # Handle mock mode
        if self.mock_mode:
            from geometry_msgs.msg import PoseStamped
            mock_pose = PoseStamped()
            self.bb_write_client.target = mock_pose
            self.feedback_message = "MOCK: Set luggage pose"
            print(f"🧳 MOCK GO TO LUGGAGE: Luggage pose set")
            return
            
        try:
            self.point = self.bb_read_client.source
            assert isinstance(self.point, PointStamped)
        except Exception as e:
            raise e

        request = SetLuggagePose.Request()
        request.point = self.point
        self.response = self.call_service_async(request)

        self.feedback_message = f"Initialized GotoLuggage for {self.point.point.x}, {self.point.point.y}, {self.point.point.z}"


    def update(self):
        # Handle mock mode
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
            
        if self.response is None:
            return py_trees.common.Status.FAILURE
            
        self.logger.debug(f"Update GotoLuggage")
        if self.response.done():
            if self.response.result().status > 0:
                self.feedback_message = f"success"
                self.bb_write_client.target = self.response.result().pose
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = f"GotoLuggage failed: {self.response.result().status}: {self.response.result().errormsg}"
                return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = "Still scanning..."
            return py_trees.common.Status.RUNNING


class BtNode_GetOrientationAngle(ServiceHandler):
    """
    Calls ``orientation_angle_service`` and writes the returned angle (rad) to a blackboard key.

    The service (``tinker_nav_msgs/srv/OrientationAngle``, advertised by
    ``tk26_navigation/orientation_angle_service``) reads ``/amcl_pose`` and
    returns the relative bearing from the robot to a server-side hardcoded
    target (currently SOFA), already negated for the pan_tilt sign convention.

    Note: the srv has no status field. The server returns ``angle = 0.0`` when
    it cannot obtain a pose within the budget, which is indistinguishable from
    a genuine 0 rad result. The node therefore returns SUCCESS as long as the
    service responds.
    """

    def __init__(
        self,
        name: str,
        bb_dest_key: str,
        max_try: int = 3,
        timeout: float = 2.0,
        service_name: str = "orientation_angle_service",
    ):
        super().__init__(name, service_name, OrientationAngle)
        self.bb_dest_key = bb_dest_key
        self.max_try = max_try
        self.timeout = timeout

    def setup(self, **kwargs):
        ServiceHandler.setup(self, **kwargs)
        self.bb_write_client = self.attach_blackboard_client(name="GetOrientationAngleWrite")
        self.bb_write_client.register_key(
            "angle",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", self.bb_dest_key),
        )
        self.logger.debug(f"Setup GetOrientationAngle -> '{self.bb_dest_key}'")

    def initialise(self) -> None:
        super().initialise()

        if self.mock_mode:
            self.bb_write_client.angle = 0.0
            self.feedback_message = "MOCK: orientation angle = 0.000 rad"
            print("🧭 MOCK GET ORIENTATION ANGLE: wrote 0.0 rad")
            return

        request = OrientationAngle.Request()
        request.max_try = self.max_try
        request.timeout = self.timeout
        self.response = self.call_service_async(request)
        self.feedback_message = (
            f"Initialized GetOrientationAngle (max_try={self.max_try}, timeout={self.timeout}s)"
        )

    def update(self):
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()

        if self.response is None:
            return py_trees.common.Status.FAILURE

        if self.response.done():
            angle = self.response.result().angle
            self.bb_write_client.angle = angle
            self.feedback_message = f"orientation angle = {angle:.3f} rad -> '{self.bb_dest_key}'"
            return py_trees.common.Status.SUCCESS

        self.feedback_message = "waiting for orientation_angle_service..."
        return py_trees.common.Status.RUNNING


# class BtNode_GoToLuggage(ServiceHandler):
#     def __init__(self, name: str, bb_key: str, service_name: str = "set_luggage_pose"):
#         super().__init__(name, service_name, SetLuggagePose)

#         self.bb_key = bb_key
    
#     def setup(self, **kwargs):
#         """
#         setup for the node, recursively called with tree.setup()
#         """
#         ServiceHandler.setup(self, **kwargs)
#         self.bb_read_client = self.attach_blackboard_client(name=f"GoToLuggage")
#         self.bb_read_client.register_key(self.bb_key,
#                                          access=py_trees.common.Access.WRITE,
#                                          remap_to=py_trees.blackboard.Blackboard.absolute_name("/", self.bb_key))

#         self.logger.debug(f"Setup Go To Luggage, reading key {self.bb_key}")

#     def initialise(self) -> None:
#         """
#         Called when the node is visited
#         """
#         try:
#             self.point = self.bb_read_client.get(self.bb_key)
#             assert isinstance(self.point, PointStamped)
#         except Exception as e:
#             self.feedback_message = f"GoToLuggage reading point failed"
#             raise e

#         request = SetLuggagePose.Request()
#         request.point = self.point
#         self.response = self.client.call_async(request)

#         self.feedback_message = f"Initialized GotoLuggage for {self.point.point.x}, {self.point.point.y}, {self.point.point.z}"


#     def update(self):
#         self.logger.debug(f"Update GotoLuggage")
#         if self.response.done():
#             if self.response.result().status > 0:
#                 self.feedback_message = f"success"
#                 return py_trees.common.Status.SUCCESS
#             else:
#                 self.feedback_message = f"GotoLuggage failed: {self.response.result().status}: {self.response.result().errormsg}"
#                 return py_trees.common.Status.FAILURE
#         else:
#             self.feedback_message = "Still scanning..."
#             return py_trees.common.Status.RUNNING


# class BtNode_Goto(ServiceHandler):
#     def __init__(self, 
#                 name: str,
#                 bb_source: str,
#                 service_name : str = "goto",
#                 target : PoseStamped = None
#                 ):
#         """
#         executed when creating tree diagram, therefor very minimal
#         Args:
#             name: the name of the pytree node
#             bb_source: path to the key in blackboard containing a geometry_msgs/PoseStamped object of the pos of trash can
#             service_name: name of the service of type tinker_decision_msgs/Drop     
#         """
#         super(BtNode_Goto, self).__init__(name, service_name, Goto)
#         self.bb_source = bb_source
#         self.bb_read_client = None
#         self.target = target
#         if target is not None:
#             assert isinstance(self.target, PoseStamped)
        

#     def setup(self, **kwargs):
#         """
#         setup for the node, recursively called with tree.setup()
#         """
#         ServiceHandler.setup(self, **kwargs)

#         if self.target is None:
#             self.bb_read_client = self.attach_blackboard_client(name="Goto Read")
#             self.bb_read_client.register_key(self.bb_source, access=py_trees.common.Access.READ)

#             # debugger info (shown with DebugVisitor)
#             self.logger.debug(f"Setup Goto, reading from {self.bb_source}")
#         else:
#             self.logger.debug(f"Setup Goto from fixed input {self.target}")

#     def initialise(self) -> None:
#         """
#         Called when the node is visited
#         """

#         if self.target is None:
#             try:
#                 self.target = self.bb_read_client.get(self.bb_source)
#                 assert isinstance(self.target, PoseStamped)
#             except Exception as e:
#                 self.feedback_message = f"Goto reading target pose failed"
#                 raise e

#         self.logger.debug(f"Initialized Goto for pose {self.target}")

#         request = Goto.Request()
#         request.target = self.target
#         # setup things that needs to be cleared
#         self.response = self.client.call_async(request)

#         self.feedback_message = f"Initialized Goto"

#     def update(self):
#         self.logger.debug(f"Update Goto")
#         if self.response.done():
#             if self.response.result().status == 0:
#                 self.feedback_message = f"Goto Successful"
#                 return py_trees.common.Status.SUCCESS
#             else:
#                 self.feedback_message = f"Goto failed with status {self.response.result().status}: {self.response.result().error_msg}"
#                 return py_trees.common.Status.FAILURE
#         else:
#             self.feedback_message = "Still navigating to target pose..."
#             return py_trees.common.Status.RUNNING


# class BtNode_GotoGrasp(ServiceHandler):
#     def __init__(self, 
#                 name: str,
#                 bb_source: str,
#                 service_name : str = "go_to_grasp",
#                 target : PointStamped = None
#                 ):
#         """
#         executed when creating tree diagram, therefor very minimal
#         Args:
#             name: the name of the pytree node
#             bb_source: path to the key in blackboard containing a geometry_msgs/PoseStamped object of the pos of trash can
#             service_name: name of the service of type tinker_decision_msgs/Drop     
#         """
#         super(BtNode_GotoGrasp, self).__init__(name, service_name, GotoGrasp)
#         self.bb_source = bb_source
#         self.bb_read_client = None
#         self.target = target
#         self.read = True
#         if target is not None:
#             assert isinstance(self.target, PointStamped)
#             self.read = False


#     def setup(self, **kwargs):
#         """
#         setup for the node, recursively called with tree.setup()
#         """
#         ServiceHandler.setup(self, **kwargs)

#         if self.read:
#             self.bb_read_client = self.attach_blackboard_client(name="GotoGrasp Read")
#             self.bb_read_client.register_key(self.bb_source, access=py_trees.common.Access.READ)

#             # debugger info (shown with DebugVisitor)
#             self.logger.debug(f"Setup GotoGrasp, reading from {self.bb_source}")
#         else:
#             self.logger.debug(f"Setup GotoGrasp from fixed point {self.target}")

#     def initialise(self) -> None:
#         """
#         Called when the node is visited
#         """
#         if self.read:
#             try:
#                 self.target = self.bb_read_client.get(self.bb_source)
#                 assert isinstance(self.target, PointStamped)
#                 self.read = True
#             except Exception as e:
#                 self.feedback_message = f"GotoGrasp reading target point failed"
#                 raise e

#         self.logger.debug(f"Initialized GotoGrasp for target point {self.target}")

#         request = GotoGrasp.Request()
#         request.target = self.target
#         # setup things that needs to be cleared
#         self.response = self.client.call_async(request)

#         self.feedback_message = f"Initialized GotoGrasp, read set to {self.read}"

#     def update(self):
#         self.logger.debug(f"Update GotoGrasp")
#         if self.response.done():
#             self.feedback_message = f"GotoGrasp returned with status {self.response.result().status}"
#             return py_trees.common.Status.SUCCESS
#             # if self.response.result().status == 0:
#             #     self.feedback_message = f"GotoGrasp Successful"
#             #     return py_trees.common.Status.SUCCESS
#             # else:
#             #     self.feedback_message = f"GotoGrasp failed with status {self.response.result().status}: {self.response.result().error_msg}"
#             #     return py_trees.common.Status.FAILURE
#         else:
#             self.feedback_message = f"Still navigating to grasping pose {self.target} from {self.bb_source} (read set to {self.read})"
#             return py_trees.common.Status.RUNNING


# class BtNode_CalcGraspPose(ServiceHandler):
#     def __init__(self, 
#                 name: str,
#                 bb_source: str,
#                 bb_dest:str,
#                 service_name : str = "compute_grasp_pos",
#                 target : PointStamped = None
#                 ):
#         """
#         executed when creating tree diagram, therefor very minimal
#         Args:
#             name: the name of the pytree node
#             bb_source: path to the key in blackboard containing a geometry_msgs/PoseStamped object of the pos of trash can
#             service_name: name of the service of type tinker_decision_msgs/Drop     
#         """
#         super(BtNode_CalcGraspPose, self).__init__(name, service_name, ComputeGrasp)
#         self.bb_source = bb_source
#         self.bb_read_client = None
#         self.bb_dest = bb_dest
#         self.target = target
#         self.read = True
#         if target is not None:
#             assert isinstance(self.target, PointStamped)
#             self.read = False


#     def setup(self, **kwargs):
#         """
#         setup for the node, recursively called with tree.setup()
#         """
#         ServiceHandler.setup(self, **kwargs)

#         self.bb_write_client = self.attach_blackboard_client(name="CalcGraspPose Write")
#         self.bb_write_client.register_key(self.bb_dest, access=py_trees.common.Access.WRITE)

#         if self.read:
#             self.bb_read_client = self.attach_blackboard_client(name="CalcGraspPose Read")
#             self.bb_read_client.register_key(self.bb_source, access=py_trees.common.Access.READ)

#             # debugger info (shown with DebugVisitor)
#             self.logger.debug(f"Setup CalcGraspPose, reading from {self.bb_source}")
#         else:
#             self.logger.debug(f"Setup CalcGraspPose from fixed point {self.target}")
        
#         self._tf_buffer = Buffer()
#         self._tf_listener = TransformListener(self._tf_buffer, self.node)

#     def initialise(self) -> None:
#         """
#         Called when the node is visited
#         """
#         if self.read:
#             try:
#                 self.target = self.bb_read_client.get(self.bb_source)
#                 assert isinstance(self.target, PointStamped)
#                 self.read = True
#             except Exception as e:
#                 self.feedback_message = f"CalcGraspPose reading target point failed"
#                 raise e

#         self.logger.debug(f"Initialized CalcGraspPose for target point {self.target}")

#         try:
#             transform = self._tf_buffer.lookup_transform(
#                                 target_frame="map",
#                                 source_frame=self.target.header.frame_id[1 if self.target.header.frame_id[0] == '/' else 0:],
#                                 time=rclpy.time.Time()
#                             )
#         except LookupException as e:
#             assert False, "Failed to lookup transform"
            
#         map_point = tf2_geometry_msgs.do_transform_point(self.target, transform)

#         request = ComputeGrasp.Request()
#         request.target = map_point
#         # setup things that needs to be cleared
#         self.response = self.client.call_async(request)

#         self.feedback_message = f"Initialized CalcGraspPose, read set to {self.read}"

#     def update(self):
#         self.logger.debug(f"Update CalcGraspPose")
#         if self.response.done():
#             self.feedback_message = f"CalcGraspPose finished"
#             goal_pose = self.response.result().target
#             self.bb_write_client.set(self.bb_dest, goal_pose, overwrite=True)

#             return py_trees.common.Status.SUCCESS
#         else:
#             self.feedback_message = f"Still calculating grasping pose {self.target} from {self.bb_source} (read set to {self.read})"
#             return py_trees.common.Status.RUNNING
