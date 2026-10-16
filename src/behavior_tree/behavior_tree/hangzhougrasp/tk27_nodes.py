"""TK27 manipulation leaves: Pick, MoveToSavedPose and a strictly checked gripper open.

Modelled on HomeServiceHangzhou.ros_nodes.Tk27SavedPose. Every leaf fails unless
the server reports success, and keeps the server's message in its feedback.
"""

import time

import py_trees
from action_msgs.msg import GoalStatus

from behavior_tree.TemplateNodes.ActionBase import ActionHandler

S = py_trees.common.Status


class _Tk27Leaf(ActionHandler):
    """Shared real-mode lifecycle for the TK27 leaves.

    - Every FAILURE, including the base class's "goal rejected" path, keeps
      its reason in feedback and logs one ERROR line.
    - A failed goal request fails the leaf instead of leaving it RUNNING.
    - A goal that already finished is not cancelled on stop(), so the
      server's failure message survives in feedback.
    """
    _logged = False
    _request_error = None

    def initialise(self):
        self._logged = False
        self._request_error = None
        super().initialise()

    def _fail(self, message):
        self.feedback_message = message
        if not self._logged:
            self._logged = True
            self.logger.error(f"{self.name} [{self.action_name}]: {message}")
        return S.FAILURE

    def goal_response_callback(self, future):
        super().goal_response_callback(future)
        if future.result() is None:
            self._request_error = self.feedback_message

    def update(self):
        if self._request_error is not None:
            return self._fail(self._request_error)
        status = super().update()
        if status == S.FAILURE:
            return self._fail(self.feedback_message)
        return status

    def terminate(self, new_status):
        finished = self.result_status is not None or self.send_goal_future is None
        message = self.feedback_message
        if finished:
            self._cancel_requested = True  # nothing live to cancel
        super().terminate(new_status)
        if finished:
            self.feedback_message = message


class BtNode_Tk27Pick(_Tk27Leaf):
    """TK27 Pick with the prompt read from the blackboard (stripped, nonempty)."""
    mock_config_name = "BtNode_Grasp"

    def __init__(self, name, prompt_bb_key, action_name, plan_effort):
        from tk27_manipulation_msgs.action import Pick
        super().__init__(name, Pick, action_name, prompt_bb_key,
                         wait_for_server_timeout_sec=-3)
        self.plan_effort = plan_effort
        self._no_goal_reason = None

    def send_goal(self):
        self._no_goal_reason = None
        if self.mock_mode:
            return super().send_goal()
        try:
            prompt = self.blackboard.goal
        except KeyError:
            prompt = None
        if not isinstance(prompt, str) or not prompt.strip():
            self._no_goal_reason = f"Pick needs a nonempty prompt, got {prompt!r}"
            return
        goal = self.action_type.Goal()
        goal.prompt = prompt.strip()
        goal.plan_effort = self.plan_effort
        self.send_goal_request(goal)
        self.feedback_message = f"Sent Pick for {goal.prompt!r}"

    def update(self):
        if self._no_goal_reason is not None:
            return self._fail(self._no_goal_reason)
        return super().update()

    def feedback_callback(self, msg):
        # Pick feedback is TaskProgress (stage, detail, elapsed_s); the base
        # callback's delay_limit/status fields do not exist and would raise
        # inside the executor.
        progress = msg.feedback.progress
        self.last_feedback_time = time.time()
        self.action_stage = progress.stage
        self.feedback_message = progress.detail

    def process_result(self):
        result = self.result_message.result
        if (self.result_status == GoalStatus.STATUS_SUCCEEDED
                and result.success and result.status == 0):
            self.feedback_message = "Pick succeeded"
            return S.SUCCESS
        return self._fail(
            f"Pick failed: action_status={self.result_status}, "
            f"success={result.success}, status={result.status}, "
            f"error={result.error_msg}")


class BtNode_Tk27SavedPose(_Tk27Leaf):
    """Move the arm to a TK27 saved pose; collision avoidance is always off."""
    mock_config_name = "BtNode_MoveArmSingle"

    def __init__(self, name, pose_name, action_name):
        from tk27_manipulation_msgs.action import MoveToSavedPose
        super().__init__(name, MoveToSavedPose, action_name, None,
                         wait_for_server_timeout_sec=-3)
        self.pose_name = pose_name

    def send_goal(self):
        if self.mock_mode:
            return super().send_goal()
        goal = self.action_type.Goal()
        goal.pose_name = self.pose_name
        goal.avoid_collisions = False
        self.send_goal_request(goal)
        self.feedback_message = f"Sent MoveToSavedPose {self.pose_name!r}"

    def process_result(self):
        result = self.result_message.result
        if (self.result_status == GoalStatus.STATUS_SUCCEEDED
                and result.success and result.status == 0):
            self.feedback_message = f"Reached saved pose {self.pose_name!r}"
            return S.SUCCESS
        return self._fail(
            f"MoveToSavedPose {self.pose_name!r} failed: "
            f"action_status={self.result_status}, success={result.success}, "
            f"status={result.status}, error={result.error_msg}")

    def feedback_callback(self, msg):
        self.action_stage = msg.feedback.stage
        self.feedback_message = msg.feedback.detail


class BtNode_Tk27GripperOpen(_Tk27Leaf):
    """Open the gripper; succeed only on the same evidence TK27 itself requires."""
    mock_config_name = "BtNode_GripperAction"

    def __init__(self, name, action_name, open_position, open_tolerance, max_effort):
        from control_msgs.action import GripperCommand
        super().__init__(name, GripperCommand, action_name, None,
                         wait_for_server_timeout_sec=-3)
        self.open_position = open_position
        self.open_tolerance = open_tolerance
        self.max_effort = max_effort

    def send_goal(self):
        if self.mock_mode:
            return super().send_goal()
        goal = self.action_type.Goal()
        goal.command.position = self.open_position
        goal.command.max_effort = self.max_effort
        self.send_goal_request(goal)
        self.feedback_message = f"Sent gripper open to {self.open_position}"

    def feedback_callback(self, msg):
        pass

    def process_result(self):
        result = self.result_message.result
        if (self.result_status == GoalStatus.STATUS_SUCCEEDED
                and result.reached_goal
                and abs(result.position - self.open_position) <= self.open_tolerance):
            self.feedback_message = "Gripper open"
            return S.SUCCESS
        return self._fail(
            f"Gripper open failed: action_status={self.result_status}, "
            f"reached_goal={result.reached_goal}, stalled={result.stalled}, "
            f"position={result.position}, open={self.open_position}, "
            f"tolerance={self.open_tolerance}")
