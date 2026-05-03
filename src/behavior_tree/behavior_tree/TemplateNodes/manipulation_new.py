from typing import Any, Optional
import py_trees
from py_trees.common import Status

from tinker_arm_msgs.action import JointMove
import action_msgs.msg as action_msgs
from .ActionBase import ActionHandler
import math

class BtNode_JointMoveAction(ActionHandler):
    def __init__(
        self,
        name: str,
        arm_pose_bb_key: str,
        action_name="joint_move_action"
        # TODO: add octomap
    ):
        super().__init__(
            name,
            JointMove,
            action_name,
            arm_pose_bb_key,
            wait_for_server_timeout_sec=-3
        )
        self.blackboard.register_key(
            key="arm_joint_pose",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", arm_pose_bb_key),
        )
    
    def setup(self, **kwargs):
        return super().setup(**kwargs)
    
    def send_goal(self):
        try:
            goal = JointMove.Goal()
            goal.joint0 = self.blackboard.arm_joint_pose[0]
            goal.joint1 = self.blackboard.arm_joint_pose[1]
            goal.joint2 = self.blackboard.arm_joint_pose[2]
            goal.joint3 = self.blackboard.arm_joint_pose[3]
            goal.joint4 = self.blackboard.arm_joint_pose[4]
            goal.joint5 = self.blackboard.arm_joint_pose[5]
            goal.joint6 = self.blackboard.arm_joint_pose[6]
            self.send_goal_request(goal)
            self.feedback_message="Send goal pose"
        except Exception as e:
            self.feedback_message = (
                f"Failed to send JointMove goal {e}"
            )
            pass
    
    def feedback_callback(self, msg: Any):
        pass

    def process_result(self):
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = (
                f"JointMoveAction failed"
            )
            self.logger.debug(
                f"MoveArmJointPC failed"
            )
            return py_trees.common.Status.FAILURE
        else:
            result = self.result_message.result
            if result.success:
                self.feedback_message = (
                    f"JointMoveAction succeeded"
                )
                self.logger.debug(f"JointMoveAction succeeded")
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = f"JointMoveAction failed"
                self.logger.debug(
                    f"JointMoveAction failed"
                )
                return py_trees.common.Status.FAILURE
