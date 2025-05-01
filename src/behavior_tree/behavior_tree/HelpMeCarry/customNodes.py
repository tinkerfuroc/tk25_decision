import py_trees
import time
from behavior_tree.TemplateNodes.BaseBehaviors import ServiceHandler
from behavior_tree.TemplateNodes.ActionBase import ActionHandler
from behavior_tree.messages import ObjectDetection,Grasp
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped
  # Make sure pytree is imported
from py_trees.common import Status
import action_msgs.msg as action_msgs  # GoalStatus
from typing import Any
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Pose, PoseStamped

class BtNode_FindPointedLuggage(ServiceHandler):
    def __init__(self, 
                 name: str,
                 bb_namespace: str,
                 bb_key: str,
                 bb_key_announce_msg: str,
                 service_name: str = "object_detection"):
        super().__init__(name, service_name, ObjectDetection)
        self.bb_namespace = bb_namespace
        self.bb_key = bb_key

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="point",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key)
        )
        self.blackboard.register_key(
            key="announce_msg",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_announce_msg)
        )

    def setup(self, **kwargs):
        super().setup(**kwargs)

    def initialise(self):
        request = ObjectDetection.Request()
        request.prompt = "person, luggage"
        request.flags = "find_pointed_object"
        request.camera = "orbbec"
        request.target_frame = "base_link"

        self.response = self.client.call_async(request)
        self.feedback_message = "Requesting pointed luggage detection..."

    def update(self):
        if not self.response.done():
            self.feedback_message = "Waiting for service response..."
            return py_trees.common.Status.RUNNING

        result = self.response.result()
        if result.status != 0:
            self.feedback_message = f"Service failed: {result.error_msg}"
            return py_trees.common.Status.FAILURE

        for obj in result.objects:
            if obj.being_pointed == 1 or obj.being_pointed == 2:
                self.blackboard.point = PointStamped
                self.blackboard.point.point = obj.centroid
                self.blackboard.point.header = result.header

                if obj.being_pointed == 1:
                    self.blackboard.announce_msg = "You are pointing to your left"
                else:
                    self.blackboard.announce_msg = "You are pointing to your right"

                self.feedback_message = f"Pointed luggage found and stored in blackboard key '{self.bb_key}'"
                return py_trees.common.Status.SUCCESS

        self.feedback_message = "No pointed luggage found in detected objects"
        return py_trees.common.Status.FAILURE

class BtNode_GotoAction(ActionHandler):
    def __init__(self, 
                 name: str, 
                 key: str = "goal", 
                 action_name: str = "navigate_to_pose", 
                 wait_for_server_timeout_sec: float = -3):
        super().__init__(name, NavigateToPose, action_name, key, wait_for_server_timeout_sec)
        
        self.blackboard = self.attach_blackboard_client(name="GotoActionRead")
        self.blackboard.register_key(
            key=key,
            access=py_trees.common.Access.READ
        )

    def send_goal(self):
        try:
            goal = NavigateToPose.Goal()
            goal.pose = self.blackboard.get("goal")
            self.send_goal_request(goal)
            self.feedback_message = "Sent goal request"
        except KeyError:
            self.feedback_message = "Goal pose not found in blackboard"
            return

    def process_result(self):
        if self.result_status == action_msgs.GoalStatus.STATUS_ABORTED:
            self.feedback_message = "Navigation action aborted"
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS

    def feedback_callback(self, msg: Any):
        feedback = msg.feedback
        self.last_feedback_time = time.time()
        self.feedback_timeout = 1000
        self.action_status = 0
        self.process_feedback(feedback)

