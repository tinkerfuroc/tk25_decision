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
                 bb_key: str = "goal",  # default key
                 service_name: str = "object_detection"):
        super().__init__(name, service_name, ObjectDetection)
        self.bb_namespace = bb_namespace
        self.bb_key = bb_key

    def setup(self, **kwargs):
        super().setup(**kwargs)

        self.bb_write_client = self.attach_blackboard_client(
            name="FindPointedLuggageWrite", 
            namespace=self.bb_namespace
        )
        self.bb_write_client.register_key(
            key=self.bb_key,
            access=py_trees.common.Access.WRITE
        )

    def initialise(self):
        request = ObjectDetection.Request()
        request.prompt = "person, luggage"
        request.flags = "find_pointed_object"
        request.camera = "realsense"
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
            if obj.being_pointed == 1:
                pose = PoseStamped()
                pose.header.frame_id = result.header.frame_id
                pose.position = obj.centroid
                # pose.orientation.w = 1.0  # Identity quaternion
                pose.orientation.x = 0.707106781
                pose.orientation.z = 0.707106781

                self.bb_write_client.set(self.bb_key, pose, overwrite=True)
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

