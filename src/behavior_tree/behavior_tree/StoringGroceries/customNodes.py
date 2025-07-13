from typing import Any
import py_trees
from behavior_tree.TemplateNodes.BaseBehaviors import ServiceHandler
from behavior_tree.TemplateNodes.ActionBase import ActionHandler
from behavior_tree.messages import ObjectDetection, Categorize
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp

import action_msgs.msg as action_msgs

class BtNode_FindObjTable(ServiceHandler):
    """
    Find object on table
    """

    def __init__(self, name: str, 
                 bb_key_prompt: str, 
                 bb_key_image: str, 
                 bb_key_segment: str, 
                 bb_key_result: str,
                 bb_key_announcement: str,
                 target_frame: str = "base_link",
                 use_realsense: bool = True,
                 service_name = "object_detection",
                 service_type = ObjectDetection,
                 ):
        super(BtNode_FindObjTable, self).__init__(name=name,
                                                  service_name=service_name,
                                                  service_type=service_type)
        self.bb_key_prompt = bb_key_prompt
        self.bb_key_image = bb_key_image
        self.bb_key_segment = bb_key_segment
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="prompt",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_prompt)
        )
        self.blackboard.register_key(
            key="image",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_image)
        )
        self.blackboard.register_key(
            key="segmentation",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_segment)
        )
        self.blackboard.register_key(
            key="result",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_result)
        )
        self.blackboard.register_key(
            key="announcement_msg",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_announcement)
        )
        self.use_realsense = use_realsense

    def initialise(self):
        request = ObjectDetection.Request()
        request.prompt = self.blackboard.prompt
        request.flags = "find_for_grasp|request_image|request_segmentation"
        if self.use_realsense:
            request.camera = "realsense"
        else:
            request.camera = "orbecc"
        self.response = self.client.call_async(request)
        self.logger.debug(f"Initialized FindObjTable with prompt: {self.blackboard.prompt}")

    def update(self):
        self.logger.debug(f"Updating FindObjTable with prompt: {self.blackboard.prompt}")
        if self.response.done():
            if self.response.result().status == 0:
                response = self.response.result()
                self.blackboard.image = response.rgb_image
                self.blackboard.segmentation = response.segments[0]
                self.blackboard.result = response
                self.blackboard.announcement_msg = f"Grasping {response.objects[0].cls}"
                self.feedback_message = f"Found object: {response.objects[0].cls}"
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = f"Failed to find object with {self.response.result().status} and error message {self.response.result().error_msg}"
                return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = "Waiting for response from find object service"
            return py_trees.common.Status.RUNNING


class BtNode_CategorizeGrocery(ActionHandler):
    def __init__(self,
                 name: str,
                 n_layers: int,
                 bb_key_prompt: str,
                 bb_key_image: str,
                 bb_key_segment: str,
                 bb_target_frame: str,
                 bb_key_result_point: str,
                 bb_key_env_points: str,
                 bb_key_reason: str,
                 bb_key_shelf_left: str,
                 bb_key_shelf_right: str,
                 action_name: str = 'grocery_categorize',
                 wait_for_server_timeout_sec: float = -3
                 ):
        super(BtNode_CategorizeGrocery, self).__init__(name, Categorize, action_name, None, wait_for_server_timeout_sec)
        self.n_layers = n_layers
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="prompt",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_prompt)
        )
        self.blackboard.register_key(
            key="image",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_image)
        )
        self.blackboard.register_key(
            key="segmentation",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_segment)
        )
        self.blackboard.register_key(
            key="target_frame",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_target_frame)
        )
        self.blackboard.register_key(
            key="shelf_left",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_shelf_left)
        )
        self.blackboard.register_key(
            key="shelf_right",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_shelf_right)
        )
        self.blackboard.register_key(
            key="env_points",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_env_points)
        )
        self.blackboard.register_key(
            key="result_point",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_result_point)
        )
        self.blackboard.register_key(
            key="reason",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_reason)
        )


    def send_goal(self):
        try:
            goal = Categorize.Goal()
            goal.n_layers = self.n_layers
            goal.prompt = self.blackboard.prompt
            goal.img_table = self.blackboard.image
            goal.segment_object = self.blackboard.segmentation
            goal.target_frame = self.blackboard.target_frame
            goal.pt_shelf_left = self.blackboard.shelf_left
            goal.pt_shelf_right = self.blackboard.shelf_right
            self.send_goal_request(goal)
            self.logger.debug(f"Sent goal to categorize grocery with prompt: {self.blackboard.prompt}")
            self.feedback_message = f"Sent goal to categorize grocery with prompt: {self.blackboard.prompt}"
        except Exception as e:
            self.feedback_message = f"Failed to send goal: {e}"
            self.logger.debug(f"Failed to send goal: {e}")
            return py_trees.common.Status.FAILURE
    
    def process_result(self):
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = f"Categorize grocery failed with status: {self.result_status} and error message {self.result_message.result.error_msg}"
            return py_trees.common.Status.FAILURE
        else:
            result = self.result_message.result
            self.blackboard.result_point = result.place_point
            self.blackboard.env_points = result.env_points
            self.blackboard.reason = result.place_reason
            self.feedback_message = f"Categorize grocery succeeded with target layer {result.shelf_layer} and target point {result.place_point}"
            return py_trees.common.Status.SUCCESS
    
    def feedback_callback(self, msg):
        feedback = msg.feedback
        if feedback.status != 0:
            self.feedback_message = f"ERROR:  {feedback.status} - {feedback.message}"
        else:
            self.feedback_message = f"INFO:  {feedback.status} - {feedback.message}"


class BtNode_GraspWithPose(BtNode_Grasp):
    def __init__(self, name: str, 
                 bb_key_vision_res: str, 
                 bb_key_pose: str, 
                 action_name: str = "grasp"):
        super().__init__(name, bb_key_vision_res, action_name)
        self.blackboard.register_key(
            key="pose",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_pose)
        )
    
    def process_result(self):
        try:
            if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED and False:
                self.feedback_message = f"Grasp feedback received with status: {self.result_status}"
                self.logger.debug(f"Grasp feedback received with status: {self.result_status}")
                return py_trees.common.Status.FAILURE
            else:
                self.logger.debug(f"Grasp feedback received with status: {self.result_status}")
                result = self.result_message.result
                if result.success:
                    self.blackboard.pose = result.grasp_pose
                    self.feedback_message = f"Grasp feedback received with success: {result.success}"
                    self.logger.debug(f"Grasp feedback received with success")
                    return py_trees.common.Status.SUCCESS
                else: 
                    self.feedback_message = f"Grasp feedback received with stage: {result.stage} and error message {result.error_msg}"
                    self.logger.debug(f"Grasp feedback received with stage: {result.stage} and error message {result.error_msg}")
                    return py_trees.common.Status.FAILURE
        except Exception as e:
            self.feedback_message = f"Failed to process grasp result: {e}"
            self.logger.debug(f"Failed to process grasp result: {e}")
            return py_trees.common.Status.FAILURE

        