from __future__ import annotations

"""PickAndPlace-specific BT nodes — copied from StoringGroceries to avoid cross-task imports.

These are functionally identical to the StoringGroceries originals.
"""

from typing import Any

import action_msgs.msg as action_msgs
import py_trees

from behavior_tree.TemplateNodes.ActionBase import ActionHandler
from behavior_tree.TemplateNodes.BaseBehaviors import ServiceHandler
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp
from behavior_tree.messages import Categorize, ObjectDetectionGeneralist, GetImage
from geometry_msgs.msg import Pose


class BtNode_WriteFoundItems(py_trees.behaviour.Behaviour):
    """Writes found items from the generalist detection result to the blackboard for TTS announcement."""

    def __init__(
        self,
        name: str,
        bb_key_vision_res: str,
        bb_key_announcement: str,
        place_seen:str = None
    ):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="vision_result",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_vision_res),
        )
        self.blackboard.register_key(
            key="announcement_msg",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_announcement),
        )
        self.place_seen = place_seen

    def update(self):
        try:
            vision_result = self.blackboard.vision_result
            if vision_result.objects:
                object_names = ["one " + obj.cls for obj in vision_result.objects]
                announcement_msg = f"I see {', '.join(object_names)}"
                if self.place_seen is not None:
                    announcement_msg += f"  {self.place_seen}."
                else:
                    announcement_msg += "."
                self.blackboard.announcement_msg = announcement_msg
                self.feedback_message = f"Wrote {announcement_msg} to blackboard"
                return py_trees.common.Status.SUCCESS
            else:
                self.blackboard.announcement_msg = "I could not find any objects"
                self.feedback_message = "No objects found in vision result"
                return py_trees.common.Status.SUCCESS
        except Exception as e:
            self.blackboard.announcement_msg = "Error processing vision result"
            self.feedback_message = f"Error processing vision result: {e}"
            return py_trees.common.Status.FAILURE


class BtNode_GetImage(ServiceHandler):
    """Gets an image and/or depth from a specified camera and writes it to the blackboard.

    Uses `tinker_vision_msgs_26/srv/GetImage` on `/get_image_service`. 
    The `camera` field specifies which camera to get the image from (e.g., "realsense" or "orbbec").
    The `depth` field specifies whether to return the depth image in addition to the RGB image.
    """

    def __init__(
        self,
        name: str,
        camera: str,
        bb_key_rgb_image: str,
        get_depth_image: bool = False,
        bb_key_depth_image: str = None,
        service_name: str = "get_image_service",
        service_type=GetImage,
    ):
        super().__init__(name=name, service_name=service_name, service_type=service_type)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="rgb_image",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_rgb_image),
        )
        if bb_key_depth_image is not None:
            self.blackboard.register_key(
                key="depth_image",
                access=py_trees.common.Access.WRITE,
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_depth_image),
            )
        self.camera = camera
        self.get_depth_image = get_depth_image
    
    def initialise(self):
        super().initialise()
        if self.mock_mode:
            from behavior_tree.mock_messages import MockMessage
            placeholder = MockMessage()
            placeholder.status = 0
            self.blackboard.rgb_image = placeholder
            if self.get_depth_image and self.blackboard.exists("depth_image"):
                self.blackboard.depth_image = placeholder
            self.feedback_message = f"MOCK: GetImage from {self.camera}"
            return
        request = GetImage.Request()
        request.camera = self.camera
        request.depth = self.get_depth_image
        self.response = self.client.call_async(request)

    def update(self):
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
        if self.response.done():
            if self.response.result().status == 0:
                response = self.response.result()
                self.blackboard.rgb_image = response.rgb_image
                if self.get_depth_image and self.blackboard.exists("depth_image"):
                    self.blackboard.depth_image = response.depth_image
                self.feedback_message = f"Successfully got image from {self.camera} camera"
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = (
                    f"Failed to get image from {self.camera} camera with status {self.response.result().status} "
                    f"and error message {self.response.result().error_msg}"
                )
                return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = f"Waiting for response from get image service for {self.camera} camera"
            return py_trees.common.Status.RUNNING



class BtNode_FindObjTable(ServiceHandler):
    """Find object on table using the generalist detection service.

    Uses `tinker_vision_msgs_26/srv/ObjectDetectionGeneralist` on
    `/object_detection_generalist`. Open-vocab prompts fall through to the
    YOLO-World / Gemini + FastSAM path when `use_vlm_sam_fallback=True`.
    Sorts closest-first so the nearest match is the grasp candidate.
    """

    def __init__(
        self,
        name: str,
        bb_key_prompt: str,
        bb_key_image: str,
        bb_key_segment: str,
        bb_key_result: str,
        bb_key_announcement: str,
        bb_key_object_label: str = None,
        target_frame: str = "base_link",
        use_realsense: bool = True,
        service_name: str = "object_detection_generalist",
        service_type=ObjectDetectionGeneralist,
        use_vlm_sam_fallback: bool = True,
    ):
        super().__init__(name=name, service_name=service_name, service_type=service_type)
        self.bb_key_prompt = bb_key_prompt
        self.bb_key_image = bb_key_image
        self.bb_key_segment = bb_key_segment
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="prompt",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_prompt),
        )
        self.blackboard.register_key(
            key="image",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_image),
        )
        self.blackboard.register_key(
            key="segmentation",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_segment),
        )
        self.blackboard.register_key(
            key="result",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_result),
        )
        self.blackboard.register_key(
            key="announcement_msg",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_announcement),
        )
        if bb_key_object_label is not None:
            self.blackboard.register_key(
                key="object_label",
                access=py_trees.common.Access.WRITE,
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_object_label),
            )
        self.use_realsense = use_realsense
        self.target_frame = target_frame
        self.use_vlm_sam_fallback = use_vlm_sam_fallback

    def initialise(self):
        super().initialise()
        if self.mock_mode:
            from behavior_tree.mock_messages import MockMessage
            mock_result = MockMessage()
            mock_result.status = 0
            mock_result.objects = []
            mock_result.segments = [MockMessage()]
            mock_result.rgb_image = MockMessage()
            mock_result.depth_image = MockMessage()
            self.blackboard.image = mock_result.rgb_image
            self.blackboard.segmentation = mock_result.segments[0]
            self.blackboard.result = mock_result
            self.blackboard.announcement_msg = "MOCK: pretending to find object"
            try:
                self.blackboard.object_label = "mock_object"
            except AttributeError:
                pass
            self.feedback_message = "MOCK: FindObjTable"
            return
        request = ObjectDetectionGeneralist.Request()
        request.prompt = self.blackboard.prompt
        request.camera = "realsense" if self.use_realsense else "orbbec"
        request.target_frame = self.target_frame
        request.sort_closest = True
        request.sort_highest = False
        request.return_rgb_image = True
        # Depth must reach the grasp action (Grasp.Goal.depth_image) — the
        # downstream BtNode_Grasp.send_goal() reads vision_result.depth_image
        # directly. Without this, AnyGrasp gets an empty image and fails.
        request.return_depth_image = True
        request.return_segments = True
        request.force_vlm_sam = False
        request.use_vlm_sam_fallback = self.use_vlm_sam_fallback
        self.response = self.client.call_async(request)

    def update(self):
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
        if self.response.done():
            if self.response.result().status == 0:
                response = self.response.result()
                self.blackboard.image = response.rgb_image
                self.blackboard.segmentation = response.segments[0]
                self.blackboard.result = response
                self.blackboard.announcement_msg = f"Grasping {response.objects[0].cls}"
                try:
                    self.blackboard.object_label = response.objects[0].cls
                except AttributeError:
                    pass
                self.feedback_message = f"Found object: {response.objects[0].cls}"
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = (
                    f"Failed to find object with {self.response.result().status} "
                    f"and error message {self.response.result().error_msg}"
                )
                return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = "Waiting for response from find object service"
            return py_trees.common.Status.RUNNING


class BtNode_CategorizeGrocery(ActionHandler):
    """Categorize grocery item for cabinet shelf placement."""

    def __init__(
        self,
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
        action_name: str = "grocery_categorize",
        wait_for_server_timeout_sec: float = -3,
    ):
        super().__init__(
            name, Categorize, action_name, None, wait_for_server_timeout_sec
        )
        self.n_layers = n_layers
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="prompt",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_prompt),
        )
        self.blackboard.register_key(
            key="image",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_image),
        )
        self.blackboard.register_key(
            key="segmentation",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_segment),
        )
        self.blackboard.register_key(
            key="target_frame",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_target_frame),
        )
        self.blackboard.register_key(
            key="shelf_left",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_shelf_left),
        )
        self.blackboard.register_key(
            key="shelf_right",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_shelf_right),
        )
        self.blackboard.register_key(
            key="env_points",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_env_points),
        )
        self.blackboard.register_key(
            key="result_point",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_result_point),
        )
        self.blackboard.register_key(
            key="reason",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_reason),
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
            self.feedback_message = f"Sent goal to categorize grocery with prompt: {self.blackboard.prompt}"
        except Exception as e:
            self.feedback_message = f"Failed to send goal: {e}"
            return py_trees.common.Status.FAILURE

    def process_result(self):
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = (
                f"Categorize grocery failed with status: {self.result_status} "
                f"and error message {self.result_message.result.error_msg}"
            )
            return py_trees.common.Status.FAILURE
        else:
            result = self.result_message.result
            self.blackboard.result_point = result.place_point
            self.blackboard.env_points = result.env_points
            self.blackboard.reason = result.place_reason
            self.feedback_message = (
                f"Categorize grocery succeeded with target layer {result.shelf_layer} "
                f"and target point {result.place_point}"
            )
            return py_trees.common.Status.SUCCESS

    def feedback_callback(self, msg):
        feedback = msg.feedback
        if feedback.status != 0:
            self.feedback_message = f"ERROR:  {feedback.status} - {feedback.message}"
        else:
            self.feedback_message = f"INFO:  {feedback.status} - {feedback.message}"
