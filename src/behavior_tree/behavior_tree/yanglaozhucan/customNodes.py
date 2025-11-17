from typing import Any,Tuple,List
import py_trees
import textwrap
from behavior_tree.TemplateNodes.BaseBehaviors import ServiceHandler
from behavior_tree.TemplateNodes.ActionBase import ActionHandler
from behavior_tree.messages import ObjectDetection, Categorize, TextToSpeech
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from geometry_msgs.msg import PointStamped

import action_msgs.msg as action_msgs

def arm_pose_reader(arm_pose_list):
    return [x / 180 * 3.1415926 for x in arm_pose_list]

def pose_reader(pose_dict):
    return PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=pose_dict["point"]["x"], y=pose_dict["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=pose_dict["orientation"]["x"], 
                                                            y=pose_dict["orientation"]["y"], 
                                                            z=pose_dict["orientation"]["z"], 
                                                            w=pose_dict["orientation"]["w"]))
                        )

class BtNode_ChangeToNextMedication(py_trees.behaviour.Behaviour):
    def __init__(self,
                 name: str,
                 bb_key_medication_list: str,
                 bb_key_medication_dict: str,
                 bb_key_current_medication: str,
                 bb_key_current_arm_scan_pos: str):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="medication_list",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_medication_list)
        )
        self.blackboard.register_key(
            key="med_dictionary",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_medication_dict)
        )
        self.blackboard.register_key(
            key="current_medication",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_current_medication)
        )
        self.blackboard.register_key(
            key="current_arm_scan_pos",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_current_arm_scan_pos)
        )
        self.current_index = -1

    def setup(self, **kwargs):
        return super().setup(**kwargs)

    def initialise(self):
        medication_list = self.blackboard.medication_list
        if not medication_list:
            self.feedback_message = "Medication list is empty."
            return
        self.feedback_message = "Initialized ChangeToNextMedication."
        return

    def update(self):
        self.current_index += 1
        med_dictionary = self.blackboard.med_dictionary
        medication_list = self.blackboard.medication_list
        if self.current_index >= len(medication_list):
            self.feedback_message = "All medications have been processed."
            self.current_index = len(medication_list) - 1
            return py_trees.common.Status.FAILURE
        current_med = medication_list[self.current_index]
        self.blackboard.current_medication = current_med
        if current_med in med_dictionary:
            current_arm_scan_pos = med_dictionary[current_med]['arm_scan_pos']
            self.blackboard.current_arm_scan_pos = arm_pose_reader(current_arm_scan_pos)
            self.feedback_message = f"Changed to next medication: {current_med}."
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"Medication {current_med} not found in dictionary."
            return py_trees.common.Status.FAILURE
        
class BtNode_WriteDropPose(py_trees.behaviour.Behaviour):
    def __init__(self, 
                 name: str,
                 bb_key_idx: str,
                 bb_key_drop_poses_list: str,
                 bb_key_drop_pose: str
    ):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="idx",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_idx)
        )
        self.blackboard.register_key(
            key="drop_poses",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_drop_poses_list)
        )
        self.blackboard.register_key(
            key="drop_pose",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_drop_pose)
        )
    def setup(self, **kwargs):
        return super().setup(**kwargs)
    def initialise(self):
        return super().initialise()
    def update(self):
        idx = self.blackboard.idx
        drop_poses = self.blackboard.drop_poses
        if idx < 0 or idx >= len(drop_poses):
            self.feedback_message = f"Index {idx} is out of bounds for drop poses list."
            return py_trees.common.Status.FAILURE
        self.blackboard.drop_pose = pose_reader(drop_poses[idx])
        self.feedback_message = f"Wrote drop pose for index {idx}."
        return py_trees.common.Status.SUCCESS

class BtNode_ProcessTrayPoint(py_trees.behaviour.Behaviour):
    def __init__(
            self,
            name: str,
            bb_vision_result: str,
            bb_tray_point: str
    ):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="vision_result",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_vision_result)
        )
        self.blackboard.register_key(
            key="tray_point",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_tray_point)
        )
    def setup(self, **kwargs):
        return super().setup(**kwargs)
    def initialise(self):
        return super().initialise()
    def update(self):
        vision_result = self.blackboard.vision_result
        if not vision_result or not vision_result.objects:
            self.feedback_message = "No objects found in vision result."
            return py_trees.common.Status.FAILURE
        tray_object = vision_result.objects[0]
        header = vision_result.header
        tray = PointStamped()
        tray.header = header
        tray.point = tray_object.centroid
        tray.point.z += 0.10  # Adjust Z coordinate upwards by 10 cm
        self.blackboard.tray_point = tray
        self.feedback_message = "Processed tray point from vision result."
        return py_trees.common.Status.SUCCESS

class BtNode_Confirm(BtNode_Announce):
    def __init__(self,
                 name: str,
                 key_confirmed: str,
                 type: str,
                 service_name: str = "announce"
                 ):
        super(BtNode_Announce, self).__init__(name=name, service_name=service_name, service_type=TextToSpeech)
        self.type = type
        self.bb_source = None #new
        self.given_msg = None #new
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="confirm_target",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key_confirmed)
        )
    
    def setup(self, **kwargs):
        self.given_msg = self.given_msg
        return super().setup(**kwargs)
    
    def initialise(self):
        # self.given_msg = "Your " + self.type + " is " + self.blackboard.confirm_target + ". Am I correct?"
        self.given_msg = "Your " + self.type + " is " + self.blackboard.confirm_target + ", correct?"
        return super().initialise()

##############################################   中关村   #####################################################
class BtNode_WriteGrid(ActionHandler):
    def __init__(self,
                 name: str,
                 bb_key_dest: str,
                 bb_key_shelf_left: str,
                 bb_key_shelf_right: str,
                 action_name: str = "write_grid",
                 ):
        super(BtNode_WriteGrid, self).__init__(name=name, action_type=ACTION, action_name=action_name)
        self.x = []
        self.y = []
        self.coor = []
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="target_grid",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name
        )("/", bb_key_dest)
        self.blackboard.register_key(
            key="shelf_left",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name
        )("/", bb_key_shelf_left)
        self.blackboard.register_key(
            key="shelf_right",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name
        )("/", bb_key_shelf_right)

    def send_goal(self):
        try:
            goal = ACTION.Goal()
            goal.shelf_left = self.blackboard.shelf_left
            goal.shelf_right = self.blackboard.shelf_right
            goal.shelf_height = [0.04, 0.33, 0.52, 0.9, 1.19, 1.47]
            goal.item_ids = ["item_1", "item_2", "item_3", "item_4", "item_5", "item_6"]
            self.send_goal_request(goal)
            self.logger.debug(f"Sent goal to get target grid")
            self.feedback_message = f"Sent goal to get target grid"
        except Exception as e:
            self.feedback_message = f"Failed to send goal: {e}"
            self.logger.debug(f"Failed to send goal: {e}")
            return py_trees.common.Status.FAILURE
    
    def process_result(self):
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = f"Coordinates failed and error message {self.result_message.result.error_msg}"
            return py_trees.common.Status.FAILURE
        else:
            result = self.result_message.result
            for i in range(len(result.item_height_grids)):
                self.y[i] = int(result.item_height_grids[i]) 
            self.x = result.item_horizontal_grids
            self.coor = list(zip(self.x, self.y))
            if len(self.coor) > 0: 
                print( f"Received target grid: {self.coor}, Type of coor: {type(self.coor)}")
            else:
                print( f"Received empty target grid.")
                return py_trees.common.Status.FAILURE
            self.blackboard.coor = self.coor
            self.feedback_message = f"Coordinates succeeded with target layer {result.coor}"
            return py_trees.common.Status.SUCCESS
    
    def feedback_callback(self, msg):
        feedback = msg.feedback
        if feedback.status != 0:
            self.feedback_message = f"ERROR:  {feedback.status} - {feedback.message}"
        else:
            self.feedback_message = f"INFO:  {feedback.status} - {feedback.message}"
##############################################   中关村   #####################################################
    
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
                 service_name = "object_detection_yolo",
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
                self.logger.debug(textwrap.dedent("""       
            'STATUS_UNKNOWN': 0,
            'STATUS_ACCEPTED': 1,
            'STATUS_EXECUTING': 2,
            'STATUS_CANCELING': 3,
            'STATUS_SUCCEEDED': 4,
            'STATUS_CANCELED': 5,
            'STATUS_ABORTED': 6,"""))
                return py_trees.common.Status.FAILURE
            else:
                self.logger.debug(f"Grasp feedback received with status: {self.result_status}")
                result = self.result_message.result
                if result.success:
                    self.blackboard.pose = result.grasp_pose
                    self.feedback_message = f"Grasp completed received with success: {result.success}"
                    self.logger.debug(f"Grasp completed received with success")
                    return py_trees.common.Status.SUCCESS
                else: 
                    self.feedback_message = f"Grasp completed received with stage: {result.stage} and error message {result.error_msg}"
                    self.logger.debug(f"Grasp completed received with stage: {result.stage} and error message {result.error_msg}")
                    return py_trees.common.Status.FAILURE
        except Exception as e:
            self.feedback_message = f"Failed to process grasp result: {e}"
            self.logger.debug(f"Failed to process grasp result: {e}")
            return py_trees.common.Status.FAILURE