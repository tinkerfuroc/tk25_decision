import py_trees
import rclpy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import json

from behavior_tree.TemplateNodes.Audio import BtNode_TTSCN, BtNode_Announce, BtNode_GetConfirmation, BtNode_PhraseExtraction, BtNode_GraspRequest
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArmSingle, BtNode_Place, BtNode_GripperAction
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj, BtNode_DoorDetection, BtNode_TurnPanTilt, BtNode_ScanFor


try:
    file = open("/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/yanglaozhucan/constants.json", "r")
    constants = json.load(file)
    file.close()
except FileNotFoundError:
    print("ERROR: constants.json not found!")
    raise FileNotFoundError

def pose_reader(pose_dict):
    return PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=pose_dict["point"]["x"], y=pose_dict["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=pose_dict["orientation"]["x"], 
                                                            y=pose_dict["orientation"]["y"], 
                                                            z=pose_dict["orientation"]["z"], 
                                                            w=pose_dict["orientation"]["w"]))
                        )

def arm_pose_reader(arm_pose_list):
    return [x / 180 * 3.1415926 for x in arm_pose_list]

KEY_ARM_NAVIGATING = "arm_navigating"
KEY_MEDICATION_DICT = "medication_dict"
KEY_DROP_POSES_LIST = "drop_poses_list"
KEY_PANTILT_ANGLE = "pantilt_angle"
ARM_NAVIGATING = arm_pose_reader(constants["arm_navigating"])
MEDICATION_DICT = constants["medication_dict"]
DROP_POSES_LIST = [pose_reader(pose_dict) for pose_dict in constants["drop_poses_list"]]
PANTILT_ANGLE = constants["pantilt_angle"]

KEY_POINT_DROP = "point_drop"
KEY_OBJECT_NAME = "object_name"
KEY_POSE_GRASP = "pose_grasp"
KEY_POSE_DROP = "pose_drop"
KEY_ARM_SCAN = "arm_scan"
KEY_MEDICATION_LIST = "medication_list"

def createConstantWriter():
    root = py_trees.composites.Parallel(name="constant writer", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    root.add_child(py_trees.behaviours.SetBlackboardVariable(name="Write arm navigating", variable_name=KEY_ARM_NAVIGATING, variable_value=ARM_NAVIGATING))
    root.add_child(py_trees.behaviours.SetBlackboardVariable(name="Write medication dict", variable_name=KEY_MEDICATION_DICT, variable_value=MEDICATION_DICT))
    root.add_child(py_trees.behaviours.SetBlackboardVariable(name="Write drop poses list", variable_name=KEY_DROP_POSES_LIST, variable_value=DROP_POSES_LIST))
    root.add_child(py_trees.behaviours.SetBlackboardVariable(name="Write pantilt angle", variable_name=KEY_PANTILT_ANGLE, variable_value=PANTILT_ANGLE))
    return root

def createStartingConfigurations():
    root = py_trees.composites.Sequence(name="Starting Configurations", memory=True)
    root.add_child(BtNode_TTSCN("Announce start", bb_source=None, message="开始比赛，正在复位"))
    root.add_child(py_trees.decorators.Retry("retry", BtNode_MoveArmSingle(name="Move arm to nav", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False), 3))
    root.add_child(BtNode_TurnPanTilt(name='turn pantilt', x=0.0, y=PANTILT_ANGLE))
    return root

def createGraspObject():
    root = py_trees.composites.Sequence(name="Grasp Object", memory=True)
    return root

def createGotoGrasp():
    root = py_trees.composites.Sequence(name="Goto Grasp", memory=True)
    return root

def createGotoDrop():
    root = py_trees.composites.Sequence(name="Goto Drop", memory=True)
    return root

def createDropObject():
    root = py_trees.composites.Sequence(name="Drop Object", memory=True)
    return root

def createGetMedicationOnce(index:int):
    root = py_trees.composites.Sequence(name="Get Medication Once", memory=True)
    return root

def createZGC2026():
    root = py_trees.composites.Sequence(name="ZGC2026", memory=True)
    root.add_child(createConstantWriter())
    for i in range(3):
        get_medication_once = createGetMedicationOnce(i)
        root.add_child(get_medication_once)
    
    return root

