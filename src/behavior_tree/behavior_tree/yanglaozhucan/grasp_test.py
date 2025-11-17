import py_trees
import py_trees_ros
import rclpy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import json

from behavior_tree.TemplateNodes.Audio import BtNode_TTSCN
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArmSingle, BtNode_GripperAction, BtNode_MoveArmJointPC, BtNode_CartesianMove
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj, BtNode_TurnPanTilt, BtNode_ScanFor, BtNode_GetPointCloud
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from .customNodes import BtNode_ChangeToNextMedication, BtNode_ProcessTrayPoint, BtNode_WriteDropPose

PRINT_DEBUG = True
PRINT_BLACKBOARD = False

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

KEY_ARM_NAVIGATING = "arm_pos_navigating"
KEY_MEDICATION_DICT = "medication_dict"
KEY_DROP_POSES_LIST = "drop_poses_list"
KEY_PANTILT_ANGLE = "pantilt_angle"
KEY_POSE_ENDING = "pose_ending"
KEY_TRAY_PROMPT = "tray_prompt"
KEY_POSE_GRASP = "pose_grasp"

# These will be initialized in main() after rclpy.init()
ARM_NAVIGATING = None
MEDICATION_DICT = None
DROP_POSES_LIST = None
PANTILT_ANGLE = None
POSE_ENDING = None
TRAY_PROMPT = None
POSE_GRASP = None

KEY_DROP_POSE_IDX = "drop_pose"
DROP_POS_IDX = constants["drop_pose_idx"]

KEY_MEDICATION_LIST = "medication_list"
MEDICATION_LIST = constants["medication_list"]

KEY_POINT_DROP = "point_drop"
KEY_OBJECT_NAME = "object_name"

KEY_POSE_DROP = "pose_drop"
KEY_ARM_SCAN = "arm_scan"
KEY_SCAN_TRAY_RESULT = "scan_tray_result"
KEY_ORBBEC_POINTCLOUD = "orbbec_pointcloud"
KEY_OBJECT = "object"

arm_service_name = "arm_joint_service"
grasp_service_name = "start_grasp"
place_service_name = "place_action_service"

def createConstantWriter():
    root = py_trees.composites.Parallel(name="constant writer", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    root.add_child(py_trees.behaviours.SetBlackboardVariable(name="Write arm navigating", variable_name=KEY_ARM_NAVIGATING, variable_value=ARM_NAVIGATING, overwrite=True))
    root.add_child(py_trees.behaviours.SetBlackboardVariable(name="Write medication dict", variable_name=KEY_MEDICATION_DICT, variable_value=MEDICATION_DICT, overwrite=True))
    root.add_child(py_trees.behaviours.SetBlackboardVariable(name="Write drop poses list", variable_name=KEY_DROP_POSES_LIST, variable_value=DROP_POSES_LIST, overwrite=True))
    root.add_child(py_trees.behaviours.SetBlackboardVariable(name="Write pantilt angle", variable_name=KEY_PANTILT_ANGLE, variable_value=PANTILT_ANGLE, overwrite=True))
    root.add_child(py_trees.behaviours.SetBlackboardVariable(name="Write pose ending", variable_name=KEY_POSE_ENDING, variable_value=POSE_ENDING, overwrite=True))
    root.add_child(py_trees.behaviours.SetBlackboardVariable(name="Write tray prompt", variable_name=KEY_TRAY_PROMPT, variable_value=TRAY_PROMPT, overwrite=True))
    root.add_child(py_trees.behaviours.SetBlackboardVariable(name="Write drop pose idx", variable_name=KEY_DROP_POSE_IDX, variable_value=DROP_POS_IDX, overwrite=True))
    root.add_child(py_trees.behaviours.SetBlackboardVariable(name="Write medication list", variable_name=KEY_MEDICATION_LIST, variable_value=MEDICATION_LIST, overwrite=True))
    root.add_child(py_trees.behaviours.SetBlackboardVariable(name="Write pose grasp", variable_name=KEY_POSE_GRASP, variable_value=POSE_GRASP, overwrite=True))
    return root

def createGetMedications():
    root = py_trees.composites.Sequence(name="Get Medications", memory=True)
    root.add_child(BtNode_WriteDropPose(
        name="Write drop pose",
        bb_key_idx=KEY_DROP_POSE_IDX,
        bb_key_drop_poses_list=KEY_DROP_POSES_LIST,
        bb_key_drop_pose=KEY_POSE_DROP
    ))
    return root

def createStartingConfigurations():
    root = py_trees.composites.Sequence(name="Starting Configurations", memory=True)
    root.add_child(BtNode_TTSCN("Announce start", bb_source=None, message="开始比赛，正在复位"))
    root.add_child(BtNode_GripperAction("Open gripper", open_gripper=True))
    root.add_child(py_trees.decorators.Retry("retry", BtNode_MoveArmSingle(name="Move arm to nav", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False), 3))
    root.add_child(BtNode_TurnPanTilt(name='turn pantilt', x=0.0, y=PANTILT_ANGLE))
    root.add_child(BtNode_TTSCN("Announce completed", bb_source=None, message="复位完成，准备就绪"))
    return root

def createMoveArmWithOctomap(arm_pose_bb_key:str):
    root = py_trees.composites.Sequence(name="Move Arm With Octomap", memory=True)
    root.add_child(BtNode_GetPointCloud(
        name="Get pointcloud",
        bb_point_cloud_key=KEY_ORBBEC_POINTCLOUD
    ))
    root.add_child(BtNode_MoveArmJointPC(
        name="Move arm with octomap",
        bb_key_pointcloud=KEY_ORBBEC_POINTCLOUD,
        bb_key_arm_pose=arm_pose_bb_key
    ))
    return root

def createDropWithOctomap(key_point:str):
    root = py_trees.composites.Sequence(name="Drop With Octomap", memory=True)
    root.add_child(BtNode_GetPointCloud(
        name="Get pointcloud",
        bb_point_cloud_key=KEY_ORBBEC_POINTCLOUD
    ))
    root.add_child(BtNode_CartesianMove(
        name="Move arm with octomap",
        bb_key_pointcloud=KEY_ORBBEC_POINTCLOUD,
        bb_key_point=key_point
    ))
    root.add_child(BtNode_GripperAction("open gripper", True))
    return root

def createGraspObject():
    root = py_trees.composites.Sequence(name="Grasp Object", memory=True)
    root.add_child(BtNode_TTSCN("Announce grasping medication", bb_source=None, message="正在抓取药物"))
    root.add_child(createMoveArmWithOctomap(KEY_ARM_SCAN))
    root.add_child(BtNode_TTSCN("Announce searching for medication", bb_source=None, message="正在定位药物"))
    root.add_child(BtNode_FindObj(
        name="find medication",
        bb_source=KEY_OBJECT_NAME,
        bb_namespace=None,
        bb_key=KEY_OBJECT,
        service_name="object_detection_yolo"
        ))
    root.add_child(BtNode_Grasp(
        name="Grasp object",
        bb_source=KEY_OBJECT,
        action_name=grasp_service_name
    ))
    root.add_child(BtNode_MoveArmSingle(
        name="Move arm to hold",
        service_name=arm_service_name,
        arm_pose_bb_key=KEY_ARM_NAVIGATING))
    root.add_child(BtNode_TTSCN("Announce grasp completed", bb_source=None, message="药物抓取完成"))
    return root

def createGotoGrasp():
    root = py_trees.composites.Sequence(name="Goto Grasp", memory=True)
    root.add_child(BtNode_MoveArmSingle(name="Move arm to nav", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=True))
    parallel = py_trees.composites.Parallel(name="Goto Grasp Parallel", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel.add_child(py_trees.decorators.Retry("retry", 
                                                 BtNode_GotoAction(name="Go to shelf grasp point", key=KEY_POSE_GRASP), 
                                                 5))
    parallel.add_child(BtNode_TTSCN("Announce navigating to grasp", bb_source=None, message="前往药架取药"))
    root.add_child(parallel)
    return root

def createGotoDrop():
    root = py_trees.composites.Sequence(name="Goto Drop", memory=True)
    root.add_child(BtNode_MoveArmSingle(name="Move arm to nav", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=True))
    parallel = py_trees.composites.Parallel(name="Goto Drop Parallel", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel.add_child(py_trees.decorators.Retry("retry", 
                                                 BtNode_GotoAction(name="Go to drop point", key=KEY_POSE_DROP), 
                                                 5))
    parallel.add_child(BtNode_TTSCN("Announce navigating to drop", bb_source=None, message="前往桌子放药"))
    root.add_child(parallel)
    return root

def createDropObject():
    root = py_trees.composites.Sequence(name="Drop Object", memory=True)
    root.add_child(BtNode_ScanFor(
        name="Scan for tray",
        bb_source=KEY_TRAY_PROMPT,
        bb_key=KEY_SCAN_TRAY_RESULT
    ))
    root.add_child(BtNode_ProcessTrayPoint(
        name="Process tray point",
        bb_vision_result=KEY_SCAN_TRAY_RESULT,
        bb_tray_point=KEY_POINT_DROP
    ))
    root.add_child(BtNode_TTSCN("Announce placing medication", bb_source=None, message="正在放置药品"))
    root.add_child(createDropWithOctomap(KEY_POINT_DROP))
    root.add_child(BtNode_MoveArmSingle(
        name="move arm to navigating",
        service_name=arm_service_name,
        arm_pose_bb_key=KEY_ARM_NAVIGATING,
        add_octomap=True
    ))
    return root

def createGetMedicationOnce(index:int):
    root = py_trees.composites.Sequence(name="Get Medication Once", memory=True)
    root.add_child(BtNode_TTSCN("Announce getting medication", bb_source=None, message=f"开始获取第{index + 1}份药物"))
    root.add_child(BtNode_ChangeToNextMedication(
        name="Change to next medication",
        bb_key_medication_list=KEY_MEDICATION_LIST,
        bb_key_medication_dict=KEY_MEDICATION_DICT,
        bb_key_current_arm_scan_pos=KEY_ARM_SCAN,
        bb_key_current_medication=KEY_OBJECT_NAME))
    root.add_child(createGotoGrasp())
    root.add_child(createGraspObject())
    root.add_child(createGotoDrop())
    root.add_child(createDropObject())
    root.add_child(BtNode_TTSCN("Announce medication delivered", bb_source=None, message="药物已送达"))
    return root

def returnToEndingPose():
    root = py_trees.composites.Sequence(name="Return to Ending Pose", memory=True)
    root.add_child(BtNode_MoveArmSingle(name="Move arm to nav", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=True))
    parallel = py_trees.composites.Parallel(name="Return to Ending Pose Parallel", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel.add_child(py_trees.decorators.Retry("retry", 
                                                 BtNode_GotoAction(name="Go to ending pose", key=KEY_POSE_ENDING), 
                                                 5))
    parallel.add_child(BtNode_TTSCN("Announce returning to ending pose", bb_source=None, message="返回结束位置"))
    root.add_child(parallel)
    return root

def createZGC2026():
    root = py_trees.composites.Sequence(name="ZGC2026", memory=True)
    root.add_child(createConstantWriter())
    root.add_child(createStartingConfigurations())
    for i in range(3):
        get_medication_once = createGetMedicationOnce(i)
        root.add_child(get_medication_once)
    root.add_child(returnToEndingPose())
    root.add_child(BtNode_TTSCN("Announce task completed", bb_source=None, message="任务完成，回到结束位置"))
    return root

def createZGC2026_test():
    root = py_trees.composites.Sequence(name="ZGC2026", memory=True)
    root.add_child(createConstantWriter())
    root.add_child(createStartingConfigurations())
    root.add_child(createGetMedications())
    root.add_child(BtNode_ChangeToNextMedication(
        name="Change to next medication",
        bb_key_medication_list=KEY_MEDICATION_LIST,
        bb_key_medication_dict=KEY_MEDICATION_DICT,
        bb_key_current_arm_scan_pos=KEY_ARM_SCAN,
        bb_key_current_medication=KEY_OBJECT_NAME))
    # root.add_child(createGraspObject())
    root.add_child(createMoveArmWithOctomap(KEY_ARM_SCAN))
    # root.add_child(createDropWithOctomap(KEY_ARM_SCAN))
    # for i in range(3):
    #     get_medication_once = createGetMedicationOnce(i)
    #     root.add_child(get_medication_once)
    # root.add_child(returnToEndingPose())
    # root.add_child(BtNode_TTSCN("Announce task completed", bb_source=None, message="任务完成，回到结束位置"))
    return root

def main():
    rclpy.init(args=None)
    
    # Initialize constants that require rclpy to be initialized
    global ARM_NAVIGATING, MEDICATION_DICT, DROP_POSES_LIST, PANTILT_ANGLE, POSE_ENDING, TRAY_PROMPT, POSE_GRASP
    ARM_NAVIGATING = arm_pose_reader(constants["arm_pos_navigating"])
    MEDICATION_DICT = constants["medication_dict"]
    DROP_POSES_LIST = [pose_reader(pose_dict) for pose_dict in constants["drop_poses_list"]]
    PANTILT_ANGLE = constants["pantilt_angle"]
    POSE_ENDING = pose_reader(constants["pose_ending"])
    TRAY_PROMPT = constants["tray_prompt"]
    POSE_GRASP = pose_reader(constants["pose_shelf"])

    root = createZGC2026_test()

    # make it a ros tree
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node_name="root_node", timeout=15)

    # function for display the tree to standard output
    def print_tree(tree):
        print(py_trees.display.unicode_tree(root=tree.root, show_status=True))
        if PRINT_BLACKBOARD:
            print(py_trees.display.unicode_blackboard())

    if PRINT_DEBUG:
        py_trees.logging.level = py_trees.logging.Level.DEBUG
    
    tree.tick_tock(period_ms=250.0,post_tick_handler=print_tree)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown() 

