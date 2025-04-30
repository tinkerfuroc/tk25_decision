import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArmSingle, BtNode_GripperAction
from .customNodes import BtNode_FindPointedLuggage

import math

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy

# 定义机械臂姿态
ARM_POS_NAVIGATING = [x / 180 * math.pi for x in [-87.0, -40.0, 28.0, 0.0, 30.0, -86.0, 0.0]]
ARM_POS_SCAN = [x / 180 * math.pi for x in [0.0, -50.0, 0.0, 66.0, 0.0, 55.0, 0.0]]

# 定义黑板键值
KEY_ARM_SCAN = "arm_scan"
KEY_ARM_NAVIGATING = "arm_navigating"

KEY_LUGGAGE_DETECTION = "luggage_detection"
KEY_PROMPT = "prompt"
KEY_GOAL = "goal"  # 用于导航的目标位置

# 服务名称
arm_service_name = "arm_joint_service"
grasp_service_name = "grasp"
point_target_frame = "base_link"

def createConstantWriter():
    """初始化常量"""
    root = py_trees.composites.Sequence("Write Constants", memory=True)
    root.add_child(BtNode_WriteToBlackboard("Write Arm Scan", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN, object=ARM_POS_SCAN))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Navigating", bb_namespace="", bb_source=None, bb_key=KEY_ARM_NAVIGATING, object=ARM_POS_NAVIGATING))
    root.add_child(BtNode_WriteToBlackboard("Write Prompt", bb_namespace="", bb_source=None, bb_key=KEY_PROMPT, object="luggage"))
    return root

def createFindPointedLuggage():
    """找到被指向的行李"""
    root = py_trees.composites.Sequence("Find Pointed Luggage", memory=True)
    
    # 宣布开始找行李
    root.add_child(BtNode_Announce(name="Announce finding luggage", bb_source=None, message="Looking for the luggage you're pointing at"))
    
    # 使用FindPointedLuggage节点寻找行李 - 不移动机械臂
    root.add_child(BtNode_FindPointedLuggage(
        name="Find pointed luggage",
        bb_namespace="",
        bb_key=KEY_GOAL
    ))
    
    # 宣布找到行李
    root.add_child(BtNode_Announce(name="Announce found luggage", bb_source=None, message="I found the luggage you're pointing at, I'll go there now"))
    
    return root

def createNavigateToLuggage():
    """导航到行李位置 - 整个机器人底盘移动"""
    root = py_trees.composites.Sequence("Navigate to Luggage", memory=True)
    
    # 宣布开始导航
    root.add_child(BtNode_Announce(name="Announce navigating", bb_source=None, message="Navigating to the luggage now"))
    
    # 导航到行李位置 - 整个机器人底盘移动
    # 这里使用KEY_GOAL作为目标位置的键值
    root.add_child(py_trees.decorators.Retry(
        name="Retry navigation",
        child=BtNode_GotoAction(name="Go to luggage", key=KEY_GOAL),
        num_failures=3
    ))
    
    # 宣布到达目标位置
    root.add_child(BtNode_Announce(name="Announce arrived", bb_source=None, message="I have arrived at the luggage location"))
    
    return root

def createFindLuggageAtLocation():
    """到达位置后再次查找行李 - 使用FindObj而不是ScanFor"""
    root = py_trees.composites.Sequence("Find Luggage at Location", memory=True)
    
    # 宣布开始在当前位置查找行李
    root.add_child(BtNode_Announce(name="Announce local search", bb_source=None, message="Looking for the luggage around me"))

        
    # 先移动机械臂到扫描位置
    move_arm_parallel = py_trees.composites.Parallel("Move Arm for Scanning", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    move_arm_parallel.add_child(BtNode_Announce(name="Announce moving arm", bb_source=None, message="Moving my arm to better see the luggage"))
    move_arm_parallel.add_child(BtNode_MoveArmSingle("Move arm to scan position", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN, add_octomap=True))
    root.add_child(move_arm_parallel)

    root.add_child(BtNode_FindObj(
        name="Find luggage",
        bb_source=KEY_PROMPT,  # 从这里获取物体名称
        bb_namespace="",
        bb_key=KEY_LUGGAGE_DETECTION,  # 结果存储在这里
        service_name="object_detection",
        object=None,  # 将从bb_source获取物体名称
        target_object_cls=None  # 不指定目标类别
    ))
    
    # 宣布找到行李
    root.add_child(BtNode_Announce(name="Announce found luggage locally", bb_source=None, message="I can see the luggage now, preparing to grasp"))
    
    return root

def createGraspLuggage():
    """抓取行李"""
    root = py_trees.composites.Sequence("Grasp Luggage", memory=True)
    
    # 现在才移动机械臂到扫描位置
    root.add_child(BtNode_MoveArmSingle("Move arm to scan position", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN, add_octomap=True))
    
    # 打开夹爪
    root.add_child(BtNode_GripperAction(name="Open gripper", open_gripper=True))
    
    # 宣布开始抓取
    root.add_child(BtNode_Announce(name="Announce grasping", bb_source=None, message="I'm going to grasp the luggage now"))
    
    # 抓取行李
    root.add_child(py_trees.decorators.Retry(
        name="Retry grasping",
        child=BtNode_Grasp(
            name="Grasp luggage",
            bb_source=KEY_LUGGAGE_DETECTION,
            service_name=grasp_service_name
        ),
        num_failures=2
    ))
    
    # 抓取后将手臂移回导航姿态
    root.add_child(BtNode_MoveArmSingle("Move arm to navigate position", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    
    # 宣布抓取成功
    root.add_child(BtNode_Announce(name="Announce grasp complete", bb_source=None, message="I have successfully grasped the luggage"))
    
    return root

def createHelpMeCarry():
    """创建"Help Me Carry"主决策树"""
    root = py_trees.composites.Sequence("Help Me Carry", memory=True)
    
    # 首先写入常量
    root.add_child(createConstantWriter())
    
    
    # 找到被指向的行李 - 不移动机械臂
    find_luggage = py_trees.decorators.Retry(
        name="Retry finding luggage",
        child=createFindPointedLuggage(),
        num_failures=2
    )
    root.add_child(find_luggage)
    
    # 导航到行李位置 - 整个机器人底盘移动
    navigate_luggage = py_trees.decorators.Retry(
        name="Retry navigating to luggage",
        child=createNavigateToLuggage(),
        num_failures=2
    )
    root.add_child(navigate_luggage)
    
    # 到达位置后再次查找行李 - 使用FindObj
    find_at_location = py_trees.decorators.Retry(
        name="Retry finding luggage at location",
        child=createFindLuggageAtLocation(),
        num_failures=2
    )
    root.add_child(find_at_location)
    
    # 抓取行李 - 现在才移动机械臂
    grasp_luggage = py_trees.decorators.Retry(
        name="Retry grasping luggage",
        child=createGraspLuggage(),
        num_failures=2
    )
    root.add_child(grasp_luggage)
    
    # 宣布任务完成
    root.add_child(BtNode_Announce(name="Announce task complete", bb_source=None, message="Help Me Carry task completed successfully. I will follow you now."))
    
    return root