import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArm, BtNode_GripperAction
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_TargetExtraction

import math

#### TODO: 改这个，arm_scan是扫描的位置，arm_handing是递给人的位置, arm_hold是待机的位置 ####
arm_scan = [x / 180 * math.pi for x in []]
arm_handing = [x / 180 * math.pi for x in []]
arm_hold = [x / 180 * math.pi for x in []]

KEY_OBJECT = "object"
KEY_OBJECT_NAME = "object_name"
KEY_ARM_SCAN = "arm_scan"
KEY_ARM_HANDING = "arm_handing"
KEY_ARM_HOLD = "arm_hold"

arm_service_name = "arm_joint_service"
grasp_service_name = "start_grasp"

def createGraspAudio():
    # create sequence root, memory set to True
    root = py_trees.composites.Sequence(name="Grasp Audio", memory=True)
    # create parallel node for writing to blackboard
    write_bb = py_trees.composites.Parallel(name="Write to blackboard", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    # add children to parallel node
    write_bb.add_child(BtNode_WriteToBlackboard(name="Write arm scan pose", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN, object=arm_scan))
    write_bb.add_child(BtNode_WriteToBlackboard(name="Write arm handing pose", bb_namespace="", bb_source=None, bb_key=KEY_ARM_HANDING, object=arm_handing))
    write_bb.add_child(BtNode_WriteToBlackboard(name="Write arm hold pose", bb_namespace="", bb_source=None, bb_key=KEY_ARM_HOLD, object=arm_hold))
    # add parallel node to root
    root.add_child(write_bb)
    
    root.add_child(BtNode_Announce("Announce start", message="Hi, I'm Tinker, you can tell me to grasp things for you"))
    
    get_target = py_trees.composites.Selector("Get target", memory=True)
    get_target.add_child(BtNode_TargetExtraction("Get command", bb_dest_key=KEY_OBJECT_NAME, timeout=7.0))
    get_target.add_child(py_trees.decorators.SuccessIsFailure(BtNode_Announce("Announce unable to extract target", message="Sorry, I didn't hear you")))
    root.add_child(py_trees.decorators.Retry(name="retry", child=get_target, num_failures=10))

    root.add_child(BtNode_MoveArm("Move arm to find", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN))
    root.add_child(BtNode_FindObj(name="find object", bb_source=KEY_OBJECT_NAME, bb_namespace=None, bb_key=KEY_OBJECT, object=None))
    root.add_child(BtNode_Grasp("Grasp object", bb_source=KEY_OBJECT, service_name=grasp_service_name))
    root.add_child(BtNode_MoveArm("Move arm to hand object", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_HANDING))
    # announce handing object
    root.add_child(BtNode_Announce("Announce object handed", message="Please take the object"))
    root.add_child(BtNode_GripperAction("Open gripper", open_gripper=True))
    # move arm back to hold
    root.add_child(BtNode_MoveArm("Move arm back to hold", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_HOLD))

    return root


