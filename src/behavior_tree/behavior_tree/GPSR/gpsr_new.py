import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_GetConfirmation, BtNode_Listen
from behavior_tree.TemplateNodes.Vision import BtNode_DoorDetection, BtNode_TurnPanTilt
from behavior_tree.TemplateNodes.Manipulation import BtNode_MoveArmSingle
from behavior_tree.StoringGroceries.customNodes import BtNode_FindObjTable, BtNode_GraspWithPose

from .node_test import DecideNextAction, CheckAndWriteAction, WriteActionSuccessful
from .custom_nodes import BtNode_QA, BtNode_ScanForWavingPerson

import json
import math

ANNOUNCE_REASONING = True

# TODO: read from json file, fill in arms poses, and poses and object dictionary,
try:
    file = open("/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/constants.json", "r")
    constants = json.load(file)
    file.close()
except FileNotFoundError:
    print("ERROR: constants.json not found!")
    raise FileNotFoundError

ARM_POS_NAVIGATING = [x / 180 * math.pi for x in constants["arm_pos_navigating"]]
ARM_POS_SCAN = [x / 180 * math.pi for x in constants["arm_pos_scan"]]

KEY_INSTRUCTION = "instruction"
KEY_TASK_STATUS = "task_status"
KEY_PAST_ACTIONS = "past_actions"
KEY_NEXT_ACTION = "next_action"
KEY_NEXT_ACTION_PARAMS = "next_action_parameters"
KEY_NEXT_ACTION_REASONING = "next_action_reasoning"
KEY_TARGET_POSE = "pose_target"
KEY_TARGET_OBJECT = "target_object"
KEY_ANNOUNCE_TEXT = "announce_text"

KEY_TABLE_IMG = "table_img"
KEY_OBJ_SEG = "object_segmentation"
KEY_OBJECT = "object"
KEY_POINT_PLACE = "point_place"
KEY_POINT_PLACE_DUMMY = "point_place_dummy"
KEY_ENV_POINTS = "env_points"
KEY_GRASP_ANNOUNCEMENT = "grasp_announcement"
KEY_ARM_NAVIGATING = "arm_navigating"
KEY_ARM_SCAN = "arm_scan"
KEY_GRASP_POSE = "grasp_pose"

KEY_QA_ANSWER = "qa_answer"
KEY_POSE_WAVING_PERSON = "waving_person"

arm_service_name = "arm_joint_service"
grasp_service_name = "start_grasp"
point_target_frame = "base_link"

def createConstantWriter():
    root = py_trees.composites.Sequence("Root", memory=True)
    root.add_child(BtNode_WriteToBlackboard("Write Arm Scan", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN, object=ARM_POS_SCAN))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Navigating", bb_namespace="", bb_source=None, bb_key=KEY_ARM_NAVIGATING, object=ARM_POS_NAVIGATING))
    return root

def createGoto():
    return py_trees.decorators.Retry(
        name="retry",
        child=BtNode_GotoAction("go to", KEY_TARGET_POSE),
        num_failures=5
    )

def createGotoWaving():
    root = py_trees.composites.Sequence("root of goto_waving", True)
    root.add_child(BtNode_ScanForWavingPerson("find waving person", KEY_POSE_WAVING_PERSON, use_orbbec=True, target_frame="base_link"))
    root.add_child(BtNode_Announce("announce found waving person", None, message="Found waving person. Dear person, could you move a little so I can reach you?"))
    root.add_child(py_trees.decorators.Retry(BtNode_GotoAction(
        name="goto waving person",
        key=KEY_POSE_WAVING_PERSON
    )))
def createGrasp():
    root = py_trees.composites.Sequence(name="Grasp Once", memory=True)
    root.add_child(BtNode_TurnPanTilt(name='turn pantilt', x=0.0, y=20.0))
    root.add_child(BtNode_MoveArmSingle("Move arm to find", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False))
    parallel_move_arm = py_trees.composites.Parallel("Move arm to find object", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_move_arm.add_child(BtNode_Announce(name="Announce moving arm", bb_source=None, message="Moving arm to find object"))
    parallel_move_arm.add_child(BtNode_MoveArmSingle("Move arm to find", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN, add_octomap=True))
    root.add_child(parallel_move_arm)

    find_and_grasp = py_trees.composites.Sequence(name="find and grasp", memory=True)
    # find object on table
    find_and_grasp.add_child(BtNode_FindObjTable("Find object on table", KEY_TARGET_OBJECT, KEY_TABLE_IMG, KEY_OBJ_SEG, KEY_OBJECT, KEY_GRASP_ANNOUNCEMENT))
    # add parallel node to grasp and announcing it is grasping
    parallel_grasp = py_trees.composites.Parallel("Parallel Grasp", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_grasp.add_child(BtNode_Announce(name="Announce grasping", bb_source=KEY_GRASP_ANNOUNCEMENT))
    parallel_grasp.add_child(BtNode_GraspWithPose(f"Grasp object on table", bb_key_vision_res=KEY_OBJECT, bb_key_pose=KEY_GRASP_POSE, action_name=grasp_service_name))
    find_and_grasp.add_child(parallel_grasp)

    root.add_child(find_and_grasp)
    root.add_child(py_trees.decorators.Retry('retry', BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING), 5))
    return py_trees.decorators.Retry(name=f"retry 3 times", child=root, num_failures=3)


def createAnnounce():
    return BtNode_Announce(name="announce message", bb_source=KEY_ANNOUNCE_TEXT)

def createQA():
    root = py_trees.composites.Sequence(name="QA root", memory=True)
    root.add_child(BtNode_QA("QA node", KEY_QA_ANSWER))
    root.add_child(BtNode_Announce("announce answer", KEY_QA_ANSWER))
    return root


def createExecuteAction():
    execute_one_step = py_trees.composites.Sequence("Root", True)

    decide_node = DecideNextAction()

    action_selector = py_trees.composites.Selector("ActionSelector", True)

    # Goto
    goto_branch = py_trees.composites.Sequence("Goto", True)
    goto_branch.add_children([
        CheckAndWriteAction("goto"),
        createGoto(),
        WriteActionSuccessful("goto")
    ])

    # Goto waving
    waving_branch = py_trees.composites.Sequence("GotoWaving", True)
    waving_branch.add_children([
        CheckAndWriteAction("goto_waving"),
        createGotoWaving(),
        WriteActionSuccessful("goto_waving")
    ])

    # Grasp
    grasp_branch = py_trees.composites.Sequence("Grasp", True)
    grasp_branch.add_children([
        CheckAndWriteAction("grasp"),
        createGrasp(),
        WriteActionSuccessful("grasp")
    ])

    # Announce
    announce_branch = py_trees.composites.Sequence("Announce", True)
    announce_branch.add_children([
        CheckAndWriteAction("announce"),
        createAnnounce(),
        WriteActionSuccessful("announce")
    ])

    # QA
    qa_branch = py_trees.composites.Sequence("QA", True)
    qa_branch.add_children([
        CheckAndWriteAction("qa"),
        createQA(),
        WriteActionSuccessful("qa")
    ])

    action_selector.add_children([
        goto_branch,
        waving_branch,
        grasp_branch,
        announce_branch,
        qa_branch
    ])

    finished_guard = CheckAndWriteAction("finished", name="check if finished")
    execute_action = py_trees.composites.Selector(
        "execute action selector", 
        True, 
        [py_trees.decorators.SuccessIsFailure("S is F", action_selector), finished_guard]
        )

    execute_one_step.add_child(BtNode_Announce("announce planning", bb_source=None, message="Planning next action"))
    execute_one_step.add_child(decide_node)
    if ANNOUNCE_REASONING:
        execute_one_step.add_child(BtNode_Announce("announce reasoning", bb_source=KEY_NEXT_ACTION_REASONING))
    execute_one_step.add_child(execute_action)

    execute_action = py_trees.decorators.Retry("Retry until success", execute_one_step, num_failures=10)
    return execute_action