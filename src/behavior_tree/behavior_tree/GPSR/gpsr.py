from py_trees.composites import Sequence, Selector, Parallel
from .custom_nodes import (
    BtNode_GetCommand,
    BtNode_DecideNextAction,
    BtNode_CheckIfMyTurn,
    BtNode_UpdateState,
    BtNode_CheckIfCompleted,
    BtNode_QA,
    BtNode_WritePose,
    BtNode_WriteVisionPrompt
)
from py_trees.trees import BehaviourTree
from py_trees import decorators
import py_trees

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy

# Import additional nodes from TemplateNodes
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_PhraseExtraction, BtNode_GetConfirmation
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj, BtNode_TrackPerson, BtNode_DoorDetection
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArmSingle
from behavior_tree.Receptionist.customNodes import BtNode_Confirm
import math

ARM_POS_NAVIGATING = [x / 180 * math.pi for x in [-87.0, -40.0, 28.0, 0.0, 30.0, -86.0, 0.0]]
ARM_POS_SCAN = [x / 180 * math.pi for x in [0.0, -50.0, 0.0, 66.0, 0.0, 55.0, 0.0]]

pose_command = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=6.3375858, y=5.419048, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=0.721331807, w=0.692589650))
                        )

pose_bed = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=-1.8183577060699463, y=-0.5918460488319397, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
                        )
pose_dresser = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=-1.8183577060699463, y=-0.5918460488319397, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
                        )
pose_desk = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=-1.8183577060699463, y=-0.5918460488319397, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
                        )
pose_dining_table = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=-1.8183577060699463, y=-0.5918460488319397, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
                        )
pose_storage_box = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=-1.8183577060699463, y=-0.5918460488319397, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
                        )
pose_wine_rack = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=-1.8183577060699463, y=-0.5918460488319397, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
                        )
pose_sofa = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=-1.8183577060699463, y=-0.5918460488319397, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
                        )
pose_side_table = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=-1.8183577060699463, y=-0.5918460488319397, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
                        )
pose_tv_cabinet = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=-1.8183577060699463, y=-0.5918460488319397, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
                        )
pose_storage_table = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=-1.8183577060699463, y=-0.5918460488319397, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
                        )
pose_sink = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=-1.8183577060699463, y=-0.5918460488319397, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
                        )
pose_dishwasher = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=-1.8183577060699463, y=-0.5918460488319397, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
                        )
pose_bedroom = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=-1.8183577060699463, y=-0.5918460488319397, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
                        )
pose_dining_room = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=-1.8183577060699463, y=-0.5918460488319397, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
                        )
pose_living_room = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=-1.8183577060699463, y=-0.5918460488319397, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
                        )
pose_kitchen = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=-1.8183577060699463, y=-0.5918460488319397, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
                        )

KEY_DEST_POSE = "dest_pose"
KEY_COMMAND_POSE = "command_pose"

KEY_ARM_POSE = "arm_pose"
KEY_GRASP_PROMPT = "grasp_prompt"
KEY_OBJECT = "object"

KEY_ARM_NAVIGATING = "arm_navigating"

KEY_QNA_ANSWER = "qna_answer"
KEY_DOOR_STATUS = "door_status"

arm_service_name = "arm_joint_service"
grasp_service_name = "start_grasp"
place_service_name = "place_service"

def createConstantWriter():
    root = Sequence("Root", memory=True)
    root.add_child(BtNode_WriteToBlackboard("Write to blackboard", "", None, KEY_ARM_POSE, ARM_POS_SCAN))
    root.add_child(BtNode_WriteToBlackboard("Write to blackboard", "", None, KEY_ARM_NAVIGATING, ARM_POS_NAVIGATING))
    root.add_child(BtNode_WriteToBlackboard("Write to blackboard", "", None, KEY_COMMAND_POSE, pose_command))
    return root

def createEnterArena():
    root = Sequence(name="Enter arena", memory=True)
    
    root.add_child(decorators.Retry(name="retry", child=BtNode_DoorDetection(name="Door detection", bb_door_state_key=KEY_DOOR_STATUS), num_failures=999))
    parallel_enter_arena = Parallel("Enter arena", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_enter_arena.add_child(BtNode_Announce(name="Announce entering arena", bb_source=None, message="Entering arena"))
    # parallel_enter_arena.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to table", service_name="move_base", target_pose=POS_TABLE, target_frame=point_target_frame)))
    parallel_enter_arena.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to table", key=KEY_COMMAND_POSE), num_failures=5))
    root.add_child(parallel_enter_arena)
    return root

def create_run_command():
    # === 根节点 ===
    root = Sequence("Root", True)

    # === Step 1: 等待指令 ===
    wait_command_node = Sequence(name=f"Get {type}", memory=True)
    loop = Sequence(name=f"get and confirm {type}", memory=True)
    loop.add_child(BtNode_Announce(name=f"Prompt for getting command", bb_source="", message=f"Please tell me your command"))
    loop.add_child(BtNode_GetCommand(name="get command", bb_dest_key="bb/command"))
    loop.add_child(BtNode_Confirm(name=f"Confirm command", key_confirmed="bb/command", type="command"))
    loop.add_child(BtNode_GetConfirmation(name=f"Get confirmation", timeout=5.0))
    wait_command_node.add_child(decorators.Retry(name="retry", child=loop, num_failures=10))

    # === Step 2: 重复执行任务的主体 ===
    task_seq = Sequence("Task Sequence", True)

    # 决策：由大模型根据指令、能力、先验、状态推理出下一步
    decide_node = BtNode_DecideNextAction(
        name="decide_next_action",
        bb_command="bb/command",
        bb_action_list="qa, announce, goto, grasp",
        bb_state="bb/state",
        bb_next_action="bb/next_action",
        bb_params="bb/params"
    )

    # === 检查是否完成 ===
    check_complete_node = BtNode_CheckIfCompleted("check_if_done", "bb/next_action")

    # === CHOOSE Selector: 根据next_action选择功能分支 ===
    choose_node = Selector("CHOOSE")

    # ------ 子分支 1: Q&A ------
    qa_seq = Sequence("scan_branch", True)
    qa_guard = BtNode_CheckIfMyTurn("check_qa", "qa", "bb/next_action")
    qa_action = BtNode_QA(name="QnA", bb_key_dest=KEY_QNA_ANSWER, timeout=7.0)
    answer = BtNode_Announce(name="Announce answer", bb_source=KEY_QNA_ANSWER)
    qa_update = BtNode_UpdateState("update_after_qa", bb_params="bb/params",
                                     bb_state_key="bb/state")
    qa_seq.add_children([qa_guard, qa_action, answer, qa_update])

    # ------ 子分支 2: Announce ------
    announce_seq = Sequence("announce_branch", True)
    announce_guard = BtNode_CheckIfMyTurn("check_announce", "announce", "bb/next_action")
    announce_action = BtNode_Announce("announce", bb_source="bb/params")
    announce_update = BtNode_UpdateState("update_after_announce", bb_params="bb/params",
                                        bb_state_key="bb/state")
    announce_seq.add_children([announce_guard, announce_action, announce_update])

    # ------ 子分支 3: Goto ------
    goto_seq = Sequence("goto_branch", True)
    goto_guard = BtNode_CheckIfMyTurn("check_goto", "goto", "bb/next_action")
    get_pose_stamped = BtNode_WritePose("get pose", "bb/params", KEY_DEST_POSE)
    goto_action = BtNode_GotoAction("goto", KEY_DEST_POSE)# BtNode_Goto("goto", bb_source="bb/target_pose")
    goto_update = BtNode_UpdateState("update_after_goto", bb_params="bb/params",
                                    bb_state_key="bb/state")
    goto_seq.add_children([goto_guard, get_pose_stamped, goto_action, goto_update])

    # ------ 子分支 4: Grasp ------
    grasp_seq = Sequence("grasp_branch")
    grasp_guard = BtNode_CheckIfMyTurn("check_grasp", "grasp", "bb/next_action")
    # TODO: add move arm pose
    move_arm = BtNode_MoveArmSingle(name="Move arm to find obj", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_POSE, add_octomap=True)
    match_prompt = BtNode_WriteVisionPrompt(name="convert class to prompt", bb_key_params="bb/params", bb_key_dest=KEY_GRASP_PROMPT)
    find_obj = BtNode_FindObj(name="Find obj", bb_source=KEY_GRASP_PROMPT, bb_namespace=None, bb_key=KEY_OBJECT)
    grasp = BtNode_Grasp(name="grasp object", bb_source=KEY_OBJECT, service_name=grasp_service_name)
    move_arm = BtNode_MoveArmSingle(name="Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING)
    grasp_update = BtNode_UpdateState("update_after_grasp", bb_params="bb/params",
                                     bb_state_key="bb/state")
    grasp_seq.add_children([grasp_guard, move_arm, match_prompt, find_obj, grasp, grasp_update])

    # Add all branches to the choose node
    choose_node.add_children([
        qa_seq, announce_seq, goto_seq, grasp_seq
    ])

    # === 组装TASK sequence ===
    task_seq.add_children([decide_node, check_complete_node, choose_node])
    task_repeat = decorators.Repeat(name="TASK", num_runs=999, child=task_seq)  # 无限循环直到被上层终止

    # === 组装根节点 ===
    goto_command_pose = BtNode_GotoAction(name="go to command position", key=KEY_COMMAND_POSE)
    root.add_children([goto_command_pose, wait_command_node, task_repeat])

    return root

def createGPSR():
    root = Sequence("GPSR", True)
    root.add_child(createConstantWriter())
    root.add_child(createEnterArena())
    root.add_child(decorators.Repeat("repeat 3 times", create_run_command(), 3))
    return root