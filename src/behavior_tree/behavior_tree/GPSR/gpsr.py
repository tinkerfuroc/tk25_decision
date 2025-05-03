from py_trees.composites import Sequence, Selector, Parallel
from .custom_nodes import (
    BtNode_GetCommand,
    BtNode_DecideNextAction,
    BtNode_CheckIfMyTurn,
    BtNode_UpdateState,
    BtNode_CheckIfCompleted,
    BtNode_QA,
    BtNode_WritePose,
    BtNode_WriteVisionPrompt,
    BtNode_ScanForWavingPerson
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
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction, BtNode_ComputeGraspPose
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArmSingle
from behavior_tree.Receptionist.customNodes import BtNode_Confirm
import math

# ARM_POS_NAVIGATING = [x / 180 * math.pi for x in [-87.0, -40.0, 28.0, 0.0, 30.0, -86.0, 0.0]]
ARM_POS_NAVIGATING = [x / 180 * math.pi for x in [-87.0, -44.0, 26.0, 20.0, 30.0, -92.0, 0.0]]
ARM_POS_SCAN = [x / 180 * math.pi for x in [0.0, -50.0, 0.0, 66.0, 0.0, 55.0, 0.0]]

pose_command = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=6.3375858, y=5.419048, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=0.721331807, w=0.692589650))
                        )

pose_bed = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=1.566056489944458, y=6.0895161628723145, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=-0.9761228067763232, w=0.21721939621293632))
                        )
pose_dresser = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=3.405961036682129, y=8.099417686462402, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=0.8461802466425077, w=0.5328967913133131))
                        )
pose_desk = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=2.5116984844207764, y=5.356934070587158, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=-0.555381997771819, w=0.8315953562586694))
                        )
pose_dining_table = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=8.548828125, y=9.025460243225098, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=-0.5326669244808017, w=0.8463249656982618))
                        )
pose_storage_box = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=7.824709892272949, y=9.984259605407715, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=0.8371160641822188, w=0.5470253148512153))
                        )
pose_wine_rack = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=11.366836547851562, y=8.5852632522583, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=-0.5425000369225286, w= 0.8400557778737404))
                        )
pose_sofa = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=4.526828289031982, y=1.6860325336456299, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=0.8425765114202682, w=0.5385766634406384))
                        )
pose_side_table = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=5.3602375984191895, y=2.7648749351501465, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=0.838311316272591, w=0.545191835053787))
                        )
pose_tv_cabinet = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=5.161995887756348, y=0.9691050052642822, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=-0.5424035116291378, w=0.8401181051330698))
                        )
pose_storage_table = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=7.924943923950195, y=9.983813285827637, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=0.8475493057578626, w=0.530716661043692))
                        )
pose_sink = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=11.239192008972168, y=3.921527862548828, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=-0.5446631727713234, w=0.8386548922093495))
                        )
pose_dishwasher = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=8.515981674194336, y=4.641963481903076, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=0.8289533697617991, w=0.5593177189760378))
                        )
pose_bedroom = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=4.347743034362793, y=6.498143672943115, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z= -0.9773902226260489, w= 0.2114434976891046))
                        )
pose_dining_room = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=6.191153049468994, y= 7.793015480041504, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=0.2109386874061353, w=0.9774992941968689))
                        )
pose_living_room = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=7.179622173309326, y=3.1644139289855957, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=-0.978254684315891, w= 0.20740726268387102))
                        )
pose_kitchen = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=8.172871589660645, y=4.122262477874756, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=0.20610733781268523, w=0.978529389083316))
                        )
pose_stove = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=10.980513572692871, y=5.860569000244141, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=0.8328154566974134, w= 0.553550733976461))
                        )
pose_wardrobe = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=12.675935745239258, y=5.432043552398682, z=0.0), 
                        orientation=Quaternion(x=0.0, y=0.0, z=0.2077634421957669, w= 0.9781791002096529))
                        )

KEY_POSE_BEDROOM = "bedroom_pose"
KEY_POSE_DININGROOM = "diningroom_pose"
KEY_POSE_KITCHEN = "kitchen_pose"
KEY_POSE_LIVINGROOM = "livingroom_pose"
# scan_poses = [KEY_POSE_BEDROOM, KEY_POSE_DININGROOM, KEY_POSE_KITCHEN, KEY_POSE_LIVINGROOM]
scan_poses = [KEY_POSE_LIVINGROOM, KEY_POSE_KITCHEN, KEY_POSE_DININGROOM, KEY_POSE_BEDROOM]
KEY_DEST_POSE = "dest_pose"
KEY_COMMAND_POSE = "command_pose"

KEY_ARM_POSE = "arm_pose"
KEY_GRASP_PROMPT = "grasp_prompt"
KEY_OBJECT = "object"
KEY_GRASP_POSE = "go_to_grasp_pose"

KEY_ARM_NAVIGATING = "arm_navigating"

KEY_QNA_ANSWER = "qna_answer"
KEY_DOOR_STATUS = "door_status"

KEY_WAVING_PERSON = "waving person"

arm_service_name = "arm_joint_service"
grasp_service_name = "start_grasp"
place_service_name = "place_service"

def createConstantWriter():
    root = Sequence("Root", memory=True)
    root.add_child(BtNode_WriteToBlackboard("Write to blackboard", "", KEY_ARM_POSE, None, ARM_POS_SCAN))
    root.add_child(BtNode_WriteToBlackboard("Write to blackboard", "", KEY_ARM_NAVIGATING, None, ARM_POS_NAVIGATING))
    root.add_child(BtNode_WriteToBlackboard("Write to blackboard", "", KEY_COMMAND_POSE, None, pose_command))
    root.add_child(BtNode_WriteToBlackboard("Write to blackboard", "", KEY_POSE_BEDROOM, None, pose_bedroom))
    root.add_child(BtNode_WriteToBlackboard("Write to blackboard", "", KEY_POSE_DININGROOM, None, pose_dining_room))
    root.add_child(BtNode_WriteToBlackboard("Write to blackboard", "", KEY_POSE_KITCHEN, None, pose_kitchen))
    root.add_child(BtNode_WriteToBlackboard("Write to blackboard", "", KEY_POSE_LIVINGROOM, None, pose_living_room))
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

def createScanForWaving(idx):
    root = Sequence("scan for waving", True)

    root.add_child(decorators.Retry("retry", BtNode_GotoAction(f"goto scan pos {idx}", scan_poses[idx]), 3))
    # TODO: Add scan for waving person, store to blackboard
    root.add_child(BtNode_ScanForWavingPerson("scan for waving person", bb_target=KEY_WAVING_PERSON, use_orbbec=True, target_frame="map"))
    return root

def createFindPerson():
    root = Selector("selector", True)
    for i in range(4):
        root.add_child(createScanForWaving(i))
    root.add_child(decorators.SuccessIsFailure("SisF", BtNode_Announce("announce failed to find person", None, message="Failed to find waving person")))

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
    loop.add_child(BtNode_Announce("received command", None, message="Command received, will execute later"))
    wait_command_node.add_child(decorators.Retry(name="retry", child=loop, num_failures=10))

    find_person_node = createFindPerson()

    go_to_person_node = Sequence("go to person", memory=True)
    go_to_person_node.add_child(BtNode_ComputeGraspPose("compute grasp pose", bb_src=KEY_WAVING_PERSON, bb_target=KEY_GRASP_POSE))
    go_to_person_node.add_child(py_trees.decorators.Retry('retry', BtNode_GotoAction('go to person', key=KEY_GRASP_POSE), 3))


    # # === Step 2: 重复执行任务的主体 ===wait_command_node
    # task_seq = Sequence("Task Sequence", True)

    # # 决策：由大模型根据指令、能力、先验、状态推理出下一步
    # decide_node = BtNode_DecideNextAction(
    #     name="decide_next_action",
    #     bb_command="bb/command",
    #     bb_action_list="qa, announce, goto, grasp",
    #     bb_state="bb/state",
    #     bb_next_action="bb/next_actiAction(name="go to command position", key=KEY_COMMAND_POSE)
    # root.add_child(goto_command_pose)on",
    #     bb_params="bb/params"Action(name="go to command position", key=KEY_COMMAND_POSE)
    # root.add_child(goto_command_pose)
    # # === CHOOSE Selector: 根据next_action选择功能分支 ===
    # choose_node = Selector("CHOOSE", True)

    # # ------ 子分支 1: Q&A ------
    # qa_seq = Sequence("scan_branch", True)
    # qa_guard = BtNode_CheckIfMyTurn("check_qa", "qa", "bb/next_action")
    # qa_action = BtNode_QA(name="QnA", bb_key_dest=KEY_QNA_ANSWER, timeout=7.0)
    # answer = BtNode_Announce(name="Announce answer", bb_source=KEY_QNA_ANSWER)
    # qa_update = BtNode_UpdateState("update_after_qa", bb_params="bb/params",
    #                                  bb_state_key="bb/state")
    # qa_seq.add_children([qa_guard, qAction(name="go to command position", key=KEY_COMMAND_POSE)
    # root.add_child(goto_command_pose)a_action, answer, qa_update])

    # # ------ 子分支 2: Announce ------
    # announce_seq = Sequence("announce_branch", True)
    # announce_guard = BtNode_CheckIfMyTurn("check_announce", "announce", "bb/next_action")
    # announce_action = BtNode_Announce("announce", bb_source="bb/params")
    # announce_update = BtNode_UpdateState("update_after_announce", bb_params="bb/params",
    #                                     bb_state_key="bb/state")
    # announce_seq.add_children([announce_guard, announce_action, announce_update])

    # # ------ 子分支 3: Goto ------
    # goto_seq = Sequence("goto_branch", True)
    # goto_guard = BtNode_CheckIfMyTurn("check_goto", "goto", "bb/next_action")
    # get_pose_stamped = BtNode_WritePose("get pose", "bb/params", KEY_DEST_POSE)
    # goto_action = BtNode_GotoAction("goto", KEY_DEST_POSE)# BtNode_Goto("goto", bb_source="bb/target_pose")
    # goto_update = BtNode_UpdateState("update_after_goto", bb_params="bb/params",
    #                                 bb_state_key="bb/state")
    # goto_seq.add_children([goto_guard, get_pose_stamped, goto_action, goto_update])

    # # ------ 子分支 4: Grasp ------
    # grasp_seq = Sequence("grasp_branch", True)
    # grasp_guard = BtNode_CheckIfMyTurn("check_grasp", "grasp", "bb/next_action")
    # # TODO: add move arm pose
    # move_arm = BtNode_MoveArmSingle(name="Move arm to find obj", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_POSE, add_octomap=True)
    # match_prompt = BtNode_WriteVisionPrompt(name="convert class to prompt", bb_key_params="bb/params", bb_key_dest=KEY_GRASP_PROMPT)
    # find_obj = BtNode_FindObj(name="Find obj", bb_source=KEY_GRASP_PROMPT, bb_namespace=None, bb_key=KEY_OBJECT)
    # grasp = BtNode_Grasp(name="grasp object", bb_source=KEY_OBJECT, service_name=grasp_service_name)
    # move_arm = BtNode_MoveArmSingle(name="Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING)
    # grasp_update = BtNode_UpdateState("update_after_grasp", bb_params="bb/params",
    #                                  bb_state_key="bb/state")
    # grasp_seq.add_children([grasp_guard, move_arm, match_prompt, find_obj, grasp, grasp_update])

    # # Add all branches to the choose node
    # choose_node.add_children([
    #     qa_seq, announce_seq, goto_seq, grasp_seq
    # ])

    # # === 组装TASK sequence ===
    # task_seq.add_children([BtNode_Announce("announce", None, message="Deciding next action"), decide_node, check_complete_node, 
    #                        BtNode_Announce("announce", "bb/command"),
    #                        choose_node])
    # task_repeat = decorators.Repeat(name="TASK", child=task_seq, num_success=999)  # 无限循环直到被上层终止

    # === 组装根节点 ===
    # goto_command_pose = BtNode_GotoAction(name="go to command position", key=KEY_COMMAND_POSE)
    # root.add_child(goto_command_pose)
    root.add_child(decorators.Retry("retry", find_person_node, 3))
    root.add_child(BtNode_Announce("getting command", None, message="Getting next command"))
    root.add_child(go_to_person_node)
    root.add_child(wait_command_node)
    # root.add_child(task_repeat)
    # root.add_children([goto_command_pose, wait_command_node, task_repeat])

    return decorators.Repeat("repeat", root, 3)

def createGPSR():
    root = Sequence("GPSR", True)
    root.add_child(createConstantWriter())
    root.add_child(createEnterArena())
    # root.add_child(create_run_command())
    root.add_child(decorators.Repeat("repeat 3 times", create_run_command(), 3))
    # root.add_child(decorators.Retry("retry", BtNode_ScanForWavingPerson("scan for waving person", bb_target=KEY_WAVING_PERSON, use_orbbec=True, target_frame=""), 5))
    return root