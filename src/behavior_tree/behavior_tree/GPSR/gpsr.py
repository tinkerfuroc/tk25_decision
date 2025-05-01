from pytree.nodes.composites import Sequence, Selector, Repeat
from custom_nodes import (  # 假设你已实现这些自定义节点
    BtNode_WaitForCommand,
    BtNode_DecideNextAction,
    BtNode_CheckIfMyTurn,
    BtNode_UpdateState,
    BtNode_CheckIfCompleted,
    BtNode_ScanFor,
    BtNode_FindObj,
)
from pytree.trees import BehaviourTree

# Import additional nodes from TemplateNodes
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_PhraseExtraction, BtNode_GetConfirmation
from behavior_tree.TemplateNodes.Vision import BtNode_FeatureExtraction, BtNode_SeatRecommend, BtNode_FeatureMatching, BtNode_TrackPerson
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction, BtNode_Goto, BtNode_GotoGrasp, BtNode_CalcGraspPose
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp

# bed, dresser, desk, dining table, storage box, wine rack, sofa, side table, TV cabinet, storage table, sink, dishwasher, bedroom, dining room, living room, kitchen
KEY_BED_POSE = "bed_pose"
KEY_DRESSER_POSE = "dresser_pose"
KEY_DESK_POSE = "desk_pose"
KEY_DINING_TABLE_POSE = "dining_table_pose"
KEY_STORAGE_BOX_POSE = "storage_box_pose"
KEY_WINE_RACK_POSE = "wine_rack_pose"
KEY_SOFA_POSE = "sofa_pose"
KEY_SIDE_TABLE_POSE = "side_table_pose"
KEY_TV_CABINET_POSE = "tv_cabinet_pose"
KEY_STORAGE_TABLE_POSE = "storage_table_pose"
KEY_SINK_POSE = "sink_pose"
KEY_DISHWASHER_POSE = "dishwasher_pose"
KEY_BEDROOM_POSE = "bedroom_pose"
KEY_DINING_ROOM_POSE = "dining_room_pose"
KEY_LIVING_ROOM_POSE = "living_room_pose"
KEY_KITCHEN_POSE = "kitchen_pose"

# chip, biscuit, bread, sprite, cola, water, dishsoap, handwash, shampoo, cookie, lays
KEY_CHIP = "chip"
KEY_BISCUIT = "biscuit"
KEY_BREAD = "bread"
KEY_SPRITE = "sprite"
KEY_COLA = "cola"
KEY_WATER = "water"
KEY_DISHSOAP = "dishsoap"
KEY_HANDWASH = "handwash"
KEY_SHAMPOO = "shampoo"
KEY_COOKIE = "cookie"
KEY_LAYS = "lays"

def create_decision_tree():
    # === 根节点 ===
    root = Sequence("Root")

    # === Step 1: 等待指令 ===
    wait_command_node = BtNode_WaitForCommand("wait_for_command", bb_command_key="bb/command")

    # === Step 2: 重复执行任务的主体 ===
    task_repeat = Repeat(name="TASK", num_runs=None)  # 无限循环直到被上层终止
    task_seq = Sequence("Task Sequence")

    # 决策：由大模型根据指令、能力、先验、状态推理出下一步
    decide_node = BtNode_DecideNextAction(
        name="decide_next_action",
        bb_command_key="bb/command",
        bb_action_key="bb/next_action",
        bb_info_key="bb/info",
        bb_state_key="bb/state"
    )

    # === 检查是否完成 ===
    check_complete_node = BtNode_CheckIfCompleted("check_if_done", "bb/next_action")

    # === CHOOSE Selector: 根据next_action选择功能分支 ===
    choose_node = Selector("CHOOSE")

    # ------ 子分支 1: Q&A ------
    qa_seq = Sequence("scan_branch")
    qa_guard = BtNode_CheckIfMyTurn("check_qa", "qa", "bb/next_action")
    qa_action = BtNode_Announce("announce", bb_source="bb/announcement") # BtNode_ScanFor("scan_for", obj_key="bb/scan_obj")
    qa_update = BtNode_UpdateState("update_after_qa", completed_action="qa(...)",
                                     bb_state_key="bb/state")
    qa_seq.add_children([qa_guard, qa_action, qa_update])

    # ------ 子分支 2: Announce ------
    announce_seq = Sequence("announce_branch")
    announce_guard = BtNode_CheckIfMyTurn("check_announce", "announce", "bb/next_action")
    announce_action = BtNode_Announce("announce", bb_source="bb/announcement")
    announce_update = BtNode_UpdateState("update_after_announce", completed_action="announce(...)",
                                        bb_state_key="bb/state")
    announce_seq.add_children([announce_guard, announce_action, announce_update])

    # ------ 子分支 3: Goto ------
    goto_seq = Sequence("goto_branch")
    goto_guard = BtNode_CheckIfMyTurn("check_goto", "goto", "bb/next_action")
    goto_action = BtNode_GotoAction("goto", "")# BtNode_Goto("goto", bb_source="bb/target_pose")
    goto_update = BtNode_UpdateState("update_after_goto", completed_action="goto(...)",
                                    bb_state_key="bb/state")
    goto_seq.add_children([goto_guard, goto_action, goto_update])

    # ------ 子分支 4: Grasp ------
    grasp_seq = Sequence("grasp_branch")
    grasp_guard = BtNode_CheckIfMyTurn("check_grasp", "grasp", "bb/next_action")
    grasp_action = BtNode_Grasp("grasp", bb_source="bb/grasp_target")
    grasp_update = BtNode_UpdateState("update_after_grasp", completed_action="grasp(...)",
                                     bb_state_key="bb/state")
    grasp_seq.add_children([grasp_guard, grasp_action, grasp_update])


    # Add all branches to the choose node
    choose_node.add_children([
        qa_seq, announce_seq, goto_seq, grasp_seq
    ])

    # === 组装TASK sequence ===
    task_seq.add_children([decide_node, check_complete_node, choose_node])
    task_repeat.add_child(task_seq)

    # === 组装根节点 ===
    root.add_children([wait_command_node, task_repeat])

    return BehaviourTree(root)
