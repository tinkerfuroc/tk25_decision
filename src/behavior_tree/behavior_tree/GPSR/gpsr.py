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

    # ------ 子分支 1: ScanFor ------
    scan_seq = Sequence("scan_branch")
    scan_guard = BtNode_CheckIfMyTurn("check_scan", "scanfor", "bb/next_action")
    scan_action = BtNode_ScanFor("scan_for", obj_key="bb/scan_obj")
    scan_update = BtNode_UpdateState("update_after_scan", completed_action="scanfor(...)",
                                     bb_state_key="bb/state")
    scan_seq.add_children([scan_guard, scan_action, scan_update])

    # ------ 子分支 2: FindObj ------
    find_seq = Sequence("find_branch")
    find_guard = BtNode_CheckIfMyTurn("check_find", "findobj", "bb/next_action")
    find_action = BtNode_FindObj("find_obj", obj_key="bb/target_obj", category_key="bb/category")
    find_update = BtNode_UpdateState("update_after_find", completed_action="findobj(...)",
                                     bb_state_key="bb/state")
    find_seq.add_children([find_guard, find_action, find_update])

    # ------ 子分支 3: Announce ------
    announce_seq = Sequence("announce_branch")
    announce_guard = BtNode_CheckIfMyTurn("check_announce", "announce", "bb/next_action")
    announce_action = BtNode_Announce("announce", bb_source="bb/announcement")
    announce_update = BtNode_UpdateState("update_after_announce", completed_action="announce(...)",
                                        bb_state_key="bb/state")
    announce_seq.add_children([announce_guard, announce_action, announce_update])

    # ------ 子分支 4: Goto ------
    goto_seq = Sequence("goto_branch")
    goto_guard = BtNode_CheckIfMyTurn("check_goto", "goto", "bb/next_action")
    goto_action = BtNode_GotoAction("goto", "")# BtNode_Goto("goto", bb_source="bb/target_pose")
    goto_update = BtNode_UpdateState("update_after_goto", completed_action="goto(...)",
                                    bb_state_key="bb/state")
    goto_seq.add_children([goto_guard, goto_action, goto_update])

    # ------ 子分支 6: Grasp ------
    grasp_seq = Sequence("grasp_branch")
    grasp_guard = BtNode_CheckIfMyTurn("check_grasp", "grasp", "bb/next_action")
    grasp_action = BtNode_Grasp("grasp", bb_source="bb/grasp_target")
    grasp_update = BtNode_UpdateState("update_after_grasp", completed_action="grasp(...)",
                                     bb_state_key="bb/state")
    grasp_seq.add_children([grasp_guard, grasp_action, grasp_update])

    # ------ 子分支 7: TrackPerson ------
    track_seq = Sequence("track_branch")
    track_guard = BtNode_CheckIfMyTurn("check_track", "track", "bb/next_action")
    track_action = BtNode_Announce("announce", bb_source="bb/announcement")# BtNode_TrackPerson("track_person", bb_namespace="tracking", bb_key="person_location")
    track_update = BtNode_UpdateState("update_after_track", completed_action="track(...)",
                                     bb_state_key="bb/state")
    track_seq.add_children([track_guard, track_action, track_update])

    # ------ 子分支 8: FeatureExtraction ------
    feature_seq = Sequence("feature_branch")
    feature_guard = BtNode_CheckIfMyTurn("check_feature", "feature", "bb/next_action")
    feature_action = BtNode_FeatureExtraction("extract_features", bb_dest_key="bb/features")
    feature_update = BtNode_UpdateState("update_after_feature", completed_action="feature(...)",
                                       bb_state_key="bb/state")
    feature_seq.add_children([feature_guard, feature_action, feature_update])

    # ------ 子分支 9: SeatRecommend ------
    seat_seq = Sequence("seat_branch")
    seat_guard = BtNode_CheckIfMyTurn("check_seat", "seat", "bb/next_action")
    seat_action = BtNode_SeatRecommend("recommend_seat", bb_dest_key="bb/seat_recommendation", 
                                      bb_source_key="bb/persons")
    seat_update = BtNode_UpdateState("update_after_seat", completed_action="seat(...)",
                                    bb_state_key="bb/state")
    seat_seq.add_children([seat_guard, seat_action, seat_update])

    # ------ 子分支 10: FeatureMatching ------
    match_seq = Sequence("match_branch")
    match_guard = BtNode_CheckIfMyTurn("check_match", "match", "bb/next_action")
    match_action = BtNode_FeatureMatching("match_features", bb_dest_key="bb/matches", 
                                         bb_persons_key="bb/persons")
    match_update = BtNode_UpdateState("update_after_match", completed_action="match(...)",
                                     bb_state_key="bb/state")
    match_seq.add_children([match_guard, match_action, match_update])

    # ------ 子分支 11: PhraseExtraction ------
    phrase_seq = Sequence("phrase_branch")
    phrase_guard = BtNode_CheckIfMyTurn("check_phrase", "phrase", "bb/next_action")
    phrase_action = BtNode_PhraseExtraction("extract_phrase", wordlist=[], bb_dest_key="bb/phrase")
    phrase_update = BtNode_UpdateState("update_after_phrase", completed_action="phrase(...)",
                                      bb_state_key="bb/state")
    phrase_seq.add_children([phrase_guard, phrase_action, phrase_update])

    # ------ 子分支 12: GetConfirmation ------
    confirm_seq = Sequence("confirm_branch")
    confirm_guard = BtNode_CheckIfMyTurn("check_confirm", "confirm", "bb/next_action")
    confirm_action = BtNode_GetConfirmation("get_confirmation")
    confirm_update = BtNode_UpdateState("update_after_confirm", completed_action="confirm(...)",
                                       bb_state_key="bb/state")
    confirm_seq.add_children([confirm_guard, confirm_action, confirm_update])

    # ------ 子分支 13: CalcGraspPose ------
    calc_grasp_seq = Sequence("calc_grasp_branch")
    calc_grasp_guard = BtNode_CheckIfMyTurn("check_calc_grasp", "calc_grasp", "bb/next_action")
    calc_grasp_action = BtNode_CalcGraspPose("calc_grasp_pose", bb_source="bb/target_point", 
                                            bb_dest="bb/grasp_pose")
    calc_grasp_update = BtNode_UpdateState("update_after_calc_grasp", completed_action="calc_grasp(...)",
                                          bb_state_key="bb/state")
    calc_grasp_seq.add_children([calc_grasp_guard, calc_grasp_action, calc_grasp_update])

    # Add all branches to the choose node
    choose_node.add_children([
        scan_seq, find_seq, announce_seq, goto_seq, grasp_seq,
        track_seq, feature_seq, seat_seq, match_seq, phrase_seq, confirm_seq,
        calc_grasp_seq
    ])

    # === 组装TASK sequence ===
    task_seq.add_children([decide_node, check_complete_node, choose_node])
    task_repeat.add_child(task_seq)

    # === 组装根节点 ===
    root.add_children([wait_command_node, task_repeat])

    return BehaviourTree(root)
