import py_trees
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArm, BtNode_GripperAction
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj, BtNode_ScanFor

KEY_ARM_POS_SCAN = "scan"
KEY_ARM_POS_ROBOT_MOVING = "robot_moving"
KEY_ARM_POS_HANDOVER = "handover"
KEY_POS_KITCHEN = "kitchen"
KEY_POS_STARTING = "starting"



def create_cleanup_trash_tree():
    # Top-level sequence for cleaning up trash on the floor
    root = py_trees.composites.Sequence(name="Cleanup Trash", memory=True)
    
    # Announce start
    root.add_child(BtNode_Announce(name="Announce start cleanup", message="Starting cleanup of trash on the floor"))
    
    # Sequence to pick up a single trash item.
    pick_trash_seq = py_trees.composites.Sequence(name="Pick Trash Sequence", memory=True)
    # Step 1: Scan for trash on the floor using the top camera.
    pick_trash_seq.add_child(BtNode_ScanFor(name="Scan for trash", obj_type="trash", key_object_position="trash_position"))
    # Step 2: Go to the location of the trash according to the scan.
    pick_trash_seq.add_child(BtNode_GotoAction(name="Go to trash", key_target="trash_position"))
    # Step 3: Position the arm to a scanning position.
    pick_trash_seq.add_child(BtNode_MoveArm(name="Move arm to scan pos", key_arm_pos=KEY_ARM_POS_SCAN))
    # Step 4: Use the arm’s camera to find the trash more precisely.
    pick_trash_seq.add_child(BtNode_FindObj(name="Find trash object", obj_type="trash", key_extracted_obj_segment="trash_segment"))
    # Step 5: Attempt to grasp the trash.
    pick_trash_seq.add_child(BtNode_Grasp(name="Grasp trash", key_obj_segment="trash_segment"))
    # Step 6: Announce that the trash has been picked up.
    pick_trash_seq.add_child(BtNode_Announce(name="Announce pickup", message="Trash picked up"))
    # Step 7: Move arm to a transport position.
    pick_trash_seq.add_child(BtNode_MoveArm(name="Move arm to carrying pos", key_arm_pos=KEY_ARM_POS_ROBOT_MOVING))
    # Step 8: Go to a designated trash bin location.
    # (Here we assume the trash bin is at the kitchen location.)
    pick_trash_seq.add_child(BtNode_GotoAction(name="Go to trash bin", key_target=KEY_POS_KITCHEN))
    # Step 9: Announce depositing.
    pick_trash_seq.add_child(BtNode_Announce(name="Announce deposit", message="Depositing trash"))
    # Step 10: Open the gripper to release the trash.
    pick_trash_seq.add_child(BtNode_GripperAction(name="Release trash", open_gripper=True))
    # Step 11: Reset the arm for the next iteration.
    pick_trash_seq.add_child(BtNode_MoveArm(name="Reset arm", key_arm_pos=KEY_ARM_POS_ROBOT_MOVING))
    
    # The RepeatUntilFailure decorator will keep executing the pick_trash_seq 
    # until it fails (i.e. until no trash is found during the scan).
    repeat_until_failure = py_trees.decorators.FailureIsSuccess(name="repeat until failure", child=py_trees.decorators.Repeat(child=pick_trash_seq, name="Repeat Trash Pickup", num_success=999))
    root.add_child(repeat_until_failure)
    
    # Once no more trash is found, announce cleanup completed.
    root.add_child(BtNode_Announce(name="Announce cleanup finished", message="Cleanup complete. No more trash found on the floor"))
    
    return root

def create_grasp_glass_milk():
    """
    This behavior moves the robotic arm to scan for a glass of milk,
    finds the object, grasps it (while announcing the action) and moves the arm back.
    """
    # Define a new blackboard key for the extracted glass of milk segment
    KEY_GLASS_MILK_SEGMENT = "glass_milk_segment"
    
    grasp_sequence = py_trees.composites.Sequence(name="Grasp Glass of Milk", memory=True)
    
    # Move arm to scan position
    grasp_sequence.add_child(BtNode_MoveArm(behavior_name="Move arm to scan position", key_arm_pos=KEY_ARM_POS_SCAN))
    
    # Find the glass of milk using the arm camera and store the segment in KEY_GLASS_MILK_SEGMENT
    grasp_sequence.add_child(BtNode_FindObj(behavior_name="Find glass of milk", obj_type="glass of milk", key_extracted_obj_segment=KEY_GLASS_MILK_SEGMENT))
    
    # In parallel, announce grasping and execute grasp operation
    parallel_grasp = py_trees.composites.Parallel("Grasp Parallel", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_grasp.add_child(BtNode_Announce(behavior_name="Announce grasping", message="Grasping the glass of milk"))
    parallel_grasp.add_child(BtNode_Grasp(behavior_name="Grasp glass of milk", key_obj_segment=KEY_GLASS_MILK_SEGMENT))
    grasp_sequence.add_child(parallel_grasp)
    
    # Move arm back to a safe pose for robot movement
    grasp_sequence.add_child(BtNode_MoveArm(behavior_name="Move arm back", key_arm_pos=KEY_ARM_POS_ROBOT_MOVING))
    
    return grasp_sequence


def create_get_glass_milk_tree():
    """
    This decision tree defines the behavior for:
    'get me a glass of milk from the kitchen counter'
    
    Steps:
      1. Announce the command.
      2. Navigate to the kitchen counter.
      3. Move arm to scan and search for a glass of milk; then grasp it.
      4. Return back to a base/handover position.
      5. Move arm to handover position.
      6. Announce and release the glass so the user can grab it.
      7. Announce task completion.
    """
    root = py_trees.composites.Sequence(name="Get Glass Milk From Kitchen", memory=True)
    
    # Announce the overall task
    root.add_child(BtNode_Announce(behavior_name="Announce task", message="Getting a glass of milk from the kitchen counter"))
    
    # Go to the kitchen counter location
    goto_kitchen = BtNode_GotoAction(behavior_name="Go to Kitchen", key_target=KEY_POS_KITCHEN)
    # Use a retry decorator in case of navigation issues
    goto_kitchen_retry = py_trees.decorators.Retry(name="Retry Kitchen Navigation", child=goto_kitchen, num_failures=5)
    root.add_child(goto_kitchen_retry)
    
    # Execute grasping routine to pick up the glass of milk
    grasp_glass_milk = create_grasp_glass_milk()
    grasp_glass_milk_retry = py_trees.decorators.Retry(name="Retry Grasping", child=grasp_glass_milk, num_failures=5)
    root.add_child(grasp_glass_milk_retry)
    
    # Navigate back for handover (using starting position as handover location)
    root.add_child(BtNode_Announce(behavior_name="Announce return", message="Returning to handover position"))
    goto_starting = BtNode_GotoAction(behavior_name="Go to Starting Position", key_target=KEY_POS_STARTING)
    goto_starting_retry = py_trees.decorators.Retry(name="Retry Return Navigation", child=goto_starting, num_failures=5)
    root.add_child(goto_starting_retry)
    
    # Handover procedure: announce, move arm to handover position, then release gripper
    root.add_child(BtNode_Announce(behavior_name="Announce handover", message="Preparing to hand you the glass of milk"))
    root.add_child(BtNode_MoveArm(behavior_name="Move arm for handover", key_arm_pos=KEY_ARM_POS_HANDOVER))
    root.add_child(BtNode_Announce(behavior_name="Announce release", message="Please take the glass of milk"))
    root.add_child(BtNode_GripperAction(behavior_name="Release gripper", open_gripper=True))
    
    # Final announcement
    root.add_child(BtNode_Announce(behavior_name="Announce completion", message="Task completed"))
    
    return root

# Create and render the behavior tree for the given task
tree = create_get_glass_milk_tree()
py_trees.display.render_dot_tree(tree, show=True)

# # Create and render the tree
# tree = create_cleanup_trash_tree()
# py_trees.display.render_dot_tree(tree, show=True)
