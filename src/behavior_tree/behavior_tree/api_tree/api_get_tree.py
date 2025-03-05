from openai import OpenAI

# client = OpenAI(
#             api_key="sk-XFUAmBS4uncf0bNn0hVwwayO8FGf0PJNP2mrqTqiUl7odXf2",
#             base_url="https://api.moonshot.cn/v1",
#         )

client = OpenAI(
            api_key="sk-x5xwuU14lqsxUYt6RPZphiSSJwWUaZMtn9tp45ZuRbCV1wTf",
            base_url="https://api.nuwaapi.com/v1",
        )

sys_prompt = 'You are in charge of a robot where decisions are programmed using py_trees.' +\
'\nThere are seven base behavior classes of the robot in addition to the composites and decorators in py_trees. They are already defined in the code' +\
' They are the following:' +\
'\nBtNode_Grasp(behavior_name: str, key_obj_segment: str) - Grasps the object within the given segment on the blackboard with key `key_obj_segment`, SUCCESS if grasped, FAILURE if cannot grasp' + \
'\nBtNode_MoveArm(behavior_name: str, key_arm_pos: str) - moves robotic arm to the given position on the blackboard with key `key_arm_pos`, SUCCESS is arm reached pos, FAILURE if otherwise' + \
'\nBtNode_GripperAction(behavior_name: str, open_gripper: bool) - opens or closes the gripper, SUCCESS is gripper reached desired state, FAILURE if otherwise' + \
'\nBtNode_FindObj(behavior_name: str, obj_type: str, key_extracted_obj_segment: str) - ' + \
'scans with camera on arm in search for objects matching descripton given by `obj_type`, stores the best matching segment on the blackboard at key `key_extracted_obj_segment`' +\
'\nBtNode_ScanFor(behavior_name: str, obj_type: str, key_object_position: str) - ' + \
'scans with camera on top of robot in search for objects matching descripton given by `obj_type`, stores the position of the best matching segment on the blackboard at key `key_object_position`' +\
', returns SUCCESS if object found, FAILURE otherwise' +\
'\nBtNode_Announce(behavior_name: str, message: str) - announces the given message, returns SUCCESS if message is announced' +\
'\nBtNode_GotoAction(behavior_name: str, key_target: str) - goes to the given position on the blackboard with key `key_target`, returns SUCCESS if target reached, FAILURE if not reached (due to obstacles, timeout, etc.)' +\
'\nYou will receive an instruction, and you are to write a decision tree in order to accomplish the task.' +\
'You should announce your actions in parallel to doing them whenever possible so the user knows what the robot is doing.' +\
'\nThe blackboard keys you already have are:' +\
'\n[KEY_ARM_POS_SCAN, KEY_ARM_POS_ROBOT_MOVING, KEY_ARM_POS_HANDOVER, KEY_POS_KITCHEN, KEY_POS_LIVING_ROOM, KEY_POS_BEDROOM, KEY_POS_SOFA, KEY_POS_TABLE, KEY_POS_STARTING]' +\
'\nYou are allowed to name new keys. Do not create new classes. Return with the python code only. You may assume py_treesa and all the base behaviors have been imported.'

# print(sys_prompt)

message1 = "go to the table and get me the red water bottle"
response1 = """
def createGraspOnce():
    root = py_trees.composites.Sequence(name="Grasp Once", memory=True)
    root.add_child(BtNode_MoveArm(name="Move arm to scan position", key_arm_pos=KEY_ARM_POS_SCAN))
    root.add_child(BtNode_FindObj(name="find object", obj_type: "red water bottle", key_extracted_obj_segment='green_bottle_segment'))
    # add parallel node to grasp and announcing it is grasping
    parallel_grasp = py_trees.composites.Parallel("Parallel Grasp", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_grasp.add_child(BtNode_Announce(name="Announce grasping", message="grasping red water bottle"))
    parallel_grasp.add_child(BtNode_Grasp("Grasp red water bottle", key_obj_segment='green_bottle_segment'))
    root.add_child(parallel_grasp)
    root.add_child(BtNode_MoveArm(name="Move arm back", key_arm_pos=KEY_ARM_POS_ROBOT_MOVING))
    return root

root = py_trees.composites.Sequence("Intelligent demo", memory=True)

root.add_child(nBtNode_MoveArm("Move arm back", KEY_ARM_POS_ROBOT_MOVING))
root.add_child(BtNode_Announce("Repeat command", "Getting water from the table"))

root.add_child(BtNode_Announce(name="Announce go to table", message="Going to table"))
# ensures the robot makes it to the table
root.add_child(py_trees.decorators.Retry(name="retry", child=nBtNode_GotoAction("go to table", KEY_POS_TABLE), num_failures=5))
root.add_child(py_trees.decorators.Retry(name="retry grasp", child=createGraspOnce(), num_failures=5))

root.add_child(BtNode_Announce(name="Announce going back", message="Going back to hand bottle over"))
root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="go back", key=KEY_POS_STARTING), num_failures=5))
root.add_child(BtNode_Announce(name="Announce hand bottle over", message="Here is your red water bottle"))
root.add_child(BtNode_MoveArm(name="Move arm to hand bottle over", key_arm_pos=KEY_ARM_POS_HANDOVER))
root.add_child(BtNode_Announce(name="Remind user to grab bottle", message="Please grab hold of the water bottle"))
root.add_child(nBtNode_GripperAction(name=release gripper", open_gripper=True))

root.add_child(BtNode_Announce(name="Announce finished", message="task completed"))

# Create and run the tree
py_trees.display.render_dot_tree(root, show=True)
"""

command = "get me a glass of milk from the kitchen counter"

print("completing")

completion = client.chat.completions.create(
    model = "o3-mini",
    messages = [
        {"role": "system", "content": sys_prompt},
        {"role": "user", "content": message1},
        {"role": "assistant", "content": response1},
        {"role": "user", "content": "The transcribed command is: " + command},
    ],
    temperature = 0.3,
)

print(completion)
result = completion.choices[0].message.content
# result = completion
print(result)