import py_trees
import py_trees.blackboard
import py_trees_ros
import rclpy
import openai
import threading
import json
import time

# === CONFIG ===
client = openai.OpenAI(
    api_key="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",
    base_url="https://openrouter.ai/api/v1"
)
# openai.api_key = "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"  # <-- Put your OpenAI API key here
MODEL_NAME = "gpt-4o"

PROMPT_TEMPLATE = """You are the decision-making module of a household robot.

Your job is to decide the robot’s next action based on:
- A natural language instruction from the user
- The current task completion status
- The list of actions the robot can perform
- Known prior knowledge, including locations and objects

You must output a JSON object containing:
{
  "reasoning": (string explaining your decision),
  "action": (name of the next action to take),
  "parameters": (the parameter to use for that action, if any)
}

Available actions:
1. qa()
   - Input: None
   - Behavior: Prompt the user to ask a question, listen, then answer

2. announce(message)
   - Input: message (string) – the message to speak aloud
   - Behavior: Speak the given message via robot's speaker

3. goto(location)
   - Input: location (string) – a known location
   - Behavior: Move the robot to the specified location

4. grasp(target)
   - Input: target (string) – a known object
   - Behavior: Use the robot arm to grasp the specified object

5. goto_waving()
   - Input: None
   - Behavior: finds and goes to the person waving

6. finished()
   - Input: None
   - Behavior: marks this task as finished

Prior knowledge:
- Known locations: ["bed", "dresser", "desk", "dining table", "storage box", "wine rack", "sofa", "side table", "TV cabinet", "storage table", "sink", "dishwasher", "bedroom", "dining room", "living room", "kitchen"]
- Known objects: ["chip", "biscuit", "bread", "sprite", "cola", "water", "dishsoap", "handwash", "shampoo", "cookie", "lays"]
"""

class DecideNextAction(py_trees.behaviour.Behaviour):
    def __init__(self, 
                 possible_locations:list=["kitchen", "living_room", "storage_cabinet"],
                 possible_objects:list=["cola", "milk"],
                 name="DecideNextAction"):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key("instruction", access=py_trees.common.Access.READ)
        self.blackboard.register_key("task_status", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("past_actions", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("next_action", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("next_action_parameters", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("next_action_reasoning", access=py_trees.common.Access.WRITE)
        self.action_list = [
            "qa",
            "announce",
            "goto",
            "grasp",
            "goto_waving",
            "finished"
        ]
        self.thread = None
        self.response = None
        self.failed = False
        self.previous_instruction = None
        self.possible_locations = possible_locations
        self.possible_objects = possible_objects
        self.prompt = f"""You are the decision-making module of a household robot.

Your job is to decide the robot’s next action based on:
- A natural language instruction from the user
- The current task completion status
- The list of actions the robot can perform
- Known prior knowledge, including locations and objects

You must output a JSON object containing:
{{
  "reasoning": (string explaining your decision),
  "action": (name of the next action to take),
  "parameters": (the parameter to use for that action, if any)
}}

Available actions:
1. qa()
   - Input: None
   - Behavior: Prompt the user to ask a question, listen, then answer

2. announce(message)
   - Input: message (string) – the message to speak aloud
   - Behavior: Speak the given message via robot's speaker

3. goto(location)
   - Input: location (string) – a known location
   - Behavior: Move the robot to the specified location

4. grasp(target)
   - Input: target (string) – a known object
   - Behavior: Use the robot arm to grasp the specified object

5. goto_waving()
   - Input: None
   - Behavior: finds and goes to the person waving

6. finished()
   - Input: None
   - Behavior: marks this task as finished

Prior knowledge:
- Known locations: {possible_locations}
- Known objects: {possible_objects}
"""

    def initialise(self):
        self.logger.info("Initialising async OpenAI call...")
        self.failed = False
        self.response = None

        instruction = self.blackboard.instruction

        # clear blackboard if new instruction is received
        if not self.previous_instruction:
            self.previous_instruction = instruction
            self.blackboard.task_status = "no action taken yet"
            self.blackboard.past_actions = []
        if not self.previous_instruction == instruction:
            self.blackboard.task_status = "no action taken yet"
            self.blackboard.past_actions = []

        # extra safeguard just in case
        if getattr(self.blackboard, "past_actions", None) is None:
            self.blackboard.past_actions = []
        status = self.blackboard.task_status
        past = self.blackboard.past_actions

        user_prompt = PROMPT_TEMPLATE + f"""

Example:
Instruction: "Go to the storage cabinet and get a cola"
Task status: "goto("storage cabinet") successful"
Past actions: []
Output:
{{
  "reasoning": "I am already at the storage cabinet, I should grasp the cola",
  "action": "grasp",
  "parameters": "cola"
}}

Instruction: "{instruction}"
Task status: "{status}"
Past actions: {json.dumps(past)}
Output:
"""

        def call_openai():
            try:
                result = client.chat.completions.create(
                    model=MODEL_NAME,
                    messages=[
                        {"role": "system", "content": "You are a reasoning robot planner."},
                        {"role": "user", "content": user_prompt}
                    ],
                    temperature=0.2
                )
                msg = result.choices[0].message.content.strip()
                # self.logger.info(f"Raw output: {msg}")
                msg = msg.split("```json")[-1]
                msg = msg.split("```")[0]
                self.response = json.loads(msg)
            except Exception as e:
                self.logger.error(f"OpenAI request failed: {e}")
                self.failed = True

        self.thread = threading.Thread(target=call_openai)
        self.thread.start()

    def update(self):
        if self.failed:
            return py_trees.common.Status.FAILURE
        if self.response:
            action_name = self.response.get("action", "no action provided")
            parameters = self.response.get("parameters")
            reasoning = self.response.get("reasoning", f"{action_name}")
            if action_name not in self.action_list:
                print(f"ERROR: action name {action_name} not in action list!")
                self.feedback_message = f"FAILED: action name {action_name} not in action list!"
                return py_trees.common.Status.FAILURE
            self.blackboard.past_actions.append(str(action_name) + "(" + str(parameters) + ")")
            self.feedback_message = f'action: {str(action_name) + "(" + str(parameters) + ")"}'
            self.blackboard.next_action = action_name
            self.blackboard.next_action_parameters = parameters
            self.blackboard.next_action_reasoning = reasoning
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.thread = None

def main():
    # Setup blackboard values
    bb = py_trees.blackboard.Client()
    bb.register_key("instruction", access=py_trees.common.Access.WRITE)
    bb.register_key("task_status", access=py_trees.common.Access.WRITE)
    bb.register_key("past_actions", access=py_trees.common.Access.WRITE)
    bb.register_key("next_step", access=py_trees.common.Access.READ)

    bb.instruction = "Find the person waving in the living room and answer their question"
    bb.task_status = "goto('living room') successful"
    bb.past_actions = []

    # Build tree
    root = py_trees.composites.Sequence("Root", True)
    decider = DecideNextAction()
    root.add_child(decider)

    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=15)

    print("=== Running Behavior Tree ===")
    while tree.tick_tock(period_ms=500, number_of_iterations=20):
        step = bb.next_step if hasattr(bb, "next_step") else None
        if step:
            print(f"\n✅ Next Step:\n{json.dumps(step, indent=2)}")
            break
        time.sleep(0.5)

POSE_DICT = {}

class CheckAndWriteAction(py_trees.behaviour.Behaviour):
    def __init__(self, 
                 expected_action: str, 
                 possible_poses: dict={"kitchen":None, "living_room":None, "storage_cabinet":None},
                 possible_objects: dict={"cola":"something", "milk":"something"},
                 name=None):
        super().__init__(name or f"CheckAndWrite-{expected_action}")
        self.expected_action = expected_action
        self.possible_poses = possible_poses
        self.possible_objects = possible_objects
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key("next_action", access=py_trees.common.Access.READ)
        self.blackboard.register_key("next_action_parameters", access=py_trees.common.Access.READ)
        self.blackboard.register_key("pose_target", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("target_object", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("announce_text", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("task_status", access=py_trees.common.Access.WRITE)

    def update(self):
        action = self.blackboard.next_action
        if action != self.expected_action:
            return py_trees.common.Status.FAILURE

        param = self.blackboard.next_action_parameters

        if self.expected_action == "goto":
            if param in self.possible_poses:
                self.blackboard.pose_target = self.possible_poses.get(param)
            else:
                self.logger.error(f"Unknown location for goto: {param}")
                return py_trees.common.Status.FAILURE

        elif self.expected_action == "grasp":
            if param in self.possible_objects:
                self.blackboard.target_object = self.possible_objects.get(param)
            else:
                self.logger.error(f"Unknown object for grasp: {param}")
                return py_trees.common.Status.FAILURE

        elif self.expected_action == "announce":
            self.blackboard.announce_text = param
        
        # assume failed. update later if successful
        self.blackboard.task_status = f'{action}({str(param)}) failed'

        return py_trees.common.Status.SUCCESS

class WriteActionSuccessful(py_trees.behaviour.Behaviour):
    def __init__(self, expected_action: str, name=None):
        super().__init__(name or f"WriteActionSuccessful-{expected_action}")
        self.expected_action = expected_action
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key("next_action", access=py_trees.common.Access.READ)
        self.blackboard.register_key("next_action_parameters", access=py_trees.common.Access.READ)
        self.blackboard.register_key("task_status", access=py_trees.common.Access.WRITE)

    def update(self):
        action = self.blackboard.next_action
        param = self.blackboard.next_action_parameters
        
        # update to successful
        self.blackboard.task_status = f'{action}({str(param)}) successful'

        return py_trees.common.Status.SUCCESS

def create_action_tree():
    execute_one_step = py_trees.composites.Sequence("Root", True)

    decide_node = DecideNextAction()

    action_selector = py_trees.composites.Selector("ActionSelector", True)

    # Goto
    goto_branch = py_trees.composites.Sequence("Goto", True)
    goto_branch.add_children([
        CheckAndWriteAction("goto"),
        py_trees.behaviours.Success("goto"),
        WriteActionSuccessful("goto")
    ])

    # Goto waving
    waving_branch = py_trees.composites.Sequence("GotoWaving", True)
    waving_branch.add_children([
        CheckAndWriteAction("goto_waving"),
        py_trees.behaviours.Success("goto waving"),
        WriteActionSuccessful("goto_waving")
        # BtNode_GoToWaving()
    ])

    # Grasp
    grasp_branch = py_trees.composites.Sequence("Grasp", True)
    grasp_branch.add_children([
        CheckAndWriteAction("grasp"),
        py_trees.behaviours.Success("grasp"),
        WriteActionSuccessful("grasp")
        # BtNode_Grasp()
    ])

    # Announce
    announce_branch = py_trees.composites.Sequence("Announce", True)
    announce_branch.add_children([
        CheckAndWriteAction("announce"),
        py_trees.behaviours.Success("announce"),
        WriteActionSuccessful("announce")
        # BtNode_Announce()
    ])

    # QA
    qa_branch = py_trees.composites.Sequence("QA", True)
    qa_branch.add_children([
        CheckAndWriteAction("qa"),
        py_trees.behaviours.Success("QA"),
        WriteActionSuccessful("qa")
        # BtNode_QA()
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

    execute_one_step.add_children([decide_node, execute_action])
    execute_action = py_trees.decorators.Retry("Retry until success", execute_one_step, num_failures=10)
    return execute_action

def main_ros2():
    rclpy.init()
    # Setup blackboard values
    bb = py_trees.blackboard.Client()
    bb.register_key("instruction", access=py_trees.common.Access.WRITE)
    bb.register_key("task_status", access=py_trees.common.Access.WRITE)
    bb.register_key("past_actions", access=py_trees.common.Access.WRITE)

    bb.instruction = "Find the person waving in the living room and answer their question"
    bb.task_status = "goto_waving() successful"
    bb.past_actions = ["got(living_room)", "goto_waving()"]

    # # Build tree
    # root = py_trees.composites.Sequence("Root", True)
    # decider = DecideNextAction()
    # root.add_child(decider)

    root = py_trees.composites.Sequence("Root", True)
    execute_action = create_action_tree()
    root.add_children([
        execute_action,
        py_trees.behaviours.Running("running")
    ])

    print("=== Running Behavior Tree ===")
    # Wrap the tree in a ROS-friendly interface
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        # node=node,
        # unicode_tree_debug=True
    )

    # Setup and spin
    tree.setup(timeout=15, node_name="root_node")
    def print_tree(tree):
        print(py_trees.display.unicode_tree(root=tree.root, show_status=True))
        # print(py_trees.display.unicode_blackboard())
    tree.tick_tock(period_ms=500.0,post_tick_handler=print_tree)

    rclpy.spin(tree.node)

    rclpy.shutdown()

if __name__ == "__main__":
    main_ros2()
