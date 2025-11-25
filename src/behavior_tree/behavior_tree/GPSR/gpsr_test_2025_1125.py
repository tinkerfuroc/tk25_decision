import os
import json
import openai

# Set your OpenAI API key
openai.api_key = os.getenv("OPENAI_API_KEY")  # or directly assign: "sk-..."

# Define the available robot actions
ACTIONS = [
    "announce(message)",
    "goto(location)",
    "grasp(target)",
    "place()",
    "scanfor(target)",
    "listen()"
]

# Prior knowledge: Locations
LOCATIONS = [
    "KEY_BED_POSE", "KEY_DRESSER_POSE", "KEY_DESK_POSE", "KEY_DINING_TABLE_POSE",
    "KEY_STORAGE_BOX_POSE", "KEY_WINE_RACK_POSE", "KEY_SOFA_POSE", "KEY_SIDE_TABLE_POSE",
    "KEY_TV_CABINET_POSE", "KEY_STORAGE_TABLE_POSE", "KEY_SINK_POSE", "KEY_DISHWASHER_POSE",
    "KEY_BEDROOM_POSE", "KEY_DINING_ROOM_POSE", "KEY_LIVING_ROOM_POSE", "KEY_KITCHEN_POSE",
    "KEY_COMMAND_POSE", "KEY_ENTRANCE_POSE"
]

# Prior knowledge: Visual targets
VISUAL_TARGETS = [
    "KEY_CHIP", "KEY_BISCUIT", "KEY_BREAD", "KEY_SPRITE", "KEY_COLA", "KEY_WATER",
    "KEY_DISHSOAP", "KEY_HANDWASH", "KEY_SHAMPOO", "KEY_COOKIE", "KEY_LAYS",
    "KEY_BOWL", "KEY_KNIFE", "KEY_HUMAN"
]

# Prior knowledge: Grasp targets
GRASP_TARGETS = [
    "KEY_CHIP_GRASP", "KEY_BISCUIT_GRASP", "KEY_BREAD_GRASP", "KEY_SPRITE_GRASP",
    "KEY_COLA_GRASP", "KEY_WATER_GRASP", "KEY_DISHSOAP_GRASP", "KEY_HANDWASH_GRASP",
    "KEY_SHAMPOO_GRASP", "KEY_COOKIE_GRASP", "KEY_LAYS_GRASP", "KEY_BOWL_GRASP", "KEY_KNIFE_GRASP"
]

# Function to generate the next robot action using GPT
def get_next_action(command, state):
    prompt = f"""
You are the decision-making layer of a service robot.
Available Actions: {ACTIONS}
Locations: {LOCATIONS}
Visual Targets: {VISUAL_TARGETS}
Grasp Targets: {GRASP_TARGETS}

Given a natural language COMMAND and the current STATE of the task, output the next robot action in JSON format.
- Format: {{"action": "action_name", "parameters": {{...}}}}
- Do NOT provide explanations, reasoning, or commentary — only JSON output.

COMMAND: {command}
STATE: {state}
"""
    response = openai.ChatCompletion.create(
        model="gpt-5-mini",
        messages=[
            {"role": "system", "content": "You are a robot task decision engine."},
            {"role": "user", "content": prompt}
        ],
        temperature=0
    )

    # Parse and return JSON
    action_json = response.choices[0].message['content'].strip()
    try:
        return json.loads(action_json)
    except json.JSONDecodeError:
        print("Error: GPT response is not valid JSON:", action_json)
        return None

# Example usage loop
if __name__ == "__main__":
    # Example task
    task_command = "retrieve the bowl from the dining table, hand it to Charlie, he is sitting in the living room"
    task_state = "start"

    while True:
        action = get_next_action(task_command, task_state)
        if action is None:
            break

        print("Next action:", action)

        # Simulate state update (in real implementation, this comes from robot sensors/feedback)
        if action["action"] == "goto":
            task_state = f"arrived at {action['parameters']['target']}"
        elif action["action"] == "scanfor":
            task_state = f"found {action['parameters']['target']}"
        elif action["action"] == "grasp":
            task_state = f"grasped {action['parameters']['target']}"
        elif action["action"] == "announce":
            task_state = f"announced message"
        elif action["action"] == "place":
            task_state = f"placed object"

        # For demonstration, stop after 10 steps
        if task_state.startswith("placed") or task_state.startswith("announced"):
            break
