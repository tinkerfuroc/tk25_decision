"""GPSR full-task entry point.

End-to-end GPSR run: enter the arena, then for each of 3 commands listen → plan
→ execute → return to the instruction point. The orchestrator from
``orchestrator.py`` handles planning, self-monitoring, and self-correction;
this module only wires it into the surrounding mission flow.

Run with::

    ros2 run behavior_tree gpsr-full           # real listen flow
    BT_GPSR_DEBUG_CMD="bring me the coke" ros2 run behavior_tree gpsr-full
"""

import math
import os

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.BaseBehaviors import (
    BtNode_WriteToBlackboard,
    BtNode_WaitTicks,
)
from behavior_tree.TemplateNodes.Audio import (
    BtNode_Announce,
    BtNode_GetConfirmationAction,
    BtNode_ListenAction,
)
from behavior_tree.TemplateNodes.Manipulation import BtNode_MoveArmSingle
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import BtNode_DoorDetection, BtNode_TurnPanTilt

from behavior_tree.visualization import create_post_tick_visualizer

from .orchestrator import (
    create_execute_command,
    create_orchestrator_init,
    load_knowledge_from_constants,
    KNOWN_LOCATIONS,
)
from .small_trees import bb_keys


CONSTANTS_PATH = (
    "/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/"
    "behavior_tree/GPSR/constants.json"
)
ARM_ACTION_NAME = "joint_move_action"
NUM_COMMANDS = 3
DEBUG_COMMAND = os.environ.get("BT_GPSR_DEBUG_CMD", "").strip()


def _load_arm_constants():
    import json
    with open(CONSTANTS_PATH, "r") as fh:
        constants = json.load(fh)
    nav = [x / 180 * math.pi for x in constants["arm_pos_navigating"]]
    scan = [x / 180 * math.pi for x in constants["arm_pos_scan"]]
    return nav, scan


def _load_arm_orbbec_look():
    """Arm pose (radians) that clears the orbbec head camera's view for scanning.
    Falls back to the navigating pose when ``arm_pos_orbbec_look`` is absent, so
    an older constants.json still works (the move is then a harmless no-op)."""
    import json
    with open(CONSTANTS_PATH, "r") as fh:
        constants = json.load(fh)
    key = "arm_pos_orbbec_look" if "arm_pos_orbbec_look" in constants else "arm_pos_navigating"
    return [x / 180 * math.pi for x in constants[key]]


def create_constant_writer():
    """Push arm poses + instruction-point pose to the blackboard."""
    arm_nav, arm_scan = _load_arm_constants()
    arm_orbbec = _load_arm_orbbec_look()
    instruction_pose = KNOWN_LOCATIONS.get("instruction_point") \
        or KNOWN_LOCATIONS.get("QA_point") \
        or next(iter(KNOWN_LOCATIONS.values()), None)

    seq = py_trees.composites.Sequence("constants", memory=True)
    seq.add_child(BtNode_WriteToBlackboard(
        "arm scan", bb_namespace="", bb_source=None,
        bb_key=bb_keys.ARM_SCAN, object=arm_scan,
    ))
    seq.add_child(BtNode_WriteToBlackboard(
        "arm nav", bb_namespace="", bb_source=None,
        bb_key=bb_keys.ARM_NAVIGATING, object=arm_nav,
    ))
    seq.add_child(BtNode_WriteToBlackboard(
        "arm orbbec look", bb_namespace="", bb_source=None,
        bb_key=bb_keys.ARM_ORBBEC_LOOK, object=arm_orbbec,
    ))
    if instruction_pose is not None:
        seq.add_child(BtNode_WriteToBlackboard(
            "instruction pose", bb_namespace="", bb_source=None,
            bb_key="gpsr/instruction_pose", object=instruction_pose,
        ))
    return seq


def create_enter_arena():
    root = py_trees.composites.Sequence("enter_arena", memory=True)
    root.add_child(py_trees.decorators.Retry(
        "retry arm to nav",
        BtNode_MoveArmSingle(
            "arm to navigating",
            action_name=ARM_ACTION_NAME,
            arm_pose_bb_key=bb_keys.ARM_NAVIGATING,
        ),
        num_failures=5,
    ))
    root.add_child(py_trees.decorators.Retry(
        "retry door detection",
        BtNode_DoorDetection("door detection", bb_door_state_key="gpsr/door_status"),
        num_failures=999,
    ))
    root.add_child(BtNode_WaitTicks("wait after door", 10))
    par = py_trees.composites.Parallel(
        "enter parallel", policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    par.add_child(BtNode_Announce("announce entering", bb_source=None, message="Entering arena"))
    par.add_child(BtNode_TurnPanTilt("pan tilt", x=0.0, y=20.0, speed=0.0))
    par.add_child(py_trees.decorators.Retry(
        "retry goto instruction",
        BtNode_GotoAction("goto instruction point", "gpsr/instruction_pose"),
        num_failures=5,
    ))
    root.add_child(par)
    return root


def create_listen_for_command():
    """Listen to the user, confirm, and write the result to ``gpsr/command``."""
    if DEBUG_COMMAND:
        return BtNode_WriteToBlackboard(
            f"debug command",
            bb_namespace="", bb_source=None,
            bb_key=bb_keys.COMMAND, object=DEBUG_COMMAND,
        )
    inner = py_trees.composites.Sequence("listen+confirm", memory=True)
    inner.add_child(BtNode_Announce(
        "prompt", bb_source=None,
        message="Please tell me your command after the beep.",
    ))
    inner.add_child(BtNode_WaitTicks("beep", 6))
    inner.add_child(BtNode_ListenAction(
        "listen", bb_dest_key=bb_keys.COMMAND, timeout=10.0,
    ))
    inner.add_child(BtNode_Announce(
        "confirm prompt", bb_source=bb_keys.COMMAND,
        message="I heard: ",
    ))
    inner.add_child(BtNode_GetConfirmationAction("confirm"))
    inner.add_child(BtNode_Announce(
        "confirmed", bb_source=None, message="Got it. Starting now.",
    ))
    return py_trees.decorators.Retry("retry listen", inner, num_failures=10)


def create_one_command_cycle():
    """Init blackboard → listen → execute_command → return to instruction point."""
    seq = py_trees.composites.Sequence("one_command", memory=True)
    seq.add_child(create_orchestrator_init())
    seq.add_child(create_listen_for_command())
    seq.add_child(create_execute_command(max_steps=25, max_corrections=3))

    return_home = py_trees.composites.Parallel(
        "return home", policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    return_home.add_child(BtNode_Announce(
        "announce returning", bb_source=None,
        message="Returning to instruction point.",
    ))
    return_home.add_child(py_trees.decorators.Retry(
        "retry return",
        BtNode_GotoAction("goto instruction", "gpsr/instruction_pose"),
        num_failures=5,
    ))
    seq.add_child(return_home)
    return seq


def create_gpsr_full(enter_arena: bool = False):
    load_knowledge_from_constants(CONSTANTS_PATH)
    root = py_trees.composites.Sequence("GPSR-full", memory=True)
    root.add_child(create_constant_writer())
    if enter_arena:
        root.add_child(create_enter_arena())

    cycle = py_trees.composites.Selector(
        "one_command_with_fallback", memory=True,
        children=[
            create_one_command_cycle(),
            py_trees.decorators.Retry(
                "retry return on failure",
                BtNode_GotoAction("goto instruction", "gpsr/instruction_pose"),
                num_failures=5,
            ),
        ],
    )
    root.add_child(py_trees.decorators.Repeat(
        "repeat per command", cycle, num_success=NUM_COMMANDS,
    ))
    return root


def main():
    rclpy.init()
    root = py_trees.composites.Sequence("Root", memory=True)
    root.add_children([
        create_gpsr_full(enter_arena=False),
        BtNode_Announce(
            "announce done", bb_source=None,
            message="GPSR task complete. All three commands attempted.",
        ),
        py_trees.behaviours.Running("idle"),
    ])

    tree = py_trees_ros.trees.BehaviourTree(root=root)
    tree.setup(timeout=15, node_name="gpsr_full_node")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(title="GPSR-full")
    tree.tick_tock(period_ms=500.0, post_tick_handler=print_tree)

    try:
        rclpy.spin(tree.node)
    finally:
        shutdown_visualizer()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
