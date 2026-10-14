"""GPSR decision tree — AUTO-GENERATED, do not hand-edit the header.

Command:
    This is a mock speech input message

Planned action chain:
    announce(text=Mock mode: planner bypassed; no network request was made. Target: This is a mock speech input message)

Generated: 2026-08-08 13:13:44

This file freezes the plan the LLM produced so the exact same tree can be
rebuilt and re-executed WITHOUT calling the LLM again (deterministic replay).
"""

import rclpy
import py_trees
import py_trees_ros

from behavior_tree.GPSR.small_trees import (
    ACTION_FACTORIES,
    bb_keys,
    BtNode_BlackboardSet,
)
from behavior_tree.GPSR.orchestrator import (
    create_execute_one_step,
    create_orchestrator_init,
    load_knowledge_from_constants,
)

CONSTANTS_PATH = '/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/constants.json'

COMMAND = 'This is a mock speech input message'

# The frozen plan, exactly as the planner produced it.
PLAN = [   {   'action': 'announce',
        'params': {   'text': 'Mock mode: planner bypassed; no network request was made. Target: '
                              'This is a mock speech input message'}}]


def create_tree(replay: bool = True):
    """Rebuild the executed GPSR tree.

    replay=True  -> inject the frozen PLAN and run the dispatch loop, so the
                    same steps execute again with NO LLM call.
    replay=False -> the bare structural composition of the small trees, in
                    plan order (for inspection / visualization).
    """
    if not replay:
        root = py_trees.composites.Sequence("GPSR (structure): " + COMMAND, memory=True)
        root.add_child(ACTION_FACTORIES['announce']())  # step 1: announce(text=Mock mode: planner bypassed; no network request was made. Target: This is a mock speech input message)
        return root

    root = py_trees.composites.Sequence("GPSR replay: " + COMMAND, memory=True)
    root.add_child(create_orchestrator_init())
    root.add_child(BtNode_BlackboardSet("inject command", bb_keys.COMMAND, COMMAND))
    root.add_child(BtNode_BlackboardSet("inject plan", bb_keys.PLAN, PLAN))
    root.add_child(BtNode_BlackboardSet("reset plan index", bb_keys.PLAN_INDEX, 0))
    loop = py_trees.decorators.Repeat(
        name="step loop",
        child=create_execute_one_step(),
        num_success=max(len(PLAN), 1),
    )
    root.add_child(py_trees.decorators.FailureIsSuccess("plan-exhausted = done", loop))
    return root


def main():
    load_knowledge_from_constants(CONSTANTS_PATH)
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=create_tree(replay=True))
    tree.setup(timeout=15, node_name="gpsr_replay")
    tree.tick_tock(period_ms=500.0)
    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
