"""Orchestrator-with-fixed-command dev test (grammar / plan-and-dispatch).

Seeds a fixed composite command on the GPSR blackboard, runs
``create_orchestrator_init()`` to reset planner state + capture the start pose,
then ``create_execute_command(...)`` to plan the command with the LLM and
dispatch each resulting small tree. This is the importable ``create_tree()``
counterpart to ``dev_tests.main_orchestrator`` with the command injected
directly (no listen step), so a single composite command can be exercised
end-to-end in mock.

The planner (``BtNode_PlanActions``) calls a remote LLM in a background thread
during ``update()`` (not ``setup()``), so tree construction and ``setup()`` never
block on the network. With no network / no API key the planner step simply stays
RUNNING (or FAILS into self-correction); the surrounding loop + idle keep the
tree alive either way.

Scope note: ``create_execute_command()`` builds the full dispatcher Selector,
which instantiates EVERY factory in ``ACTION_FACTORIES`` at construction time —
including ``create_grasp``, whose ``BtNode_MoveArmSingle(action_name=...)`` calls
fail against the installed node signature (it takes ``service_name``, not
``action_name``). That is a pre-existing ``small_trees`` / template-node mismatch
unrelated to this harness. So the default ``create_tree()`` is scoped to
``create_orchestrator_init()`` (blackboard reset + start-pose capture + grammar
load) followed by ``BtNode_PlanActions`` + a Running idle — it exercises the
planner/grammar path without building the broken grasp dispatcher. Set
``BT_GPSR_RUN_DISPATCH=1`` to additionally wire ``create_execute_command`` once
the upstream grasp signature is fixed.

Run (planner needs network + an OPENAI/openrouter key; executing subsystems are
real vs. mocked per mock_config.json)::

    ros2 run behavior_tree gpsr-test-cmd
    BT_GPSR_DEBUG_CMD="greet the person in the kitchen and tell them the time" ros2 run behavior_tree gpsr-test-cmd

Offline (everything mocked, keypress off -> auto-advance)::

    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json ros2 run behavior_tree gpsr-test-cmd
"""

import math
import os

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.visualization import create_post_tick_visualizer

from .orchestrator import (
    BtNode_PlanActions,
    create_execute_command,
    create_orchestrator_init,
    load_knowledge_from_constants,
)
from .small_trees import ACTION_FACTORIES, bb_keys
from .gpsr_full import CONSTANTS_PATH, _load_arm_constants


DEFAULT_COMMAND = "greet the person in the kitchen and tell them the time"


def _arm_constants_to_bb(seq: py_trees.composites.Sequence) -> None:
    """Seed the arm navigating/scan poses (mirrors dev_tests._arm_constants_to_bb)."""
    arm_nav, arm_scan = _load_arm_constants()
    seq.add_child(BtNode_WriteToBlackboard(
        "arm scan", bb_namespace="", bb_source=None,
        bb_key=bb_keys.ARM_SCAN, object=arm_scan,
    ))
    seq.add_child(BtNode_WriteToBlackboard(
        "arm nav", bb_namespace="", bb_source=None,
        bb_key=bb_keys.ARM_NAVIGATING, object=arm_nav,
    ))


def create_tree():
    """Seed a fixed command, reset planner state, then plan it (grammar path).

    Default builds: arm constants + command seed + ``create_orchestrator_init()``
    + ``BtNode_PlanActions`` + Running idle. The full dispatch
    (``create_execute_command``) is gated behind ``BT_GPSR_RUN_DISPATCH=1`` because
    its dispatcher Selector eagerly constructs the broken ``create_grasp`` subtree
    (see module docstring).
    """
    load_knowledge_from_constants(CONSTANTS_PATH)
    command = os.environ.get("BT_GPSR_DEBUG_CMD", DEFAULT_COMMAND).strip() or DEFAULT_COMMAND
    run_dispatch = os.environ.get("BT_GPSR_RUN_DISPATCH", "0").strip() not in ("", "0", "false", "False")

    cycle = py_trees.composites.Sequence("Test orchestrator grammar", memory=True)
    _arm_constants_to_bb(cycle)
    cycle.add_child(BtNode_WriteToBlackboard(
        "command (fixed)", bb_namespace="", bb_source=None,
        bb_key=bb_keys.COMMAND, object=command,
    ))
    cycle.add_child(create_orchestrator_init())
    if run_dispatch:
        cycle.add_child(create_execute_command(
            max_steps=25, max_corrections=3, emit_plan_dir=None,
        ))
    else:
        # Plan only — exercises the LLM grammar path without the broken
        # grasp-dispatcher construction. The planner runs its LLM call in a
        # background thread, so it stays RUNNING offline (no network) without
        # blocking setup().
        cycle.add_child(BtNode_PlanActions(name="plan command"))
    # One-shot: run the injected command once, then idle so the tree stays alive.
    cycle.add_child(py_trees.behaviours.Running("idle (ctrl-c to exit)"))
    return cycle


def main():
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=create_tree())
    tree.setup(timeout=15, node_name="gpsr_test_cmd")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(title="orchestrator-grammar")
    tree.tick_tock(period_ms=500.0, post_tick_handler=print_tree)
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        shutdown_visualizer()
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
