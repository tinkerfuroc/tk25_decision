"""Live GPSR orchestrator — production entry point.

Receives a natural-language command, asks the LLM for a plan, **builds the
behaviour tree in memory and ticks it to drive the robot**, and freezes the
plan to a standalone ``.py`` for check-after-run / replay.

This is the real runtime (not a visualizer): ``BtNode_PlanActions`` calls the
LLM at tick time, the dispatcher routes each planned step to its small tree,
and the self-correction sub-tree re-plans on failure.

Run::

    ros2 run behavior_tree gpsr-orchestrator
    # command source (first match wins):
    BT_GPSR_CMD="bring me a coke from the kitchen" ros2 run behavior_tree gpsr-orchestrator
    # generated plan modules land in $BT_GPSR_PLAN_DIR (default: ./gpsr_runs)
"""

import os
from pathlib import Path

import py_trees
import py_trees_ros
import rclpy
from rclpy.executors import ExternalShutdownException

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.visualization import create_post_tick_visualizer

from .gpsr_full import CONSTANTS_PATH, _load_arm_constants
from .orchestrator import (
    create_execute_command,
    create_orchestrator_init,
    create_goto_command_point,
    has_command_point,
    load_knowledge_from_constants,
)
from .small_trees import bb_keys, create_enter_arena

DEFAULT_COMMAND = "Go to the kitchen and bring me a coke."
DEFAULT_PLAN_DIR = Path(os.environ.get("BT_GPSR_PLAN_DIR", "gpsr_runs")).resolve()


def _arm_constants_to_bb(seq: py_trees.composites.Sequence) -> None:
    """Seed the arm navigating/scan poses the grasp small tree consumes."""
    arm_nav, arm_scan = _load_arm_constants()
    seq.add_child(BtNode_WriteToBlackboard(
        "arm scan", bb_namespace="", bb_source=None,
        bb_key=bb_keys.ARM_SCAN, object=arm_scan,
    ))
    seq.add_child(BtNode_WriteToBlackboard(
        "arm nav", bb_namespace="", bb_source=None,
        bb_key=bb_keys.ARM_NAVIGATING, object=arm_nav,
    ))


def createGPSROrchestrator(
    command: str = None,
    plan_dir: Path = DEFAULT_PLAN_DIR,
    max_steps: int = 25,
    max_corrections: int = 3,
) -> py_trees.behaviour.Behaviour:
    """Build the live orchestrator root.

    Command source: explicit ``command`` arg > ``BT_GPSR_CMD`` env > default.
    (Swap the BtNode_WriteToBlackboard below for an audio ``BtNode_ListenAction``
    + confirm sub-tree to take the command by voice in competition.)
    """
    load_knowledge_from_constants(CONSTANTS_PATH)
    cmd = command or os.environ.get("BT_GPSR_CMD", DEFAULT_COMMAND)

    root = py_trees.composites.Sequence("GPSR orchestrator", memory=True)
    # The robot starts OUTSIDE the arena in front of the door: wait for it to
    # open and enter — once, before anything else.
    root.add_child(create_enter_arena())
    _arm_constants_to_bb(root)
    # GPSR: go to the command point to receive the command first.
    if has_command_point():
        root.add_child(create_goto_command_point())
    root.add_child(BtNode_WriteToBlackboard(
        "write command", bb_namespace="", bb_source=None,
        bb_key=bb_keys.COMMAND, object=cmd,
    ))
    root.add_child(create_orchestrator_init())
    root.add_child(create_execute_command(
        max_steps=max_steps,
        max_corrections=max_corrections,
        emit_plan_dir=str(plan_dir),
    ))
    root.add_child(py_trees.behaviours.Running("idle"))
    return root


def main():
    rclpy.init(args=None)
    DEFAULT_PLAN_DIR.mkdir(parents=True, exist_ok=True)
    print(f"[gpsr-orchestrator] plan modules -> {DEFAULT_PLAN_DIR}")

    root = createGPSROrchestrator()
    tree = py_trees_ros.trees.BehaviourTree(root=root)
    tree.setup(timeout=15, node_name="gpsr_orchestrator")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(title="GPSR orchestrator")
    # Per-command logging (plan + each step's result + the failing node's feedback).
    from .command_logger import create_command_logger, combine_post_tick_handlers
    log_tree, shutdown_logger = create_command_logger(str(DEFAULT_PLAN_DIR / "logs"))
    tree.tick_tock(
        period_ms=500.0,
        post_tick_handler=combine_post_tick_handlers(print_tree, log_tree),
    )
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        shutdown_logger()
        shutdown_visualizer()
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
