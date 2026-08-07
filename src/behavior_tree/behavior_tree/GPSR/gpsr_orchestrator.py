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

from .gpsr_full import CONSTANTS_PATH, _load_arm_constants, _load_arm_orbbec_look
from .orchestrator import (
    create_batch_command_flow_new,
    make_inject_intake,
    make_listen_intake,
    create_orchestrator_init,
    create_goto_command_point,
    has_command_point,
    load_knowledge_from_constants,
)
from .planner import GPSRPlanner
from .small_trees import bb_keys, create_enter_arena

# Module-level decoupled orchestrator: the two-layer planner invoked repeatedly
# (per slot, per target, per replan) by the bridge nodes + DynamicExecutor. It
# never holds a tree/node reference and its planner threads never touch the BB.
PLANNER = GPSRPlanner()

DEFAULT_PLAN_DIR = Path(os.environ.get("BT_GPSR_PLAN_DIR", "gpsr_runs")).resolve()
# How many commands to collect up front before executing (RoboCup GPSR = 3).
NUM_COMMANDS = int(os.environ.get("BT_GPSR_NUM_COMMANDS", "3"))


def _arm_constants_to_bb(seq: py_trees.composites.Sequence) -> None:
    """Seed the arm navigating/scan/orbbec-look poses the small trees consume."""
    arm_nav, arm_scan = _load_arm_constants()
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
        bb_key=bb_keys.ARM_ORBBEC_LOOK, object=_load_arm_orbbec_look(),
    ))


def createGPSROrchestrator(
    commands=None,
    plan_dir: Path = DEFAULT_PLAN_DIR,
    max_steps: int = 25,
    max_corrections: int = 3,
    num_commands: int = NUM_COMMANDS,
) -> py_trees.behaviour.Behaviour:
    """Build the live orchestrator root.

    New flow: enter arena → go to the command point ONCE → collect all
    ``num_commands`` commands, planning + announcing each up front → then execute
    them one by one (announcing before each).

    Command source: an explicit ``commands`` list (or ``BT_GPSR_CMD`` as a
    ``|``-separated list) is injected — used for desktop / mock e2e tests. With
    neither, the robot prompts + listens for each command by voice (competition).
    """
    load_knowledge_from_constants(CONSTANTS_PATH)
    if commands is None:
        cmd_env = os.environ.get("BT_GPSR_CMD", "").strip()
        if cmd_env:
            commands = [c.strip() for c in cmd_env.split("|") if c.strip()]
    if commands:
        make_intake = make_inject_intake(commands)
        n = len(commands)
    else:
        make_intake = make_listen_intake()
        n = num_commands

    root = py_trees.composites.Sequence("GPSR orchestrator", memory=True)
    # The robot starts OUTSIDE the arena in front of the door: wait for it to
    # open and enter — once, before anything else.
    root.add_child(create_enter_arena())
    _arm_constants_to_bb(root)
    # GPSR: go to the command point ONCE to receive all commands there.
    if has_command_point():
        root.add_child(create_goto_command_point())
    # Capture the command-point start pose once (operator spot for deliveries).
    root.add_child(create_orchestrator_init())
    # TWO-LAYER batch flow: top split + parallel per-target planning up front,
    # then a DynamicExecutor per slot swaps ready target subtrees into the
    # RUNNING tree at runtime. PLANNER is the module-level decoupled orchestrator.
    root.add_child(create_batch_command_flow_new(
        PLANNER,
        num_commands=n,
        make_intake=make_intake,
        max_replans_per_target=max_corrections,
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
    # gpsr_tree=tree forwards the running tree into every behaviour's setup()
    # (py_trees_ros relays extra kwargs). DynamicExecutor reads it so it can
    # swap in freshly-planned target subtrees at runtime.
    tree.setup(timeout=15, node_name="gpsr_orchestrator", gpsr_tree=tree)
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
