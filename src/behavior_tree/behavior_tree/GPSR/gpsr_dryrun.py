"""GPSR plan rehearsal (dry run) — speak to the robot, plan, announce + visualize.

Exercises the front half of the pipeline WITHOUT driving the robot, so you can
test it with only the **audio** stack up (no navigation / vision / manipulation
modules needed):

    1. prompt the human + beep    (BtNode_Announce "...after the beep")
    2. take the command by voice  (BtNode_ListenAction — the listen server beeps)
    3. plan it with the LLM        (BtNode_PlanActions)
    4. freeze the plan to a .py    (BtNode_GeneratePlanFile)   — check-after-run
    5. draw the decision tree      (BtNode_RenderPlanTree)     — PNG/SVG/DOT
    6. speak the plan aloud         (create_announce_plan)     — step-by-step

The dispatcher (which would open nav/vision/arm clients at ``setup()``) is never
built, so ``tree.setup()`` only touches the audio + planning nodes.

In the voice path the whole cycle LOOPS, so you can speak one command after
another without relaunching. The beep before listening follows the HRI pattern
(``HRI/hri.py``): the BT announces "speak after the beep" and the listen/ASR
server emits the actual beep when it starts recording.

Run::

    # real voice in/out (needs listen_action + announce services up):
    ros2 run behavior_tree gpsr-dryrun

    # desktop test, no robot at all — inject the command, mock the audio,
    # runs once then idles:
    BT_MOCK_MODE=true BT_GPSR_CMD="bring me a coke from the kitchen" \
        ros2 run behavior_tree gpsr-dryrun

Artifacts (``<HHMMSS>_<command>``) land in ``$BT_GPSR_PLAN_DIR`` (default
``./gpsr_runs``): ``gpsr_plan_*.py`` (replay module) + ``gpsr_tree_*.png`` (tree).
"""

import os
from pathlib import Path

import py_trees
import py_trees_ros
import rclpy
from rclpy.executors import ExternalShutdownException

from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_ListenAction
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.visualization import create_post_tick_visualizer

from .gpsr_full import CONSTANTS_PATH
from .orchestrator import (
    BtNode_GeneratePlanFile,
    BtNode_PlanActions,
    BtNode_RenderPlanTree,
    create_announce_plan,
    create_orchestrator_init,
    load_knowledge_from_constants,
)
from .small_trees import bb_keys

DEFAULT_PLAN_DIR = Path(os.environ.get("BT_GPSR_PLAN_DIR", "gpsr_runs")).resolve()
# Max length of a spoken command (= speech_recognition phrase_time_limit). Set
# generously so a long command read with hesitations is not cut short. Pair it
# with the listen server's LISTEN_PAUSE_THRESHOLD (how long a mid-command pause
# is tolerated before the phrase ends). Override either via env.
LISTEN_TIMEOUT_SEC = float(os.environ.get("BT_GPSR_LISTEN_TIMEOUT", "30.0"))
# Spoken cue before the robot listens. The human waits for the beep the
# listen/ASR server plays when recording starts (same pattern as HRI/hri.py).
PROMPT_MESSAGE = "Please tell me the task after the beep."


def _command_intake() -> py_trees.behaviour.Behaviour:
    """Command source: ``BT_GPSR_CMD`` env (inject) else voice (prompt+beep+listen).

    The env override keeps a desktop / mock path working; the default is the
    real audio intake the competition uses — a spoken prompt ("...after the
    beep") followed by the listen action, whose server emits the beep.
    """
    env_cmd = os.environ.get("BT_GPSR_CMD")
    if env_cmd:
        return BtNode_WriteToBlackboard(
            "inject command (env)", bb_namespace="", bb_source=None,
            bb_key=bb_keys.COMMAND, object=env_cmd,
        )
    seq = py_trees.composites.Sequence("prompt + listen", memory=True)
    seq.add_child(BtNode_Announce(
        "prompt for command", bb_source=None, message=PROMPT_MESSAGE,
    ))
    seq.add_child(BtNode_ListenAction(
        "listen for command",
        bb_dest_key=bb_keys.COMMAND,
        timeout=LISTEN_TIMEOUT_SEC,
    ))
    return seq


def createGPSRDryRun(
    plan_dir: Path = DEFAULT_PLAN_DIR,
) -> py_trees.behaviour.Behaviour:
    """Build the dry-run rehearsal root.

    Voice path: the cycle is a ``memory`` Sequence with no trailing idle, so
    ``tick_tock`` re-runs it from the prompt after each command (loop).
    Env-inject path: append a ``Running`` idle so it runs the one command once
    then sits idle.
    """
    load_knowledge_from_constants(CONSTANTS_PATH)
    env_cmd = os.environ.get("BT_GPSR_CMD")

    cycle = py_trees.composites.Sequence("GPSR dry-run", memory=True)
    cycle.add_child(_command_intake())
    # No TF/localization in this test — skip the start-pose capture.
    cycle.add_child(create_orchestrator_init(capture_pose=False))
    cycle.add_child(BtNode_PlanActions(name="plan command"))
    cycle.add_child(BtNode_GeneratePlanFile(out_dir=str(plan_dir)))   # .py replay
    cycle.add_child(BtNode_RenderPlanTree(out_dir=str(plan_dir)))      # tree PNG
    cycle.add_child(create_announce_plan())                           # speak plan

    if env_cmd:
        # One-shot desktop test: run the injected command once, then idle.
        cycle.add_child(py_trees.behaviours.Running("idle (ctrl-c to exit)"))
    # Voice path: return the bare cycle so it loops command-after-command.
    return cycle


def main():
    rclpy.init(args=None)
    DEFAULT_PLAN_DIR.mkdir(parents=True, exist_ok=True)
    print(f"[gpsr-dryrun] artifacts -> {DEFAULT_PLAN_DIR}")

    root = createGPSRDryRun()
    tree = py_trees_ros.trees.BehaviourTree(root=root)
    tree.setup(timeout=15, node_name="gpsr_dryrun")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(title="GPSR dry-run")
    tree.tick_tock(period_ms=500.0, post_tick_handler=print_tree)
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        shutdown_visualizer()
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
