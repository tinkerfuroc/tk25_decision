"""Per-small-tree dev test runners.

Each ``main_<action>`` function builds a single small tree, pre-fills the
shared GPSR blackboard with reasonable test values, and ticks it from a
ROS2-friendly tree. Used for development-stage execution checks of individual
actions in mock mode (or against real services if everything is up).

Entry points are wired in ``src/setup.py``::

    ros2 run behavior_tree gpsr-test-goto
    ros2 run behavior_tree gpsr-test-find-object
    ...
"""

import os
import math
from typing import Callable

import py_trees
import py_trees_ros
import rclpy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.visualization import create_post_tick_visualizer

from .orchestrator import load_knowledge_from_constants, KNOWN_LOCATIONS, KNOWN_OBJECT_PROMPTS
from .small_trees import ACTION_FACTORIES, bb_keys
from .gpsr_full import CONSTANTS_PATH, _load_arm_constants


def _identity_pose(location_key: str = "kitchen") -> PoseStamped:
    """Return a known pose if available, else a 0,0 fallback."""
    pose = KNOWN_LOCATIONS.get(location_key)
    if pose is not None:
        return pose
    return PoseStamped(
        header=Header(stamp=rclpy.time.Time().to_msg(), frame_id="map"),
        pose=Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def _build_runner(action_name: str, fill_bb: Callable[[py_trees.composites.Sequence], None]):
    """Build a runnable tree: write defaults to BB, then tick the small tree."""
    load_knowledge_from_constants(CONSTANTS_PATH)
    seq = py_trees.composites.Sequence(f"test:{action_name}", memory=True)
    fill_bb(seq)
    seq.add_child(ACTION_FACTORIES[action_name]())
    seq.add_child(py_trees.behaviours.Running("idle"))
    return seq


def _arm_constants_to_bb(seq: py_trees.composites.Sequence) -> None:
    arm_nav, arm_scan = _load_arm_constants()
    seq.add_child(BtNode_WriteToBlackboard(
        "arm scan", bb_namespace="", bb_source=None,
        bb_key=bb_keys.ARM_SCAN, object=arm_scan,
    ))
    seq.add_child(BtNode_WriteToBlackboard(
        "arm nav", bb_namespace="", bb_source=None,
        bb_key=bb_keys.ARM_NAVIGATING, object=arm_nav,
    ))


def _spin(tree_root: py_trees.behaviour.Behaviour, title: str) -> None:
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=tree_root)
    tree.setup(timeout=15, node_name=f"gpsr_test_{title}")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(title=title)
    tree.tick_tock(period_ms=500.0, post_tick_handler=print_tree)
    try:
        rclpy.spin(tree.node)
    finally:
        shutdown_visualizer()
        rclpy.shutdown()


# ---- per-action mains ----

def main_goto():
    location = os.environ.get("BT_GPSR_TEST_LOCATION", "kitchen")

    def fill(seq):
        seq.add_child(BtNode_WriteToBlackboard(
            "loc name", bb_namespace="", bb_source=None,
            bb_key=bb_keys.TARGET_LOCATION, object=location,
        ))
        seq.add_child(BtNode_WriteToBlackboard(
            "target pose", bb_namespace="", bb_source=None,
            bb_key=bb_keys.TARGET_POSE, object=_identity_pose(location),
        ))

    _spin(_build_runner("goto", fill), "goto")


def main_find_object():
    obj = os.environ.get("BT_GPSR_TEST_OBJECT", "coke")
    prompt = KNOWN_OBJECT_PROMPTS.get(obj, obj)

    def fill(seq):
        seq.add_child(BtNode_WriteToBlackboard(
            "obj name", bb_namespace="", bb_source=None,
            bb_key=bb_keys.TARGET_OBJECT_NAME, object=obj,
        ))
        seq.add_child(BtNode_WriteToBlackboard(
            "obj prompt", bb_namespace="", bb_source=None,
            bb_key=bb_keys.TARGET_OBJECT_PROMPT, object=prompt,
        ))

    _spin(_build_runner("find_object", fill), "find_object")


def main_find_person():
    descriptor = os.environ.get("BT_GPSR_TEST_PERSON", "waving person")

    def fill(seq):
        seq.add_child(BtNode_WriteToBlackboard(
            "person prompt", bb_namespace="", bb_source=None,
            bb_key=bb_keys.TARGET_PERSON_PROMPT, object=descriptor,
        ))

    _spin(_build_runner("find_person", fill), "find_person")


def main_follow():
    def fill(seq):
        pass
    _spin(_build_runner("follow", fill), "follow")


def main_guide():
    location = os.environ.get("BT_GPSR_TEST_LOCATION", "kitchen")

    def fill(seq):
        seq.add_child(BtNode_WriteToBlackboard(
            "loc name", bb_namespace="", bb_source=None,
            bb_key=bb_keys.TARGET_LOCATION, object=location,
        ))
        seq.add_child(BtNode_WriteToBlackboard(
            "target pose", bb_namespace="", bb_source=None,
            bb_key=bb_keys.TARGET_POSE, object=_identity_pose(location),
        ))

    _spin(_build_runner("guide", fill), "guide")


def main_greet():
    person = os.environ.get("BT_GPSR_TEST_PERSON", "Alex")

    def fill(seq):
        seq.add_child(BtNode_WriteToBlackboard(
            "person prompt", bb_namespace="", bb_source=None,
            bb_key=bb_keys.TARGET_PERSON_PROMPT, object=person,
        ))

    _spin(_build_runner("greet", fill), "greet")


def main_grasp():
    obj = os.environ.get("BT_GPSR_TEST_OBJECT", "coke")

    def fill(seq):
        _arm_constants_to_bb(seq)
        seq.add_child(BtNode_WriteToBlackboard(
            "obj name", bb_namespace="", bb_source=None,
            bb_key=bb_keys.TARGET_OBJECT_NAME, object=obj,
        ))

    _spin(_build_runner("grasp", fill), "grasp")


def main_place():
    location = os.environ.get("BT_GPSR_TEST_LOCATION", "kitchen")

    def fill(seq):
        seq.add_child(BtNode_WriteToBlackboard(
            "loc name", bb_namespace="", bb_source=None,
            bb_key=bb_keys.TARGET_LOCATION, object=location,
        ))
        seq.add_child(BtNode_WriteToBlackboard(
            "target pose", bb_namespace="", bb_source=None,
            bb_key=bb_keys.TARGET_POSE, object=_identity_pose(location),
        ))

    _spin(_build_runner("place", fill), "place")


def main_deliver():
    obj = os.environ.get("BT_GPSR_TEST_OBJECT", "coke")
    location = os.environ.get("BT_GPSR_TEST_LOCATION", "living_room")

    def fill(seq):
        seq.add_child(BtNode_WriteToBlackboard(
            "obj name", bb_namespace="", bb_source=None,
            bb_key=bb_keys.TARGET_OBJECT_NAME, object=obj,
        ))
        seq.add_child(BtNode_WriteToBlackboard(
            "loc name", bb_namespace="", bb_source=None,
            bb_key=bb_keys.TARGET_LOCATION, object=location,
        ))
        seq.add_child(BtNode_WriteToBlackboard(
            "target pose", bb_namespace="", bb_source=None,
            bb_key=bb_keys.TARGET_POSE, object=_identity_pose(location),
        ))

    _spin(_build_runner("deliver", fill), "deliver")


def main_count():
    obj = os.environ.get("BT_GPSR_TEST_OBJECT", "coke")
    prompt = KNOWN_OBJECT_PROMPTS.get(obj, obj)

    def fill(seq):
        seq.add_child(BtNode_WriteToBlackboard(
            "obj prompt", bb_namespace="", bb_source=None,
            bb_key=bb_keys.TARGET_OBJECT_PROMPT, object=prompt,
        ))

    _spin(_build_runner("count", fill), "count")


def main_answer_question():
    def fill(seq):
        pass
    _spin(_build_runner("answer_question", fill), "answer_question")


def main_tell_info():
    text = os.environ.get(
        "BT_GPSR_TEST_TEXT",
        "Our team is Tinker Furo. We are from Tsinghua University.",
    )

    def fill(seq):
        seq.add_child(BtNode_WriteToBlackboard(
            "announce text", bb_namespace="", bb_source=None,
            bb_key=bb_keys.ANNOUNCE_TEXT, object=text,
        ))

    _spin(_build_runner("tell_info", fill), "tell_info")


def main_say():
    text = os.environ.get("BT_GPSR_TEST_TEXT", "Hello, this is a test announcement.")

    def fill(seq):
        seq.add_child(BtNode_WriteToBlackboard(
            "announce text", bb_namespace="", bb_source=None,
            bb_key=bb_keys.ANNOUNCE_TEXT, object=text,
        ))

    _spin(_build_runner("say", fill), "say")


# ---- orchestrator-with-fixed-command dev test ----

def main_orchestrator():
    """Speak ONE command -> plan -> execute, honouring per-module mock config.

    This is the per-module integration harness: the planner is always real (it
    "splits" the spoken command and generates the tree), while which *executing*
    subsystems are real vs. stubbed is read from ``mock_config.json`` BEFORE the
    run. Set ``mock_mode.enabled: true`` and flip each subsystem's ``enabled``
    flag (``true`` = MOCKED, ``false`` = REAL) so only the module under test
    drives hardware. ``keyboard_control.enabled: false`` makes mocked nodes
    auto-succeed (smooth flow) instead of waiting for a keypress.

    Command source:
      * ``BT_GPSR_DEBUG_CMD`` set -> use it verbatim (deterministic, no audio).
      * unset -> SPEAK it after the beep (needs audio_input REAL + audio stack up).
    """
    from .orchestrator import create_execute_command, create_orchestrator_init
    from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_ListenAction
    from .small_trees import BtNode_AnnounceFromBB

    load_knowledge_from_constants(CONSTANTS_PATH)
    command = os.environ.get("BT_GPSR_DEBUG_CMD", "").strip()

    rclpy.init()
    root = py_trees.composites.Sequence("Test orchestrator", memory=True)
    _arm_constants_to_bb(root)
    if command:
        root.add_child(BtNode_WriteToBlackboard(
            "command (env)", bb_namespace="", bb_source=None,
            bb_key=bb_keys.COMMAND, object=command,
        ))
    else:
        # Voice intake: prompt, then the listen server beeps and records.
        root.add_child(BtNode_Announce(
            "prompt", bb_source=None,
            message="Please tell me the command after the beep.",
        ))
        listen_timeout = float(os.environ.get("BT_GPSR_LISTEN_TIMEOUT", "30.0"))
        root.add_child(BtNode_ListenAction(
            "listen", bb_dest_key=bb_keys.COMMAND, timeout=listen_timeout,
        ))
        # Echo the transcription so STT accuracy is audible before planning.
        root.add_child(BtNode_AnnounceFromBB(
            "echo heard", bb_keys.COMMAND, prefix="I heard: ",
        ))
    root.add_child(create_orchestrator_init())
    root.add_child(create_execute_command(max_steps=25, max_corrections=3))
    root.add_child(py_trees.behaviours.Running("idle"))

    tree = py_trees_ros.trees.BehaviourTree(root=root)
    tree.setup(timeout=15, node_name="gpsr_test_orchestrator")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(title="orchestrator")
    tree.tick_tock(period_ms=500.0, post_tick_handler=print_tree)
    try:
        rclpy.spin(tree.node)
    finally:
        shutdown_visualizer()
        rclpy.shutdown()
