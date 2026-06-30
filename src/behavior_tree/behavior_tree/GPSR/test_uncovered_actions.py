"""Per-small-tree dev test runners for the GPSR factories not yet covered by
``dev_tests.py``.

``dev_tests.py`` wires ``gpsr-test-*`` runners for goto / find_object /
find_person / follow / guide / grasp / place / deliver / count /
answer_question. This module adds importable ``create_tree`` builders + one
``main_<action>`` per *uncovered* factory in ``small_trees.ACTION_FACTORIES``:

    announce, approach_person, ask_person, describe_person, record_position,
    vlm_fallback, llm_fallback

Each ``create_tree_<action>()`` builds the single-action small tree by
pre-seeding only that action's blackboard inputs (mirroring
``dev_tests._build_runner`` / ``_arm_constants_to_bb``), then ticks it. The
module-level ``create_tree()`` builds the ``announce`` small tree so the offline
smoke harness can import a no-arg builder.

Run (real or partially-mocked, per mock_config.json)::

    ros2 run behavior_tree gpsr-test-announce
    ros2 run behavior_tree gpsr-test-approach-person
    ros2 run behavior_tree gpsr-test-ask-person
    ros2 run behavior_tree gpsr-test-describe-person
    ros2 run behavior_tree gpsr-test-record-position
    ros2 run behavior_tree gpsr-test-vlm
    ros2 run behavior_tree gpsr-test-llm

Offline (everything mocked, keypress off -> auto-advance)::

    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json ros2 run behavior_tree gpsr-test-announce
"""

import os
from typing import Callable

import py_trees
import py_trees_ros
import rclpy
from geometry_msgs.msg import PoseStamped, PointStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.visualization import create_post_tick_visualizer

from .orchestrator import load_knowledge_from_constants
from .small_trees import ACTION_FACTORIES, bb_keys
from .gpsr_full import CONSTANTS_PATH


# ---------------------------------------------------------------------------
# Builders (mirror dev_tests._build_runner / _spin precisely)
# ---------------------------------------------------------------------------

def _build_runner(action_name: str,
                  fill_bb: Callable[[py_trees.composites.Sequence], None]):
    """Write defaults to the BB, then tick the named small tree, then idle."""
    load_knowledge_from_constants(CONSTANTS_PATH)
    seq = py_trees.composites.Sequence(f"test:{action_name}", memory=True)
    fill_bb(seq)
    seq.add_child(ACTION_FACTORIES[action_name]())
    seq.add_child(py_trees.behaviours.Running("idle"))
    return seq


def _spin(tree_root: py_trees.behaviour.Behaviour, title: str) -> None:
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=tree_root)
    tree.setup(timeout=15, node_name=f"gpsr_test_{title}")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(title=title)
    tree.tick_tock(period_ms=500.0, post_tick_handler=print_tree)
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        shutdown_visualizer()
        tree.shutdown()
        rclpy.try_shutdown()


def _identity_point(frame_id: str = "map") -> PointStamped:
    return PointStamped(
        header=Header(stamp=rclpy.time.Time().to_msg(), frame_id=frame_id),
        point=Point(x=1.0, y=0.0, z=0.0),
    )


# ---------------------------------------------------------------------------
# Per-action fill functions + create_tree builders
# ---------------------------------------------------------------------------

def _fill_announce(seq):
    text = os.environ.get(
        "BT_GPSR_TEST_TEXT",
        "Our team is Tinker Furo. We are from Tsinghua University.",
    )
    seq.add_child(BtNode_WriteToBlackboard(
        "announce text", bb_namespace="", bb_source=None,
        bb_key=bb_keys.ANNOUNCE_TEXT, object=text,
    ))


def create_tree_announce():
    return _build_runner("announce", _fill_announce)


def _fill_approach_person(seq):
    # find_person normally writes TARGET_PERSON_POSE (a PointStamped); seed it.
    seq.add_child(BtNode_WriteToBlackboard(
        "person pose", bb_namespace="", bb_source=None,
        bb_key=bb_keys.TARGET_PERSON_POSE, object=_identity_point(),
    ))


def create_tree_approach_person():
    return _build_runner("approach_person", _fill_approach_person)


def _fill_ask_person(seq):
    question = os.environ.get("BT_GPSR_TEST_QUESTION", "What is your name?")
    seq.add_child(BtNode_WriteToBlackboard(
        "ask question", bb_namespace="", bb_source=None,
        bb_key=bb_keys.ASK_QUESTION, object=question,
    ))


def create_tree_ask_person():
    return _build_runner("ask_person", _fill_ask_person)


def _fill_describe_person(seq):
    # describe_person reads no upstream BB inputs (the orchestrator runs
    # find_person/approach_person first to position the robot); nothing to seed.
    pass


def create_tree_describe_person():
    return _build_runner("describe_person", _fill_describe_person)


def _fill_record_position(seq):
    label = os.environ.get("BT_GPSR_TEST_LABEL", "this_spot")
    seq.add_child(BtNode_WriteToBlackboard(
        "dynlabel", bb_namespace="", bb_source=None,
        bb_key=bb_keys.CURRENT_DYNLABEL, object=label,
    ))


def create_tree_record_position():
    return _build_runner("record_position", _fill_record_position)


def _fill_vlm_fallback(seq):
    question = os.environ.get(
        "BT_GPSR_TEST_QUESTION", "What is on the table?",
    )
    seq.add_child(BtNode_WriteToBlackboard(
        "vlm question", bb_namespace="", bb_source=None,
        bb_key=bb_keys.VLM_QUESTION, object=question,
    ))


def create_tree_vlm_fallback():
    return _build_runner("vlm_fallback", _fill_vlm_fallback)


def _fill_llm_fallback(seq):
    question = os.environ.get(
        "BT_GPSR_TEST_QUESTION", "What is today's date?",
    )
    seq.add_child(BtNode_WriteToBlackboard(
        "llm question", bb_namespace="", bb_source=None,
        bb_key=bb_keys.LLM_QUESTION, object=question,
    ))


def create_tree_llm_fallback():
    return _build_runner("llm_fallback", _fill_llm_fallback)


# ---------------------------------------------------------------------------
# Smoke-importable no-arg builder (the 'announce' small tree)
# ---------------------------------------------------------------------------

def create_tree():
    """Default builder for the offline smoke harness: the announce small tree."""
    return create_tree_announce()


# ---------------------------------------------------------------------------
# Per-action mains
# ---------------------------------------------------------------------------

def main_announce():
    _spin(create_tree_announce(), "announce")


def main_approach_person():
    _spin(create_tree_approach_person(), "approach_person")


def main_ask_person():
    _spin(create_tree_ask_person(), "ask_person")


def main_describe_person():
    _spin(create_tree_describe_person(), "describe_person")


def main_record_position():
    _spin(create_tree_record_position(), "record_position")


def main_vlm_fallback():
    _spin(create_tree_vlm_fallback(), "vlm_fallback")


def main_llm_fallback():
    _spin(create_tree_llm_fallback(), "llm_fallback")


def main():
    """Default entry: run the announce small tree."""
    main_announce()


if __name__ == "__main__":
    main()
