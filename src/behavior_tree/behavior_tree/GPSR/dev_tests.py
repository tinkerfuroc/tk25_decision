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
    # Seed arm navigating/scan poses so standalone nav/grasp small trees can tuck
    # the arm (they read ARM_NAVIGATING/ARM_SCAN, normally seeded by the
    # orchestrator entry point). Harmless if a fill_bb re-writes them.
    _arm_constants_to_bb(seq)
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


def main_grasp_diag():
    """Hardware grasp diagnostic — RealSense (arm camera), NOT the head Orbbec.

    Runs the exact sequence the GPSR ``grasp`` action uses and prints a clear
    per-stage result so you can see whether the *vision module* (RealSense
    detection) actually finds the object and whether the grasp picks it up:

      1. arm -> base_moving (navigating pose)
      2. arm -> table_grasp (scan pose)
      3. RealSense detect on the table (``object_detection_yolo``)   <- VISION CHECK
      4. grasp the detected object (``start_grasp``)
      5. RealSense RE-SCAN to check the object is gone (picked up)   <- GRABBED CHECK
      6. arm -> base_moving

    REQUIRES these servers running first (launch the manipulation + realsense +
    yolo bringup): ``joint_move_action``, ``object_detection_yolo``,
    ``start_grasp``. Object via ``BT_GPSR_TEST_OBJECT`` (default "coke").
    Setup will BLOCK waiting for a server that is not up — start them first.
    """
    import time
    from py_trees.behaviour import Behaviour
    from py_trees.common import Access, Status
    from behavior_tree.TemplateNodes.Manipulation import BtNode_MoveArmSingle
    from behavior_tree.StoringGroceries.customNodes import BtNode_GraspWithPose
    from behavior_tree.TemplateNodes.Vision import BtNode_ScanForGeneralist

    obj = os.environ.get("BT_GPSR_TEST_OBJECT", "coke")
    arm_nav, arm_scan = _load_arm_constants()

    class _Report(Behaviour):
        def __init__(self, name, src_key, label):
            super().__init__(name)
            self._src, self._label, self._c = src_key, label, None

        def setup(self, **kw):
            self._c = self.attach_blackboard_client(name=self.name)
            self._c.register_key(self._src, access=Access.READ)

        def update(self):
            try:
                r = self._c.get(self._src)
            except Exception:
                r = None
            objs = list(getattr(r, "objects", []) or []) if r is not None else []
            cls = [getattr(o, "cls", "?") for o in objs]
            print(f"[DIAG] {self._label}: {len(objs)} object(s) {cls}")
            return Status.SUCCESS

    seq = py_trees.composites.Sequence("grasp_diag", memory=True)
    seq.add_child(BtNode_WriteToBlackboard("arm scan", bb_namespace="", bb_source=None,
                                           bb_key=bb_keys.ARM_SCAN, object=arm_scan))
    seq.add_child(BtNode_WriteToBlackboard("arm nav", bb_namespace="", bb_source=None,
                                           bb_key=bb_keys.ARM_NAVIGATING, object=arm_nav))
    seq.add_child(BtNode_WriteToBlackboard("obj", bb_namespace="", bb_source=None,
                                           bb_key=bb_keys.TARGET_OBJECT_NAME, object=obj))
    seq.add_child(BtNode_WriteToBlackboard("obj prompt", bb_namespace="", bb_source=None,
                                           bb_key=bb_keys.TARGET_OBJECT_PROMPT, object=obj))
    seq.add_child(BtNode_MoveArmSingle("1. arm to base_moving",
                                       arm_pose_bb_key=bb_keys.ARM_NAVIGATING, add_octomap=False))
    seq.add_child(BtNode_MoveArmSingle("2. arm to table_grasp",
                                       arm_pose_bb_key=bb_keys.ARM_SCAN, add_octomap=True))
    seq.add_child(BtNode_ScanForGeneralist(name="3. realsense generalist detect",
                                           bb_source=bb_keys.TARGET_OBJECT_PROMPT,
                                           bb_key=bb_keys.TARGET_OBJECT, use_orbbec=False,
                                           transform_to_map=False, use_vlm_sam_fallback=True,
                                           sort_closest=True, return_rgb_image=True,
                                           return_depth_image=True, return_segments=True))
    seq.add_child(_Report("detect report", bb_keys.TARGET_OBJECT, "RealSense BEFORE grasp"))
    seq.add_child(BtNode_GraspWithPose("4. grasp", bb_key_vision_res=bb_keys.TARGET_OBJECT,
                                       bb_key_pose=bb_keys.GRASP_POSE, action_name="start_grasp"))
    seq.add_child(BtNode_ScanForGeneralist(name="5. realsense generalist recheck",
                                           bb_source=bb_keys.TARGET_OBJECT_PROMPT,
                                           bb_key="gpsr/recheck_result", use_orbbec=False,
                                           transform_to_map=False, use_vlm_sam_fallback=True,
                                           sort_closest=True, return_segments=True))
    seq.add_child(_Report("recheck report", "gpsr/recheck_result", "RealSense AFTER grasp"))
    seq.add_child(BtNode_MoveArmSingle("6. arm back to base_moving",
                                       arm_pose_bb_key=bb_keys.ARM_NAVIGATING, add_octomap=False))

    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=seq)
    print(f"[DIAG] grasp diagnostic for object={obj!r} — connecting to arm/realsense/grasp servers...")
    tree.setup(timeout=30, node_name="gpsr_grasp_diag")
    print("[DIAG] all servers connected. Running sequence:")
    result = None
    for _ in range(240):  # ~120 s budget
        tree.tick()
        if seq.status == Status.SUCCESS:
            result = "SUCCESS"; break
        if seq.status == Status.FAILURE:
            result = "FAILURE"
            for n in seq.iterate():
                if n.status == Status.FAILURE and (n.feedback_message or ""):
                    print(f"[DIAG] FAIL @ {n.name}: {n.feedback_message}")
            break
        time.sleep(0.5)
    print(f"[DIAG] ===== RESULT: {result or 'TIMEOUT'} =====")
    rclpy.shutdown()


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
    """Type/speak a command -> plan -> execute, looping, saving each plan.

    Per-module integration harness: the planner always runs (it "splits" the
    command and generates the tree); which *executing* subsystems are real vs.
    stubbed is read from ``mock_config.json`` BEFORE the run (``mock_mode.enabled:
    true`` + per-subsystem ``enabled``: ``true`` = MOCKED, ``false`` = REAL).
    ``keyboard_control.enabled: false`` makes mocked nodes auto-succeed.

    Command source:
      * ``BT_GPSR_DEBUG_CMD`` set -> use it verbatim once, then idle.
      * unset -> intake LOOP. If ``audio_input`` is MOCKED and you're on an
        interactive terminal, you TYPE each command (remote / no microphone);
        if audio is REAL you speak it. After each command the cycle loops so you
        can enter another.

    Every generated plan is frozen to a re-runnable ``.py`` under
    ``BT_GPSR_PLAN_DIR`` (default ``./gpsr_runs``) so a command can be replayed
    later for debugging (``python <that_file>.py``).
    """
    from pathlib import Path
    from .orchestrator import create_execute_command, create_orchestrator_init
    from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_ListenAction
    from .small_trees import BtNode_AnnounceFromBB

    load_knowledge_from_constants(CONSTANTS_PATH)
    command = os.environ.get("BT_GPSR_DEBUG_CMD", "").strip()
    plan_dir = Path(os.environ.get("BT_GPSR_PLAN_DIR", "gpsr_runs")).resolve()
    plan_dir.mkdir(parents=True, exist_ok=True)
    print(f"[gpsr-test-orchestrator] saved plans -> {plan_dir}")

    rclpy.init()
    cycle = py_trees.composites.Sequence("Test orchestrator", memory=True)
    _arm_constants_to_bb(cycle)
    if command:
        cycle.add_child(BtNode_WriteToBlackboard(
            "command (env)", bb_namespace="", bb_source=None,
            bb_key=bb_keys.COMMAND, object=command,
        ))
    else:
        # Intake: spoken prompt (silent when announcement is mocked); a MOCKED
        # listen on an interactive terminal then prompts you to TYPE the command.
        cycle.add_child(BtNode_Announce(
            "prompt", bb_source=None,
            message="Please give me a command.",
        ))
        listen_timeout = float(os.environ.get("BT_GPSR_LISTEN_TIMEOUT", "30.0"))
        cycle.add_child(BtNode_ListenAction(
            "listen", bb_dest_key=bb_keys.COMMAND, timeout=listen_timeout,
        ))
        cycle.add_child(BtNode_AnnounceFromBB(
            "echo heard", bb_keys.COMMAND, prefix="I heard: ",
        ))
    cycle.add_child(create_orchestrator_init())
    cycle.add_child(create_execute_command(
        max_steps=25, max_corrections=3, emit_plan_dir=str(plan_dir),
    ))
    if command:
        # One-shot: run the injected command once, then idle.
        cycle.add_child(py_trees.behaviours.Running("idle (ctrl-c to exit)"))
    # Typed/voice path: no trailing idle -> the memory Sequence re-runs from the
    # top after each command, so you can enter another.

    tree = py_trees_ros.trees.BehaviourTree(root=cycle)
    tree.setup(timeout=15, node_name="gpsr_test_orchestrator")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(title="orchestrator")
    # Per-command logging (plan + each step's result + the failing node's feedback)
    # lands beside the saved plans, under <plan_dir>/logs.
    from .command_logger import create_command_logger, combine_post_tick_handlers
    log_tree, shutdown_logger = create_command_logger(str(plan_dir / "logs"))
    tree.tick_tock(
        period_ms=500.0,
        post_tick_handler=combine_post_tick_handlers(print_tree, log_tree),
    )
    try:
        rclpy.spin(tree.node)
    finally:
        shutdown_logger()
        shutdown_visualizer()
        rclpy.shutdown()
