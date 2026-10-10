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
from .gpsr_full import CONSTANTS_PATH, _load_arm_constants, _load_arm_orbbec_look


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
    seq.add_child(BtNode_WriteToBlackboard(
        "arm orbbec look", bb_namespace="", bb_source=None,
        bb_key=bb_keys.ARM_ORBBEC_LOOK, object=_load_arm_orbbec_look(),
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
    seq.add_child(BtNode_WriteToBlackboard("arm orbbec look", bb_namespace="", bb_source=None,
                                           bb_key=bb_keys.ARM_ORBBEC_LOOK, object=_load_arm_orbbec_look()))
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
    """Collect N commands + plans up front, then execute them one by one.

    Per-module integration harness: the planner always runs (it "splits" each
    command and generates the tree); which *executing* subsystems are real vs.
    stubbed is read from ``mock_config.json`` BEFORE the run (``mock_mode.enabled:
    true`` + per-subsystem ``enabled``: ``true`` = MOCKED, ``false`` = REAL).
    ``keyboard_control.enabled: false`` makes mocked nodes auto-succeed.

    Flow: enter arena → go to the command point once → for each of
    ``BT_GPSR_NUM_COMMANDS`` (default 3) commands: ask → plan → announce the plan;
    then execute the collected plans one by one (announcing before each).

    Command source:
      * ``BT_GPSR_DEBUG_CMD`` set -> injected. One command, or several joined by
        ``|`` (e.g. ``"go to the kitchen|count the apples|say hello"``) to drive
        the whole batch non-interactively.
      * unset -> for each slot the robot prompts + listens. If ``audio_input`` is
        MOCKED on an interactive terminal you TYPE each command; if audio is REAL
        you speak it.

    Every generated plan is frozen to a re-runnable ``.py`` under
    ``BT_GPSR_PLAN_DIR`` (default ``./gpsr_runs``) so a command can be replayed
    later for debugging (``python <that_file>.py``).
    """
    from pathlib import Path
    from .orchestrator import (
        create_batch_command_flow, make_inject_intake, make_listen_intake,
        create_orchestrator_init, create_goto_command_point, has_command_point,
    )
    from .small_trees import create_enter_arena

    load_knowledge_from_constants(CONSTANTS_PATH)
    plan_dir = Path(os.environ.get("BT_GPSR_PLAN_DIR", "gpsr_runs")).resolve()
    plan_dir.mkdir(parents=True, exist_ok=True)
    num_commands = int(os.environ.get("BT_GPSR_NUM_COMMANDS", "3"))
    # Command source: BT_GPSR_DEBUG_CMD (one command, or several joined by '|')
    # is injected for desktop tests; unset -> prompt + listen for each of
    # num_commands commands (type them when audio is mocked).
    debug = os.environ.get("BT_GPSR_DEBUG_CMD", "").strip()
    if debug:
        commands = [c.strip() for c in debug.split("|") if c.strip()]
        make_intake = make_inject_intake(commands)
        num_commands = len(commands)
    else:
        listen_timeout = float(os.environ.get("BT_GPSR_LISTEN_TIMEOUT", "30.0"))
        make_intake = make_listen_intake(listen_timeout=listen_timeout)
    print(f"[gpsr-test-orchestrator] saved plans -> {plan_dir}  "
          f"(collecting {num_commands} command(s) up front, then executing)")

    rclpy.init()
    root = py_trees.composites.Sequence("Test orchestrator", memory=True)
    # Enter the arena through the door once, before collecting commands.
    root.add_child(create_enter_arena())
    _arm_constants_to_bb(root)
    # GPSR: go to the command point ONCE; all commands are collected there.
    if has_command_point():
        root.add_child(create_goto_command_point())
    else:
        print("[gpsr-test-orchestrator] 'command_point' has no pose in "
              "constants.json possible_poses — skipping the go-to-command-point step.")
    # Capture the command-point start pose once (operator spot for deliveries).
    root.add_child(create_orchestrator_init())
    root.add_child(create_batch_command_flow(
        num_commands=num_commands, make_intake=make_intake,
        max_steps=25, max_corrections=3, emit_plan_dir=str(plan_dir),
    ))
    root.add_child(py_trees.behaviours.Running("idle (ctrl-c to exit)"))

    tree = py_trees_ros.trees.BehaviourTree(root=root)
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


# ---- two-layer planner / executor dev tests (offline, no ROS needed) ----

def main_orchestrator_two_layer():
    """TWO-LAYER end-to-end harness: split + parallel-plan up front, then a
    DynamicExecutor per slot swaps target subtrees into the RUNNING tree.

    Same command source / arena / command-point shape as ``main_orchestrator``,
    but uses ``create_batch_command_flow_new`` so the top layer splits each
    command into targets and the lower layer plans them in parallel, then the
    executor phase drives them with runtime subtree swaps (``gpsr_tree=tree``).
    Full-mock preset makes the whole run deterministic and network-free.
    """
    from pathlib import Path
    from .orchestrator import (
        create_batch_command_flow_new, make_inject_intake, make_listen_intake,
        create_orchestrator_init, create_goto_command_point, has_command_point,
    )
    from .planner import GPSRPlanner
    from .small_trees import create_enter_arena

    load_knowledge_from_constants(CONSTANTS_PATH)
    plan_dir = Path(os.environ.get("BT_GPSR_PLAN_DIR", "gpsr_runs")).resolve()
    plan_dir.mkdir(parents=True, exist_ok=True)
    num_commands = int(os.environ.get("BT_GPSR_NUM_COMMANDS", "3"))
    debug = os.environ.get("BT_GPSR_DEBUG_CMD", "").strip()
    if debug:
        commands = [c.strip() for c in debug.split("|") if c.strip()]
        make_intake = make_inject_intake(commands)
        num_commands = len(commands)
    else:
        listen_timeout = float(os.environ.get("BT_GPSR_LISTEN_TIMEOUT", "30.0"))
        make_intake = make_listen_intake(listen_timeout=listen_timeout)
    print(f"[gpsr-test-orchestrator-2layer] saved plans -> {plan_dir}  "
          f"(collecting {num_commands} command(s) up front, then executing)")

    rclpy.init()
    root = py_trees.composites.Sequence("Test orchestrator (two-layer)", memory=True)
    root.add_child(create_enter_arena())
    _arm_constants_to_bb(root)
    if has_command_point():
        root.add_child(create_goto_command_point())
    root.add_child(create_orchestrator_init())
    planner = GPSRPlanner()
    root.add_child(create_batch_command_flow_new(
        planner, num_commands=num_commands, make_intake=make_intake,
        max_replans_per_target=3, emit_plan_dir=str(plan_dir),
    ))
    root.add_child(py_trees.behaviours.Running("idle (ctrl-c to exit)"))

    tree = py_trees_ros.trees.BehaviourTree(root=root)
    tree.setup(timeout=15, node_name="gpsr_test_orchestrator_2layer",
               gpsr_tree=tree)
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="orchestrator (two-layer)")
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


def main_split_command():
    """TOP LAYER: split a command into self-contained targets (offline).

    Reads ``BT_GPSR_CMD`` (default: "fetch Susan a coke, she is in the living
    room"). Uses a fresh GPSRPlanner so full-mock mode stays deterministic and
    network-free. Prints the split; no tree is built, no ROS involved.
    """
    command = os.environ.get("BT_GPSR_CMD",
                             "fetch Susan a coke, she is in the living room")
    from .planner import GPSRPlanner
    planner = GPSRPlanner()
    targets = planner.split_command(command)
    print(f"[split] command: {command!r}")
    print(f"[split] {len(targets)} target(s):")
    for i, t in enumerate(targets):
        print(f"  {i}. {t}")
    print(f"[split] descs: {[t.get('desc') if isinstance(t, dict) else t for t in targets]}")
    return targets


def main_lower_layer():
    """LOWER LAYER: plan ONE target and show its subtree (offline).

    Reads ``BT_GPSR_TARGET`` (default: "grab a coke"). Plans it, validates the
    plan, builds + prints the executing subtree via ``py_trees.display``. Uses
    full-mock planner so no network/ROS is touched.
    """
    from .planner import GPSRPlanner
    from .planner_validators import validate_plan
    from .small_trees import ACTION_FACTORIES
    from .orchestrator import KNOWN_LOCATIONS, START_LOCATION_ALIASES
    target = os.environ.get("BT_GPSR_TARGET", "grab a coke")
    planner = GPSRPlanner()
    planner.request_plan_all(0, [target])
    import time
    for _ in range(200):
        if planner.all_targets_ready(0, 1):
            break
        time.sleep(0.01)
    plan = planner.get_action_plan(0, 0)
    known_loc_arg = (set(KNOWN_LOCATIONS.keys()) | START_LOCATION_ALIASES) or None
    ok, reason = validate_plan(plan, target, set(ACTION_FACTORIES.keys()),
                               known_locations=known_loc_arg)
    print(f"[lower] target: {target!r}")
    print(f"[lower] plan ({len(plan)} step(s)): {plan}")
    print(f"[lower] validate_plan -> {ok}"
          + (f" ({reason})" if reason else ""))
    subtree = planner.get_target_subtree(0, 0)
    if subtree is not None:
        print(py_trees.display.unicode_tree(subtree))
    return plan, ok


def main_dynamic_executor():
    """DynamicExecutor unit test (offline, FakePlanner, no LLM/robot).

    Drives a 3-target command where target 0 fails once then succeeds, target 1
    succeeds, and target 2 fails through its replan budget and is skipped. Runs
    a ``py_trees.trees.BehaviourTree`` with a stub ROS node + ``gpsr_tree=tree``
    and asserts the executor swaps subtrees at runtime, advances TARGET_INDEX,
    logs target-level STATE_LOG, and ends SUCCESS. Prints PASS/FAIL + the log.
    """
    import types
    from py_trees.common import Status, Access
    from py_trees.blackboard import Client
    from .orchestrator import DynamicExecutor

    class TicksBehaviour(py_trees.behaviour.Behaviour):
        def __init__(self, name, status):
            super().__init__(name)
            self._status = status
        def update(self):
            return self._status

    def make_subtree(name, status):
        root = py_trees.composites.Sequence(name, memory=True)
        root.add_child(TicksBehaviour(name + "/leaf", status))
        return root

    class FakePlanner:
        def __init__(self):
            self.replans = []
            self.t0_first = True
        def _get_desc(self, slot, index):
            return {0: "grab coke", 1: "count apples", 2: "wave"}.get(index, "?")
        def get_target_subtree(self, slot, index):
            if index == 0 and self.t0_first:
                return make_subtree("target:0:0-fail", Status.FAILURE)
            if index == 2:
                return make_subtree("target:0:2-fail", Status.FAILURE)
            return make_subtree(f"target:0:{index}", Status.SUCCESS)
        def replan_target(self, slot, index, reason):
            self.replans.append((index, reason))
            if index == 0:
                self.t0_first = False

    fp = FakePlanner()
    node = types.SimpleNamespace(get_name=lambda: "stub")
    root = py_trees.composites.Sequence("root", memory=True)
    executor = DynamicExecutor("exec", 0, fp, max_replans_per_target=2)
    root.add_child(executor)
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=15, node_name="dynamic_executor_test", node=node,
               gpsr_tree=tree)
    bb = Client(name="exec_test")
    bb.register_key(bb_keys.SAVED_TARGETS_PREFIX + "0", access=Access.WRITE)
    bb.register_key(bb_keys.TARGETS, access=Access.WRITE)
    bb.register_key(bb_keys.TARGET_INDEX, access=Access.READ)
    bb.register_key(bb_keys.STATE_LOG, access=Access.READ)
    bb.set(bb_keys.SAVED_TARGETS_PREFIX + "0",
           ["grab coke", "count apples", "wave"], overwrite=True)
    bb.set(bb_keys.TARGETS, ["grab coke", "count apples", "wave"], overwrite=True)

    import time
    for _ in range(60):
        tree.tick()
        if tree.root.status != Status.RUNNING:
            break
        time.sleep(0.005)
    # Target 0 replans exactly once (first attempt fails, then succeeds).
    # Target 2 fails 3 times through its budget of 2 replans, then is skipped.
    t0_replans = [r for r in fp.replans if r[0] == 0]
    t2_replans = [r for r in fp.replans if r[0] == 2]
    ok = (tree.root.status == Status.SUCCESS
          and bb.get(bb_keys.TARGET_INDEX) == 3
          and len(t0_replans) == 1 and len(t2_replans) == 2)
    print(f"[exec] root status: {tree.root.status}")
    print(f"[exec] target_index: {bb.get(bb_keys.TARGET_INDEX)}")
    print(f"[exec] replans: {fp.replans}")
    for line in (bb.get(bb_keys.STATE_LOG) or []):
        print(f"[exec] {line}")
    print(f"[exec] {'PASS' if ok else 'FAIL'}")
    return ok
