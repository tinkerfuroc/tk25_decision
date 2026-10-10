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
from .small_trees import bb_keys, create_enter_arena, _one_shot
from .telemetry import GpsrTelemetry, set_default_telemetry
from .supervision.controller import MissionSupervisor
from .supervision.runtime import (
    configure_default_supervisor,
    get_default_supervisor,
    set_default_supervisor,
)

# Module-level decoupled orchestrator: the two-layer planner invoked repeatedly
# (per slot, per target, per replan) by the bridge nodes + DynamicExecutor. It
# never holds a tree/node reference and its planner threads never touch the BB.
PLANNER = GPSRPlanner()

DEFAULT_PLAN_DIR = Path(os.environ.get("BT_GPSR_PLAN_DIR", "gpsr_runs")).resolve()
# How many commands to collect up front before executing (RoboCup GPSR = 3).
NUM_COMMANDS = int(os.environ.get("BT_GPSR_NUM_COMMANDS", "3"))


def _start_outside_arena() -> bool:
    """Whether to run the physical door-entry gate at mission startup.

    Real GPSR deployments begin in front of the door, so the default remains
    ``True``.  The RCW2026 simulator scenario places the robot at its command
    point inside the arena (``initial_pose=[-2, -2, 0]``); its bench launcher
    sets ``GPSR_START_OUTSIDE_ARENA=0`` to avoid asking for a door that is
    already behind the robot.
    """
    raw = os.environ.get("GPSR_START_OUTSIDE_ARENA", "1").strip().lower()
    return raw not in {"0", "false", "no", "off"}


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
    telemetry: GpsrTelemetry | None = None,
    supervisor: MissionSupervisor | None = None,
) -> py_trees.behaviour.Behaviour:
    """Build the live orchestrator root.

    New flow: enter arena → go to the command point ONCE → collect all
    ``num_commands`` commands, planning + announcing each up front → then execute
    them one by one (announcing before each).

    Command source: an explicit ``commands`` list (or ``BT_GPSR_CMD`` as a
    ``|``-separated list) is injected — used for desktop / mock e2e tests. With
    neither, the robot prompts + listens for each command by voice (competition).
    """
    if telemetry is not None:
        set_default_telemetry(telemetry)
    if supervisor is not None:
        set_default_supervisor(supervisor)
    elif get_default_supervisor() is None:
        configure_default_supervisor(telemetry=telemetry)
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

    if telemetry is not None:
        telemetry.emit(
            "run.configured",
            {"expected_task_count": n, "mode": "injected" if commands else "voice"},
            phase="intake",
        )

    root = py_trees.composites.Sequence("GPSR orchestrator", memory=True)
    # Stable trajectory identity is kept on the blackboard for all task slots
    # and is independent of the natural-language command text.
    run_id = telemetry.trajectory_id if telemetry is not None else f"gpsr-{os.getpid()}"
    root.add_child(BtNode_WriteToBlackboard(
        "set GPSR run id", bb_namespace="", bb_source=None,
        bb_key=bb_keys.RUN_ID, object=run_id,
    ))
    # Arm poses must be on the blackboard before arena entry: the door watch
    # stows the arm to the orbbec-look pose before sensing the door.
    _arm_constants_to_bb(root)
    # Physical robots start OUTSIDE the arena in front of the door. The seeded
    # simulator scenario starts at command_point (-2,-2), already inside, so
    # its battery opts out with GPSR_START_OUTSIDE_ARENA=0. In that mode latch
    # the entry state before the gate; this prevents a false door request and
    # keeps the production default safe for hardware.
    start_outside = _start_outside_arena()
    if not start_outside:
        root.add_child(BtNode_WriteToBlackboard(
            "seed simulator arena entered", bb_namespace="", bb_source=None,
            bb_key=bb_keys.ARENA_ENTERED, object=True,
        ))
        if telemetry is not None:
            telemetry.emit("arena.entry_skipped", {
                "reason": "simulator starts inside arena",
            }, phase="startup")
    # Real deployments start OUTSIDE the arena in front of the door: wait for
    # it to open and enter once.  The simulator starts at command_point inside
    # the arena; its bench sets GPSR_START_OUTSIDE_ARENA=0 and skips this gate.
    # K1: OneShot prevents replay after a later sibling failure restarts the
    # memory root sequence.
    if _start_outside_arena():
        root.add_child(_one_shot(create_enter_arena()))
    # GPSR: go to the command point ONCE to receive all commands there.  The
    # RCW2026 scenario already spawns Tinker at that exact pose; sending a
    # second Nav2 goal there only spins/replans while AMCL settles and can
    # restart the outer sequence.  Hardware and any explicit outside start
    # retain the normal command-point drive.
    if has_command_point() and start_outside:
        # A later task failure can restart the root sequence.  Latch this
        # successful entry navigation so recovery retries the failed task
        # instead of repeatedly sending the robot back to (-2, -2).
        root.add_child(_one_shot(create_goto_command_point()))
    elif has_command_point() and telemetry is not None:
        telemetry.emit("arena.command_point_skipped", {
            "reason": "simulator starts at command_point",
        }, phase="startup")
    # Capture the command-point start pose once (operator spot for deliveries).
    root.add_child(create_orchestrator_init())
    # TWO-LAYER batch flow: top split + parallel per-target planning up front,
    # then a DynamicExecutor per slot swaps ready target subtrees into the
    # RUNNING tree at runtime. PLANNER is the module-level decoupled orchestrator.
    # The batch is the mission boundary.  A failed plan or execution must be
    # terminal for this run: allowing FAILURE to bubble into this outer memory
    # Sequence restarts command intake on the next tick and can livelock the
    # planner.  Keep the wrapper name stable for telemetry and structure tests.
    batch_flow = create_batch_command_flow_new(
        PLANNER,
        num_commands=n,
        make_intake=make_intake,
        max_replans_per_target=max_corrections,
        emit_plan_dir=str(plan_dir),
    )
    root.add_child(py_trees.decorators.FailureIsSuccess(
        "batch command flow", batch_flow,
    ))
    root.add_child(py_trees.behaviours.Running("idle"))
    return root


class TickStallWatchdog:
    """Dump every thread's Python stack once the tree stops ticking.

    GPSR sim battery 2026-09-30 fix6: three runs froze right after
    ``plan.materialized`` (no tick, no telemetry, no log line) for the whole
    1500 s budget, and the live process could not be inspected from outside.
    A daemon thread watches the post-tick timestamp; after ``stall_s`` without
    a tick it writes ``faulthandler`` tracebacks of all threads to stderr (the
    run's orchestrator.log) and repeats every ``stall_s`` while still stalled.
    ``GPSR_TICK_STALL_S`` overrides the threshold (<= 0 disables).
    """

    def __init__(self, stall_s: float = 60.0):
        import threading
        import time
        self._time = time
        self.stall_s = float(os.environ.get("GPSR_TICK_STALL_S", stall_s) or 0)
        self.last_tick = time.monotonic()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="tick-stall-watchdog", daemon=True)

    def post_tick(self, _tree=None) -> None:
        self.last_tick = self._time.monotonic()

    def start(self) -> None:
        if self.stall_s > 0:
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def check(self) -> bool:
        """True (and stacks dumped) when the tree has been silent too long."""
        import faulthandler
        import sys
        silent = self._time.monotonic() - self.last_tick
        if self.stall_s <= 0 or silent < self.stall_s:
            return False
        print(f"[gpsr-orchestrator] TICK STALL: no tick for {silent:.0f}s -- thread stacks follow",
              file=sys.stderr, flush=True)
        faulthandler.dump_traceback(file=sys.stderr, all_threads=True)
        sys.stderr.flush()
        return True

    def _run(self) -> None:
        while not self._stop.wait(min(self.stall_s, 10.0)):
            if self.check():
                self.last_tick = self._time.monotonic()  # next dump after another stall_s


def main():
    rclpy.init(args=None)
    DEFAULT_PLAN_DIR.mkdir(parents=True, exist_ok=True)
    telemetry = GpsrTelemetry(DEFAULT_PLAN_DIR)
    print(f"[gpsr-orchestrator] plan modules -> {DEFAULT_PLAN_DIR}; trajectory={telemetry.trajectory_id}")

    root = createGPSROrchestrator(telemetry=telemetry)
    tree = py_trees_ros.trees.BehaviourTree(root=root)
    # gpsr_tree=tree forwards the running tree into every behaviour's setup()
    # (py_trees_ros relays extra kwargs). DynamicExecutor reads it so it can
    # swap in freshly-planned target subtrees at runtime.
    tree.setup(timeout=15, node_name="gpsr_orchestrator", gpsr_tree=tree)
    telemetry.attach_ros(tree.node)
    # ``py_trees_ros`` supplies a SnapshotVisitor; telemetry reuses it so its
    # tick records describe nodes actually visited, not merely the topology.
    telemetry.attach_tick_visitor(tree)
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(title="GPSR orchestrator")
    # Per-command logging (plan + each step's result + the failing node's feedback).
    from .command_logger import create_command_logger, combine_post_tick_handlers
    log_tree, shutdown_logger = create_command_logger(str(DEFAULT_PLAN_DIR / "logs"))
    trace_tree = telemetry.post_tick_handler()
    watchdog = TickStallWatchdog()
    tree.tick_tock(
        period_ms=500.0,
        post_tick_handler=combine_post_tick_handlers(print_tree, log_tree, trace_tree, watchdog.post_tick),
    )
    watchdog.start()
    try:
        # A malformed message on one of the simulator bridge subscriptions can
        # make rclpy's executor raise while converting a queued sample.  The
        # old one-shot ``rclpy.spin`` let that transient conversion error kill
        # the whole battery halfway through a command.  ``spin_once`` consumes
        # the offending sample and lets the behaviour-tree timer continue.
        while rclpy.ok():
            try:
                rclpy.spin_once(tree.node, timeout_sec=1.0)
            except RuntimeError as exc:
                if "Unable to convert call argument" not in str(exc):
                    raise
                print(
                    f"[gpsr-orchestrator] ignoring malformed bridge message: {exc}",
                    flush=True,
                )
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        watchdog.stop()
        supervisor = get_default_supervisor()
        if supervisor is not None:
            supervisor.close()
        set_default_supervisor(None)
        shutdown_logger()
        shutdown_visualizer()
        tree.shutdown()
        telemetry.close()
        set_default_telemetry(None)
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
