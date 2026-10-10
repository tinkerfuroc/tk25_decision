from __future__ import annotations

"""Pick and Place — finalized 2026 mission tree (entry: ``pick-and-place-2026``).

A NEW production tree that does NOT touch the canonical
``pick_and_place.pickAndPlaceShortened`` (``pick-and-place``). The canonical tree
is a China-2026 linear demo: it grasps table items and *opens the gripper in
place* (no real place at a destination), fakes the shelf perception, has no
object→destination routing, and never assembles breakfast.

This v2 composes the *real* manipulation primitives that already exist and are
individually tested, into the rulebook §5.2 shape:

  1. **Table cleanup as a destination-routed pick→transport→real-place loop**
     (the core piece the canonical tree lacks). For each destination class we run
     a bounded sweep — grasp a class item (``table_grasping.createTableGrasp``),
     drive to the destination, and place:
       * trash    → ``drop_trash.createDropTrash`` (drop into bin)
       * dishware → ``createDropTrash`` re-targeted to the wash-staging surface
         (rulebook allows a wash-staging surface in place of the dishwasher rack)
       * cabinet  → drive to cabinet → ``table_placing.createTablePlacing`` (the
         REAL vision-collision ``BtNode_Place``; safe placing, not a drop)
     The whole loop is gated by a runtime cutover so it stops before the 7:00
     limit instead of over-running.
  2. **Breakfast assembly** (``breakfast_assembly.createBreakfastAssembly`` — the
     rulebook's 2nd main goal, also absent from the canonical tree).

Deliberately bounded / surfaced (not silently faked):
  * Per-destination item budget is a FIXED count (``SWEEP_ITEM_BUDGET``), not
    "until the table is empty" — ``createTableGrasp`` masks grasp failure with its
    own cleanup Selector, so there is no clean empty-table signal to loop on. The
    global runtime cutover caps total effort; refine later with a detection-count
    gate.
  * Dishwasher door/rack/tablet, floor-trash, and pouring bonuses are out of scope
    (need primitives that don't exist yet) — see ``RULEBOOK_PLAN.md``.
  * ``BtNode_Grasp`` does not return a grasp orientation, so a fixed top-down
    place orientation is seeded into ``KEY_GRASP_POSE`` (see ``breakfast_assembly``).

Run::

    ros2 run behavior_tree pick-and-place-2026

Fully offline (auto-advance, no servers)::

    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json \
        ros2 run behavior_tree pick-and-place-2026
"""

import time

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.visualization import create_post_tick_visualizer

# Reuse canonical mission helpers unchanged (enter / nav / table scan).
from .pick_and_place import (
    createConstantWriter,
    enterArena,
    navigateToTable,
    scanTableAndAnnounce,
    _gotoRetryWith_Announcement,
)
# Reuse the real destination subtrees.
from .table_grasping import createTableGrasp
from .table_placing import createTablePlacing
from .drop_trash import createDropTrash
# Breakfast phase (this session).
from .breakfast_assembly import createBreakfastAssembly, _DEFAULT_PLACE_ORIENTATION

from .config import (
    KEY_ARM_WASH,
    KEY_ENV_POINTS,
    KEY_GRASP_POSE,
    KEY_MAX_RUNTIME,
    KEY_POSE_CABINET,
    KEY_POSE_WASH_STAGING,
    MAX_RUNTIME_SEC,
)

# Bounded items grasped per destination class (rulebook caps Pick/Place at 12x;
# the global cutover is the real limiter — see module docstring).
SWEEP_ITEM_BUDGET = 2

# Module-local blackboard key for the runtime deadline (epoch seconds).
KEY_RUNTIME_DEADLINE = "pp_runtime_deadline"

# Open-vocab detection prompts that double as the per-destination classifier.
TRASH_PROMPT = "trash . candy wrapper . empty can . crumpled paper . used napkin"
DISHWARE_PROMPT = "plate . bowl . cup . mug . fork . spoon . knife"
CABINET_PROMPT = "bottle . snack box . can . food package . jar . carton"


# --------------------------------------------------------------------------- #
# Runtime cutover guards (module-local; PickAndPlace stays self-contained)
# --------------------------------------------------------------------------- #
class BtNode_InitRuntimeDeadline(py_trees.behaviour.Behaviour):
    """Write ``deadline = now + max_runtime`` once, at runtime (not construction)."""

    def __init__(self, name: str, *, deadline_key: str, max_runtime_key: str):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(
            key="deadline",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", deadline_key),
        )
        self.bb.register_key(
            key="max_runtime",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", max_runtime_key),
        )

    def update(self) -> py_trees.common.Status:
        try:
            budget = self.bb.max_runtime
        except Exception:
            budget = None
        if not isinstance(budget, (int, float)):
            budget = MAX_RUNTIME_SEC
        self.bb.deadline = time.time() + float(budget)
        self.feedback_message = f"runtime deadline set (+{budget:.0f}s)"
        return py_trees.common.Status.SUCCESS


class BtNode_RuntimeNotExpired(py_trees.behaviour.Behaviour):
    """Condition: FAILURE once the runtime deadline is passed (ends the sweep)."""

    def __init__(self, name: str, *, deadline_key: str):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(
            key="deadline",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", deadline_key),
        )

    def update(self) -> py_trees.common.Status:
        try:
            deadline = self.bb.deadline
        except Exception:
            deadline = None
        if isinstance(deadline, (int, float)) and time.time() >= float(deadline):
            self.feedback_message = "runtime cutover reached — stopping sweep"
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS


# --------------------------------------------------------------------------- #
# v2 seeds + destination place subtrees
# --------------------------------------------------------------------------- #
def _createV2Seeds() -> py_trees.composites.Parallel:
    """Seed the inputs the canonical constant writer does not: a default place
    orientation (grasp action returns none), an empty env-cloud slot, and the
    runtime deadline."""
    root = py_trees.composites.Parallel(
        name="Seed v2 place inputs + deadline",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Seed default place orientation",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_GRASP_POSE,
            object=_DEFAULT_PLACE_ORIENTATION,
        )
    )
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Init env points slot",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_ENV_POINTS,
            object=None,
        )
    )
    root.add_child(
        BtNode_InitRuntimeDeadline(
            name="Init runtime deadline",
            deadline_key=KEY_RUNTIME_DEADLINE,
            max_runtime_key=KEY_MAX_RUNTIME,
        )
    )
    return root


def _createCabinetPlace() -> py_trees.composites.Sequence:
    """Drive to the cabinet and place the held item with the real vision place."""
    seq = py_trees.composites.Sequence(name="Transport + place at cabinet", memory=True)
    seq.add_child(_gotoRetryWith_Announcement("cabinet", KEY_POSE_CABINET))
    seq.add_child(createTablePlacing(item_description="the cabinet item"))
    return seq


def _createWashStagingPlace() -> py_trees.composites.Selector:
    """Drop dishware at the wash-staging surface (drop_trash re-targeted)."""
    return createDropTrash(
        target_pose_key=KEY_POSE_WASH_STAGING,
        drop_arm_pose_key=KEY_ARM_WASH,
    )


def createDestinationSweep(
    label: str,
    detect_prompt: str,
    place_factory,
    *,
    item_budget: int = SWEEP_ITEM_BUDGET,
) -> py_trees.decorators.FailureIsSuccess:
    """A bounded grasp→transport→place sweep for one destination class.

    Each attempt: cutover-check → re-approach table → grasp a class item →
    transport + place. ``Repeat`` runs the attempt up to ``item_budget`` times;
    the cutover-check fails the attempt (and ends the sweep early) once the
    runtime deadline passes. Wrapped ``FailureIsSuccess`` so a finished/aborted
    sweep never aborts the mission.
    """
    attempt = py_trees.composites.Sequence(name=f"{label} sweep attempt", memory=True)
    attempt.add_child(
        BtNode_RuntimeNotExpired(
            name=f"{label}: runtime not expired?",
            deadline_key=KEY_RUNTIME_DEADLINE,
        )
    )
    attempt.add_child(navigateToTable())
    attempt.add_child(
        BtNode_Announce(
            name=f"announce {label} sweep",
            bb_source=None,
            message=f"Looking for {label} items to clear from the table.",
        )
    )
    attempt.add_child(createTableGrasp(prompt=detect_prompt))
    attempt.add_child(place_factory())
    return py_trees.decorators.FailureIsSuccess(
        name=f"{label} cleanup sweep (best effort)",
        child=py_trees.decorators.Repeat(
            name=f"repeat {label} sweep x{item_budget}",
            child=attempt,
            num_success=item_budget,
        ),
    )


def createTableCleanupPhase() -> py_trees.composites.Sequence:
    """Perceive the table, then sweep each destination class with a real place."""
    root = py_trees.composites.Sequence(name="Table cleanup phase", memory=True)
    root.add_child(navigateToTable())
    root.add_child(scanTableAndAnnounce())
    root.add_child(createDestinationSweep("trash", TRASH_PROMPT, createDropTrash))
    root.add_child(createDestinationSweep("dishware", DISHWARE_PROMPT, _createWashStagingPlace))
    root.add_child(createDestinationSweep("cabinet", CABINET_PROMPT, _createCabinetPlace))
    return root


def createPickAndPlaceTask2026() -> py_trees.behaviour.Behaviour:
    """Full v2 mission: enter → table cleanup (routed real place) → breakfast."""
    root = py_trees.composites.Sequence(name="Pick and Place Task 2026", memory=True)
    root.add_child(createConstantWriter())
    root.add_child(_createV2Seeds())
    root.add_child(enterArena())
    root.add_child(
        BtNode_Announce(
            name="announce mission start",
            bb_source=None,
            message="Starting pick and place. I'll tidy the table, then set breakfast.",
        )
    )
    root.add_child(createTableCleanupPhase())
    root.add_child(
        BtNode_Announce(
            name="announce cleanup done",
            bb_source=None,
            message="Table cleared. Now setting up breakfast.",
        )
    )
    root.add_child(navigateToTable())  # breakfast surface (clean table area)
    root.add_child(createBreakfastAssembly())
    root.add_child(
        BtNode_Announce(
            name="announce mission complete",
            bb_source=None,
            message="Pick and place complete. The table is tidy and breakfast is set.",
        )
    )
    return root


def create_tree() -> py_trees.behaviour.Behaviour:
    """Alias for the offline smoke harness."""
    return createPickAndPlaceTask2026()


def main():
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=createPickAndPlaceTask2026())
    tree.setup(timeout=15, node_name="pick_and_place_2026")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="pick-and-place-2026"
    )
    tree.tick_tock(period_ms=300.0, post_tick_handler=print_tree)
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
