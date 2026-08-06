from __future__ import annotations

"""Pick-and-Place rulebook mission tree (RoboCup@Home 2026 §5.2).

Three main scored goals: (1) dining-table cleanup, (2) serve breakfast,
(3) extra-surface cleanup. Cleanup is data-driven (inventory -> queue ->
generic per-item loop); breakfast is an explicit fixed-point 4-item table.

`place_policy` ('vlm' default | 'hardcoded') is threaded to every surface
place leaf via BtNode_PopWorkItem; it is inert under BT_MOCK_MODE (place nodes
auto-succeed) and matters only on the real robot. The old narrow demo lives on
in pick_and_place.pickAndPlaceShortened (entry `pick-and-place-demo`).
"""

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import (
    BtNode_Grasp,
    BtNode_GripperAction,
    BtNode_ScanAndPlace,
)
from behavior_tree.TemplateNodes.Vision import BtNode_TurnPanTilt

from .config import (
    GRASP_RETRY_LIMIT,
    KEY_ARM_CABINET,
    KEY_ARM_TABLE,
    KEY_ARM_TRASH,
    KEY_ARM_WASH,
    KEY_ACTIVE_OBJECT_CLASS,
    KEY_ACTIVE_SOURCE_POSE,
    KEY_ACTIVE_TARGET_POSE,
    KEY_ANNOUNCEMENT_MSG,
    KEY_INVENTORY_TABLE,
    KEY_OBJECT_LABEL,
    KEY_POINT_BREAKFAST_BOWL,
    KEY_POINT_BREAKFAST_CEREAL,
    KEY_POINT_BREAKFAST_MILK,
    KEY_POINT_BREAKFAST_SPOON,
    KEY_POINT_EXTRA_SURFACE,
    KEY_POSE_CABINET,
    KEY_POSE_EXTRA_SURFACE,
    KEY_POSE_KITCHEN_SHELF,
    KEY_POSE_TABLE,
    KEY_SCAN_RESULTS_TABLE,
    KEY_SCORE_TRACE,
    KEY_SUMMARY_MESSAGE,
    KEY_VISION_RESULT,
    KEY_WORK_QUEUE,
    MAX_RUNTIME_SEC,
    PLACEMENT_MODE_FIXED_POINT,
    POINT_EXTRA_SURFACE,
    POSE_EXTRA_SURFACE,
    TABLE_BUDGET_SEC,
    BREAKFAST_BUDGET_SEC,
    EXTRA_BUDGET_SEC,
    TABLE_SCAN_PROMPT,
)
from .custom_nodes import (
    BtNode_BuildInventory,
    BtNode_DeadlineGuard,
    BtNode_GuardActiveClass,
    BtNode_MarkPhase,
    BtNode_PopWorkItem,
    BtNode_WriteFoundItems,
    record_event,
)
from .pick_and_place import (
    createConstantWriter as _writeBaseConstants,
    _gotoRetryWith_Announcement,
    _moveArmRetry,
    _scanForGeneralistRetry,
)

# 'pp_active_placement_mode' is BtNode_ScanAndPlace's default bb_placement_mode
# key. PopWorkItem writes it per-policy for cleanup; breakfast writes FIXED_POINT.
_KEY_ACTIVE_PLACEMENT_MODE = "pp_active_placement_mode"

# 'pp_active_skip_scan' is BtNode_ScanAndPlace's default bb_skip_scan_move key.
# PopWorkItem writes it True for cleanup; breakfast writes True too (the BT
# positions the arm itself, so the server must not re-move it).
_KEY_ACTIVE_SKIP_SCAN = "pp_active_skip_scan"

# Dedicated key for the per-item "<label> going to <class>" announce string
# composed by _ComposeItemAnnounceLeaf and spoken by the following Announce.
_KEY_ITEM_ANNOUNCE_MSG = "pp_item_announce_msg"

# Frozen breakfast table: (item, source_pose_key, arm_pose_key, point_key).
BREAKFAST = [
    ("bowl", KEY_POSE_KITCHEN_SHELF, KEY_ARM_TABLE, KEY_POINT_BREAKFAST_BOWL),
    ("spoon", KEY_POSE_KITCHEN_SHELF, KEY_ARM_TABLE, KEY_POINT_BREAKFAST_SPOON),
    ("cereal", KEY_POSE_CABINET, KEY_ARM_CABINET, KEY_POINT_BREAKFAST_CEREAL),
    ("milk", KEY_POSE_CABINET, KEY_ARM_CABINET, KEY_POINT_BREAKFAST_MILK),
]


# --------------------------------------------------------------------------- #
# Plain-Behaviour leaves (run REAL logic in mock; NEVER listed in mock_config) #
# --------------------------------------------------------------------------- #


class _RecordEventLeaf(py_trees.behaviour.Behaviour):
    """Append one score-trace event for the active item via record_event().

    Plain Behaviour: under BT_MOCK_MODE it runs its real logic (it is not a
    Handler and must NOT be registered in mock_config). Always SUCCESS.
    """

    def __init__(self, name, *, action, outcome, phase, points_est=0):
        super().__init__(name=name)
        self._action = action
        self._outcome = outcome
        self._phase = phase
        self._points_est = points_est
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(
            key="label",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", KEY_OBJECT_LABEL),
        )
        # record_event reads+appends the score-trace through this client.
        self.bb.register_key(
            key="score",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", KEY_SCORE_TRACE),
        )
        self.bb.register_key(
            key="score_r",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", KEY_SCORE_TRACE),
        )

    def update(self):
        try:
            item = str(self.bb.label)
        except Exception:  # pragma: no cover - label unset in standalone runs
            item = ""
        record_event(self.bb, self._phase, item, self._action, self._outcome, self._points_est)
        return py_trees.common.Status.SUCCESS


class _SummarizeScoreLeaf(py_trees.behaviour.Behaviour):
    """Build a spoken rollup from the score-trace; write it to KEY_SUMMARY_MESSAGE."""

    def __init__(self, name):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(
            key="score",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", KEY_SCORE_TRACE),
        )
        self.bb.register_key(
            key="summary",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", KEY_SUMMARY_MESSAGE),
        )

    def update(self):
        try:
            trace = self.bb.score or {}
        except Exception:
            trace = {}
        events = trace.get("events", [])
        placed = sum(1 for e in events if e.get("action") == "place")
        points = sum(int(e.get("points_est", 0) or 0) for e in events)
        phases = ", ".join(trace.get("visited_phases", [])) or "none"
        self.bb.summary = (
            f"Mission summary. Visited phases: {phases}. "
            f"Placed {placed} item{'s' if placed != 1 else ''}. "
            f"Estimated {points} points."
        )
        return py_trees.common.Status.SUCCESS


class _ComposeItemAnnounceLeaf(py_trees.behaviour.Behaviour):
    """Compose the per-item perception+destination announce string.

    Spec §8.3/§3 (communicate-perception): the per-item announce must speak the
    object label AND its destination class. Reads KEY_OBJECT_LABEL +
    KEY_ACTIVE_OBJECT_CLASS (both stamped by BtNode_PopWorkItem) and writes
    "<label> going to <class>" to _KEY_ITEM_ANNOUNCE_MSG for the following
    BtNode_Announce. Plain Behaviour: runs its real logic under BT_MOCK_MODE
    (it is not a Handler and must NOT be registered in mock_config). Always
    SUCCESS.
    """

    def __init__(self, name):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(
            key="label",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", KEY_OBJECT_LABEL),
        )
        self.bb.register_key(
            key="klass",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", KEY_ACTIVE_OBJECT_CLASS),
        )
        self.bb.register_key(
            key="msg",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", _KEY_ITEM_ANNOUNCE_MSG),
        )

    def update(self):
        try:
            label = str(self.bb.label)
        except Exception:  # pragma: no cover - label unset in standalone runs
            label = ""
        try:
            klass = str(self.bb.klass)
        except Exception:  # pragma: no cover - class unset in standalone runs
            klass = ""
        self.bb.msg = f"{label} going to {klass}"
        return py_trees.common.Status.SUCCESS


# --------------------------------------------------------------------------- #
# Small wiring helpers                                                         #
# --------------------------------------------------------------------------- #


def _goto(label, pose_key):
    return _gotoRetryWith_Announcement(label, pose_key)


def _arm(label, arm_key):
    return _moveArmRetry(label, arm_key, add_octomap=True)


# Head-tilt presets applied right before each grasp/place (operator spec): the
# pan-tilt head looks 0 deg at flat tables/surfaces and 25 deg at a
# cabinet/shelf. Pan (x) stays 0 = facing forward. Trash uses a gripper release
# (no surface scan) so it gets no tilt set.
HEAD_TILT_SURFACE = 0.0
HEAD_TILT_CABINET_SHELF = 25.0


def _headTilt(label, tilt_deg):
    return BtNode_TurnPanTilt(
        name=f"head tilt {int(tilt_deg)}deg ({label})", x=0.0, y=float(tilt_deg)
    )


def _reDetectActive():
    # NOTE(hardware): narrow the prompt to KEY_ACTIVE_PROMPT once the generalist
    # node exposes a bb-sourced prompt; the literal is mock-inert.
    # Generic re-detect is acceptable because Grasp targets the popped item via
    # object_label=KEY_OBJECT_LABEL; narrowing the scan prompt to the per-item
    # label is an on-robot refinement.
    return _scanForGeneralistRetry(
        name="re-detect active item",
        bb_source=None,
        bb_key=KEY_VISION_RESULT,
        object=TABLE_SCAN_PROMPT,
        use_orbbec=False,
    )


# --------------------------------------------------------------------------- #
# Constant writer + summary                                                    #
# --------------------------------------------------------------------------- #


def createConstantWriter(place_policy="vlm"):
    """Reuse the base PP constant writer, add extra-surface + score-trace init."""
    seq = py_trees.composites.Sequence(
        "write constants + init score-trace", memory=True
    )
    seq.add_child(_writeBaseConstants())
    seq.add_child(
        BtNode_WriteToBlackboard(
            name="write extra-surface pose",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_POSE_EXTRA_SURFACE,
            object=POSE_EXTRA_SURFACE,
        )
    )
    seq.add_child(
        BtNode_WriteToBlackboard(
            name="write extra-surface point",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_POINT_EXTRA_SURFACE,
            object=POINT_EXTRA_SURFACE,
        )
    )
    seq.add_child(
        BtNode_WriteToBlackboard(
            name="init score-trace",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_SCORE_TRACE,
            object={"visited_phases": [], "events": [], "place_policy": place_policy},
        )
    )
    return seq


def phaseSummary():
    seq = py_trees.composites.Sequence("phase: summary", memory=True)
    seq.add_child(_SummarizeScoreLeaf(name="summarize score-trace"))
    seq.add_child(
        BtNode_Announce(name="announce summary", bb_source=KEY_SUMMARY_MESSAGE)
    )
    return py_trees.decorators.FailureIsSuccess(
        name="summary (always success)", child=seq
    )


# --------------------------------------------------------------------------- #
# Enter arena                                                                  #
# --------------------------------------------------------------------------- #


def phaseEnterArena():
    # Door detection is DEPRECATED (no door_detection_srv server; under a real
    # run its setup() blocks tree.setup() on wait_for_service -> timeout). Skip
    # the wait-for-open-door step entirely and just announce entry; the first
    # phase's nav (_goto "dining table") drives the robot into the arena. To
    # restore a real door-wait, re-add a gated detection node here.
    seq = py_trees.composites.Sequence("phase: enter arena", memory=True)
    seq.add_child(
        BtNode_Announce(
            name="announce enter arena",
            bb_source=None,
            message="Entering the arena.",
        )
    )
    return py_trees.decorators.FailureIsSuccess(
        name="enter arena (always success)", child=seq
    )


# --------------------------------------------------------------------------- #
# Per-item handling (cleanup + extra-surface)                                  #
# --------------------------------------------------------------------------- #


def trashRelease(phase="cleanup"):
    """Kinematic controlled release over the bin (low) + best-effort gripper open.

    NOTE: if a dedicated `start_drop` server lands, swap the gripper open for
    BtNode_Drop. Records a 'place' event so trashed items count in the trace.
    """
    seq = py_trees.composites.Sequence("trash release", memory=True)
    seq.add_child(
        BtNode_Announce(
            name="announce trash release",
            bb_source=None,
            message="Releasing item into the trash bin.",
        )
    )
    seq.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="open gripper over bin (best effort)",
            child=BtNode_GripperAction(name="open gripper over bin", open_gripper=True),
        )
    )
    seq.add_child(
        _RecordEventLeaf(
            name="record trash place", action="place", outcome="released",
            phase=phase, points_est=40
        )
    )
    return seq


def maybeHelpOrSkip(allow_human_assistance=False, phase="cleanup"):
    """Terminal always-SUCCESS fallback: skip the item (partial credit) by default,
    or request human assistance if `allow_human_assistance`. Records a 'skip' event."""
    if allow_human_assistance:
        msg = "Please hand me or reposition the item."
        outcome = "assist_requested"
    else:
        msg = "Skipping this item."
        outcome = "skipped"
    seq = py_trees.composites.Sequence("maybe help or skip", memory=True)
    seq.add_child(
        _RecordEventLeaf(
            name="record skip/help", action="skip", outcome=outcome,
            phase=phase, points_est=0
        )
    )
    seq.add_child(
        BtNode_Announce(name="announce skip/help", bb_source=None, message=msg)
    )
    return py_trees.decorators.FailureIsSuccess(
        name="skip/help is always success", child=seq
    )


def _routeByDestination(place_policy, phase="cleanup"):
    """Selector of guard-routed destination branches. PopWorkItem has already set
    KEY_ACTIVE_OBJECT_CLASS + the per-policy placement_mode/fixed_target, so the
    place leaves are policy-agnostic (vlm -> FREE_SPACE/NEAR_SIMILAR, hardcoded ->
    FIXED_POINT)."""
    sel = py_trees.composites.Selector("route by destination", memory=True)

    wash = py_trees.composites.Sequence("route wash-staging", memory=True)
    wash.add_child(
        BtNode_GuardActiveClass(name="is wash_staging?", expected="wash_staging")
    )
    wash.add_child(_goto("wash staging", KEY_ACTIVE_TARGET_POSE))
    wash.add_child(_headTilt("wash-staging place", HEAD_TILT_SURFACE))
    wash.add_child(_arm("arm to wash place", KEY_ARM_WASH))
    wash.add_child(BtNode_ScanAndPlace(name="place at wash-staging"))
    wash.add_child(
        _RecordEventLeaf(
            name="record wash place", action="place", outcome="placed",
            phase=phase, points_est=40
        )
    )
    sel.add_child(wash)

    cab = py_trees.composites.Sequence("route cabinet", memory=True)
    cab.add_child(BtNode_GuardActiveClass(name="is cabinet?", expected="cabinet"))
    cab.add_child(_goto("cabinet", KEY_ACTIVE_TARGET_POSE))
    cab.add_child(_headTilt("cabinet place", HEAD_TILT_CABINET_SHELF))
    cab.add_child(_arm("arm to cabinet place", KEY_ARM_CABINET))
    cab.add_child(BtNode_ScanAndPlace(name="place at cabinet (grouped)"))
    cab.add_child(
        _RecordEventLeaf(
            name="record cabinet place", action="place", outcome="placed",
            phase=phase, points_est=40
        )
    )
    sel.add_child(cab)

    tr = py_trees.composites.Sequence("route trash", memory=True)
    tr.add_child(BtNode_GuardActiveClass(name="is trash?", expected="trash"))
    tr.add_child(_goto("trash bin", KEY_ACTIVE_TARGET_POSE))
    tr.add_child(_arm("arm to trash", KEY_ARM_TRASH))
    tr.add_child(trashRelease(phase))
    sel.add_child(tr)

    return sel


def handleOneItem(place_policy="vlm", phase="cleanup"):
    """Source-nav + arm + re-detect + announce + grasp + route, with a terminal
    skip fallback.

    The grasp + route sit under an outer Selector whose terminal child is
    maybeHelpOrSkip(): a grasp FAILURE OR an unroutable class falls through to
    the skip leaf (records 'skip', returns SUCCESS). So the only way handleOneItem
    FAILS is a goto/arm/re-detect failure above the Selector, which the cleanup
    loop's FailureIsSuccess masks. `place_policy` is consumed by PopWorkItem, not
    here; it is inert in mock.
    """
    body = py_trees.composites.Sequence("handle one item", memory=True)
    body.add_child(_goto("active item source", KEY_ACTIVE_SOURCE_POSE))
    body.add_child(_headTilt("surface grasp", HEAD_TILT_SURFACE))
    body.add_child(_arm("arm to scan (active item)", KEY_ARM_TABLE))
    body.add_child(_reDetectActive())
    # Per-object perception+destination announce (spec §8.3): speak the popped
    # label AND its destination class as "<label> going to <class>". The compose
    # leaf stages the string into _KEY_ITEM_ANNOUNCE_MSG; the single Announce
    # speaks it.
    body.add_child(_ComposeItemAnnounceLeaf(name="compose active item announce"))
    body.add_child(
        BtNode_Announce(name="announce active item", bb_source=_KEY_ITEM_ANNOUNCE_MSG)
    )

    happy = py_trees.composites.Sequence("grasp then route", memory=True)
    happy.add_child(
        py_trees.decorators.Retry(
            name="Retry grasp active item",
            child=BtNode_Grasp(
                name="grasp active item",
                bb_key_vision_res=KEY_VISION_RESULT,
                bb_key_object_label=KEY_OBJECT_LABEL,
            ),
            num_failures=GRASP_RETRY_LIMIT,
        )
    )
    happy.add_child(_routeByDestination(place_policy, phase))

    grasp_or_skip = py_trees.composites.Selector("grasp+route or skip", memory=True)
    grasp_or_skip.add_child(happy)
    grasp_or_skip.add_child(maybeHelpOrSkip(phase=phase))  # terminal, always SUCCESS
    body.add_child(grasp_or_skip)
    return body


def _cleanupLoop(place_policy="vlm", phase="cleanup"):
    body = py_trees.composites.Sequence("cleanup loop body", memory=True)
    # PopWorkItem is UNWRAPPED — the one node allowed to FAIL the body. It FAILS
    # exactly on an empty queue, which is the loop's only exit.
    body.add_child(BtNode_PopWorkItem(name="pop work item", place_policy=place_policy))
    body.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="item failure -> continue loop", child=handleOneItem(place_policy, phase)
        )
    )
    # LOAD-BEARING: loop exit == child FAILURE (empty queue). num_success=-1 never
    # reaches the success count, so Repeat re-ticks until the body FAILS. The
    # FailureIsSuccess(handleOneItem) above is REQUIRED — without it, a per-item
    # failure would FAIL the body and break the loop early (or, worse, if the body
    # could not fail, the loop would be infinite). Do not remove it.
    return py_trees.decorators.Repeat(
        name="drain work queue (num_success=-1)", child=body, num_success=-1
    )


# --------------------------------------------------------------------------- #
# Phases                                                                       #
# --------------------------------------------------------------------------- #


def phaseTableCleanup(place_policy="vlm"):
    seq = py_trees.composites.Sequence("phase: table cleanup", memory=True)
    seq.add_child(BtNode_MarkPhase(name="mark table", phase="table"))
    seq.add_child(_goto("dining table", KEY_POSE_TABLE))
    seq.add_child(_arm("arm to table scan", KEY_ARM_TABLE))
    seq.add_child(
        _scanForGeneralistRetry(
            name="scan table for cleanup",
            bb_source=None,
            bb_key=KEY_SCAN_RESULTS_TABLE,
            object=TABLE_SCAN_PROMPT,
            use_orbbec=True,
        )
    )
    # Perception summary (spec §8.3): announce what the scan found before building
    # the inventory. WriteFoundItems is a plain Behaviour (runs real logic in mock;
    # empty objects -> "could not find any objects", still SUCCESS).
    seq.add_child(
        BtNode_WriteFoundItems(
            name="write found table items",
            bb_key_vision_res=KEY_SCAN_RESULTS_TABLE,
            bb_key_announcement=KEY_ANNOUNCEMENT_MSG,
        )
    )
    seq.add_child(
        BtNode_Announce(name="announce found table items", bb_source=KEY_ANNOUNCEMENT_MSG)
    )
    seq.add_child(
        BtNode_BuildInventory(
            name="build table inventory",
            in_key=KEY_SCAN_RESULTS_TABLE,
            out_inventory=KEY_INVENTORY_TABLE,
            out_queue=KEY_WORK_QUEUE,
            source_pose_key=KEY_POSE_TABLE,
            mock_seed=None,  # BuildInventory seeds a canned queue under mock if empty
        )
    )
    seq.add_child(_cleanupLoop(place_policy, "table"))
    return seq


def _breakfastItem(item, src_key, arm_key, point_key):
    s = py_trees.composites.Sequence(f"retrieve+place {item}", memory=True)
    s.add_child(_goto(f"{item} source", src_key))
    s.add_child(_headTilt(f"{item} retrieve (shelf/cabinet)", HEAD_TILT_CABINET_SHELF))
    s.add_child(_arm(f"arm to {item} retrieve", arm_key))
    s.add_child(
        _scanForGeneralistRetry(
            name=f"re-detect {item}",
            bb_source=None,
            bb_key=KEY_VISION_RESULT,
            object=item,
            use_orbbec=False,
        )
    )
    s.add_child(
        BtNode_Announce(
            name=f"announce retrieving {item}",
            bb_source=None,
            message=f"Retrieving {item}.",
        )
    )
    s.add_child(
        py_trees.decorators.Retry(
            name=f"Retry grasp {item}",
            child=BtNode_Grasp(
                name=f"grasp {item}", bb_key_vision_res=KEY_VISION_RESULT
            ),
            num_failures=GRASP_RETRY_LIMIT,
        )
    )
    s.add_child(_goto("clean table (breakfast)", KEY_POSE_TABLE))
    s.add_child(_headTilt("breakfast table place", HEAD_TILT_SURFACE))
    s.add_child(_arm("arm to table place (breakfast)", KEY_ARM_TABLE))
    # Breakfast is ALWAYS FIXED_POINT at the item's own point (policy-independent).
    s.add_child(
        BtNode_WriteToBlackboard(
            name=f"set FIXED_POINT mode ({item})",
            bb_namespace="",
            bb_source=None,
            bb_key=_KEY_ACTIVE_PLACEMENT_MODE,
            object=PLACEMENT_MODE_FIXED_POINT,
        )
    )
    # The BT already positioned the arm (_arm above), so tell the server to skip
    # its own scan-pose move (object is in the gripper). Same key the cleanup
    # path uses (PopWorkItem 'skip_scan' -> BtNode_ScanAndPlace bb_skip_scan_move).
    s.add_child(
        BtNode_WriteToBlackboard(
            name=f"skip server scan move ({item})",
            bb_namespace="",
            bb_source=None,
            bb_key=_KEY_ACTIVE_SKIP_SCAN,
            object=True,
        )
    )
    s.add_child(
        BtNode_ScanAndPlace(name=f"place {item} (FIXED_POINT)", bb_fixed_target=point_key)
    )
    s.add_child(
        _RecordEventLeaf(
            name=f"record place {item}", action="place", outcome="placed",
            phase="breakfast", points_est=40
        )
    )
    return s


def phaseServeBreakfast(place_policy="vlm"):
    seq = py_trees.composites.Sequence("phase: serve breakfast", memory=True)
    seq.add_child(BtNode_MarkPhase(name="mark breakfast", phase="breakfast"))
    for item, src_key, arm_key, point_key in BREAKFAST:
        # Per-item FailureIsSuccess: one failed retrieval doesn't abort breakfast.
        seq.add_child(
            py_trees.decorators.FailureIsSuccess(
                name=f"breakfast {item} (best effort)",
                child=_breakfastItem(item, src_key, arm_key, point_key),
            )
        )
    seq.add_child(
        BtNode_Announce(
            name="announce breakfast served",
            bb_source=None,
            message="Breakfast is served.",
        )
    )
    return seq


def phaseExtraSurfaceCleanup(place_policy="vlm"):
    seq = py_trees.composites.Sequence("phase: extra-surface cleanup", memory=True)
    seq.add_child(BtNode_MarkPhase(name="mark extra", phase="extra"))
    seq.add_child(_goto("extra surface", KEY_POSE_EXTRA_SURFACE))
    seq.add_child(_arm("arm to extra-surface scan", KEY_ARM_TABLE))
    seq.add_child(
        _scanForGeneralistRetry(
            name="scan extra surface",
            bb_source=None,
            bb_key=KEY_SCAN_RESULTS_TABLE,
            object=TABLE_SCAN_PROMPT,
            use_orbbec=True,
        )
    )
    # Perception summary (spec §8.3): announce found items before BuildInventory.
    seq.add_child(
        BtNode_WriteFoundItems(
            name="write found extra items",
            bb_key_vision_res=KEY_SCAN_RESULTS_TABLE,
            bb_key_announcement=KEY_ANNOUNCEMENT_MSG,
        )
    )
    seq.add_child(
        BtNode_Announce(name="announce found extra items", bb_source=KEY_ANNOUNCEMENT_MSG)
    )
    seq.add_child(
        BtNode_BuildInventory(
            name="build extra inventory",
            in_key=KEY_SCAN_RESULTS_TABLE,
            out_inventory=KEY_INVENTORY_TABLE,
            out_queue=KEY_WORK_QUEUE,
            source_pose_key=KEY_POSE_EXTRA_SURFACE,
            mock_seed=None,
        )
    )
    seq.add_child(_cleanupLoop(place_policy, "extra"))
    return seq


# --------------------------------------------------------------------------- #
# Mission assembly                                                             #
# --------------------------------------------------------------------------- #


def budgeted(body, budget_sec, label="phase"):
    par = py_trees.composites.Parallel(
        name=f"budgeted {label}",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
    )
    par.add_child(BtNode_DeadlineGuard(name=f"{label} deadline", budget_sec=budget_sec))
    par.add_child(
        py_trees.decorators.FailureIsSuccess(
            name=f"{label} body (never fails the budget)", child=body
        )
    )
    return par


def missionPhases(place_policy="vlm"):
    seq = py_trees.composites.Sequence("mission phases", memory=True)
    seq.add_child(budgeted(phaseTableCleanup(place_policy), TABLE_BUDGET_SEC, "table cleanup"))
    seq.add_child(
        budgeted(phaseServeBreakfast(place_policy), BREAKFAST_BUDGET_SEC, "serve breakfast")
    )
    seq.add_child(
        budgeted(phaseExtraSurfaceCleanup(place_policy), EXTRA_BUDGET_SEC, "extra cleanup")
    )
    # EXTENSION HOOK: a future optional-goals phase (dishwasher / pour / tablet)
    # would attach here, after extra-surface cleanup. No code (out of scope).
    return seq


def pickAndPlaceRulebook(place_policy="vlm"):
    root = py_trees.composites.Sequence("Pick and Place (rulebook)", memory=True)
    root.add_child(createConstantWriter(place_policy))
    root.add_child(phaseEnterArena())

    mission_par = py_trees.composites.Parallel(
        name="mission under global deadline",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
    )
    # DeadlineGuard returns RUNNING->SUCCESS (never FAILURE); the mission is
    # FailureIsSuccess-wrapped, so the Parallel NEVER sees FAILURE from either
    # child. Normal finish: mission SUCCEEDS -> SuccessOnOne -> SUCCESS. Timeout:
    # guard SUCCEEDS -> SUCCESS (running place leaf cancelled by terminate). Either
    # way phaseSummary (OUTSIDE the guard) still runs.
    mission_par.add_child(
        BtNode_DeadlineGuard(name="global deadline", budget_sec=MAX_RUNTIME_SEC)
    )
    mission_par.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="mission never fails the parallel", child=missionPhases(place_policy)
        )
    )
    root.add_child(mission_par)
    root.add_child(phaseSummary())
    return root
