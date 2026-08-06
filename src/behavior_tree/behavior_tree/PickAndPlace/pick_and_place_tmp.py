from __future__ import annotations

"""Pick and Place — manipulation-free scan probe (entry: ``pick-and-place-tmp``).

A stripped-down sibling of ``pick_and_place_v2.createPickAndPlaceTask2026`` that
exercises ONLY the navigation, vision and audio subsystems. Every manipulation
primitive is removed — there is no grasp, no place, and (crucially) none of the
``BtNode_MoveArmSingle`` "arm to navigating pose" moves that the canonical
``navigateToTable`` / ``scanTableAndAnnounce`` helpers embed. That makes this
tree safe to run with the arm powered down / absent.

Process (per request):

  1. **Enter the arena** — ``enterArena()`` door detection (vision), reused from
     the canonical tree.
  2. **Detour + clear chair** — drive to the ``pose_detour`` staging waypoint
     (a PLACEHOLDER identity pose in ``constants.json`` — fill at Setup Days),
     announce "Please remove the chair right in front of me.", then wait 3 s.
  3. **Navigate to the table pose** — announce + pan-tilt + ``BtNode_GotoAction``
     against ``KEY_POSE_TABLE`` (no pre-nav arm move).
  4. **Scan the table and announce** — pan-tilt to the table, run the tk26
     generalist detector over the **Orbbec** camera, format the found items,
     and announce them.
  5. **Navigate to the shelf pose** — same no-arm goto, against
     ``KEY_POSE_KITCHEN_SHELF``.
  6. **Scan the shelf and announce** — same Orbbec scan-and-announce, pointed at
     the shelf.

Because the shared helpers in ``pick_and_place.py`` bake the arm moves in, this
module re-implements the phases locally instead of importing them. The constant
writer is slimmed to just the two poses navigation needs.

Run::

    ros2 run behavior_tree pick-and-place-tmp

Fully offline (auto-advance, no servers)::

    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json \
        ros2 run behavior_tree pick-and-place-tmp
"""

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import (
    BtNode_ScanForGeneralist,
    BtNode_TurnPanTilt,
)
from behavior_tree.visualization import create_post_tick_visualizer

# enterArena() is pure door-detection (vision) — no manipulation — so reuse it.
from .pick_and_place import enterArena
from .custom_nodes import BtNode_WriteFoundItems
from .config import (
    KEY_ANNOUNCEMENT_MSG,
    KEY_POSE_DETOUR,
    KEY_POSE_KITCHEN_SHELF,
    KEY_POSE_TABLE,
    KEY_SCAN_RESULTS_SHELF,
    KEY_SCAN_RESULTS_TABLE,
    NAV_RETRY_LIMIT,
    POSE_DETOUR,
    POSE_KITCHEN_SHELF,
    POSE_TABLE,
)

# Open-vocab detection prompts per surface (tk26 generalist detector format:
# class names separated by " . ").
TABLE_PROMPT = "bottle . cup . chip can . hand sanitizer . plate"
SHELF_PROMPT = "bottle . milk . cereal . food package . snack box . can . cup . bowl"


# --------------------------------------------------------------------------- #
# Constant writer (slim — navigation only needs the surface poses)
# --------------------------------------------------------------------------- #
def createConstantWriter() -> py_trees.composites.Parallel:
    """Seed just the poses ``BtNode_GotoAction`` reads (table + shelf)."""
    root = py_trees.composites.Parallel(
        name="Write nav poses",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    for name, key, value in (
        ("Write detour pose", KEY_POSE_DETOUR, POSE_DETOUR),
        ("Write table pose", KEY_POSE_TABLE, POSE_TABLE),
        ("Write kitchen shelf pose", KEY_POSE_KITCHEN_SHELF, POSE_KITCHEN_SHELF),
    ):
        root.add_child(
            BtNode_WriteToBlackboard(
                name=name,
                bb_namespace="",
                bb_source=None,
                bb_key=key,
                object=value,
            )
        )
    return root


# --------------------------------------------------------------------------- #
# Shared helpers (no manipulation)
# --------------------------------------------------------------------------- #
def _gotoNoArm(location_name: str, pose_key: str) -> py_trees.composites.Parallel:
    """Announce + face forward + drive to ``pose_key``, WITHOUT any arm move.

    Mirrors ``pick_and_place._gotoRetryWith_Announcement`` but drops the
    ``BtNode_MoveArmSingle`` "arm to nav" step so no manipulation is touched.
    """
    root = py_trees.composites.Parallel(
        name=f"parallel announce and goto {location_name}",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    root.add_child(
        BtNode_Announce(
            name=f"Announce going to {location_name}",
            bb_source=None,
            message=f"Going to {location_name}.",
        )
    )
    root.add_child(
        BtNode_TurnPanTilt(name="turn pan tilt to face forward", x=0.0, y=45.0)
    )
    root.add_child(
        py_trees.decorators.Retry(
            name=f"Retry goto {location_name}",
            child=BtNode_GotoAction(name=f"Go to {location_name}", key=pose_key),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    return root


def _scanAndAnnounce(
    location_name: str,
    scan_key: str,
    prompt: str,
    *,
    pan_tilt_y: float,
) -> py_trees.composites.Sequence:
    """Pan-tilt to the surface, run the Orbbec generalist detector, announce items.

    Mirrors ``pick_and_place.scanTableAndAnnounce`` but drops the
    ``BtNode_MoveArmSingle`` "clear orbbec" arm move. ``use_orbbec=True`` forces
    the scan through the Orbbec camera.
    """
    root = py_trees.composites.Sequence(f"Scan {location_name} and announce", memory=True)
    root.add_child(
        BtNode_TurnPanTilt(name=f"Turn head to {location_name}", x=0.0, y=pan_tilt_y)
    )

    parallel_scan_and_announce = py_trees.composites.Parallel(
        f"parallel scan {location_name} and announce",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    parallel_scan_and_announce.add_child(
        BtNode_Announce(
            f"announce scanning {location_name}",
            bb_source=None,
            message=f"Scanning {location_name} for items.",
        )
    )
    parallel_scan_and_announce.add_child(
        py_trees.decorators.Retry(
            name=f"retry scan {location_name} for generalist",
            child=BtNode_ScanForGeneralist(
                name=f"scan {location_name}",
                bb_source=None,
                bb_key=scan_key,
                object=prompt,
                use_orbbec=True,
                return_rgb_image=True,
                return_depth_image=True,
                force_vlm_sam=True,
                return_segments=True,
            ),
            num_failures=5,
        )
    )
    root.add_child(parallel_scan_and_announce)

    root.add_child(
        BtNode_WriteFoundItems(
            name=f"write scanned {location_name} items to announcement message",
            bb_key_vision_res=scan_key,
            bb_key_announcement=KEY_ANNOUNCEMENT_MSG,
        )
    )
    root.add_child(
        BtNode_Announce(f"announce found {location_name} items", KEY_ANNOUNCEMENT_MSG)
    )
    return root


# --------------------------------------------------------------------------- #
# Phase factories
# --------------------------------------------------------------------------- #
def navigateToDetourAndClearChair() -> py_trees.composites.Sequence:
    """Drive to the detour/staging waypoint, ask for the chair to be cleared, wait 3 s.

    Runs BEFORE the table goto so the operator has a beat to remove a chair in
    front of the robot. ``py_trees.timers.Timer`` holds the sequence RUNNING for
    3 seconds, then SUCCESS lets the mission proceed to the table.
    """
    root = py_trees.composites.Sequence("Detour then clear chair", memory=True)
    root.add_child(_gotoNoArm("the detour point", KEY_POSE_DETOUR))
    root.add_child(
        BtNode_Announce(
            name="announce clear the chair",
            bb_source=None,
            message="Please remove the chair right in front of me.",
        )
    )
    root.add_child(
        py_trees.timers.Timer(name="wait 3 seconds for chair removal", duration=3.0)
    )
    return root


def navigateToTable() -> py_trees.composites.Parallel:
    return _gotoNoArm("table", KEY_POSE_TABLE)


def scanTableAndAnnounce() -> py_trees.composites.Sequence:
    return _scanAndAnnounce(
        "table", KEY_SCAN_RESULTS_TABLE, TABLE_PROMPT, pan_tilt_y=20.0
    )


def navigateToShelf() -> py_trees.composites.Parallel:
    return _gotoNoArm("shelf", KEY_POSE_KITCHEN_SHELF)


def scanShelfAndAnnounce() -> py_trees.composites.Sequence:
    return _scanAndAnnounce(
        "shelf", KEY_SCAN_RESULTS_SHELF, SHELF_PROMPT, pan_tilt_y=25.0
    )


# --------------------------------------------------------------------------- #
# Mission assembly
# --------------------------------------------------------------------------- #
def createPickAndPlaceScanProbe() -> py_trees.behaviour.Behaviour:
    """Nav/vision/audio only: enter arena → detour+clear-chair → table scan → shelf scan."""
    root = py_trees.composites.Sequence("Pick and place scan probe", memory=True)
    root.add_child(createConstantWriter())
    root.add_child(enterArena())
    root.add_child(navigateToDetourAndClearChair())
    root.add_child(navigateToTable())
    root.add_child(scanTableAndAnnounce())
    root.add_child(navigateToShelf())
    root.add_child(scanShelfAndAnnounce())
    return root


def create_tree() -> py_trees.behaviour.Behaviour:
    """Alias for the offline smoke harness."""
    return createPickAndPlaceScanProbe()


def main():
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=createPickAndPlaceScanProbe())
    tree.setup(timeout=15, node_name="pick_and_place_tmp")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="pick-and-place-tmp"
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
