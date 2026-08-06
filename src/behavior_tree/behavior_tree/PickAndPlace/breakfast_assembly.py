from __future__ import annotations

"""PickAndPlace — breakfast assembly phase (RoboCup@Home 2026 §5.2, 2nd main goal).

The rulebook's second main goal is to set breakfast on a clean table area:
**bowl + spoon + cereal + milk**, arranged typically and uncluttered. The
canonical mission (`pick_and_place.pickAndPlaceShortened`) does not implement
this at all — the four `KEY_POINT_BREAKFAST_*` constants are declared in
`config.py` but nothing reads them.

This NEW module adds the phase by composing the *real* manipulation primitives
that already exist and are individually tested:

  * pick  : `table_grasping.createTableGrasp(prompt)` — realsense scan -> real
            `BtNode_Grasp` -> stow. Reused unchanged.
  * place : the real vision-collision `BtNode_Place` (`place_action`), but at a
            **fixed** breakfast point (bowl here, spoon there, ...) instead of a
            VLM-found free spot — breakfast must be a typical, deliberate layout,
            so we skip `BtNode_FindPlacingLocation` and target the preset point.

Known approximation (surfaced, not hidden): `BtNode_Grasp` returns only
success/failure — it does NOT publish the grasped object's orientation, yet
`BtNode_Place` reads a `grasp_pose` orientation. So we seed a fixed top-down
place orientation into `KEY_GRASP_POSE` in the constant writer (same approach as
`test_grasp_to_place_e2e._seed_inputs`). When the grasp action later returns a
real orientation, point `grasp_pose_key` at that key instead.

Standalone smoke test (assemble all four breakfast items, fixed-point place)::

    ros2 run behavior_tree pp-test-breakfast-assembly

Fully offline (auto-advance, no servers)::

    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json \
        ros2 run behavior_tree pp-test-breakfast-assembly
"""

import py_trees
import py_trees_ros
import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import BtNode_GripperAction, BtNode_Place
from behavior_tree.TemplateNodes.Vision import BtNode_GetPointCloud
from behavior_tree.TemplateNodes.manipulation_new import BtNode_JointMoveAction
from behavior_tree.visualization import create_post_tick_visualizer

from .table_grasping import createTableGrasp
from .config import (
    ARM_ACTION_NAME,
    ARM_POS_NAVIGATING,
    ARM_POS_TABLE,
    KEY_ARM_NAVIGATING,
    KEY_ARM_TABLE,
    KEY_ENV_POINTS,
    KEY_GRASP_POSE,
    KEY_POINT_BREAKFAST_BOWL,
    KEY_POINT_BREAKFAST_CEREAL,
    KEY_POINT_BREAKFAST_MILK,
    KEY_POINT_BREAKFAST_SPOON,
    PLACE_ACTION_NAME,
    POINT_BREAKFAST_BOWL,
    POINT_BREAKFAST_CEREAL,
    POINT_BREAKFAST_MILK,
    POINT_BREAKFAST_SPOON,
)

# (label, detection prompt, fixed-point blackboard key, fixed-point value).
# Order is the typical breakfast layout: bowl & spoon first (the place setting),
# then cereal and milk beside it.
BREAKFAST_ITEMS = [
    ("bowl", "a cereal bowl . bowl", KEY_POINT_BREAKFAST_BOWL, POINT_BREAKFAST_BOWL),
    ("spoon", "a spoon . spoon", KEY_POINT_BREAKFAST_SPOON, POINT_BREAKFAST_SPOON),
    ("cereal", "a box of cereal . cereal box . bag of cereal", KEY_POINT_BREAKFAST_CEREAL, POINT_BREAKFAST_CEREAL),
    ("milk", "a milk carton . bottle of milk . milk", KEY_POINT_BREAKFAST_MILK, POINT_BREAKFAST_MILK),
]

# Fixed top-down-ish place orientation seeded into KEY_GRASP_POSE because the
# grasp action does not return one. Matches test_grasp_to_place_e2e's seed.
_DEFAULT_PLACE_ORIENTATION = Pose(
    position=Point(x=0.0, y=0.0, z=0.0),
    orientation=Quaternion(x=0.707106781, y=0.0, z=0.707106781, w=0.0),
)


def createBreakfastConstantWriter() -> py_trees.composites.Parallel:
    """Seed the breakfast points, arm poses, default place orientation, env cloud.

    Scoped to breakfast assembly; standalone-safe. The full mission writes the
    same arm/point keys upstream — re-writing them here is idempotent and the two
    entry points never run concurrently.
    """
    root = py_trees.composites.Parallel(
        name="Write breakfast constants",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    writes = [
        ("Write arm navigating", KEY_ARM_NAVIGATING, ARM_POS_NAVIGATING),
        ("Write arm table", KEY_ARM_TABLE, ARM_POS_TABLE),
        ("Write breakfast bowl pt", KEY_POINT_BREAKFAST_BOWL, POINT_BREAKFAST_BOWL),
        ("Write breakfast spoon pt", KEY_POINT_BREAKFAST_SPOON, POINT_BREAKFAST_SPOON),
        ("Write breakfast cereal pt", KEY_POINT_BREAKFAST_CEREAL, POINT_BREAKFAST_CEREAL),
        ("Write breakfast milk pt", KEY_POINT_BREAKFAST_MILK, POINT_BREAKFAST_MILK),
        # Default place orientation — grasp action does not publish one.
        ("Seed default place orientation", KEY_GRASP_POSE, _DEFAULT_PLACE_ORIENTATION),
        # Env cloud key must exist for BtNode_Place's READ; GetPointCloud
        # overwrites it live before each place.
        ("Init env points", KEY_ENV_POINTS, None),
    ]
    for name, key, value in writes:
        root.add_child(
            BtNode_WriteToBlackboard(
                name=name, bb_namespace="", bb_source=None, bb_key=key, object=value
            )
        )
    return root


def _moveArmRetry(name: str, arm_pose_key: str, *, retries: int = 3):
    return py_trees.decorators.Retry(
        name=f"Retry {name}",
        child=BtNode_JointMoveAction(
            name=name, action_name=ARM_ACTION_NAME, arm_pose_bb_key=arm_pose_key
        ),
        num_failures=retries,
    )


def createPlaceAtFixedPoint(
    item_label: str,
    point_key: str,
    *,
    place_retries: int = 3,
) -> py_trees.composites.Selector:
    """Place the held object at a PRESET point (no VLM free-spot search).

    Mirrors ``table_placing.createTablePlacing`` but drops ``BtNode_FindPlacingLocation``
    — breakfast layout is deliberate, so we place at the fixed ``point_key``
    rather than wherever the VLM finds room. Keeps the real ``BtNode_GetPointCloud``
    (collision env) + ``BtNode_Place`` (vision-guided place) and the same
    failure-cleanup Selector so downstream items start from a known arm state.
    """
    place_sequence = py_trees.composites.Sequence(
        name=f"Place {item_label} at breakfast point", memory=True
    )
    # Env cloud from the head camera BEFORE the arm moves into the place pose
    # (so the arm does not occlude the view).
    place_sequence.add_child(
        BtNode_GetPointCloud(
            name=f"get env cloud for {item_label} place",
            bb_point_cloud_key=KEY_ENV_POINTS,
            camera_name="orbbec",
        )
    )
    place_sequence.add_child(
        _moveArmRetry(name=f"move arm to table place pose ({item_label})", arm_pose_key=KEY_ARM_TABLE)
    )
    place_sequence.add_child(
        py_trees.decorators.Retry(
            name=f"Retry place {item_label}",
            child=BtNode_Place(
                name=f"place {item_label} at breakfast point",
                bb_key_point=point_key,
                bb_key_pose=KEY_GRASP_POSE,
                bb_key_env_points=KEY_ENV_POINTS,
                action_name=PLACE_ACTION_NAME,
            ),
            num_failures=place_retries,
        )
    )
    place_sequence.add_child(
        BtNode_GripperAction(name=f"open gripper after placing {item_label}", open_gripper=True)
    )
    place_sequence.add_child(
        _moveArmRetry(name=f"move arm to nav after {item_label}", arm_pose_key=KEY_ARM_NAVIGATING)
    )

    failure_cleanup = py_trees.composites.Sequence(
        name=f"Reset state on {item_label} place failure",
        memory=True,
        children=[
            BtNode_GripperAction(name=f"open gripper on {item_label} failure", open_gripper=True),
            _moveArmRetry(name=f"move arm to nav on {item_label} failure", arm_pose_key=KEY_ARM_NAVIGATING),
        ],
    )

    root = py_trees.composites.Selector(
        name=f"Place {item_label} with failure cleanup", memory=True
    )
    root.add_child(place_sequence)
    root.add_child(failure_cleanup)
    return root


def createAssembleOneBreakfastItem(
    item_label: str, prompt: str, point_key: str
) -> py_trees.decorators.FailureIsSuccess:
    """Pick one breakfast item and place it at its fixed point (best-effort).

    Best-effort (``FailureIsSuccess``) so a single un-graspable item does not
    abort the whole breakfast — the rulebook scores each placed item, and a
    partial setting still scores.
    """
    seq = py_trees.composites.Sequence(name=f"Assemble {item_label}", memory=True)
    seq.add_child(
        BtNode_Announce(
            name=f"announce fetching {item_label}",
            bb_source=None,
            message=f"Setting up the {item_label} for breakfast.",
        )
    )
    seq.add_child(createTableGrasp(prompt=prompt))
    seq.add_child(
        BtNode_Announce(
            name=f"announce placing {item_label}",
            bb_source=None,
            message=f"Placing the {item_label}.",
        )
    )
    seq.add_child(createPlaceAtFixedPoint(item_label, point_key))
    return py_trees.decorators.FailureIsSuccess(
        name=f"Assemble {item_label} (best effort)", child=seq
    )


def createBreakfastAssembly() -> py_trees.composites.Sequence:
    """Full breakfast assembly: bowl, spoon, cereal, milk at their fixed points."""
    root = py_trees.composites.Sequence(name="Breakfast assembly", memory=True)
    root.add_child(createBreakfastConstantWriter())
    root.add_child(
        BtNode_Announce(
            name="announce breakfast start",
            bb_source=None,
            message="Setting the table for breakfast.",
        )
    )
    for label, prompt, point_key, _value in BREAKFAST_ITEMS:
        root.add_child(createAssembleOneBreakfastItem(label, prompt, point_key))
    root.add_child(
        BtNode_Announce(
            name="announce breakfast done",
            bb_source=None,
            message="Breakfast is set. Bowl, spoon, cereal, and milk are arranged.",
        )
    )
    return root


def create_tree() -> py_trees.behaviour.Behaviour:
    """Offline smoke harness for the breakfast phase."""
    root = py_trees.composites.Sequence(name="Breakfast assembly test", memory=True)
    root.add_child(createBreakfastAssembly())
    root.add_child(py_trees.behaviours.Running(name="idle (Ctrl+C to exit)"))
    return root


def main():
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=create_tree())
    tree.setup(timeout=15, node_name="pp_breakfast_assembly")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="pp-test-breakfast-assembly"
    )
    tree.tick_tock(period_ms=400.0, post_tick_handler=print_tree)
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
