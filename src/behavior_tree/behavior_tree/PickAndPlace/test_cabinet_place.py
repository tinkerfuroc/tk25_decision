from __future__ import annotations

"""Cabinet categorize -> place smoke tree.

PickAndPlace defines ``BtNode_CategorizeGrocery`` in ``custom_nodes.py`` but —
unlike StoringGroceries — has no production call site that wires it to a place
yet (``cabinet_categorization.py`` is a survey-only step). This module is that
first real call site: it runs the survey factory
``createCabinetCategorization()`` and then a real categorize -> place chain
built from the PickAndPlace nodes (``BtNode_CategorizeGrocery`` -> arm move ->
``BtNode_Place``), mirroring StoringGroceries' ``createPlaceOnShelf`` but with
PickAndPlace's blackboard keys.

Inputs the chain reads are seeded up front (prompt, table image, segmentation,
target frame, shelf left/right anchor points, grasp orientation, dynamic
placing point + env cloud fallbacks).

Run command:
    ros2 run behavior_tree pp-test-cabinet-place

Offline run command:
    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json ros2 run behavior_tree pp-test-cabinet-place
"""

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import BtNode_MoveArmSingle, BtNode_Place
from behavior_tree.visualization import create_post_tick_visualizer

from geometry_msgs.msg import Point, PointStamped, Pose, Quaternion
from std_msgs.msg import Header

from .cabinet_categorization import createCabinetCategorization
from .config import (
    ARM_SERVICE_NAME,
    KEY_ARM_CABINET,
    KEY_ENV_POINTS,
    KEY_GRASP_POSE,
    KEY_OBJ_SEG,
    KEY_PLACE_REASON,
    KEY_POINT_PLACING_DYNAMIC,
    KEY_POINT_SHELF_LEFT,
    KEY_POINT_SHELF_RIGHT,
    KEY_TABLE_IMG,
    KEY_TARGET_FRAME,
    N_LAYERS,
    PLACE_ACTION_NAME,
    POINT_SHELF_LEFT,
    POINT_SHELF_RIGHT,
    TABLE_SCAN_PROMPT,
    TARGET_FRAME,
    ARM_POS_CABINET,
)
from .custom_nodes import BtNode_CategorizeGrocery

# Local blackboard keys for inputs CategorizeGrocery reads that config doesn't
# expose under dedicated KEY_* names (prompt). Keep them module-local.
KEY_CABINET_PROMPT = "pp_test_cabinet_prompt"


def _seed_inputs() -> py_trees.composites.Parallel:
    """Seed every blackboard READ input the categorize -> place chain needs."""
    root = py_trees.composites.Parallel(
        name="Seed cabinet place inputs",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    grasp_pose = Pose(
        position=Point(x=0.5, y=0.0, z=0.4),
        orientation=Quaternion(x=0.707106781, y=0.0, z=0.707106781, w=0.0),
    )
    placing_point = PointStamped(
        header=Header(frame_id=TARGET_FRAME),
        point=Point(x=0.5, y=0.0, z=0.4),
    )
    seeds = [
        ("Seed prompt", KEY_CABINET_PROMPT, TABLE_SCAN_PROMPT),
        ("Seed table image", KEY_TABLE_IMG, None),
        ("Seed object segmentation", KEY_OBJ_SEG, None),
        ("Seed target frame", KEY_TARGET_FRAME, TARGET_FRAME),
        ("Seed shelf left", KEY_POINT_SHELF_LEFT, POINT_SHELF_LEFT),
        ("Seed shelf right", KEY_POINT_SHELF_RIGHT, POINT_SHELF_RIGHT),
        ("Seed grasp pose", KEY_GRASP_POSE, grasp_pose),
        ("Seed placing point", KEY_POINT_PLACING_DYNAMIC, placing_point),
        ("Seed env points", KEY_ENV_POINTS, None),
        ("Seed arm cabinet pose", KEY_ARM_CABINET, ARM_POS_CABINET),
    ]
    for name, key, value in seeds:
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


def _create_categorize_and_place() -> py_trees.composites.Sequence:
    """Real categorize -> arm move -> place chain (PickAndPlace keys)."""
    root = py_trees.composites.Sequence(name="Categorize and place on cabinet", memory=True)
    root.add_child(
        BtNode_CategorizeGrocery(
            name="categorize grocery for cabinet shelf",
            n_layers=N_LAYERS,
            bb_key_prompt=KEY_CABINET_PROMPT,
            bb_key_image=KEY_TABLE_IMG,
            bb_key_segment=KEY_OBJ_SEG,
            bb_target_frame=KEY_TARGET_FRAME,
            bb_key_result_point=KEY_POINT_PLACING_DYNAMIC,
            bb_key_env_points=KEY_ENV_POINTS,
            bb_key_reason=KEY_PLACE_REASON,
            bb_key_shelf_left=KEY_POINT_SHELF_LEFT,
            bb_key_shelf_right=KEY_POINT_SHELF_RIGHT,
        )
    )
    root.add_child(
        BtNode_MoveArmSingle(
            name="move arm to cabinet pose",
            service_name=ARM_SERVICE_NAME,
            arm_pose_bb_key=KEY_ARM_CABINET,
            add_octomap=True,
        )
    )
    root.add_child(
        BtNode_Announce(name="announce placing on cabinet", bb_source=None, message="Placing on cabinet shelf")
    )
    root.add_child(
        BtNode_Place(
            name="place object on cabinet shelf",
            bb_key_point=KEY_POINT_PLACING_DYNAMIC,
            bb_key_pose=KEY_GRASP_POSE,
            bb_key_env_points=KEY_ENV_POINTS,
            action_name=PLACE_ACTION_NAME,
        )
    )
    return root


def create_tree() -> py_trees.behaviour.Behaviour:
    """Survey (categorization) -> real categorize -> place chain."""
    root = py_trees.composites.Sequence(name="PickAndPlace cabinet place smoke", memory=True)
    root.add_child(_seed_inputs())
    root.add_child(createCabinetCategorization())
    root.add_child(_create_categorize_and_place())
    # Long-running manipulation/vision actions — keep the tree alive.
    root.add_child(py_trees.behaviours.Running(name="idle"))
    return root


def main():
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=create_tree())
    tree.setup(timeout=15, node_name="pp_test_cabinet_place")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="PickAndPlace Cabinet Place"
    )
    tree.tick_tock(period_ms=500.0, post_tick_handler=print_tree)
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        shutdown_visualizer()
        tree.shutdown()
        rclpy.try_shutdown()
