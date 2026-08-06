from __future__ import annotations

"""End-to-end PickAndPlace grasp -> place smoke tree.

Composes the *real* vision-guided pick->place chain by reusing the production
subtree factories:

    table_grasping.createTableGrasp()    # scan (realsense) -> grasp -> stow arm
    table_placing.createTablePlacing()   # find placing pt + env cloud -> place

Both factories prepend their own ``createConstantWriter()`` so the arm-pose
constants are on the blackboard. We additionally seed the inter-phase inputs
that ``BtNode_Place`` reads but that no upstream mock writes:
  - KEY_GRASP_POSE          (orientation captured at grasp; read by Place)
  - KEY_POINT_PLACING_DYNAMIC (placing PointStamped; written by FindPlacingLocation,
                               seeded here as a fallback for offline runs)
  - KEY_ENV_POINTS          (env point cloud; written by GetPointCloud,
                               seeded here as a fallback)

Run command:
    ros2 run behavior_tree pp-test-grasp-place-e2e

Offline run command:
    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json ros2 run behavior_tree pp-test-grasp-place-e2e
"""

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.visualization import create_post_tick_visualizer

from geometry_msgs.msg import Point, PointStamped, Pose, Quaternion
from std_msgs.msg import Header

from .config import (
    KEY_ENV_POINTS,
    KEY_GRASP_POSE,
    KEY_POINT_PLACING_DYNAMIC,
)
from .table_grasping import createTableGrasp
from .table_placing import createTablePlacing


def _seed_inputs() -> py_trees.composites.Parallel:
    """Seed the cross-phase blackboard inputs the place subtree reads.

    These are normally produced live (grasp returns an orientation, vision
    returns a placing point + env cloud). Under full mock those producers
    short-circuit on keypress without writing, so we pre-seed plausible values
    so a real (non-mock) place subtree would still have its READ keys present.
    """
    root = py_trees.composites.Parallel(
        name="Seed grasp->place inputs",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    grasp_pose = Pose(
        position=Point(x=0.5, y=0.0, z=0.3),
        orientation=Quaternion(x=0.707106781, y=0.0, z=0.707106781, w=0.0),
    )
    placing_point = PointStamped(
        header=Header(frame_id="base_link"),
        point=Point(x=0.5, y=0.0, z=0.3),
    )
    seeds = [
        ("Seed grasp pose", KEY_GRASP_POSE, grasp_pose),
        ("Seed placing point", KEY_POINT_PLACING_DYNAMIC, placing_point),
        ("Seed env points", KEY_ENV_POINTS, None),
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


def create_tree() -> py_trees.behaviour.Behaviour:
    """Full scan -> grasp -> find placing -> get cloud -> place chain."""
    root = py_trees.composites.Sequence(name="PickAndPlace grasp->place e2e", memory=True)
    root.add_child(_seed_inputs())
    root.add_child(createTableGrasp(prompt="bottle . cup . can"))
    root.add_child(createTablePlacing(item_description="the grasped object"))
    # Long-running manipulation/vision actions — keep the tree alive after the
    # one-shot chain so the spin loop has something to tick.
    root.add_child(py_trees.behaviours.Running(name="idle"))
    return root


def main():
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=create_tree())
    tree.setup(timeout=15, node_name="pp_test_grasp_place_e2e")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="PickAndPlace Grasp->Place E2E"
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
