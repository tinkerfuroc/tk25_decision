"""Standalone harness for the HRI seat-recommend → arm-point subtree.

Seeds the bare minimum on the blackboard, calls
:class:`BtNode_SeatRecommendBbox` to obtain a 3D centroid for the
recommended empty seat, wraps it as a list, and dispatches
:class:`BtNode_PointTo`. No prior tree phases (arrival, intake,
feature extraction, navigation) are run — pointing depends only on
the seat-recommend service answering against an empty seated-guests
list.

Usage::

    ros2 run behavior_tree hri-point-at-seat

In mock mode (``BT_MOCK_MODE=true``) the seat node returns a fixed
``PointStamped`` (1.0, 0.5, 0.6) in ``base_link`` and the arm node
KEYPRESS-waits. In live mode it hits ``seat_recommend_bbox_service``
(kimi_api) and ``arm_joint_service`` (arm_api).
"""
from __future__ import annotations

import py_trees


def build() -> py_trees.behaviour.Behaviour:
    # Deferred imports: HRI/config.py touches rclpy.time at module scope,
    # only populated after rclpy.init(). run_tree inits rclpy before
    # calling build(). Same deferral pattern as HRI/intro.py and
    # HRI/intake.py.
    from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
    from behavior_tree.TemplateNodes.Vision import BtNode_SeatRecommendBbox
    from behavior_tree.TemplateNodes.Manipulation import BtNode_PointTo
    from behavior_tree.TemplateNodes.Audio import BtNode_Announce
    from behavior_tree.TemplateNodes.structs import Person

    from .config import (
        ARM_POS_POINT_TO,
        KEY_ARM_POINT_TO,
        KEY_PERSONS,
        KEY_SEAT_BBOX,
        KEY_SEAT_POINT,
        KEY_SEAT_POINTS,
        KEY_SEAT_RECOMMENDATION,
    )
    from .hri import BtNode_WrapPointAsList

    # One placeholder Person satisfies two pre-existing contracts:
    #   - BtNode_SeatRecommendBbox builds names/features from
    #     persons[:-1] (excludes the "newest guest"); a single empty
    #     entry yields names=[], features=[] — exactly the no-prior
    #     intent for this isolated test.
    #   - BtNode_PointTo fast-fails when len(persons) <= target_id; we
    #     use target_id=0 so we need at least one entry.
    placeholder_persons = [Person()]

    seed = py_trees.composites.Parallel(
        name="Seed blackboard",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    seed.add_child(
        BtNode_WriteToBlackboard(
            name="Seed placeholder persons",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_PERSONS,
            object=placeholder_persons,
        )
    )
    seed.add_child(
        BtNode_WriteToBlackboard(
            name="Seed arm point-to pose",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_ARM_POINT_TO,
            object=ARM_POS_POINT_TO,
        )
    )

    root = py_trees.composites.Sequence(name="Point-at-seat test", memory=True)
    root.add_child(seed)
    root.add_child(
        BtNode_SeatRecommendBbox(
            name="Seat recommend (bbox)",
            bb_recommendation_key=KEY_SEAT_RECOMMENDATION,
            bb_bbox_key=KEY_SEAT_BBOX,
            bb_point_key=KEY_SEAT_POINT,
            bb_source_key=KEY_PERSONS,
            target_frame="base_link",
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Announce recommendation",
            bb_source=KEY_SEAT_RECOMMENDATION,
        )
    )
    root.add_child(
        BtNode_WrapPointAsList(
            name="Wrap seat point as list",
            bb_source_key=KEY_SEAT_POINT,
            bb_dest_key=KEY_SEAT_POINTS,
        )
    )
    root.add_child(
        BtNode_PointTo(
            name="Point arm to seat",
            bb_key_persons=KEY_PERSONS,
            bb_key_points=KEY_SEAT_POINTS,
            bb_key_init_pose=KEY_ARM_POINT_TO,
            target_id=0,
        )
    )
    return root


def main():
    from behavior_tree.runtime import run_tree

    run_tree(build, period_ms=500.0, title="Point-at-seat")
