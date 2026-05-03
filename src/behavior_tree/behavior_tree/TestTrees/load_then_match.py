"""
Isolated test harness: load a host reference photo + .txt description from
disk, pack into the same Person struct the Receptionist would build, then
run feature matching against the live scene.

End-to-end smoke for BtNode_LoadPersonReference + the production combiner
+ /feature_matching_service. Operator inspects the result via the
vision_log/ dump that kimi_api/feature_matching.py already writes.

Configuration (env vars, all optional):
    HOST_IMAGE_PATH      default: ~/tk25_ws/host_reference/host.jpg
    HOST_DESC_PATH       default: ~/tk25_ws/host_reference/host.txt
    MATCH_MAX_DISTANCE   default: 5.0
    MATCH_TARGET_FRAME   default: base_link
    MATCH_USE_ORBBEC     default: true

Run: ros2 run behavior_tree load-then-match
"""

import os
import threading

import py_trees
import py_trees_ros
import rclpy
from rclpy.executors import ExternalShutdownException

from behavior_tree.TemplateNodes.Vision import (
    BtNode_LoadPersonReference,
    BtNode_FeatureMatching,
)
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.Receptionist.customNodes import BtNode_CombinePerson


KEY_HOST_NAME = "lpm_host_name"
KEY_HOST_DRINK = "lpm_host_drink"
KEY_HOST_FEATURES = "lpm_host_features"
KEY_HOST_IMAGE = "lpm_host_image"
KEY_PERSONS = "lpm_persons"
KEY_PERSON_CENTROIDS = "lpm_person_centroids"


def createTree(
    image_path: str,
    description_path: str,
    max_distance: float,
    target_frame: str,
    use_orbbec: bool,
) -> py_trees.composites.Sequence:
    root = py_trees.composites.Sequence(name="load-then-match", memory=True)

    root.add_child(BtNode_WriteToBlackboard(
        name="seed name",
        bb_namespace="",
        bb_source=None,
        bb_key=KEY_HOST_NAME,
        object="John",
    ))
    root.add_child(BtNode_WriteToBlackboard(
        name="seed drink",
        bb_namespace="",
        bb_source=None,
        bb_key=KEY_HOST_DRINK,
        object="water",
    ))
    root.add_child(BtNode_WriteToBlackboard(
        name="init persons",
        bb_namespace="",
        bb_source=None,
        bb_key=KEY_PERSONS,
        object=[],
    ))
    root.add_child(BtNode_LoadPersonReference(
        name="load host ref",
        image_path=image_path,
        description_path=description_path,
        bb_features_key=KEY_HOST_FEATURES,
        bb_image_key=KEY_HOST_IMAGE,
    ))
    root.add_child(BtNode_CombinePerson(
        name="pack host",
        key_dest=KEY_PERSONS,
        key_name=KEY_HOST_NAME,
        key_drink=KEY_HOST_DRINK,
        key_features=KEY_HOST_FEATURES,
        key_image=KEY_HOST_IMAGE,
    ))
    root.add_child(BtNode_FeatureMatching(
        name="match host",
        bb_dest_key=KEY_PERSON_CENTROIDS,
        bb_persons_key=KEY_PERSONS,
        max_distance=max_distance,
        target_frame=target_frame,
        use_orbbec=use_orbbec,
        trim_last_person=False,
    ))
    return root


def main():
    rclpy.init(args=None)

    image_path = os.environ.get("HOST_IMAGE_PATH", "/home/tinker/tk25_ws/img_host.jpg")
    description_path = os.environ.get("HOST_DESC_PATH", "/home/tinker/tk25_ws/host.txt")
    max_distance = float(os.environ.get("MATCH_MAX_DISTANCE", "5.0"))
    target_frame = os.environ.get("MATCH_TARGET_FRAME", "base_link")
    use_orbbec = os.environ.get("MATCH_USE_ORBBEC", "true").lower() in ("1", "true", "yes")

    print(
        f"[load-then-match] image={image_path} desc={description_path} "
        f"max_distance={max_distance} target_frame={target_frame} "
        f"use_orbbec={use_orbbec}"
    )

    root = createTree(image_path, description_path, max_distance, target_frame, use_orbbec)
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node_name="load_then_match", timeout=15)
    assert tree.node is not None, "tree.setup() did not initialise the ROS node"

    completed = threading.Event()

    def post_tick_handler(t):
        if t.root.status != py_trees.common.Status.RUNNING:
            print(f"[load-then-match] terminal status: {t.root.status}")
            completed.set()

    tree.tick_tock(period_ms=500.0, post_tick_handler=post_tick_handler)

    try:
        while not completed.is_set():
            rclpy.spin_once(tree.node, timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
