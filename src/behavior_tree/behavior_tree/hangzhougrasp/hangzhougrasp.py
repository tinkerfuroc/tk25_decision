"""Phase factories and mission composition, matching HRI/PickAndPlace style."""

import operator

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_AnnounceDetections
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import (
    BtNode_GripperAction, BtNode_Grasp, BtNode_MoveArmSingle,
)
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import BtNode_ScanForYolo26

from .config import (
    KEY_ARM_NAVIGATING, KEY_ARM_TABLE,
    KEY_SCAN_RESULT, KEY_GRASP_SCAN_RESULT, KEY_GRASP_VISION_RES,
    KEY_TARGET_LABEL, KEY_GRASP_LABEL, KEY_ANNOUNCEMENT, KEY_HOLDING,
    load_config, pose_key,
)
from .custom_nodes import BtNode_LogError, BtNode_SelectGraspTarget, BtNode_SetHolding


def createConstantWriter(config, *, enable_grasp=False):
    root = py_trees.composites.Parallel(
        name="Write HangzhouGrasp constants",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    writes, seen = [], set()
    named = [(rec.name, rec.pose) for stop in config.route
             for rec in stop.viewpoints]
    named.append((config.destination_name, config.destination))
    for name, pose in named:
        if name not in seen:
            seen.add(name)
            writes.append((f"Write {name}", pose_key(name), pose))
    if enable_grasp:
        writes.extend([
            ("Write arm navigating", KEY_ARM_NAVIGATING, config.arm_pos_navigating),
            ("Write arm table", KEY_ARM_TABLE, config.arm_pos_table),
        ])
    for name, key, value in writes:
        root.add_child(BtNode_WriteToBlackboard(
            name=name, bb_namespace="", bb_source=None, bb_key=key, object=value,
        ))
    return root


def _moveArmRetry(name, arm_pose_key, config):
    return py_trees.decorators.Retry(
        name=f"Retry {name}",
        child=BtNode_MoveArmSingle(
            name=name, action_name=config.arm_action,
            arm_pose_bb_key=arm_pose_key, add_octomap=False,
        ),
        num_failures=config.arm_retry_limit,
    )


def _gotoRetry(name, pose_key, config):
    return py_trees.decorators.Retry(
        name=f"Retry {name}",
        child=BtNode_GotoAction(
            name=name, key=pose_key, action_name=config.navigation_action,
        ),
        num_failures=config.nav_retry_limit,
    )


def _scanYolo26Retry(name, config, *, bb_key, use_orbbec,
                     bb_source=None, prompt=None, grasp_payload=False):
    return py_trees.decorators.Retry(
        name=f"Retry {name}",
        child=BtNode_ScanForYolo26(
            name=name, bb_source=bb_source, bb_key=bb_key, object=prompt,
            service_name=config.detection_service, use_orbbec=use_orbbec,
            transform_to_map=False, sort_closest=True, sort_highest=False,
            return_rgb_image=grasp_payload, return_depth_image=grasp_payload,
            return_segments=grasp_payload,
        ),
        num_failures=config.scan_retry_limit,
    )


def createScanAnnounce(config, *, grasp=False):
    """Scan on yolo26_sam and announce; grasp=True adds arm pose + payload.

    A scan with no detections FAILS; wrap in a Selector ending in
    py_trees.behaviours.Success where a miss must not abort the mission.
    """
    root = py_trees.composites.Sequence(name="Scan and announce objects", memory=True)
    if grasp:
        root.add_child(_moveArmRetry("Arm to table scan pose", KEY_ARM_TABLE, config))
    root.add_child(_scanYolo26Retry(
        "Scan objects with YOLO26 SAM", config, bb_key=KEY_SCAN_RESULT,
        use_orbbec=not grasp, prompt=config.prompt, grasp_payload=grasp,
    ))
    announce = py_trees.decorators.Retry(
        name="Retry announce detected objects",
        child=BtNode_AnnounceDetections(
            name="Announce detected objects in Chinese", bb_source=KEY_SCAN_RESULT,
            label_map=config.label_map, bb_text_key=KEY_ANNOUNCEMENT,
            service_name=config.announce_service,
        ),
        num_failures=config.announce_retry_limit,
    )
    # A speech failure is logged but must not turn a detection into a miss.
    root.add_child(py_trees.composites.Selector(
        name="Announce or log failure", memory=True,
        children=[announce, BtNode_LogError(
            name="Log announce failure",
            message="Announcing detections failed after retries; continuing")]))
    return root


def createGrasp(config):
    """Select once, then reacquire the chosen class before each grasp attempt."""
    root = py_trees.composites.Sequence(name="Grasp nearest object", memory=True)
    root.add_child(BtNode_SelectGraspTarget(
        name="Select nearest detected object", bb_source=KEY_SCAN_RESULT,
        bb_key=KEY_GRASP_VISION_RES, label_key=KEY_TARGET_LABEL,
    ))
    attempt = py_trees.composites.Sequence(name="Reacquire and grasp", memory=True)
    attempt.add_child(_scanYolo26Retry(
        "Reacquire selected label with RealSense", config,
        bb_key=KEY_GRASP_SCAN_RESULT, use_orbbec=False,
        bb_source=KEY_TARGET_LABEL, grasp_payload=True,
    ))
    attempt.add_child(BtNode_SelectGraspTarget(
        name="Prepare fresh single-object grasp bundle",
        bb_source=KEY_GRASP_SCAN_RESULT,
        bb_key=KEY_GRASP_VISION_RES, label_key=KEY_GRASP_LABEL,
    ))
    attempt.add_child(BtNode_Grasp(
        name="Grasp selected object", action_name=config.grasp_action,
        bb_key_vision_res=KEY_GRASP_VISION_RES,
        bb_key_object_label=KEY_GRASP_LABEL, use_mesh=True, stay=False,
    ))
    root.add_child(py_trees.decorators.Retry(
        name="Retry reacquire and grasp", child=attempt,
        num_failures=config.grasp_retry_limit,
    ))
    # Holding is true the moment the grasp succeeds, even if the retract fails.
    root.add_child(BtNode_SetHolding(name="Set holding after grasp", value=True))
    root.add_child(_moveArmRetry(
        "Retract arm with object", KEY_ARM_NAVIGATING, config))
    return root


def createScanGraspWithCleanup(config, *, miss_is_success=False):
    """Selector cleanup like PickAndPlace.

    The grasp sets holding as soon as it succeeds. On any failure the arm is
    retracted; if that retract succeeds the result is FAILURE (a recovered
    miss) unless miss_is_success, in which case SUCCESS. If the cleanup
    retract itself fails, FAILURE always propagates.
    """
    attempt = py_trees.composites.Sequence(
        name="Scan announce and grasp", memory=True,
        children=[createScanAnnounce(config, grasp=True), createGrasp(config)],
    )
    outcome = (py_trees.behaviours.Success(name="Report recovered miss")
               if miss_is_success else
               py_trees.behaviours.Failure(name="Report scan or grasp failure"))
    cleanup = py_trees.composites.Sequence(
        name="Retract on scan or grasp failure", memory=True,
        children=[
            _moveArmRetry("Retract arm on failure", KEY_ARM_NAVIGATING, config),
            outcome,
        ],
    )
    return py_trees.composites.Selector(
        name="Scan and grasp with failure cleanup", memory=True,
        children=[attempt, cleanup],
    )


def _holding_is_set():
    return py_trees.behaviours.CheckBlackboardVariableValue(
        name="Already holding an object?",
        check=py_trees.common.ComparisonExpression(
            variable=KEY_HOLDING, value=True,
            operator=operator.eq),
    )


def createVisitTable(config, viewpoints):
    """Try each viewpoint in order; stop after the first successful grasp.

    Each viewpoint succeeds when the attempt succeeded or failed and the arm
    was retracted afterwards (a miss, or holding already set). A failed
    navigation or a failed cleanup retract fails the table.
    """
    root = py_trees.composites.Sequence(name="Visit table", memory=True)
    for rec in viewpoints:
        visit = py_trees.composites.Sequence(
            name=f"Visit {rec.name}", memory=True)
        visit.add_child(_gotoRetry(
            f"Go to {rec.name}", pose_key(rec.name), config))
        visit.add_child(createScanGraspWithCleanup(config, miss_is_success=True))
        root.add_child(py_trees.composites.Selector(
            name=f"Skip or visit {rec.name}", memory=True,
            children=[_holding_is_set(), visit]))
    return root


def _release(name, config, goto_name, goto_key):
    root = py_trees.composites.Sequence(name=name, memory=True)
    root.add_child(_moveArmRetry(
        "Arm to navigation pose", KEY_ARM_NAVIGATING, config))
    root.add_child(_gotoRetry(goto_name, goto_key, config))
    return root


def _open_and_clear(root, where):
    root.add_child(BtNode_GripperAction(
        name=f"Open gripper at {where}", open_gripper=True))
    root.add_child(BtNode_SetHolding(
        name=f"Clear holding after release at {where}", value=False))


def createDeliver(config):
    dest = config.destination_name
    root = _release("Deliver object", config, f"Go to destination {dest}",
                    pose_key(dest))
    _open_and_clear(root, dest)
    return root


def createDropElsewhere(config, next_pose):
    """Drop at the next scan pose; with no next pose, the real drop."""
    if next_pose is None:
        return createDeliver(config)
    root = _release("Drop object elsewhere", config,
                    f"Go to drop pose {next_pose.name}", pose_key(next_pose.name))
    root.add_child(BtNode_Announce(
        name="Announce unreachable drop target", bb_source=None,
        message="无法到达真正的放置位置，我将在这里放下物体。"))
    _open_and_clear(root, next_pose.name)
    return root


def _scan_or_miss(config, name):
    return py_trees.composites.Selector(
        name=f"Scan or miss at {name}", memory=True,
        children=[createScanAnnounce(config, grasp=False),
                  py_trees.behaviours.Success(name="Miss, skip")])


def _delivery_or_nothing(config, stop, next_pose):
    if stop.room == config.enter_room:
        delivery = createDeliver(config)
    else:
        delivery = createDropElsewhere(config, next_pose)
    holding = py_trees.composites.Sequence(
        name="Deliver held object", memory=True,
        children=[_holding_is_set(), delivery])
    return py_trees.composites.Selector(
        name="Deliver if holding", memory=True,
        children=[py_trees.decorators.Inverter(
            name="Not holding", child=_holding_is_set()), holding])


def createTablesPhase(config, stops, *, enable_grasp):
    root = py_trees.composites.Sequence(name="Tables phase", memory=True)
    for stop, next_pose in stops:
        if stop.kind != "table":
            continue
        if enable_grasp:
            root.add_child(createVisitTable(config, stop.viewpoints))
            root.add_child(_delivery_or_nothing(config, stop, next_pose))
        else:
            for rec in stop.viewpoints:
                visit = py_trees.composites.Sequence(
                    name=f"Visit {rec.name}", memory=True)
                visit.add_child(_gotoRetry(
                    f"Go to {rec.name}", pose_key(rec.name), config))
                visit.add_child(_scan_or_miss(config, rec.name))
                root.add_child(visit)
    return root


def createGroundPhase(config, stops):
    root = py_trees.composites.Sequence(name="Ground phase", memory=True)
    for stop, _ in stops:
        if stop.kind != "ground":
            continue
        rec = stop.viewpoints[0]
        visit = py_trees.composites.Sequence(
            name=f"Visit {rec.name}", memory=True)
        visit.add_child(_gotoRetry(
            f"Go to {rec.name}", pose_key(rec.name), config))
        visit.add_child(_scan_or_miss(config, rec.name))
        root.add_child(visit)
    return root


def createRoom(config, room, route_slice, *, enable_grasp):
    root = py_trees.composites.Sequence(name=f"Room {room}", memory=True)
    root.add_child(createTablesPhase(
        config, route_slice, enable_grasp=enable_grasp))
    root.add_child(createGroundPhase(config, route_slice))
    return root


def createHangzhouGrasp(*, enable_grasp=False, config=None):
    """Build grasp nodes only when enable_grasp=True; one mission per run."""
    if type(enable_grasp) is not bool:
        raise TypeError("enable_grasp must be a boolean")
    config = config if config is not None else load_config()
    root = py_trees.composites.Sequence(name="HangzhouGrasp Root", memory=True)
    root.add_child(createConstantWriter(config, enable_grasp=enable_grasp))
    if enable_grasp:
        root.add_child(_moveArmRetry(
            "Arm to nav before first navigation", KEY_ARM_NAVIGATING, config))
    route = config.route
    paired = [
        (stop, route[i + 1].viewpoints[0] if i + 1 < len(route) else None)
        for i, stop in enumerate(route)]
    for room in config.rooms:
        root.add_child(createRoom(
            config, room, [p for p in paired if p[0].room == room],
            enable_grasp=enable_grasp))
    return root
