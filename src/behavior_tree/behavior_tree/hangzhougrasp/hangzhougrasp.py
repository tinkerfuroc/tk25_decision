"""Phase factories and mission composition, matching HRI/PickAndPlace style."""

import operator

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_AnnounceDetections
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import BtNode_ScanForYolo26

from .config import (
    KEY_SCAN_RESULT, KEY_TARGET_LABEL,
    KEY_ANNOUNCEMENT, KEY_HOLDING, load_config, pose_key,
)
from .custom_nodes import (
    BtNode_ClearKey, BtNode_HeadTilt, BtNode_LogError, BtNode_ScanSawObjects,
    BtNode_SelectNearestLabel, BtNode_SetHolding,
)
from .tk27_nodes import BtNode_Tk27GripperOpen, BtNode_Tk27Pick, BtNode_Tk27SavedPose


def createConstantWriter(config):
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
    for name, key, value in writes:
        root.add_child(BtNode_WriteToBlackboard(
            name=name, bb_namespace="", bb_source=None, bb_key=key, object=value,
        ))
    return root


def _savedPoseRetry(name, pose_name, config):
    """TK27 MoveToSavedPose (avoid_collisions=False), bounded retries."""
    return py_trees.decorators.Retry(
        name=f"Retry {name}",
        child=BtNode_Tk27SavedPose(
            name=name, pose_name=pose_name, action_name=config.saved_pose_action,
        ),
        num_failures=config.arm_retry_limit,
    )


def _gripperOpen(name, config):
    return BtNode_Tk27GripperOpen(
        name=name, action_name=config.gripper_action,
        open_position=config.gripper_open_position,
        open_tolerance=config.gripper_open_tolerance,
        max_effort=config.gripper_max_effort,
    )


def _gotoRetry(name, pose_key, config):
    """Every navigation stows the arm in its navigation pose first (all modes)."""
    return py_trees.composites.Sequence(
        name=f"Stow then {name}", memory=True,
        children=[
            _savedPoseRetry(f"Arm to nav before {name}",
                            config.arm_pose_navigating, config),
            py_trees.decorators.Retry(
                name=f"Retry {name}",
                child=BtNode_GotoAction(
                    name=name, key=pose_key, action_name=config.navigation_action,
                ),
                num_failures=config.nav_retry_limit,
            ),
        ])


def _scanYolo26Retry(name, config, *, bb_key, use_orbbec, prompt):
    """yolo26_sam scan, nearest first. No images or masks: Pick observes itself."""
    return py_trees.decorators.Retry(
        name=f"Retry {name}",
        child=BtNode_ScanForYolo26(
            name=name, bb_source=None, bb_key=bb_key, object=prompt,
            service_name=config.detection_service, use_orbbec=use_orbbec,
            transform_to_map=False, sort_closest=True, sort_highest=False,
            return_rgb_image=False, return_depth_image=False,
            return_segments=False,
        ),
        num_failures=config.scan_retry_limit,
    )


def _headTilt(config, surface):
    """Point the Orbbec head at the surface; a head that will not settle is
    logged and the scan goes ahead anyway (the run keeps scoring)."""
    tilt = {"table": config.table_scan_tilt_deg,
            "ground": config.ground_scan_tilt_deg}[surface]
    name = f"Head to {surface} scan tilt"
    return py_trees.composites.Selector(
        name=f"{name} or log", memory=True,
        children=[
            py_trees.decorators.Retry(
                name=f"Retry {name}",
                child=BtNode_HeadTilt(name=name, x=config.head_pan_deg, y=tilt),
                num_failures=config.head_retry_limit),
            BtNode_LogError(
                name="Log head failure",
                message=f"Orbbec head did not settle at tilt {tilt} for the "
                        f"{surface} scan; scanning anyway"),
        ])


def createScanAnnounce(config, *, surface):
    """Point the Orbbec head for the surface, scan on yolo26_sam, announce.

    surface is "table" or "ground"; each has its own tilt in constants.json.
    A scan with no detections FAILS; wrap in a Selector ending in
    py_trees.behaviours.Success where a miss must not abort the mission.
    """
    if surface not in ("table", "ground"):
        raise ValueError(f"surface must be 'table' or 'ground', got {surface!r}")
    root = py_trees.composites.Sequence(name="Scan and announce objects", memory=True)
    root.add_child(_headTilt(config, surface))
    root.add_child(_scanYolo26Retry(
        "Scan objects with YOLO26 SAM", config, bb_key=KEY_SCAN_RESULT,
        use_orbbec=True, prompt=config.prompt,
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
    """Label from the Orbbec scan, then pick45 + open + TK27 Pick, then stow.

    TK27 Pick does its own RealSense observation and segmentation, so the
    tree trusts it and sends only the nearest label the Orbbec scan saw.
    Once grasping has started, the attempt is retried up to grasp_retry_limit
    times: back to pick45, a fresh open and a fresh Pick goal.
    """
    root = py_trees.composites.Sequence(name="Grasp nearest object", memory=True)
    root.add_child(BtNode_SelectNearestLabel(
        name="Select nearest label", bb_source=KEY_SCAN_RESULT,
        label_key=KEY_TARGET_LABEL,
    ))
    attempt = py_trees.composites.Sequence(name="Grasp attempt", memory=True)
    # Re-sent every round: a failed Pick may leave the arm off pick45.
    attempt.add_child(_savedPoseRetry("Arm to table pose", config.arm_pose_table, config))
    # Voice note: always open before Pick; a failed open means no Pick.
    attempt.add_child(_gripperOpen("Open gripper before pick", config))
    attempt.add_child(BtNode_Tk27Pick(
        name="Pick selected object", prompt_bb_key=KEY_TARGET_LABEL,
        action_name=config.pick_action, plan_effort=config.pick_plan_effort,
    ))
    root.add_child(py_trees.decorators.Retry(
        name="Retry grasp attempt", child=attempt,
        num_failures=config.grasp_retry_limit,
    ))
    # Holding is true the moment Pick succeeds, even if the retract fails.
    root.add_child(BtNode_SetHolding(name="Set holding after grasp", value=True))
    root.add_child(_savedPoseRetry(
        "Retract arm with object", config.arm_pose_navigating, config))
    return root


def createScanGraspWithCleanup(config, *, miss_is_success=False):
    """Selector cleanup like PickAndPlace.

    Pick sets holding as soon as it succeeds. On any failure the arm is
    retracted; if that retract succeeds the result is FAILURE (a recovered
    miss) unless miss_is_success, in which case SUCCESS. If the cleanup
    retract itself fails, FAILURE always propagates.
    """
    outcome = (py_trees.behaviours.Success(name="Report recovered miss")
               if miss_is_success else
               py_trees.behaviours.Failure(name="Report scan or grasp failure"))
    cleanup = py_trees.composites.Sequence(
        name="Retract on scan or grasp failure", memory=True,
        children=[
            _savedPoseRetry("Retract arm on failure", config.arm_pose_navigating,
                            config),
            outcome,
        ],
    )
    return py_trees.composites.Selector(
        name="Scan and grasp with failure cleanup", memory=True,
        children=[createGrasp(config), cleanup],
    )


def createViewpointScanGrasp(config, *, miss_is_success):
    """Orbbec scan + announce with the arm stowed; grasp only if it saw objects.

    An Orbbec miss never reaches the cleanup branch, so the arm does not move
    and a failed cleanup stow can still fail the run.
    """
    root = py_trees.composites.Sequence(name="Viewpoint scan then grasp", memory=True)
    root.add_child(BtNode_ClearKey(name="Clear previous scan result", key=KEY_SCAN_RESULT))
    root.add_child(py_trees.composites.Selector(
        name="Scan or nothing seen", memory=True,
        children=[createScanAnnounce(config, surface="table"),
                  py_trees.behaviours.Success(name="Nothing seen")]))
    root.add_child(py_trees.composites.Selector(
        name="Grasp only if seen", memory=True,
        children=[
            py_trees.decorators.Inverter(
                name="Nothing to grasp",
                child=BtNode_ScanSawObjects(
                    name="Scan saw objects?", bb_source=KEY_SCAN_RESULT)),
            createScanGraspWithCleanup(config, miss_is_success=miss_is_success),
        ]))
    return root


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
        visit.add_child(createViewpointScanGrasp(config, miss_is_success=True))
        root.add_child(py_trees.composites.Selector(
            name=f"Skip or visit {rec.name}", memory=True,
            children=[_holding_is_set(), visit]))
    return root


def _release(name, config, goto_name, goto_key):
    root = py_trees.composites.Sequence(name=name, memory=True)
    root.add_child(_gotoRetry(goto_name, goto_key, config))  # stows first
    return root


def _open_and_clear(root, where, config):
    """Retry the release; if it still fails, log and go on (scans still score).

    Holding is cleared either way, or every later viewpoint would be skipped.
    """
    root.add_child(py_trees.composites.Selector(
        name=f"Release or log at {where}", memory=True,
        children=[
            py_trees.decorators.Retry(
                name=f"Retry open gripper at {where}",
                child=_gripperOpen(f"Open gripper at {where}", config),
                num_failures=config.arm_retry_limit),
            BtNode_LogError(
                name=f"Log release failure at {where}",
                message=f"Gripper did not open at {where} after retries; "
                        "continuing the run"),
        ]))
    root.add_child(BtNode_SetHolding(
        name=f"Clear holding after release at {where}", value=False))


def createDeliver(config):
    dest = config.destination_name
    root = _release("Deliver object", config, f"Go to destination {dest}",
                    pose_key(dest))
    _open_and_clear(root, dest, config)
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
    _open_and_clear(root, next_pose.name, config)
    return root


def _scan_or_miss(config, name, surface):
    return py_trees.composites.Selector(
        name=f"Scan or miss at {name}", memory=True,
        children=[createScanAnnounce(config, surface=surface),
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
                visit.add_child(_scan_or_miss(config, rec.name, "table"))
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
        visit.add_child(_scan_or_miss(config, rec.name, "ground"))
        root.add_child(visit)
    return root


def createRoom(config, room, route_slice, *, enable_grasp):
    root = py_trees.composites.Sequence(name=f"Room {room}", memory=True)
    root.add_child(createTablesPhase(
        config, route_slice, enable_grasp=enable_grasp))
    root.add_child(createGroundPhase(config, route_slice))
    return root


def createHangzhouGrasp(*, enable_grasp=False, config=None):
    """Build Pick/gripper nodes only when enable_grasp=True; one mission per run.

    In every mode the arm is stowed before entering the arena and before
    every navigation (see _gotoRetry).
    """
    if type(enable_grasp) is not bool:
        raise TypeError("enable_grasp must be a boolean")
    config = config if config is not None else load_config()
    root = py_trees.composites.Sequence(name="HangzhouGrasp Root", memory=True)
    root.add_child(createConstantWriter(config))
    root.add_child(_savedPoseRetry(
        "Arm to nav before entering the arena", config.arm_pose_navigating, config))
    route = config.route
    paired = [
        (stop, route[i + 1].viewpoints[0] if i + 1 < len(route) else None)
        for i, stop in enumerate(route)]
    for room in config.rooms:
        root.add_child(createRoom(
            config, room, [p for p in paired if p[0].room == room],
            enable_grasp=enable_grasp))
    return root
