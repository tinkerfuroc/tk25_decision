"""Independently runnable parts: same creators as the full tree, own constants."""

import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard

from . import hangzhougrasp as task
from .config import pose_key

PARTS = {
    "scan-announce": "Scan with the Orbbec camera and announce where the robot stands (or after --at); never moves the arm to the table pose",
    "scan-grasp": "Orbbec scan and announce, then nearest label, pick45, open gripper and TK27 Pick, with failure cleanup (needs --grasp)",
    "visit-table": "All viewpoints of --room --table N without delivery (needs --grasp)",
    "deliver": "Arm to nav pose, go to the destination, open gripper (needs --grasp)",
    "drop-elsewhere": "Drop the object at --next-pose (needs --grasp)",
    "ground": "Ground phase of --room",
    "room": "Whole --room: tables phase then ground phase",
    "full": "The full multi-room mission",
}
_NEEDS_GRASP = {"scan-grasp", "visit-table", "deliver", "drop-elsewhere"}


def _known_poses(config):
    poses = {config.destination_name: config.destination}
    for stop in config.route:
        for rec in stop.viewpoints:
            poses[rec.name] = rec.pose
    return poses


def _writer(config, pose_names):
    """Constants writer limited to the named poses."""
    known = _known_poses(config)
    root = py_trees.composites.Parallel(
        name="Write part constants",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    writes, seen = [], set()
    for name in pose_names:
        if name not in seen:
            seen.add(name)
            writes.append((f"Write {name}", pose_key(name), known[name]))
    for name, key, value in writes:
        root.add_child(BtNode_WriteToBlackboard(
            name=name, bb_namespace="", bb_source=None, bb_key=key, object=value))
    return root


def _goto(config, at, *, stow):
    return task._gotoRetry(f"Go to {at}", pose_key(at), config, stow=stow)


def _room_stops(config, room):
    route = config.route
    paired = [(stop, route[i + 1].viewpoints[0] if i + 1 < len(route) else None)
              for i, stop in enumerate(route)]
    return [p for p in paired if p[0].room == room]


def _names(stops):
    return [rec.name for stop, _ in stops for rec in stop.viewpoints]


def _require(value, option, name):
    if value is None:
        raise ValueError(f"part {name!r} requires {option}")


def _check_pose(config, pose, option):
    if pose is not None and pose not in _known_poses(config):
        raise ValueError(f"{option} pose {pose!r} does not exist in the map")


def createPart(name, config, *, enable_grasp, room=None, table=None,
               at=None, next_pose=None):
    if name not in PARTS:
        raise ValueError(f"unknown part {name!r}; choose from {sorted(PARTS)}")
    if name in _NEEDS_GRASP and not enable_grasp:
        raise ValueError(f"part {name!r} needs the arm; enable grasping")
    if name in ("visit-table", "ground", "room"):
        _require(room, "--room", name)
        if room not in {s.room for s in config.route}:
            raise ValueError(
                f"unknown room {room!r}; rooms are {sorted({s.room for s in config.route})}")
    if name == "visit-table":
        _require(table, "--table", name)
    if name == "drop-elsewhere":
        _require(next_pose, "--next-pose", name)
    _check_pose(config, at, "--at")
    _check_pose(config, next_pose, "--next-pose")

    if name == "full":
        return task.createHangzhouGrasp(enable_grasp=enable_grasp, config=config)

    poses, body = [], None
    if name == "scan-announce":
        body = task.createScanAnnounce(config, surface="table")
    elif name == "scan-grasp":
        body = task.createViewpointScanGrasp(config, miss_is_success=False)
    elif name == "visit-table":
        stops = [s for s, _ in _room_stops(config, room)
                 if s.kind == "table" and s.viewpoints[0].n == table]
        if not stops:
            raise ValueError(f"room {room!r} has no table {table}")
        poses = [rec.name for rec in stops[0].viewpoints]
        body = task.createVisitTable(config, stops[0].viewpoints)
    elif name == "deliver":
        poses = [config.destination_name]
        body = task.createDeliver(config)
    elif name == "drop-elsewhere":
        rec = next((r for s in config.route for r in s.viewpoints
                    if r.name == next_pose), None)
        if rec is None:  # the destination is a pose but not a route viewpoint
            raise ValueError(f"--next-pose {next_pose!r} is not a route viewpoint")
        poses = [next_pose]
        body = task.createDropElsewhere(config, rec)
    elif name == "ground":
        stops = _room_stops(config, room)
        poses = _names([p for p in stops if p[0].kind == "ground"])
        body = task.createGroundPhase(config, stops, enable_grasp=enable_grasp)
    elif name == "room":
        stops = _room_stops(config, room)
        poses = _names(stops)
        poses += [p[1].name for p in stops[-1:] if p[1] is not None]
        poses.append(config.destination_name)
        body = task.createRoom(config, room, stops, enable_grasp=enable_grasp)

    # With --grasp the arm must be in its navigation pose before the part
    # navigates at all (every --grasp goto also stows; see task._gotoRetry).
    # --no-grasp sends no arm commands.
    arm_first = enable_grasp and (
        name in ("room", "visit-table", "ground") or bool(at))
    root = py_trees.composites.Sequence(name=f"Part {name}", memory=True)
    root.add_child(_writer(config, ([at] if at else []) + poses))
    if arm_first:
        root.add_child(task._savedPoseRetry(
            "Arm to nav before first navigation", config.arm_pose_navigating, config))
    if at:
        root.add_child(_goto(config, at, stow=enable_grasp))
    root.add_child(body)
    return root
