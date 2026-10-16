"""Independently runnable parts: same creators as the full tree, own constants."""

import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard

from . import hangzhougrasp as task
from .config import KEY_ARM_NAVIGATING, KEY_ARM_TABLE, pose_key

PARTS = {
    "scan-announce": "Scan and announce where the robot stands (or after --at); no arm",
    "scan-grasp": "Scan, announce, select and grasp, with failure cleanup (needs --grasp)",
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


def _writer(config, pose_names, *, arm_nav=False, arm_table=False):
    """Constants writer limited to the named poses (and chosen arm poses)."""
    known = _known_poses(config)
    root = py_trees.composites.Parallel(
        name="Write part constants",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    writes, seen = [], set()
    for name in pose_names:
        if name not in seen:
            seen.add(name)
            writes.append((f"Write {name}", pose_key(name), known[name]))
    if arm_nav:
        writes.append(("Write arm navigating", KEY_ARM_NAVIGATING,
                       config.arm_pos_navigating))
    if arm_table:
        writes.append(("Write arm table", KEY_ARM_TABLE, config.arm_pos_table))
    for name, key, value in writes:
        root.add_child(BtNode_WriteToBlackboard(
            name=name, bb_namespace="", bb_source=None, bb_key=key, object=value))
    return root


def _goto(config, at):
    return task._gotoRetry(f"Go to {at}", pose_key(at), config)


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

    arm_nav = arm_table = False
    poses, body = [], None
    if name == "scan-announce":
        body = task.createScanAnnounce(config, grasp=False)
    elif name == "scan-grasp":
        arm_nav = arm_table = True
        body = task.createScanGraspWithCleanup(config)
    elif name == "visit-table":
        stops = [s for s, _ in _room_stops(config, room)
                 if s.kind == "table" and s.viewpoints[0].n == table]
        if not stops:
            raise ValueError(f"room {room!r} has no table {table}")
        arm_nav = arm_table = True
        poses = [rec.name for rec in stops[0].viewpoints]
        body = task.createVisitTable(config, stops[0].viewpoints)
    elif name == "deliver":
        arm_nav = True
        poses = [config.destination_name]
        body = task.createDeliver(config)
    elif name == "drop-elsewhere":
        arm_nav = True
        rec = next((r for s in config.route for r in s.viewpoints
                    if r.name == next_pose), None)
        if rec is None:  # the destination is a pose but not a route viewpoint
            raise ValueError(f"--next-pose {next_pose!r} is not a route viewpoint")
        poses = [next_pose]
        body = task.createDropElsewhere(config, rec)
    elif name == "ground":
        stops = _room_stops(config, room)
        poses = _names([p for p in stops if p[0].kind == "ground"])
        body = task.createGroundPhase(config, stops)
    elif name == "room":
        stops = _room_stops(config, room)
        poses = _names(stops)
        poses += [p[1].name for p in stops[-1:] if p[1] is not None]
        poses.append(config.destination_name)
        arm_nav = arm_table = enable_grasp
        body = task.createRoom(config, room, stops, enable_grasp=enable_grasp)

    # The arm must be in its navigation pose before the part navigates at all
    # (full tree does the same); deliver/drop-elsewhere do it in their own subtree.
    arm_first = enable_grasp and (name in ("room", "visit-table") or bool(at))
    arm_nav = arm_nav or arm_first
    root = py_trees.composites.Sequence(name=f"Part {name}", memory=True)
    root.add_child(_writer(config, ([at] if at else []) + poses,
                           arm_nav=arm_nav, arm_table=arm_table))
    if arm_first:
        root.add_child(task._moveArmRetry(
            "Arm to nav before first navigation", KEY_ARM_NAVIGATING, config))
    if at:
        root.add_child(_goto(config, at))
    root.add_child(body)
    return root
