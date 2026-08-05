"""Headless xArm7 URDF visualizer for hardware-free supervisor fixtures.

The renderer consumes the same visual STL files referenced by the ROS xArm
description. It does not need an OpenGL display, so it is stable in CI and over
SSH where RViz itself cannot create a window.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import math
import os
from pathlib import Path
import struct
from typing import Any
import xml.etree.ElementTree as ET


_ROBOT_LINKS = {
    "base_link",
    "link_base",
    *(f"link{index}" for index in range(1, 8)),
    "xarm_gripper_base_link",
    "left_outer_knuckle",
    "left_finger",
    "left_inner_knuckle",
    "right_outer_knuckle",
    "right_finger",
    "right_inner_knuckle",
}
_GRIPPER_LINKS = {
    "xarm_gripper_base_link",
    "left_outer_knuckle",
    "left_finger",
    "left_inner_knuckle",
    "right_outer_knuckle",
    "right_finger",
    "right_inner_knuckle",
}


@dataclass(frozen=True)
class _Visual:
    link: str
    mesh_path: Path
    origin: Any
    scale: Any


@dataclass(frozen=True)
class _Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin: Any
    axis: Any
    mimic: tuple[str, float, float] | None


@dataclass(frozen=True)
class _RobotModel:
    urdf_path: Path
    visuals: tuple[_Visual, ...]
    joints: tuple[_Joint, ...]


def render_xarm_urdf(
    joints: tuple[float, ...],
    output_path: Path,
    *,
    gripper_position: float = 0.42,
    pose_name: str = "joint-state snapshot",
) -> dict[str, str]:
    """Render an xArm7 pose from ROS visual meshes and return provenance."""
    try:
        import numpy as np
        from PIL import Image, ImageDraw
    except ImportError as exc:  # pragma: no cover - fixture-only dependencies
        raise RuntimeError("NumPy and Pillow are required for xArm rendering") from exc

    values = tuple(float(value) for value in joints)
    if len(values) != 7:
        raise ValueError("xArm7 renderer requires exactly seven joint positions")
    model = _load_model(_find_xarm_urdf())
    configuration = {
        **{f"joint{index}": value for index, value in enumerate(values, start=1)},
        "drive_joint": float(gripper_position),
    }
    link_transforms, joint_points = _forward_kinematics(model, configuration, np)
    robot_origin = np.linalg.inv(link_transforms["base_link"])

    width, height = 768, 768
    # High-contrast diagnostic palette: white xArm, blue Tinker chassis, and
    # orange gripper/camera cues on a near-black background.
    image = Image.new("RGB", (width, height), (10, 16, 25))
    draw = ImageDraw.Draw(image)

    eye = np.asarray((2.0, -2.35, 1.65), dtype=float)
    target = np.asarray((-0.12, 0.0, 0.55), dtype=float)
    forward = _unit(target - eye, np)
    right = _unit(np.cross(forward, np.asarray((0.0, 0.0, 1.0))), np)
    up = _unit(np.cross(right, forward), np)

    projected_points: list[Any] = []
    transformed_meshes: list[tuple[str, Any]] = []
    for visual in model.visuals:
        if visual.link not in link_transforms:
            continue
        local_triangles = _load_stl(visual.mesh_path)
        # Keep every triangle. Dropping faces is quick, but creates visible
        # holes in the detailed gripper mesh and weakens pose diagnosis.
        local_triangles = local_triangles * visual.scale
        transform = robot_origin @ link_transforms[visual.link] @ visual.origin
        flat = local_triangles.reshape((-1, 3))
        homogeneous = np.column_stack((flat, np.ones(len(flat))))
        world = (transform @ homogeneous.T).T[:, :3].reshape((-1, 3, 3))
        transformed_meshes.append((visual.link, world))
        projected_points.append(
            _camera_coordinates(
                world.reshape((-1, 3)), eye, right, up, forward, np
            )
        )

    all_camera = np.concatenate(projected_points, axis=0)
    min_x, max_x = float(all_camera[:, 0].min()), float(all_camera[:, 0].max())
    min_y, max_y = float(all_camera[:, 1].min()), float(all_camera[:, 1].max())
    viewport = (32.0, 32.0, width - 32.0, height - 32.0)
    scale = min(
        (viewport[2] - viewport[0]) / max(max_x - min_x, 0.01),
        (viewport[3] - viewport[1]) / max(max_y - min_y, 0.01),
    ) * 0.88
    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0
    screen_center = (
        (viewport[0] + viewport[2]) / 2.0,
        (viewport[1] + viewport[3]) / 2.0,
    )

    def project(camera_points):
        result = np.empty((len(camera_points), 2), dtype=float)
        result[:, 0] = screen_center[0] + (camera_points[:, 0] - center_x) * scale
        result[:, 1] = screen_center[1] - (camera_points[:, 1] - center_y) * scale
        return result

    # RViz-like ground grid gives scale and orientation without inventing a
    # robot environment.
    grid_color = (57, 78, 98)
    for coordinate in [value / 10.0 for value in range(-6, 7)]:
        for start, end in (
            ((coordinate, -0.6, 0.0), (coordinate, 0.6, 0.0)),
            ((-0.6, coordinate, 0.0), (0.6, coordinate, 0.0)),
        ):
            camera_line = _camera_coordinates(
                np.asarray((start, end)), eye, right, up, forward, np
            )
            draw.line(tuple(map(tuple, project(camera_line))), fill=grid_color, width=1)

    triangles: list[
        tuple[float, tuple[tuple[float, float], ...], tuple[int, int, int]]
    ] = []
    light = _unit(np.asarray((-0.45, -0.65, 1.0), dtype=float), np)
    for link, world_triangles in transformed_meshes:
        camera = _camera_coordinates(
            world_triangles.reshape((-1, 3)), eye, right, up, forward, np
        ).reshape((-1, 3, 3))
        screen = project(camera[:, :, :2].reshape((-1, 2))).reshape((-1, 3, 2))
        edges_a = world_triangles[:, 1] - world_triangles[:, 0]
        edges_b = world_triangles[:, 2] - world_triangles[:, 0]
        normals = np.cross(edges_a, edges_b)
        norm = np.linalg.norm(normals, axis=1)
        good = norm > 1e-10
        normals[good] /= norm[good, None]
        intensity = 0.44 + 0.56 * np.abs(normals @ light)
        base = _link_color(link)
        for index in range(len(world_triangles)):
            if not good[index]:
                continue
            color = tuple(
                max(0, min(255, round(channel * float(intensity[index]))))
                for channel in base
            )
            polygon = tuple((float(x), float(y)) for x, y in screen[index])
            triangles.append(
                (float(camera[index, :, 2].mean()), polygon, color)
            )

    for _, polygon, color in sorted(
        triangles, key=lambda item: item[0], reverse=True
    ):
        draw.polygon(polygon, fill=color)

    # Overlay the seven joint origins after the meshes, like an RViz joint-state
    # diagnostic. Labels stay small so they do not obscure the model.
    for index in range(1, 8):
        point = joint_points.get(f"joint{index}")
        if point is None:
            continue
        local = (robot_origin @ np.asarray((*point, 1.0)))[:3]
        camera_point = _camera_coordinates(
            local[None, :], eye, right, up, forward, np
        )
        sx, sy = project(camera_point)[0]
        radius = 5
        draw.ellipse(
            (sx - radius, sy - radius, sx + radius, sy + radius),
            fill=(255, 171, 64),
            outline=(67, 35, 5),
            width=2,
        )
        draw.text((sx + 7, sy - 12), f"J{index}", fill=(255, 214, 151))

    # Calibrated Tinker2 wrist optical axis. This is the semantic bridge
    # between the supplied joint pose and the corresponding wrist image.
    camera_mount = _matrix_from_xyz_rpy(
        (0.06572972, -0.02489187, 0.12288894),
        (2.50995298, -1.53906218, 0.68646904),
        np,
    )
    color_offset = _matrix_from_xyz_rpy(
        (0.0, 0.015, 0.0), (0.0, 0.0, 0.0), np
    )
    optical_rotation = _matrix_from_xyz_rpy(
        (0.0, 0.0, 0.0), (-math.pi / 2, 0.0, -math.pi / 2), np
    )
    camera_tf = (
        robot_origin
        @ link_transforms["link_eef"]
        @ camera_mount
        @ color_offset
        @ optical_rotation
    )
    camera_start = camera_tf[:3, 3]
    camera_direction = camera_tf[:3, 2]
    camera_elevation_deg = math.degrees(
        math.asin(float(camera_direction[2]))
    )
    if camera_elevation_deg > 15:
        camera_direction_label = f"UPWARD {camera_elevation_deg:+.0f}°"
    elif camera_elevation_deg < -15:
        camera_direction_label = f"DOWNWARD {camera_elevation_deg:+.0f}°"
    else:
        camera_direction_label = f"LEVEL {camera_elevation_deg:+.0f}°"
    camera_end = camera_start + camera_direction * 0.32
    camera_screen = project(
        _camera_coordinates(
            np.asarray((camera_start, camera_end)),
            eye,
            right,
            up,
            forward,
            np,
        )
    )
    draw.line(
        tuple(map(tuple, camera_screen)),
        fill=(41, 225, 230),
        width=8,
    )
    cx, cy = camera_screen[1]
    draw.ellipse(
        (cx - 7, cy - 7, cx + 7, cy + 7),
        fill=(41, 225, 230),
        outline=(3, 45, 52),
        width=2,
    )
    draw.text(
        (cx + 10, cy - 15),
        f"WRIST CAMERA: {camera_direction_label}",
        fill=(113, 246, 249),
    )

    # Orientation axes anchored at the arm base.
    axis_origin = np.asarray((0.0, 0.0, 0.0))
    axis_points = {
        "X": (np.asarray((0.18, 0.0, 0.0)), (210, 55, 55)),
        "Y": (np.asarray((0.0, 0.18, 0.0)), (43, 153, 82)),
        "Z": (np.asarray((0.0, 0.0, 0.18)), (45, 101, 210)),
    }
    origin_screen = project(
        _camera_coordinates(axis_origin[None, :], eye, right, up, forward, np)
    )[0]
    for label, (endpoint, color) in axis_points.items():
        end_screen = project(
            _camera_coordinates(endpoint[None, :], eye, right, up, forward, np)
        )[0]
        draw.line((*origin_screen, *end_screen), fill=color, width=4)
        draw.text((end_screen[0] + 4, end_screen[1] - 8), label, fill=color)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    resampling = getattr(Image, "Resampling", Image)
    image.resize((768, 768), resampling.LANCZOS).save(output_path)
    return {
        "renderer": "xarm_urdf_headless",
        "urdf": str(model.urdf_path),
        "geometry": "Tinker base + xarm_description visual STL",
        "layout": "scene_only",
        "image_size": "768x768",
        "pose_name": pose_name,
        "camera_axis": "calibrated Tinker2 wrist optical frame",
        "camera_axis_up_dot": f"{float(camera_direction[2]):.3f}",
        "camera_elevation_deg": f"{camera_elevation_deg:.1f}",
        "camera_direction": camera_direction_label.lower(),
    }


def _find_xarm_urdf() -> Path:
    override = os.environ.get("GPSR_XARM_URDF", "").strip()
    if override:
        path = Path(override).expanduser()
        if path.is_file():
            return path.resolve()
        raise FileNotFoundError(f"GPSR_XARM_URDF does not exist: {path}")

    relative = Path("src/tk25_basic/src/cumotion_description/config/xarm7.urdf")
    candidates: list[Path] = []
    for anchor in (Path.cwd(), Path(__file__).resolve()):
        for parent in (anchor, *anchor.parents):
            candidates.append(parent / relative)
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError(
        "xArm7 URDF not found; set GPSR_XARM_URDF to the generated xarm7.urdf"
    )


@lru_cache(maxsize=2)
def _load_model(urdf_path: Path) -> _RobotModel:
    try:
        import numpy as np
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError("NumPy is required for xArm rendering") from exc

    root = ET.parse(urdf_path).getroot()
    package_roots = _package_roots(urdf_path)
    visuals: list[_Visual] = []
    for link_element in root.findall("link"):
        link = link_element.get("name", "")
        if link not in _ROBOT_LINKS:
            continue
        visual_element = link_element.find("visual")
        if visual_element is None:
            continue
        mesh_element = visual_element.find("geometry/mesh")
        if mesh_element is None or not mesh_element.get("filename"):
            continue
        visuals.append(
            _Visual(
                link=link,
                mesh_path=_resolve_mesh(
                    mesh_element.get("filename", ""), package_roots
                ),
                origin=_origin_matrix(visual_element.find("origin"), np),
                scale=np.asarray(
                    _floats(
                        mesh_element.get("scale"), (1.0, 1.0, 1.0)
                    ),
                    dtype=float,
                ),
            )
        )
    camera_mesh = (
        package_roots["xarm_description"]
        / "meshes/camera/realsense/visual/d435_with_cam_stand.stl"
    )
    if camera_mesh.is_file():
        visuals.append(
            _Visual(
                link="link_eef",
                mesh_path=camera_mesh.resolve(),
                origin=np.eye(4),
                scale=np.ones(3),
            )
        )

    parsed_joints: list[_Joint] = []
    for joint_element in root.findall("joint"):
        parent_element = joint_element.find("parent")
        child_element = joint_element.find("child")
        if parent_element is None or child_element is None:
            continue
        mimic_element = joint_element.find("mimic")
        mimic = None
        if mimic_element is not None:
            mimic = (
                mimic_element.get("joint", ""),
                float(mimic_element.get("multiplier", "1")),
                float(mimic_element.get("offset", "0")),
            )
        axis_element = joint_element.find("axis")
        parsed_joints.append(
            _Joint(
                name=joint_element.get("name", ""),
                kind=joint_element.get("type", "fixed"),
                parent=parent_element.get("link", ""),
                child=child_element.get("link", ""),
                origin=_origin_matrix(joint_element.find("origin"), np),
                axis=np.asarray(
                    _floats(
                        axis_element.get("xyz")
                        if axis_element is not None
                        else None,
                        (1.0, 0.0, 0.0),
                    ),
                    dtype=float,
                ),
                mimic=mimic,
            )
        )
    if not visuals or not any(
        joint.name == "joint7" for joint in parsed_joints
    ):
        raise ValueError(
            f"URDF does not contain a complete xArm7 visual model: {urdf_path}"
        )
    return _RobotModel(
        urdf_path=urdf_path,
        visuals=tuple(visuals),
        joints=tuple(parsed_joints),
    )


def _package_roots(urdf_path: Path) -> dict[str, Path]:
    workspace = None
    for parent in urdf_path.parents:
        if (parent / "src/tk25_manipulation").is_dir():
            workspace = parent
            break
    if workspace is None:
        raise FileNotFoundError(
            f"cannot locate workspace assets beside {urdf_path}"
        )
    return {
        "xarm_description": workspace
        / "src/tk25_manipulation/src/xarm_ros2/xarm_description",
        "tinker_urdf": workspace / "src/tk25_basic/src/tinker_urdf",
    }


def _resolve_mesh(filename: str, package_roots: dict[str, Path]) -> Path:
    if filename.startswith("package://"):
        remainder = filename[len("package://") :]
        package, separator, relative = remainder.partition("/")
        if not separator or package not in package_roots:
            raise FileNotFoundError(f"cannot resolve URDF mesh: {filename}")
        path = package_roots[package] / relative
    else:
        path = Path(filename)
    if not path.is_file():
        raise FileNotFoundError(f"URDF mesh not found: {path}")
    return path.resolve()


def _forward_kinematics(
    model: _RobotModel, configuration: dict[str, float], np
):
    transforms = {"world": np.eye(4)}
    joint_points: dict[str, Any] = {}
    remaining = list(model.joints)
    while remaining:
        progressed = False
        for joint in tuple(remaining):
            if joint.parent not in transforms:
                continue
            value = configuration.get(joint.name, 0.0)
            if joint.mimic is not None:
                source, multiplier, offset = joint.mimic
                value = configuration.get(source, 0.0) * multiplier + offset
            parent = transforms[joint.parent]
            fixed = parent @ joint.origin
            joint_points[joint.name] = fixed[:3, 3].copy()
            motion = np.eye(4)
            if joint.kind in {"revolute", "continuous"}:
                motion[:3, :3] = _axis_rotation(joint.axis, value, np)
            elif joint.kind == "prismatic":
                motion[:3, 3] = _unit(joint.axis, np) * value
            transforms[joint.child] = fixed @ motion
            remaining.remove(joint)
            progressed = True
        if not progressed:
            break
    if "link_base" not in transforms:
        raise ValueError("xArm base is disconnected in the supplied URDF")
    return transforms, joint_points


def _camera_coordinates(points, eye, right, up, forward, np):
    relative = points - eye
    return np.column_stack(
        (relative @ right, relative @ up, relative @ forward)
    )


def _origin_matrix(element: ET.Element | None, np):
    xyz = _floats(
        element.get("xyz") if element is not None else None,
        (0.0, 0.0, 0.0),
    )
    rpy = _floats(
        element.get("rpy") if element is not None else None,
        (0.0, 0.0, 0.0),
    )
    return _matrix_from_xyz_rpy(xyz, rpy, np)


def _matrix_from_xyz_rpy(xyz, rpy, np):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.asarray(((1, 0, 0), (0, cr, -sr), (0, sr, cr)), dtype=float)
    ry = np.asarray(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)), dtype=float)
    rz = np.asarray(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)), dtype=float)
    result = np.eye(4)
    result[:3, :3] = rz @ ry @ rx
    result[:3, 3] = xyz
    return result


def _axis_rotation(axis, angle: float, np):
    x, y, z = _unit(axis, np)
    c, s, one_minus_c = (
        math.cos(angle),
        math.sin(angle),
        1.0 - math.cos(angle),
    )
    return np.asarray(
        (
            (
                c + x * x * one_minus_c,
                x * y * one_minus_c - z * s,
                x * z * one_minus_c + y * s,
            ),
            (
                y * x * one_minus_c + z * s,
                c + y * y * one_minus_c,
                y * z * one_minus_c - x * s,
            ),
            (
                z * x * one_minus_c - y * s,
                z * y * one_minus_c + x * s,
                c + z * z * one_minus_c,
            ),
        ),
        dtype=float,
    )


def _unit(vector, np):
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-12:
        raise ValueError("cannot normalize a zero vector")
    return vector / norm


def _floats(
    raw: str | None, default: tuple[float, ...]
) -> tuple[float, ...]:
    if raw is None or not raw.strip():
        return default
    return tuple(float(item) for item in raw.split())


@lru_cache(maxsize=32)
def _load_stl(path: Path):
    try:
        import numpy as np
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError("NumPy is required for xArm rendering") from exc

    data = path.read_bytes()
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        expected = 84 + count * 50
        if expected == len(data):
            record = np.dtype(
                [
                    ("normal", "<f4", (3,)),
                    ("vertices", "<f4", (3, 3)),
                    ("attribute", "<u2"),
                ]
            )
            return np.frombuffer(
                data, dtype=record, count=count, offset=84
            )["vertices"].astype(float)
    vertices = []
    for line in data.decode("ascii", errors="ignore").splitlines():
        fields = line.strip().split()
        if len(fields) == 4 and fields[0].lower() == "vertex":
            vertices.append(tuple(float(value) for value in fields[1:]))
    if not vertices or len(vertices) % 3:
        raise ValueError(f"unsupported STL encoding: {path}")
    return np.asarray(vertices, dtype=float).reshape((-1, 3, 3))


def _link_color(link: str) -> tuple[int, int, int]:
    if link == "base_link":
        return (44, 137, 218)
    if link == "link_eef":
        return (255, 154, 48)
    if link in _GRIPPER_LINKS:
        return (242, 164, 65)
    if link == "link_base":
        return (211, 222, 232)
    try:
        index = int(link.removeprefix("link"))
    except ValueError:
        index = 0
    return (248, 250, 252) if index % 2 else (214, 225, 235)


__all__ = ["render_xarm_urdf"]
