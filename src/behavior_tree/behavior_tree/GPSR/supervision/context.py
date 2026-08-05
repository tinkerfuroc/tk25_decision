"""Checkpoint assembly and deterministic hardware-free context fixtures."""
from __future__ import annotations

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import tempfile
from typing import Any, Callable, Mapping, Protocol

from .models import ArtifactRef, CaptureRequest, SnapshotBundle


REQUIRED_ARTIFACT_ROLES = ("front_camera", "wrist_camera", "map", "arm")


def gpsr_arm_pose(name: str) -> tuple[float, ...]:
    """Load one GPSR runtime arm pose and convert degrees to radians."""
    constants_path = Path(__file__).resolve().parents[1] / "constants.json"
    constants = json.loads(constants_path.read_text(encoding="utf-8"))
    degrees = constants.get(name)
    if not isinstance(degrees, list) or len(degrees) != 7:
        raise ValueError(f"GPSR {name} must contain seven joints")
    return tuple(math.radians(float(value)) for value in degrees)


def gpsr_named_pose(name: str) -> tuple[float, float, float]:
    """Resolve one named arena pose from the GPSR runtime constants."""
    constants_path = Path(__file__).resolve().parents[1] / "constants.json"
    constants = json.loads(constants_path.read_text(encoding="utf-8"))
    raw = constants.get("possible_poses", {}).get(name)
    if not isinstance(raw, Mapping):
        raise KeyError(f"unknown GPSR named pose {name!r}")
    point = raw.get("point", {})
    orientation = raw.get("orientation", {})
    x = float(point["x"])
    y = float(point["y"])
    qx = float(orientation.get("x", 0.0))
    qy = float(orientation.get("y", 0.0))
    qz = float(orientation.get("z", 0.0))
    qw = float(orientation.get("w", 1.0))
    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    return (x, y, yaw)


def gpsr_arm_pose_navigating() -> tuple[float, ...]:
    return gpsr_arm_pose("arm_pos_navigating")


def gpsr_arm_pose_orbbec_look() -> tuple[float, ...]:
    return gpsr_arm_pose("arm_pos_orbbec_look")


class ContextProvider(Protocol):
    def capture(self, request: CaptureRequest) -> SnapshotBundle:
        """Return artifacts for a checkpoint without changing robot state."""


class FixtureContextProvider:
    """Use committed camera/map fixtures and render pose/arm state on demand."""

    def __init__(
        self,
        fixture_dir: Path | None = None,
        output_dir: Path | None = None,
        *,
        clock: Callable[[], datetime] | None = None,
        scenario_id: str | None = None,
        require_urdf_renderer: bool = False,
    ) -> None:
        self.fixture_dir = fixture_dir or Path(__file__).with_name("fixtures")
        self.output_dir = output_dir or Path(tempfile.gettempdir()) / "gpsr-supervisor-fixtures"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._clock = clock or (lambda: datetime.now(timezone.utc))
        self.scenario_id = scenario_id
        self.require_urdf_renderer = require_urdf_renderer
        self._verify_manifest()

    def capture(self, request: CaptureRequest) -> SnapshotBundle:
        scenario = None
        if self.scenario_id:
            from .scenarios import get_scenario

            scenario = get_scenario(self.scenario_id)
        captured_at = self._clock().isoformat()
        safe_checkpoint = "".join(
            char if char.isalnum() or char in "-_" else "_" for char in request.checkpoint_id
        )[:120]
        map_output = self.output_dir / f"{safe_checkpoint}-map.png"
        arm_output = self.output_dir / f"{safe_checkpoint}-arm.png"
        render_map_pose(
            self.fixture_dir / "arena_map.pgm",
            self.fixture_dir / "arena_map.yaml",
            request.robot_pose,
            map_output,
            goal_pose=(
                gpsr_named_pose(scenario.map_goal_name)
                if scenario and scenario.map_goal_name
                else None
            ),
            last_known_pose=(
                gpsr_named_pose(scenario.map_last_known_name)
                if scenario and scenario.map_last_known_name
                else None
            ),
            uncertainty_radius_m=(
                scenario.map_uncertainty_radius_m if scenario else None
            ),
        )
        pose_name = str(
            request.blackboard.get(
                "gpsr/arm_pose_name", "joint-state snapshot"
            )
        )
        arm_metadata = render_arm_pose(
            request.arm_joints,
            arm_output,
            pose_name=pose_name,
        )
        if (
            self.require_urdf_renderer
            and arm_metadata.get("renderer") != "xarm_urdf_headless"
        ):
            raise RuntimeError(
                "the GPSR validation suite requires the xArm URDF renderer"
            )
        front_path = self.fixture_dir / (
            scenario.front_image if scenario else "front_camera.jpg"
        )
        wrist_path = self.fixture_dir / (
            scenario.wrist_image if scenario else "wrist_camera.jpg"
        )
        wrist_direction = (
            scenario.wrist_view_direction if scenario else "upward"
        )
        artifacts = (
            ArtifactRef.from_path(
                role="front_camera",
                mime_type=_image_mime(front_path),
                path=front_path,
                captured_at=captured_at,
                metadata={
                    "fixture": True,
                    "camera": "orbbec",
                    "scenario_id": self.scenario_id,
                    "provenance": (
                        "generated_hardware_free"
                        if scenario and scenario.generated
                        else "vision_log"
                    ),
                },
            ),
            ArtifactRef.from_path(
                role="wrist_camera",
                mime_type=_image_mime(wrist_path),
                path=wrist_path,
                captured_at=captured_at,
                metadata={
                    "fixture": True,
                    "camera": "wrist",
                    "scenario_id": self.scenario_id,
                    "view_direction": wrist_direction,
                    "provenance": (
                        "real_calibration_negative_control"
                        if scenario
                        and scenario.scenario_id == "case08-sensor-mismatch-stop"
                        else "synthetic_hardware_free"
                    ),
                },
            ),
            ArtifactRef.from_path(
                role="map",
                mime_type="image/png",
                path=map_output,
                captured_at=captured_at,
                metadata={
                    "fixture": True,
                    "robot_pose": list(request.robot_pose),
                    "goal_pose_name": (
                        scenario.map_goal_name if scenario else None
                    ),
                    "last_known_pose_name": (
                        scenario.map_last_known_name if scenario else None
                    ),
                    "uncertainty_radius_m": (
                        scenario.map_uncertainty_radius_m if scenario else None
                    ),
                },
            ),
            ArtifactRef.from_path(
                role="arm",
                mime_type="image/png",
                path=arm_output,
                captured_at=captured_at,
                metadata={
                    "fixture": True,
                    "joints": list(request.arm_joints),
                    "pose_name": pose_name,
                    **arm_metadata,
                },
            ),
        )
        return SnapshotBundle(request=request, artifacts=artifacts)

    def _verify_manifest(self) -> None:
        manifest_path = self.fixture_dir / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        for name, record in manifest.get("assets", {}).items():
            expected = record.get("sha256")
            if not expected:
                continue
            actual = ArtifactRef.from_path(
                role="fixture",
                mime_type="application/octet-stream",
                path=self.fixture_dir / name,
                captured_at="manifest-check",
            ).sha256
            if actual != expected:
                raise ValueError(f"fixture checksum mismatch for {name}")


class StaticContextProvider:
    """Small test provider for already-created artifact references."""

    def __init__(self, artifacts: tuple[ArtifactRef, ...]) -> None:
        self.artifacts = artifacts

    def capture(self, request: CaptureRequest) -> SnapshotBundle:
        by_role = {artifact.role: artifact for artifact in self.artifacts}
        timestamp = datetime.now(timezone.utc).isoformat()
        complete = tuple(
            by_role.get(role, ArtifactRef.absent(role, timestamp, "not supplied"))
            for role in REQUIRED_ARTIFACT_ROLES
        )
        return SnapshotBundle(request=request, artifacts=complete)


def render_map_pose(
    map_path: Path,
    metadata_path: Path,
    robot_pose: tuple[float, float, float],
    output_path: Path,
    *,
    goal_pose: tuple[float, float, float] | None = None,
    last_known_pose: tuple[float, float, float] | None = None,
    uncertainty_radius_m: float | None = None,
) -> None:
    try:
        from PIL import Image, ImageDraw
    except ImportError as exc:  # pragma: no cover - fixture-only dependency
        raise RuntimeError("Pillow is required to render GPSR map fixtures") from exc

    metadata = _read_map_yaml(metadata_path)
    resolution = float(metadata["resolution"])
    origin_x, origin_y, _ = metadata["origin"]
    source = Image.open(map_path).convert("RGB")
    display_scale = 3
    resampling = getattr(Image, "Resampling", Image)
    image = source.resize(
        (source.width * display_scale, source.height * display_scale),
        resampling.NEAREST,
    )
    draw = ImageDraw.Draw(image)
    x, y, yaw = robot_pose
    px = (x - origin_x) / resolution * display_scale
    py = image.height - (y - origin_y) / resolution * display_scale
    radius = max(9, int(0.22 / resolution) * display_scale)
    if goal_pose is not None:
        gx, gy, _ = goal_pose
        goal_x = (gx - origin_x) / resolution * display_scale
        goal_y = image.height - (gy - origin_y) / resolution * display_scale
        goal_radius = radius + 7
        draw.ellipse(
            (
                goal_x - goal_radius,
                goal_y - goal_radius,
                goal_x + goal_radius,
                goal_y + goal_radius,
            ),
            outline=(21, 171, 96),
            width=6,
        )
        draw.line(
            (
                goal_x - goal_radius,
                goal_y,
                goal_x + goal_radius,
                goal_y,
            ),
            fill=(21, 171, 96),
            width=3,
        )
        draw.line(
            (
                goal_x,
                goal_y - goal_radius,
                goal_x,
                goal_y + goal_radius,
            ),
            fill=(21, 171, 96),
            width=3,
        )
        draw.text(
            (goal_x + goal_radius + 5, goal_y - goal_radius),
            f"goal ({gx:.2f}, {gy:.2f})",
            fill=(7, 91, 48),
        )
    if last_known_pose is not None:
        lx, ly, _ = last_known_pose
        last_x = (lx - origin_x) / resolution * display_scale
        last_y = image.height - (ly - origin_y) / resolution * display_scale
        marker = radius
        draw.line(
            (last_x - marker, last_y - marker, last_x + marker, last_y + marker),
            fill=(231, 126, 34),
            width=6,
        )
        draw.line(
            (last_x - marker, last_y + marker, last_x + marker, last_y - marker),
            fill=(231, 126, 34),
            width=6,
        )
        draw.text(
            (last_x + marker + 5, last_y - marker),
            f"last reliable ({lx:.2f}, {ly:.2f})",
            fill=(132, 66, 9),
        )
    if uncertainty_radius_m is not None and uncertainty_radius_m > 0:
        uncertainty_px = uncertainty_radius_m / resolution * display_scale
        draw.ellipse(
            (
                px - uncertainty_px,
                py - uncertainty_px,
                px + uncertainty_px,
                py + uncertainty_px,
            ),
            outline=(222, 50, 73),
            width=7,
        )
        draw.text(
            (px - uncertainty_px, py + uncertainty_px + 5),
            f"pose uncertainty {uncertainty_radius_m:.1f} m",
            fill=(151, 24, 42),
        )
    draw.ellipse(
        (px - radius, py - radius, px + radius, py + radius),
        fill=(26, 115, 232),
        outline=(255, 255, 255),
        width=2,
    )
    arrow = max(radius * 2, 14)
    tip = (px + arrow * math.cos(yaw), py - arrow * math.sin(yaw))
    draw.line((px, py, *tip), fill=(255, 210, 40), width=4)
    draw.text((px + radius + 3, py - radius), f"robot ({x:.2f}, {y:.2f})", fill=(10, 10, 10))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)


def _image_mime(path: Path) -> str:
    return "image/png" if path.suffix.lower() == ".png" else "image/jpeg"


def render_arm_pose(
    joints: tuple[float, ...],
    output_path: Path,
    *,
    pose_name: str = "joint-state snapshot",
) -> dict[str, str]:
    """Render ROS xArm7 visual geometry, with a portable fallback."""
    try:
        from .xarm_render import render_xarm_urdf

        return render_xarm_urdf(
            joints,
            output_path,
            pose_name=pose_name,
        )
    except (FileNotFoundError, ImportError, RuntimeError):
        return _render_arm_pose_fallback(joints, output_path)


def _render_arm_pose_fallback(
    joints: tuple[float, ...], output_path: Path
) -> dict[str, str]:
    try:
        from PIL import Image, ImageDraw
    except ImportError as exc:  # pragma: no cover - fixture-only dependency
        raise RuntimeError("Pillow is required to render GPSR arm fixtures") from exc

    values = tuple(float(value) for value in joints)
    if len(values) != 7:
        raise ValueError("arm fixture requires exactly seven joint positions")
    image = Image.new("RGB", (720, 480), (245, 247, 250))
    draw = ImageDraw.Draw(image)
    origin = (100.0, 380.0)
    lengths = (75, 70, 65, 58, 50, 42, 34)
    angle = -math.pi / 2
    points = [origin]
    for joint, length in zip(values, lengths):
        angle += joint
        previous = points[-1]
        points.append(
            (previous[0] + length * math.cos(angle), previous[1] + length * math.sin(angle))
        )
    draw.line(points, fill=(35, 88, 170), width=12, joint="curve")
    for index, point in enumerate(points):
        draw.ellipse(
            (point[0] - 8, point[1] - 8, point[0] + 8, point[1] + 8),
            fill=(244, 166, 35),
            outline=(70, 70, 70),
        )
        if index:
            draw.text((point[0] + 10, point[1] - 12), f"J{index}", fill=(20, 20, 20))
    draw.text((20, 20), "Portable xArm kinematic fallback", fill=(20, 20, 20))
    draw.text((20, 45), ", ".join(f"{value:+.2f}" for value in values), fill=(60, 60, 60))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)
    return {"renderer": "kinematic_fallback", "geometry": "link centerlines"}


def blackboard_keys_from_tree(tree_document: Mapping[str, Any]) -> tuple[str, ...]:
    keys: set[str] = set()
    for node in tree_document.get("nodes", []) or []:
        access = node.get("blackboard_access", {}) if isinstance(node, Mapping) else {}
        if not isinstance(access, Mapping):
            continue
        for mode in ("read", "write", "exclusive"):
            values = access.get(mode, [])
            if isinstance(values, list):
                keys.update(str(value) for value in values)
    return tuple(sorted(keys))


def snapshot_blackboard(
    keys: tuple[str, ...],
    getter: Callable[[str], Any],
) -> dict[str, Any]:
    snapshot: dict[str, Any] = {}
    for key in keys:
        lowered = key.lower()
        if any(token in lowered for token in ("password", "secret", "token", "api_key")):
            snapshot[key] = "<redacted>"
            continue
        try:
            snapshot[key] = _safe_value(getter(key))
        except (KeyError, AttributeError):
            snapshot[key] = {"missing": True}
        except Exception as exc:
            snapshot[key] = {"unavailable": type(exc).__name__}
    return snapshot


def next_unticked_node(
    tree_document: Mapping[str, Any], terminal_node_id: str
) -> Mapping[str, Any] | None:
    nodes = list(tree_document.get("nodes", []) or [])
    terminal_index = next(
        (index for index, node in enumerate(nodes) if node.get("node_id") == terminal_node_id),
        -1,
    )
    for node in nodes[terminal_index + 1 :]:
        semantics = node.get("semantics", {})
        if node.get("status") == "INVALID" and semantics.get("category") == "leaf":
            return node
    return None


def _read_map_yaml(path: Path) -> dict[str, Any]:
    values: dict[str, Any] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        if ":" not in raw_line:
            continue
        key, _, raw_value = raw_line.partition(":")
        key, raw_value = key.strip(), raw_value.strip()
        if raw_value.startswith("[") and raw_value.endswith("]"):
            values[key] = [float(item.strip()) for item in raw_value[1:-1].split(",")]
        elif key in {"resolution", "occupied_thresh", "free_thresh"}:
            values[key] = float(raw_value)
        else:
            values[key] = raw_value
    if "resolution" not in values or "origin" not in values:
        raise ValueError(f"invalid map metadata: {path}")
    return values


def _safe_value(value: Any, depth: int = 0) -> Any:
    if depth > 6:
        return "<max-depth>"
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Mapping):
        return {str(key): _safe_value(item, depth + 1) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_safe_value(item, depth + 1) for item in value]
    fields = getattr(value, "__dict__", None)
    if isinstance(fields, Mapping):
        return {
            str(key): _safe_value(item, depth + 1)
            for key, item in fields.items()
            if not str(key).startswith("_")
        }
    return str(value)


__all__ = [
    "ContextProvider",
    "FixtureContextProvider",
    "REQUIRED_ARTIFACT_ROLES",
    "StaticContextProvider",
    "blackboard_keys_from_tree",
    "gpsr_arm_pose",
    "gpsr_arm_pose_navigating",
    "gpsr_arm_pose_orbbec_look",
    "gpsr_named_pose",
    "next_unticked_node",
    "render_arm_pose",
    "render_map_pose",
    "snapshot_blackboard",
]
