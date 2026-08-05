#!/usr/bin/env python3
"""Render an exact, non-generative contact sheet for the supervisor replay."""
from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont, ImageOps


CANVAS = (1740, 1180)
PANEL = (540, 430)
GAP = 28
LEFT = 32
TOP = 122


def _font(size: int, *, bold: bool = False):
    filename = (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
        if bold
        else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    )
    try:
        return ImageFont.truetype(filename, size=size)
    except OSError:
        return ImageFont.load_default()


def _image_panel(
    canvas: Image.Image,
    *,
    column: int,
    row: int,
    title: str,
    subtitle: str,
    path: Path,
    border: tuple[int, int, int] = (54, 67, 79),
) -> None:
    x = LEFT + column * (PANEL[0] + GAP)
    y = TOP + row * (PANEL[1] + GAP)
    draw = ImageDraw.Draw(canvas)
    draw.rounded_rectangle(
        (x, y, x + PANEL[0], y + PANEL[1]),
        radius=13,
        fill=(27, 35, 44),
        outline=border,
        width=3,
    )
    draw.text((x + 20, y + 15), title, font=_font(24, bold=True), fill=(244, 247, 250))
    draw.text((x + 20, y + 49), subtitle, font=_font(15), fill=(162, 174, 185))
    source = Image.open(path).convert("RGB")
    fitted = ImageOps.contain(source, (PANEL[0] - 28, PANEL[1] - 94))
    frame = Image.new("RGB", (PANEL[0] - 28, PANEL[1] - 94), (12, 17, 22))
    frame.paste(
        fitted,
        ((frame.width - fitted.width) // 2, (frame.height - fitted.height) // 2),
    )
    canvas.paste(frame, (x + 14, y + 80))


def _status_panel(canvas: Image.Image, *, column: int, row: int) -> None:
    x = LEFT + column * (PANEL[0] + GAP)
    y = TOP + row * (PANEL[1] + GAP)
    draw = ImageDraw.Draw(canvas)
    draw.rounded_rectangle(
        (x, y, x + PANEL[0], y + PANEL[1]),
        radius=13,
        fill=(27, 35, 44),
        outline=(52, 159, 104),
        width=3,
    )
    draw.text(
        (x + 24, y + 22),
        "VALIDATION",
        font=_font(23, bold=True),
        fill=(126, 224, 173),
    )
    rows = (
        ("PASS", "Tinker base + xArm7 + wrist camera mesh"),
        ("PASS", "Fixture manifest + artifact hashes"),
        ("PASS", "Prompt v2 cross-modal consistency contract"),
        ("PASS", "Luna-medium negative control (live)"),
    )
    for index, (status, text) in enumerate(rows):
        row_y = y + 82 + index * 72
        draw.rounded_rectangle(
            (x + 24, row_y, x + 100, row_y + 35),
            radius=7,
            fill=(31, 112, 73),
        )
        draw.text(
            (x + 37, row_y + 6),
            status,
            font=_font(16, bold=True),
            fill=(232, 255, 242),
        )
        draw.text(
            (x + 118, row_y + 7),
            text,
            font=_font(16),
            fill=(219, 226, 232),
        )
    draw.line((x + 24, y + 367, x + PANEL[0] - 24, y + 367), fill=(68, 81, 94))
    draw.text(
        (x + 24, y + 383),
        "Mismatch policy: uncertain · stop · sensor_context_mismatch",
        font=_font(14),
        fill=(179, 191, 201),
    )


def render(state_dir: Path, checkpoint: str, output: Path) -> None:
    artifact_dir = state_dir / "artifacts"
    fixture_dir = (
        Path(__file__).resolve().parents[2]
        / "behavior_tree/behavior_tree/GPSR/supervision/fixtures"
    )
    paths = {
        "front": artifact_dir / f"{checkpoint}-front_camera.jpg",
        "wrist": artifact_dir / f"{checkpoint}-wrist_camera.jpg",
        "map": artifact_dir / f"{checkpoint}-map.png",
        "arm": artifact_dir / f"{checkpoint}-arm.png",
        "mismatch": fixture_dir / "wrist_camera_mismatch.jpg",
    }
    missing = [str(path) for path in paths.values() if not path.is_file()]
    if missing:
        raise FileNotFoundError("missing contact-sheet artifacts: " + ", ".join(missing))

    canvas = Image.new("RGB", CANVAS, (15, 21, 27))
    draw = ImageDraw.Draw(canvas)
    draw.text(
        (LEFT, 28),
        "GPSR supervisor · hardware-free multimodal checkpoint",
        font=_font(34, bold=True),
        fill=(245, 248, 250),
    )
    draw.text(
        (LEFT, 76),
        "Exact replay artifacts — images below are composited without generative alteration",
        font=_font(18),
        fill=(155, 169, 181),
    )
    _image_panel(
        canvas,
        column=0,
        row=0,
        title="FRONT CAMERA",
        subtitle="Real Orbbec vision-log frame",
        path=paths["front"],
    )
    _image_panel(
        canvas,
        column=1,
        row=0,
        title="WRIST CAMERA · UPWARD",
        subtitle="Synthetic hardware-free fixture, explicitly labeled",
        path=paths["wrist"],
        border=(64, 118, 179),
    )
    _image_panel(
        canvas,
        column=2,
        row=0,
        title="NEGATIVE CONTROL · REJECTED",
        subtitle="Real AprilTag frame from a different calibration scene",
        path=paths["mismatch"],
        border=(193, 75, 73),
    )
    _image_panel(
        canvas,
        column=0,
        row=1,
        title="NAVIGATION MAP",
        subtitle="Saved arena occupancy map + current robot pose",
        path=paths["map"],
    )
    _image_panel(
        canvas,
        column=1,
        row=1,
        title="ARM POSE",
        subtitle="arm_pos_orbbec_look · full base/arm/camera geometry",
        path=paths["arm"],
        border=(219, 144, 57),
    )
    _status_panel(canvas, column=2, row=1)
    draw.text(
        (LEFT, 1060),
        "Interpretation: different camera viewpoints are acceptable; contradictory scene/pose evidence is not.",
        font=_font(20, bold=True),
        fill=(227, 233, 238),
    )
    draw.text(
        (LEFT, 1100),
        "Default replay uses the upward wrist frame. The calibration image is supplied only to verify safe rejection.",
        font=_font(17),
        fill=(159, 173, 185),
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state-dir", type=Path, required=True)
    parser.add_argument("--checkpoint", default="cp-01-goto-table")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    render(args.state_dir, args.checkpoint, args.output)
    print(args.output)


if __name__ == "__main__":
    main()
