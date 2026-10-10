#!/usr/bin/env python3
"""Render a 5x2 visual report for the ten GPSR VLM scenarios."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw, ImageFont, ImageOps

from behavior_tree.GPSR.supervision.scenarios import SCENARIO_CASES


CARD = (970, 550)
GAP = 24
MARGIN = 28
HEADER = 120
CANVAS = (
    MARGIN * 2 + CARD[0] * 2 + GAP,
    HEADER + MARGIN + CARD[1] * 5 + GAP * 4 + 54,
)
THUMB = (220, 155)


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


def _asset(artifact_dir: Path, stage_id: str, role: str) -> Path:
    matches = sorted(artifact_dir.glob(f"{stage_id}-{role}.*"))
    if not matches:
        raise FileNotFoundError(f"missing {stage_id}-{role} artifact")
    return matches[0]


def _load_results(path: Path | None) -> list[dict[str, Any]]:
    if path is None or not path.is_file():
        return []
    return list(
        json.loads(path.read_text(encoding="utf-8")).get("records", [])
    )


def _fit(path: Path) -> Image.Image:
    source = Image.open(path).convert("RGB")
    resampling = getattr(Image, "Resampling", Image)
    fitted = ImageOps.contain(source, THUMB, method=resampling.LANCZOS)
    frame = Image.new("RGB", THUMB, (7, 12, 18))
    frame.paste(
        fitted,
        ((frame.width - fitted.width) // 2, (frame.height - fitted.height) // 2),
    )
    return frame


def _expected(case) -> str:
    values = []
    for stage in case.stages:
        verify = stage.verification
        route = (
            f" → {stage.planner.action}"
            if stage.planner is not None
            else ""
        )
        values.append(
            f"{stage.stage_id}: {verify.verdict}/{verify.bt_assessment}/"
            f"{verify.escalation}{route}"
        )
    return " | ".join(values)


def _observed_lines(case, results: list[dict[str, Any]]) -> list[str]:
    stage_ids = {stage.stage_id for stage in case.stages}
    selected = [item for item in results if item.get("stage_id") in stage_ids]
    if not selected:
        return ["Run 1: NOT RUN", "Run 2: NOT RUN", "Run 3: NOT RUN"]
    lines = []
    for repetition in range(1, 4):
        records = [
            item
            for item in selected
            if item.get("repetition") == repetition
        ]
        passed = sum(bool(item.get("passed")) for item in records)
        details = []
        for item in records:
            observed = item.get("observed") or {}
            value = (
                observed.get("verdict")
                or observed.get("kind")
                or observed.get("action")
                or "error"
            )
            details.append(f"{item.get('role')}={value}")
        lines.append(
            f"Run {repetition}: {passed}/{len(records)} PASS · "
            + ", ".join(details)
        )
    return lines


def _draw_wrapped(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int],
    text: str,
    *,
    width_chars: int,
    font,
    fill,
    line_gap: int = 4,
) -> int:
    words = text.split()
    lines: list[str] = []
    current = ""
    for word in words:
        candidate = f"{current} {word}".strip()
        if len(candidate) > width_chars and current:
            lines.append(current)
            current = word
        else:
            current = candidate
    if current:
        lines.append(current)
    x, y = xy
    for line in lines:
        draw.text((x, y), line, font=font, fill=fill)
        y += font.size + line_gap
    return y


def render(
    state_dir: Path,
    output: Path,
    *,
    live_results: Path | None = None,
) -> None:
    artifact_dir = state_dir / "artifacts"
    results = _load_results(live_results)
    canvas = Image.new("RGB", CANVAS, (10, 16, 24))
    draw = ImageDraw.Draw(canvas)
    draw.text(
        (MARGIN, 24),
        "GPSR VLM · ten-case hardware-free validation",
        font=_font(34, bold=True),
        fill=(247, 250, 252),
    )
    draw.text(
        (MARGIN, 70),
        "Each card: front camera · wrist camera · navigation map · scene-only Tinker/xArm render",
        font=_font(18),
        fill=(157, 174, 191),
    )

    labels = ("FRONT", "WRIST", "MAP", "ARM")
    for index, case in enumerate(SCENARIO_CASES):
        column = index % 2
        row = index // 2
        x = MARGIN + column * (CARD[0] + GAP)
        y = HEADER + row * (CARD[1] + GAP)
        stage_id = case.stages[0].stage_id
        case_results = [
            item
            for item in results
            if item.get("case_number") == case.number
        ]
        passed = bool(case_results) and all(
            item.get("passed") for item in case_results
        )
        border = (
            (47, 184, 119)
            if passed
            else (218, 153, 55) if not case_results else (220, 71, 86)
        )
        draw.rounded_rectangle(
            (x, y, x + CARD[0], y + CARD[1]),
            radius=14,
            fill=(21, 29, 39),
            outline=border,
            width=4,
        )
        draw.text(
            (x + 18, y + 14),
            f"CASE {case.number:02d} · {case.title}",
            font=_font(21, bold=True),
            fill=(245, 248, 250),
        )
        status = "PASS" if passed else "NOT RUN" if not case_results else "FAIL"
        draw.text(
            (x + CARD[0] - 110, y + 16),
            status,
            font=_font(18, bold=True),
            fill=border,
        )
        paths = (
            _asset(artifact_dir, stage_id, "front_camera"),
            _asset(artifact_dir, stage_id, "wrist_camera"),
            _asset(artifact_dir, stage_id, "map"),
            _asset(artifact_dir, stage_id, "arm"),
        )
        for thumb_index, (label, path) in enumerate(zip(labels, paths)):
            thumb_x = x + 18 + thumb_index * (THUMB[0] + 14)
            thumb_y = y + 58
            draw.text(
                (thumb_x, thumb_y),
                label,
                font=_font(13, bold=True),
                fill=(154, 184, 211),
            )
            canvas.paste(_fit(path), (thumb_x, thumb_y + 21))
        text_y = y + 250
        text_y = _draw_wrapped(
            draw,
            (x + 18, text_y),
            "EXPECTED · " + _expected(case),
            width_chars=108,
            font=_font(14, bold=True),
            fill=(224, 231, 237),
        )
        for line in _observed_lines(case, results):
            text_y = _draw_wrapped(
                draw,
                (x + 18, text_y + 5),
                line,
                width_chars=115,
                font=_font(13),
                fill=(163, 181, 198),
                line_gap=3,
            )
        provenance = (
            "generated, frozen + checksummed"
            if case.generated
            else "real log / deterministic render"
        )
        draw.text(
            (x + 18, y + CARD[1] - 28),
            (
                f"{case.arm_pose_name} · map={case.map_pose_name} · "
                f"{provenance}"
            ),
            font=_font(12),
            fill=(116, 139, 160),
        )
    draw.text(
        (MARGIN, CANVAS[1] - 36),
        "Case 08 is the only deliberate camera mismatch. Arm images contain no right-side dashboard.",
        font=_font(16, bold=True),
        fill=(175, 190, 204),
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state-dir", type=Path, required=True)
    parser.add_argument("--live-results", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    render(
        args.state_dir,
        args.output,
        live_results=args.live_results,
    )
    print(args.output)


if __name__ == "__main__":
    main()
