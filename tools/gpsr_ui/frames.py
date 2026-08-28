# tools/gpsr_ui/frames.py
"""Per-run camera frame listing and traversal-safe file resolution.

Frame filenames encode a SIMULATOR-clock millisecond stamp
(`NNNN_<ms>.jpg`), not wall time. Sim and wall clocks are not
proportional -- real-time factor drifts roughly 0.2-0.5 within a run --
so a frame's wall time must come from `Clock.sim_to_wall`, never from
scaling the filename's stamp directly.

`frame_path` is the highest-risk function in this project: unlike every
route in `corpus.py`/`app.py`, which only ever compares strings against
names discovered by a real directory walk, it builds a filesystem path
out of two request-supplied strings (`label`, `file`). It is deliberately
NOT a blacklist ("reject if it contains '..' or starts with '/'"): a
blacklist misses encoded traversal, and it misses pathlib's own `/`
operator silently discarding the left-hand path when the right-hand one
is absolute (`Path("/a/b") / "/etc/passwd" == Path("/etc/passwd")`, no
".." in sight). Instead every candidate is resolved against the real
filesystem and then checked positively: does it land exactly one
filesystem hop below where it is supposed to, given what really exists
on disk. A candidate that fails that check is rejected regardless of
what its literal characters looked like.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .clock import load_clock, parse_frame_name


@dataclass(frozen=True)
class FrameRef:
    index: int
    stamp_s: float
    file: str
    wall: float | None


def list_frames(run_dir: Path) -> dict[str, list[FrameRef]]:
    """List every camera's frames for a run, sorted by sim stamp.

    Returns {} if the run has no `frames/` directory at all, and omits
    any label sub-directory that has no parseable frame files. A run may
    have only one camera (e.g. the real s2026-003-findObjInRoom has
    `frames/head` but no `frames/arena`) -- that is normal, not an error.
    """
    run_dir = Path(run_dir)
    frames_dir = run_dir / "frames"
    if not frames_dir.is_dir():
        return {}
    clock = load_clock(run_dir)

    out: dict[str, list[FrameRef]] = {}
    for label_dir in sorted(p for p in frames_dir.iterdir() if p.is_dir()):
        refs: list[FrameRef] = []
        for f in label_dir.iterdir():
            parsed = parse_frame_name(f.name)
            if parsed is None:
                continue
            index, stamp_s = parsed
            refs.append(FrameRef(
                index=index,
                stamp_s=stamp_s,
                file=f.name,
                wall=clock.sim_to_wall(label_dir.name, stamp_s),
            ))
        if refs:
            out[label_dir.name] = sorted(refs, key=lambda r: r.stamp_s)
    return out


def frame_path(run_dir: Path, label: str, file: str) -> Path | None:
    """Resolve a single frame file, or None if it is not a real frame
    genuinely inside `run_dir/frames/<label>/`.

    Validation is entirely positive, in two independent layers:

    1. `file` must match the frame filename grammar (`^\\d+_\\d+\\.jpg$`,
       via `parse_frame_name`). That grammar alone excludes "/", "..",
       null bytes and absolute paths, since nothing but ASCII digits, one
       underscore and the literal ".jpg" suffix can appear.
    2. Both `label` and `file` are resolved against the real filesystem
       (`Path.resolve(strict=True)`, which follows symlinks) and the
       result is required to land exactly one hop below the real
       `frames/` directory (for `label`) or exactly inside the real
       label directory (for `file`) -- not merely "resolves to something
       under frames_dir.parents", which encoded traversal or a symlink
       could still satisfy incidentally, but the literal immediate
       parent.
    """
    if parse_frame_name(file) is None:
        return None

    frames_dir = Path(run_dir) / "frames"
    try:
        frames_root = frames_dir.resolve(strict=True)
    except (OSError, ValueError):
        return None

    try:
        label_dir = (frames_dir / label).resolve(strict=True)
    except (OSError, ValueError):
        return None
    if label_dir.parent != frames_root or not label_dir.is_dir():
        return None

    try:
        resolved = (label_dir / file).resolve(strict=True)
    except (OSError, ValueError):
        return None
    if resolved.parent != label_dir or not resolved.is_file():
        return None
    return resolved
