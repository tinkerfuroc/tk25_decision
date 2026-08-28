# tools/gpsr_ui/frames.py
"""Per-run camera frame listing and traversal-safe file resolution.

Frame filenames encode a SIMULATOR-clock millisecond stamp
(`NNNN_<ms>.jpg`), not wall time. Sim and wall clocks are not
proportional -- real-time factor drifts roughly 0.2-0.5 within a run --
so a frame's wall time must come from `Clock.sim_to_wall`, never from
scaling the filename's stamp directly.

Both `list_frames` and `frame_path` build filesystem paths out of names
that were not necessarily placed there by the bench itself: `frame_path`
takes two request-supplied strings (`label`, `file`) directly, and
`list_frames` walks whatever directory entries actually exist on disk,
which -- should the corpus we don't control ever contain a symlink, via a
recording bug or a bad archive extraction -- could include a label
directory or a frame file that is not really where it appears to be.
Round-1 review found that `list_frames` trusted `iterdir()` unconditionally
while `frame_path` did not, so a symlinked label directory or frame file
was *listed* (leaking another run's real frame names, or a label that
doesn't really belong to this run) even though `frame_path` would still
correctly refuse to *serve* it. That asymmetry -- two functions in one
module disagreeing about what counts as a valid frame -- is itself a
defect: it is exactly how a future change could reintroduce a real hole
on the serving side. Both functions now share one validator,
`_resolve_child`, and neither trusts a directory entry until it passes
the same positive check.

Nothing here is a blacklist ("reject if it contains '..' or starts with
'/'"): a blacklist misses encoded traversal, and it misses pathlib's own
`/` operator silently discarding the left-hand path when the right-hand
one is absolute (`Path("/a/b") / "/etc/passwd" == Path("/etc/passwd")`,
no ".." in sight). Instead every candidate -- whether it came from a
request or from `iterdir()` -- is resolved against the real filesystem
and checked positively: does it land exactly one hop below where it is
supposed to, given what genuinely exists on disk. A candidate that fails
that check is rejected regardless of what its literal characters, or its
apparent directory listing, looked like.
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


def _resolve_child(parent: Path, name: str, *, want_dir: bool) -> Path | None:
    """Resolve `parent / name` and return it only if it is a real entry
    whose immediate parent, once both sides are resolved on the real
    filesystem, is genuinely `parent` -- not a symlink escape (to
    anywhere: outside the run, or into another run's frames), not an
    absolute-path substitution, not a multi-hop traversal, and not a
    non-regular file. `parent` must already be a resolved, real
    directory. Shared by `list_frames` (validating what `iterdir()`
    reports) and `frame_path` (validating what a request claims).
    """
    try:
        resolved = (parent / name).resolve(strict=True)
    except (OSError, ValueError):
        return None
    if resolved.parent != parent:
        return None
    if want_dir:
        if not resolved.is_dir():
            return None
    elif not resolved.is_file():
        return None
    return resolved


def list_frames(run_dir: Path) -> dict[str, list[FrameRef]]:
    """List every camera's frames for a run, sorted by sim stamp.

    Returns {} if the run has no `frames/` directory at all, and omits
    any label sub-directory that has no parseable frame files. A run may
    have only one camera (e.g. the real s2026-003-findObjInRoom has
    `frames/head` but no `frames/arena`) -- that is normal, not an error.

    Every label directory and every frame file is validated with
    `_resolve_child` before being trusted: a symlinked label directory or
    frame file -- wherever it points, including at another run's
    `frames/` tree -- is silently skipped rather than listed, matching
    what `frame_path` would (refuse to) serve.

    The bench reuses a run directory in place when it re-runs a corpus
    entry, and does not archive the old occupant first. Frame filenames
    encode the SIMULATOR clock, not wall time, so a re-run does not
    overwrite the previous occupant's frame files -- it interleaves new
    ones alongside them on disk. `frames/index.jsonl`, when present, is
    the current run's own record of exactly which files belong to it; a
    directory listing is not. So when the index exists, only files it
    names are returned -- everything else on disk under that label is an
    orphan from a previous occupancy and is silently excluded. When no
    index exists (a run from before the index was introduced), there is
    no way to distinguish an orphan from a real frame, so every
    parseable file on disk is returned, same as before.
    """
    run_dir = Path(run_dir)
    frames_dir = run_dir / "frames"
    try:
        frames_root = frames_dir.resolve(strict=True)
    except (OSError, ValueError):
        return {}
    if not frames_root.is_dir():
        return {}
    clock = load_clock(run_dir)

    try:
        label_names = sorted(p.name for p in frames_dir.iterdir())
    except OSError:
        return {}

    out: dict[str, list[FrameRef]] = {}
    for label in label_names:
        label_dir = _resolve_child(frames_root, label, want_dir=True)
        if label_dir is None:
            continue
        try:
            file_names = sorted(f.name for f in label_dir.iterdir())
        except OSError:
            continue

        # When the current run has an exact index, it is the sole
        # source of truth for which on-disk files are this run's own --
        # anything else is an orphan from a previous occupancy of this
        # directory. No index means no way to tell, so keep every
        # parseable file (the pre-index behaviour for older runs).
        indexed = clock.indexed_files(label)

        refs: list[FrameRef] = []
        for name in file_names:
            if indexed is not None and name not in indexed:
                continue
            parsed = parse_frame_name(name)
            if parsed is None:
                continue
            if _resolve_child(label_dir, name, want_dir=False) is None:
                continue
            index, stamp_s = parsed
            refs.append(FrameRef(
                index=index,
                stamp_s=stamp_s,
                file=name,
                wall=clock.sim_to_wall(label, stamp_s),
            ))
        if refs:
            out[label] = sorted(refs, key=lambda r: r.stamp_s)
    return out


def frame_path(run_dir: Path, label: str, file: str) -> Path | None:
    """Resolve a single frame file, or None if it is not a real frame
    genuinely inside `run_dir/frames/<label>/`.

    Validation is entirely positive, in two independent layers:

    1. `file` must match the frame filename grammar (`^\\d+_\\d+\\.jpg$`,
       via `parse_frame_name`). That grammar alone excludes "/", "..",
       null bytes and absolute paths, since nothing but ASCII digits, one
       underscore and the literal ".jpg" suffix can appear.
    2. Both `label` and `file` are checked with `_resolve_child` (the same
       validator `list_frames` uses), which resolves each against the
       real filesystem and requires it to land exactly one hop below
       where it is supposed to be -- not merely "resolves to something
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

    label_dir = _resolve_child(frames_root, label, want_dir=True)
    if label_dir is None:
        return None

    return _resolve_child(label_dir, file, want_dir=False)
