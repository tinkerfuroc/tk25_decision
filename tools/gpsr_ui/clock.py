# tools/gpsr_ui/clock.py
"""Map between the sim clock (frame filenames) and wall time (events).

The two are NOT proportionally related: real-time factor varies between
roughly 0.2 and 0.5 within a single run. Frames therefore cannot be placed
on the event timeline by scaling. Two strategies exist:

  exact        join on frames/index.jsonl, which carries stamp_s and wall
               per frame. Correct.
  approximate  linear interpolation between recorder-meta.json's
               started_wall/ended_wall and first_stamp/last_stamp. Wrong
               mid-run by seconds wherever RTF deviates from its average.

The mode is surfaced so the UI can badge it. An approximate alignment must
never be mistakable for a real one.
"""
from __future__ import annotations

import bisect
import json
import re
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

_FRAME_RE = re.compile(r"^(\d+)_(\d+)\.jpg$")


def parse_frame_name(name: str) -> tuple[int, float] | None:
    m = _FRAME_RE.match(name)
    if m is None:
        return None
    return int(m.group(1)), int(m.group(2)) / 1000.0


def parse_wall(iso: str | None) -> float | None:
    if not isinstance(iso, str) or not iso:
        return None
    try:
        return datetime.fromisoformat(iso.replace("Z", "+00:00")).timestamp()
    except ValueError:
        return None


@dataclass
class Clock:
    mode: str = "none"
    labels: list[str] = field(default_factory=list)
    # label -> (sorted sim stamps, parallel wall times, parallel filenames)
    _stamps: dict[str, list[float]] = field(default_factory=dict)
    _walls: dict[str, list[float]] = field(default_factory=dict)
    _files: dict[str, list[str]] = field(default_factory=dict)
    # label -> (first_stamp, last_stamp, started_wall, ended_wall)
    _linear: dict[str, tuple[float, float, float, float]] = field(
        default_factory=dict)

    def sim_to_wall(self, label: str, stamp_s: float) -> float | None:
        if self.mode == "exact":
            stamps = self._stamps.get(label)
            walls = self._walls.get(label)
            if not stamps or not walls:
                return None
            i = bisect.bisect_left(stamps, stamp_s)
            if i < len(stamps) and stamps[i] == stamp_s:
                return walls[i]
            return _interpolate(stamps, walls, stamp_s)
        if self.mode == "approximate":
            span = self._linear.get(label)
            if span is None:
                return None
            first, last, started, ended = span
            if last == first:
                return started
            if stamp_s <= first:
                return started
            if stamp_s >= last:
                return ended
            frac = (stamp_s - first) / (last - first)
            return started + frac * (ended - started)
        return None

    def indexed_files(self, label: str) -> frozenset[str] | None:
        """Bare filenames `index.jsonl` lists for `label`, or None if
        there is no exact index for this run (mode != "exact") -- i.e.
        there is no way to tell an indexed frame from an orphan left by
        a previous occupancy of this run directory. When the index
        exists but has no rows at all for `label`, returns an empty
        frozenset: every on-disk file under that label is an orphan.
        """
        if self.mode != "exact":
            return None
        return frozenset(self._files.get(label, ()))

    # There used to be a `wall_to_frame` here (filename of the frame
    # nearest at-or-before a given wall time). It had no caller anywhere
    # in this app -- the frontend never round-trips to the server to
    # scrub frames, it holds the raw `frames/index.jsonl` rows client-side
    # (from /api/frames) and resolves at-or-before locally, in
    # `static/frames.js`'s `frameAt`. That JS function is the sole live
    # implementation of "which frame is at-or-before this moment" and the
    # sole place to fix the next bug in that rule; a Python copy that
    # nothing calls would just be a decoy for whoever goes looking next.
    # Removed rather than wired up: the whole point of shipping the raw
    # arrays to the client is so scrubbing needs no round trip, which
    # rules out routing this through the API instead.


def _interpolate(xs: list[float], ys: list[float], x: float) -> float | None:
    if not xs:
        return None
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    i = bisect.bisect_left(xs, x)
    x0, x1 = xs[i - 1], xs[i]
    y0, y1 = ys[i - 1], ys[i]
    if x1 == x0:
        return y0
    return y0 + (x - x0) / (x1 - x0) * (y1 - y0)


def _scan_frames(run_dir: Path) -> dict[str, list[tuple[float, str]]]:
    out: dict[str, list[tuple[float, str]]] = {}
    frames_dir = run_dir / "frames"
    if not frames_dir.is_dir():
        return out
    for label_dir in sorted(p for p in frames_dir.iterdir() if p.is_dir()):
        items: list[tuple[float, str]] = []
        for f in label_dir.iterdir():
            parsed = parse_frame_name(f.name)
            if parsed is not None:
                items.append((parsed[1], f.name))
        if items:
            out[label_dir.name] = sorted(items)
    return out


def _load_exact(run_dir: Path, clock: Clock) -> bool:
    """Load frames/index.jsonl into `clock`, returning True whenever the
    file itself EXISTS -- independent of whether any of its lines parsed
    into a usable row.

    The bench reuses a run directory in place on a re-run without
    archiving the previous occupant, so a fresh index.jsonl can exist
    (created, possibly truncated to zero bytes) for a window before its
    first line is flushed -- exactly the moment a live dashboard is most
    likely to be looking. Treating "index exists but has zero valid
    rows" the same as "no index file at all" would fall through to an
    unfiltered directory scan and leak the previous occupant's orphan
    frames. So an existing-but-empty (or all-garbage) index must still
    flip mode to "exact", just with empty per-label rows -- `indexed_files`
    then correctly reports "no indexed frames yet" for every label,
    rather than "no way to tell, show everything on disk".
    """
    path = run_dir / "frames" / "index.jsonl"
    try:
        text = path.read_text()
    except OSError:
        return False  # no index file at all: caller may fall back

    rows: dict[str, list[tuple[float, float, str]]] = {}
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rec = json.loads(line)
        except ValueError:
            continue  # a live run may have a torn final line
        if not isinstance(rec, dict):
            continue
        label, file = rec.get("label"), rec.get("file")
        stamp, wall = rec.get("stamp_s"), parse_wall(rec.get("wall"))
        if not isinstance(label, str) or not isinstance(file, str):
            continue
        if not isinstance(stamp, (int, float)) or wall is None:
            continue
        # `file` is a run-relative path (e.g. "frames/head/0000_1.jpg"),
        # not a bare filename -- frames.py and static/frames.js both deal
        # in bare filenames, so normalize here, once, at the source.
        rows.setdefault(label, []).append(
            (float(stamp), wall, Path(file).name))

    # The file existing (even empty, even all-garbage) is itself the
    # signal that this run has an exact index and is not pre-index-era.
    for label, items in rows.items():
        items.sort()
        clock._stamps[label] = [i[0] for i in items]
        clock._walls[label] = [i[1] for i in items]
        clock._files[label] = [i[2] for i in items]
    clock.mode = "exact"
    clock.labels = sorted(rows)
    return True


def _load_approximate(run_dir: Path, clock: Clock) -> bool:
    try:
        meta = json.loads((run_dir / "recorder-meta.json").read_text())
    except (OSError, ValueError):
        return False
    started = parse_wall(meta.get("started_wall"))
    ended = parse_wall(meta.get("ended_wall"))
    labels = meta.get("labels")
    if started is None or ended is None or not isinstance(labels, dict):
        return False

    scanned = _scan_frames(run_dir)
    ok = False
    for label, info in labels.items():
        if not isinstance(info, dict):
            continue
        first, last = info.get("first_stamp"), info.get("last_stamp")
        if not isinstance(first, (int, float)):
            continue
        if not isinstance(last, (int, float)):
            continue
        clock._linear[label] = (float(first), float(last), started, ended)
        items = scanned.get(label, [])
        clock._stamps[label] = [i[0] for i in items]
        clock._files[label] = [i[1] for i in items]
        ok = True

    if ok:
        clock.mode = "approximate"
        clock.labels = sorted(clock._linear)
    return ok


def load_clock(run_dir: Path) -> Clock:
    clock = Clock()
    if _load_exact(run_dir, clock):
        return clock
    if _load_approximate(run_dir, clock):
        return clock
    scanned = _scan_frames(run_dir)
    clock.labels = sorted(scanned)
    for label, items in scanned.items():
        clock._stamps[label] = [i[0] for i in items]
        clock._files[label] = [i[1] for i in items]
    return clock
