# tools/tests/test_clock.py
from __future__ import annotations

import json

from gpsr_ui.clock import load_clock, parse_frame_name, parse_wall


def test_parse_frame_name_reads_index_and_sim_seconds():
    assert parse_frame_name("0042_2123833.jpg") == (42, 2123.833)
    assert parse_frame_name("0000_2081833.jpg") == (0, 2081.833)
    assert parse_frame_name("not-a-frame.jpg") is None
    assert parse_frame_name("0001.jpg") is None


def test_parse_wall_handles_the_z_suffix_used_in_events():
    assert parse_wall("2026-08-28T10:52:30.735414Z") == parse_wall(
        "2026-08-28T10:52:30.735414+00:00")
    assert parse_wall("nonsense") is None


def test_exact_mode_when_index_jsonl_present(make_run):
    run = make_run(
        name="s9999-010-x",
        frames={"head": [(0, 1000), (1, 2000)]},
        index_lines=[
            {"label": "head", "file": "0000_1000.jpg",
             "stamp_s": 1.0, "wall": "2026-08-28T10:00:00.000000Z"},
            {"label": "head", "file": "0001_2000.jpg",
             "stamp_s": 2.0, "wall": "2026-08-28T10:00:05.000000Z"},
        ],
    )
    clock = load_clock(run)
    assert clock.mode == "exact"
    assert clock.labels == ["head"]
    # Exact join: 1 sim-second maps to 5 wall-seconds here (RTF 0.2).
    assert clock.sim_to_wall("head", 2.0) == parse_wall(
        "2026-08-28T10:00:05.000000Z")
    # Query wall (10:00:04) is 1s *before* frame 1 was captured (10:00:05),
    # so the at-or-before frame is still frame 0 (10:00:00). Returning
    # frame 1 here would show a frame from the future relative to the
    # query, which is the exact failure mode this module exists to avoid.
    assert clock.wall_to_frame(
        "head", parse_wall("2026-08-28T10:00:04.000000Z")) == "0000_1000.jpg"


def test_approximate_mode_interpolates_from_recorder_meta(make_run):
    run = make_run(
        name="s9999-011-x",
        frames={"head": [(0, 1000), (1, 2000), (2, 3000)]},
        recorder_meta={
            "labels": {"head": {
                "frames": 3, "first_stamp": 1.0, "last_stamp": 3.0}},
            "started_wall": "2026-08-28T10:00:00+00:00",
            "ended_wall": "2026-08-28T10:00:10+00:00",
        },
    )
    clock = load_clock(run)
    assert clock.mode == "approximate"
    # Midpoint of a 2 sim-second span across 10 wall-seconds.
    assert clock.sim_to_wall("head", 2.0) == parse_wall(
        "2026-08-28T10:00:05+00:00")


def test_no_metadata_yields_mode_none_rather_than_a_crash(make_run):
    run = make_run(name="s9999-012-x", frames={"head": [(0, 1000)]})
    clock = load_clock(run)
    assert clock.mode == "none"
    assert clock.sim_to_wall("head", 1.0) is None


def test_index_jsonl_wins_over_recorder_meta(make_run):
    """Both present: the exact join must be preferred."""
    run = make_run(
        name="s9999-013-x",
        frames={"head": [(0, 1000)]},
        index_lines=[{"label": "head", "file": "0000_1000.jpg",
                      "stamp_s": 1.0, "wall": "2026-08-28T10:00:00.000000Z"}],
        recorder_meta={
            "labels": {"head": {
                "frames": 1, "first_stamp": 1.0, "last_stamp": 1.0}},
            "started_wall": "2026-08-28T09:00:00+00:00",
            "ended_wall": "2026-08-28T09:00:10+00:00",
        },
    )
    clock = load_clock(run)
    assert clock.mode == "exact"
    assert clock.sim_to_wall("head", 1.0) == parse_wall(
        "2026-08-28T10:00:00.000000Z")


def test_wall_to_frame_before_first_frame_is_none_not_the_first_frame(
        make_run):
    """A query earlier than every known frame has no at-or-before frame.

    Clamping to the first frame here would return a frame from the
    future relative to the query -- the same bug class as the brief's
    original (corrected) test assertion, just on the wall_to_frame side.
    Every run emits run.started/run.configured before the first camera
    frame exists, so this is the common case, not a corner case.
    """
    run = make_run(
        name="s9999-014-x",
        frames={"head": [(0, 1000), (1, 2000)]},
        index_lines=[
            {"label": "head", "file": "0000_1000.jpg",
             "stamp_s": 1.0, "wall": "2026-08-28T10:00:00.000000Z"},
            {"label": "head", "file": "0001_2000.jpg",
             "stamp_s": 2.0, "wall": "2026-08-28T10:00:05.000000Z"},
        ],
    )
    clock = load_clock(run)
    assert clock.mode == "exact"
    # Before the first frame's wall time: no frame is at-or-before it.
    assert clock.wall_to_frame(
        "head", parse_wall("2026-08-28T09:59:59.000000Z")) is None
    # After the last frame's wall time: the last frame genuinely IS
    # at-or-before it. This end is not symmetric with the first.
    assert clock.wall_to_frame(
        "head", parse_wall("2026-08-28T10:05:00.000000Z")) == "0001_2000.jpg"


def test_approximate_sim_to_wall_clamps_past_last_stamp(make_run):
    """A stamp past last_stamp must clamp to ended_wall, not extrapolate.

    recorder-meta.json is routinely half-written on a live run, so
    first_stamp/last_stamp can be stale relative to frames already on
    disk. Exact mode's sibling _interpolate() already clamps at both
    ends; approximate mode must match that, not extrapolate unbounded
    past the end of the run.
    """
    run = make_run(
        name="s9999-015-x",
        frames={"head": [(0, 1000), (1, 2000), (2, 3000)]},
        recorder_meta={
            "labels": {"head": {
                "frames": 3, "first_stamp": 1.0, "last_stamp": 3.0}},
            "started_wall": "2026-08-28T10:00:00+00:00",
            "ended_wall": "2026-08-28T10:00:10+00:00",
        },
    )
    clock = load_clock(run)
    assert clock.mode == "approximate"
    # 10.0 sim-seconds is well past last_stamp (3.0); must clamp to
    # ended_wall, not extrapolate 35 seconds past the run's end.
    assert clock.sim_to_wall("head", 10.0) == parse_wall(
        "2026-08-28T10:00:10+00:00")
    # Symmetric clamp before first_stamp, for good measure.
    assert clock.sim_to_wall("head", -5.0) == parse_wall(
        "2026-08-28T10:00:00+00:00")
