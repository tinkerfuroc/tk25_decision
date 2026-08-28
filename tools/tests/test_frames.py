# tools/tests/test_frames.py
from __future__ import annotations

from gpsr_ui.frames import frame_path, list_frames


def test_frames_are_listed_per_label_in_stamp_order(make_run):
    run = make_run(
        name="s9999-050-x",
        frames={"head": [(1, 2000), (0, 1000)], "arena": [(0, 1000)]},
    )
    frames = list_frames(run)
    assert sorted(frames) == ["arena", "head"]
    assert [f.stamp_s for f in frames["head"]] == [1.0, 2.0]
    assert frames["head"][0].file == "0000_1000.jpg"


def test_frames_carry_wall_times_when_the_clock_is_exact(make_run):
    run = make_run(
        name="s9999-051-x",
        frames={"head": [(0, 1000)]},
        index_lines=[{"label": "head", "file": "0000_1000.jpg",
                      "stamp_s": 1.0, "wall": "2026-08-28T10:00:00.000000Z"}],
    )
    assert list_frames(run)["head"][0].wall is not None


def test_a_run_with_only_one_camera_is_fine(make_run):
    """s2026-003-findObjInRoom in the real corpus has head but no arena."""
    run = make_run(name="s9999-052-x", frames={"head": [(0, 1000)]})
    assert list(list_frames(run)) == ["head"]


def test_a_run_with_no_frames_yields_an_empty_mapping(make_run):
    assert list_frames(make_run(name="s9999-053-x")) == {}


def test_frame_path_refuses_traversal(make_run):
    run = make_run(name="s9999-054-x", frames={"head": [(0, 1000)]})
    assert frame_path(run, "head", "0000_1000.jpg") is not None
    assert frame_path(run, "head", "../../run.json") is None
    assert frame_path(run, "../head", "0000_1000.jpg") is None
    assert frame_path(run, "head", "nope.jpg") is None
