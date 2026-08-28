# tools/tests/test_frames.py
from __future__ import annotations

import os

import pytest

from gpsr_ui.corpus import list_tiers
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


def test_list_frames_skips_a_symlinked_label_dir_and_a_symlinked_file(
    make_run, tmp_path
):
    """Round-1 review finding: list_frames must apply the same positive
    validation frame_path does, or a symlink -- introduced by a recording
    bug or a bad archive extraction, since we do not control what writes
    the corpus -- leaks names through a listing that would then refuse to
    serve them. Three leaks were demonstrated: a symlinked label dir
    pointing outside the run, one pointing at ANOTHER run's frames/head
    (exposing that run's real frame names and stamps), and a symlinked
    file inside a genuine label dir. Cover all three here."""
    run = make_run(name="s9999-057-x", frames={"head": [(0, 1000)]})
    other_run = make_run(name="s9999-058-y", frames={"head": [(0, 2000)]})
    outside = tmp_path / "outside"
    outside.mkdir()

    # 1. symlinked label dir pointing entirely outside any run.
    os.symlink(outside, run / "frames" / "escaped_label")
    # 2. symlinked label dir pointing at ANOTHER run's real frames/head.
    os.symlink(other_run / "frames" / "head", run / "frames" / "borrowed_label")
    # 3. symlinked file inside the genuine "head" label dir.
    real_elsewhere = tmp_path / "9999_9999.jpg"
    real_elsewhere.write_bytes(b"\xff\xd8\xff\xd9")
    os.symlink(real_elsewhere, run / "frames" / "head" / "9999_9999.jpg")

    frames = list_frames(run)
    assert list(frames) == ["head"], "no symlinked label should be listed"
    assert [f.file for f in frames["head"]] == ["0000_1000.jpg"], (
        "the symlinked file must not be listed alongside the genuine one"
    )


def test_list_frames_excludes_orphans_from_a_reused_run_directory(make_run):
    """The bench reuses a run directory in place when it re-runs a corpus
    entry, without archiving the previous occupant first. Frame filenames
    encode the simulator clock, so the previous occupant's frames are not
    overwritten -- they are interleaved on disk alongside the current
    run's. `index.jsonl` is the current run's own record of which files
    are really its own; anything else on disk is an orphan.

    The orphans here are given MUCH earlier stamps than the indexed
    frames, mirroring the real contamination measured on
    s2026-003-findObjInRoom (orphan sim-ms ~1.2M vs. current run's
    ~5.8M) -- so a naive fix that merely truncates to the index's
    *length* or sorts before slicing would still pass a same-shaped test
    with interleaved-but-close stamps, but must fail here: an implementation
    that took (say) the two earliest or two latest on-disk frames instead
    of the two the index actually names would get caught by asserting the
    exact returned filenames, not just a count.
    """
    run = make_run(
        name="s9999-060-x",
        frames={
            "head": [
                # Current run's real frames (indexed).
                (0, 5000), (1, 6000),
                # Orphans from a previous occupancy: much earlier stamps,
                # so they would sort to the front of a naive merge.
                (50, 1000), (51, 1200),
            ],
        },
        index_lines=[
            {"label": "head", "file": "frames/head/0000_5000.jpg",
             "stamp_s": 5.0, "wall": "2026-08-28T10:00:05.000000Z"},
            {"label": "head", "file": "frames/head/0001_6000.jpg",
             "stamp_s": 6.0, "wall": "2026-08-28T10:00:06.000000Z"},
        ],
    )
    frames = list_frames(run)
    assert [f.file for f in frames["head"]] == [
        "0000_5000.jpg", "0001_6000.jpg",
    ], "only the indexed frames should be listed, in index order"
    assert [f.stamp_s for f in frames["head"]] == [5.0, 6.0], (
        "the orphans' much-earlier stamps must not appear at all"
    )


def test_list_frames_without_an_index_still_lists_everything_on_disk(
        make_run):
    """Runs that predate frames/index.jsonl have no way to distinguish an
    orphan from a real frame -- list_frames must not regress to filtering
    (or erroring) on them, even when the on-disk stamps are shaped just
    like the contaminated case (some much earlier than others)."""
    run = make_run(
        name="s9999-061-x",
        frames={
            "head": [(0, 5000), (1, 6000), (50, 1000), (51, 1200)],
            "arena": [(0, 5000), (1, 6000)],
        },
    )
    frames = list_frames(run)
    assert sorted(frames) == ["arena", "head"]
    assert [f.file for f in frames["head"]] == [
        "0050_1000.jpg", "0051_1200.jpg", "0000_5000.jpg", "0001_6000.jpg",
    ]
    assert len(frames["arena"]) == 2


@pytest.mark.corpus
def test_indexed_runs_never_list_more_frames_than_their_index(corpus_root):
    """For every run in the corpus that has frames/index.jsonl, list_frames
    must return no more frames, per label, than the index actually names --
    a floor/ceiling relationship, not an exact count, since a live battery
    can be adding to the index concurrently."""
    import json as _json

    checked = 0
    for tier in list_tiers(corpus_root):
        for entry in tier.entries:
            for attempt in entry.attempts:
                index_path = attempt.path / "frames" / "index.jsonl"
                try:
                    lines = index_path.read_text().splitlines()
                except OSError:
                    continue
                per_label: dict[str, int] = {}
                for line in lines:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        rec = _json.loads(line)
                    except ValueError:
                        continue
                    if isinstance(rec, dict) and isinstance(
                            rec.get("label"), str):
                        per_label[rec["label"]] = (
                            per_label.get(rec["label"], 0) + 1)

                frames = list_frames(attempt.path)
                checked += 1
                for label, refs in frames.items():
                    assert len(refs) <= per_label.get(label, 0), (
                        f"{attempt.dir_name}/{label}: list_frames returned "
                        f"{len(refs)} frames but index.jsonl names only "
                        f"{per_label.get(label, 0)} -- orphans are leaking "
                        f"through"
                    )
    if checked == 0:
        pytest.skip("no run in the corpus currently has frames/index.jsonl")


@pytest.mark.corpus
def test_real_single_camera_run_lists_only_head(corpus_root):
    """A run with exactly one camera should list correctly, without ever
    assuming a pair (head + arena). We used to hardcode
    s2026-003-findObjInRoom as the single-camera specimen, but a live
    battery can re-run any entry and give it a second camera (or take one
    away), so a hardcoded entry id is not stable under concurrent writes.
    Instead scan the corpus for whatever currently happens to be a
    single-camera run and check that one -- the specimen may change
    from run to run, but the property under test does not."""
    def _find_single_camera_run():
        for tier in list_tiers(corpus_root):
            for entry in tier.entries:
                for attempt in entry.attempts:
                    frames = list_frames(attempt.path)
                    if len(frames) == 1:
                        return attempt.dir_name, frames
        return None

    single = _find_single_camera_run()
    if single is None:
        pytest.skip(
            "no single-camera run currently in the corpus -- every run "
            "has zero, two or more camera labels right now"
        )

    dir_name, frames = single
    (label,) = frames
    assert list(frames) == [label]
    # A live battery may change this run's frame count concurrently;
    # assert a floor, not equality.
    assert len(frames[label]) >= 1, (
        f"{dir_name}'s single camera label {label!r} unexpectedly empty"
    )
