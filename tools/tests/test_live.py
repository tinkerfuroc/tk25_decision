# tools/tests/test_live.py
from __future__ import annotations

import json
import os
import time

from gpsr_ui.clock import parse_wall
from gpsr_ui.live import (
    find_in_flight,
    find_progress_failures,
    live_summary,
    tail_events,
)


def test_a_run_without_run_json_is_in_flight(make_run):
    run = make_run(name="s9999-060-x", verdict=None, finished=False)
    found = find_in_flight(run.parents[2])
    assert [f.dir_name for f in found] == ["s9999-060-x"]


def test_a_finished_run_is_not_in_flight(make_run):
    run = make_run(name="s9999-061-x", verdict="PASS")
    assert find_in_flight(run.parents[2]) == []


def test_a_stale_run_without_run_json_is_dead_not_live(make_run):
    """A crashed run never gets its run.json; staleness is the guard."""
    run = make_run(name="s9999-062-x", verdict=None, finished=False)
    future = time.time() + 3600
    assert find_in_flight(run.parents[2], stale_after=60.0, now=future) == []


def test_a_reused_directory_with_a_newer_event_log_is_in_flight(make_run):
    """The core correction over the brief: the bench reuses a run
    directory in place on a re-run and does NOT archive the previous
    occupant's run.json. So a directory can have a run.json (from the
    PREVIOUS run) while a brand new events.jsonl -- newer than that
    run.json -- is actively being written for the run in progress right
    now. Absence-of-run.json alone (the brief's rule) would miss this
    entirely: the directory always has *a* run.json throughout the
    re-run, and the brief's own test suite never constructed this case.
    """
    run = make_run(name="s9999-063-x", verdict="PASS")  # finished once

    # Simulate the bench reusing this directory: a fresh debug/gpsr-*/
    # events.jsonl subdirectory appears, newer than the stale run.json,
    # while run.json itself is untouched. Back-date run.json explicitly
    # with os.utime rather than relying on real elapsed wall-clock time
    # between the two writes above/below: this sandbox's filesystem mtime
    # resolution is coarse enough that two writes microseconds apart can
    # legitimately round to the identical timestamp, which would make
    # this test flaky (or silently non-discriminating) without it.
    os.utime(run / "run.json", (time.time() - 120, time.time() - 120))

    new_debug = run / "debug" / "gpsr-20260828T235959000000Z-rerun"
    new_debug.mkdir(parents=True)
    (new_debug / "events.jsonl").write_text(
        json.dumps({"event_type": "run.started",
                    "occurred_at": "2026-08-28T23:59:59.000000Z"}) + "\n"
    )

    found = find_in_flight(run.parents[2])
    assert [f.dir_name for f in found] == ["s9999-063-x"]


def test_a_reused_directory_gone_stale_is_not_in_flight(make_run):
    """Liveness matrix case 5: run.json present AND events.jsonl newer
    than it (the reused-directory signal that makes the case above IN
    flight) -- but the event log itself has gone quiet past
    `stale_after`. The staleness guard must still win here: a re-run
    that crashed or was torn down right after starting must not look
    live forever just because its stale run.json happens to predate its
    equally-stale new events.jsonl. This combination (reused directory +
    staleness) was previously covered only separately, never together.
    """
    run = make_run(name="s9999-063-z", verdict="PASS")
    now = time.time()
    os.utime(run / "run.json", (now - 1000, now - 1000))

    new_debug = run / "debug" / "gpsr-20260828T235959000000Z-rerun-stale"
    new_debug.mkdir(parents=True)
    events = new_debug / "events.jsonl"
    events.write_text(
        json.dumps({"event_type": "run.started",
                    "occurred_at": "2026-08-28T23:59:59.000000Z"}) + "\n"
    )
    # Newer than run.json (on its own, the reused-directory signal that
    # would mark this in flight) but itself far older than stale_after
    # relative to "now" -- gone quiet, not live.
    os.utime(events, (now - 500, now - 500))
    assert events.stat().st_mtime > (run / "run.json").stat().st_mtime

    found = find_in_flight(run.parents[2], stale_after=60.0, now=now)
    assert found == []


def test_tail_events_resumes_from_a_byte_offset(make_run):
    run = make_run(name="s9999-063-y", verdict=None, finished=False)
    events = next((run / "debug").glob("gpsr-*")) / "events.jsonl"

    first, offset = tail_events(events, 0)
    assert len(first) > 0
    again, offset2 = tail_events(events, offset)
    assert again == []
    assert offset2 == offset

    with events.open("a") as fh:
        fh.write(json.dumps({"event_type": "run.heartbeat",
                             "payload": {"tick": 99}}) + "\n")
    more, _ = tail_events(events, offset)
    assert len(more) == 1
    assert more[0]["payload"]["tick"] == 99


def test_tail_events_does_not_consume_a_torn_final_line(make_run):
    run = make_run(name="s9999-064-x", verdict=None, finished=False)
    events = next((run / "debug").glob("gpsr-*")) / "events.jsonl"
    _, offset = tail_events(events, 0)

    with events.open("a") as fh:
        fh.write('{"event_type": "tree.gen')  # no trailing newline yet
    torn, torn_offset = tail_events(events, offset)
    assert torn == []
    assert torn_offset == offset, "offset must not advance past a partial line"

    with events.open("a") as fh:
        fh.write('erated", "payload": {"nodes": []}}\n')
    complete, _ = tail_events(events, torn_offset)
    assert len(complete) == 1
    assert complete[0]["event_type"] == "tree.generated"


def test_live_summary_reports_regenerations_and_elapsed(make_run):
    run = make_run(
        name="s9999-065-x", verdict=None, finished=False,
        epochs=[["a"], ["a", "b"], ["a", "b", "c"]],
        transitions=[("2026-08-28T10:00:30.000000Z", "executor/root/0",
                      "FAILURE", "goto target failed")],
    )
    summary = live_summary(run)
    assert summary["tree_regenerations"] == 1
    assert summary["elapsed_s"] is not None
    assert summary["last_failure"]["feedback"] == "goto target failed"


def test_live_summary_elapsed_tracks_real_time_not_a_frozen_log_mtime(
    make_run,
):
    """The brief's reference computed elapsed as `events.stat().st_mtime
    - started_wall`, which freezes for as long as the log goes quiet. A
    run can legitimately go quiet for a stretch (a long nav goal with no
    ticks) and still count as in flight, so elapsed must keep advancing
    with real wall-clock time instead of stalling at whatever the log's
    last write happened to be.

    Back-date the event log's own mtime to something wildly different
    from "now" (1970-01-01) to simulate a quiet stretch, and confirm
    elapsed_s tracks real time.time() -- not that frozen mtime, which
    would put elapsed_s at roughly negative 56 years instead.
    """
    run = make_run(name="s9999-069-x", verdict=None, finished=False)
    events = next((run / "debug").glob("gpsr-*")) / "events.jsonl"
    os.utime(events, (0, 0))  # 1970-01-01: deliberately absurd

    started_wall = parse_wall("2026-08-28T10:00:00.000000Z")
    summary = live_summary(run)

    expected_real_time_elapsed = time.time() - started_wall
    frozen_mtime_elapsed = events.stat().st_mtime - started_wall

    assert summary["elapsed_s"] is not None
    assert abs(summary["elapsed_s"] - expected_real_time_elapsed) < 5, (
        "elapsed_s should track real wall-clock time"
    )
    assert abs(summary["elapsed_s"] - frozen_mtime_elapsed) > 1000, (
        "elapsed_s must not have frozen at the (backdated) log's own mtime"
    )


def test_live_summary_prefers_the_most_recent_plan_step_across_kinds(make_run):
    """sheet_events.py tags the active plan step onto BOTH milestone and
    judge events (_apply_step_context), in event-arrival order. A judge
    gate FAILURE can be followed, later in the same run, by a milestone
    (e.g. the nav goal it gated) that carries the SAME plan-step tag but
    is chronologically newer -- the milestone must win, not whichever
    kind happens to be scanned. (The task brief's reference
    implementation scanned only judge_events for this, which would
    return the stale "blocked" judge text here instead of "arrived".)
    """
    run = make_run(
        name="s9999-066-x", verdict=None, finished=False,
        epochs=[[
            "materialise:plan:task:0", "goto target kitchen",
            "precondition gate check",
        ]],
        transitions=[
            ("2026-08-28T10:01:00.000000Z", "executor/root/0", "SUCCESS",
             "step 0: goto({'location': 'kitchen'})"),
            ("2026-08-28T10:01:05.000000Z", "executor/root/2", "FAILURE",
             "blocked"),
            ("2026-08-28T10:01:10.000000Z", "executor/root/1", "SUCCESS",
             "arrived"),
        ],
    )
    summary = live_summary(run)
    assert summary["plan_step"] is not None
    assert summary["plan_step"].startswith("arrived"), (
        f"expected the later NAV milestone's tag to win, got "
        f"{summary['plan_step']!r}"
    )
    assert "plan-step 0 goto" in summary["plan_step"]


def test_live_summary_reports_the_newest_frame_per_camera(make_run):
    run = make_run(
        name="s9999-067-x", verdict=None, finished=False,
        frames={
            "head": [(0, 1000), (1, 2000)],
            "arena": [(0, 1000)],
        },
    )
    summary = live_summary(run)
    assert summary["newest_frames"] == {
        "head": "0001_2000.jpg", "arena": "0000_1000.jpg",
    }


def test_find_progress_failures_is_none_when_nothing_is_there(tmp_path):
    assert find_progress_failures([tmp_path / "nope-here"]) is None


def test_find_progress_failures_counts_the_newest_bridge_log(tmp_path):
    root = tmp_path / "tinker-sim"
    older = root / "6.0.1" / "gpsr_stack_logs" / "20260101T000000"
    newer = (
        root / "6.0.1" / ".claude" / "worktrees" / "some-spec"
        / "gpsr_stack_logs" / "20260828T120000"
    )
    older.mkdir(parents=True)
    newer.mkdir(parents=True)
    (older / "02-bridge.log").write_text("Failed to make progress\n" * 5)
    (newer / "02-bridge.log").write_text(
        "ok\nFailed to make progress\nFailed to make progress\nok\n")

    found = find_progress_failures([root])
    assert found is not None
    assert found["recent_count"] == 2
    assert str(newer) in found["path"]
    # Small file: read in full, so it is NOT reported as truncated.
    assert found["truncated"] is False
    assert found["window_bytes"] == len(
        (newer / "02-bridge.log").read_bytes())


def test_find_progress_failures_never_raises_on_an_unreadable_root(tmp_path):
    # A root that is itself a file, not a directory: glob() on it must
    # degrade to "nothing found", not raise.
    not_a_dir = tmp_path / "not-a-dir"
    not_a_dir.write_text("x")
    assert find_progress_failures([not_a_dir]) is None


def test_find_progress_failures_never_raises_on_binary_garbage(tmp_path):
    """The bridge log is read as bytes and decoded with errors="replace" --
    a chunk that happens to end mid-multibyte-character, or is genuinely
    non-UTF-8, must never raise or crash the panel."""
    root = tmp_path / "tinker-sim"
    stack = root / "6.0.1" / "gpsr_stack_logs" / "20260828T000000"
    stack.mkdir(parents=True)
    (stack / "02-bridge.log").write_bytes(
        b"\xff\xfe\x00Failed to make progress\x00\x80\x81binary junk"
    )
    found = find_progress_failures([root])
    assert found is not None
    assert found["recent_count"] >= 1


def test_find_progress_failures_only_reads_a_bounded_tail_window(tmp_path):
    """The core fix over the naive whole-file read: a log far larger than
    the window must never have its full contents read, and the marker
    count must reflect only the tail, not the whole file -- an old
    occurrence outside the window must NOT be counted, while a recent one
    inside it must be."""
    root = tmp_path / "tinker-sim"
    stack = root / "6.0.1" / "gpsr_stack_logs" / "20260828T000000"
    stack.mkdir(parents=True)
    log = stack / "02-bridge.log"

    window = 4096
    old_marker = "Failed to make progress -- OLD, outside the window\n"
    filler = "x" * (window * 3)  # comfortably larger than the window
    recent_marker = "Failed to make progress -- RECENT, inside the window\n"
    log.write_text(old_marker + filler + recent_marker)

    found = find_progress_failures([root], window_bytes=window)
    assert found is not None
    assert found["truncated"] is True
    assert found["window_bytes"] == window
    assert found["recent_count"] == 1, (
        "must count only the recent, in-window occurrence -- the old one "
        "outside the window must not be read at all"
    )


def test_find_progress_failures_window_bytes_never_exceeds_the_cap(tmp_path):
    """A large real 02-bridge.log has been measured at ~95MB; this proves
    the function never reads more than `window_bytes` off such a file --
    the whole point of the fix, not just a count-correctness detail."""
    root = tmp_path / "tinker-sim"
    stack = root / "6.0.1" / "gpsr_stack_logs" / "20260828T000000"
    stack.mkdir(parents=True)
    log = stack / "02-bridge.log"
    # A few MB stand in for a real ~100MB log; still large enough that a
    # full read would be trivially detectable as the wrong behaviour if
    # it happened, without actually writing 100MB in a unit test.
    log.write_text("z" * (4 * 1024 * 1024))

    found = find_progress_failures([root], window_bytes=1024)
    assert found is not None
    assert found["window_bytes"] == 1024
    assert found["truncated"] is True
