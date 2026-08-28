# tools/tests/test_cache.py
from __future__ import annotations

from gpsr_ui.cache import cache_key, cached_run_model


def test_cache_key_changes_when_the_event_log_grows(make_run):
    run = make_run(name="s9999-030-x")
    first = cache_key(run)
    events = next((run / "debug").glob("gpsr-*")) / "events.jsonl"
    with events.open("a") as fh:
        fh.write("\n")
    assert cache_key(run) != first, "a growing live log must invalidate"


def test_cached_model_matches_a_fresh_one(make_run, tmp_path):
    run = make_run(name="s9999-031-x", epochs=[["a"], ["a", "b"]])
    state = tmp_path / "state"
    once = cached_run_model(run, state)
    twice = cached_run_model(run, state)
    assert once.tree_regenerations == twice.tree_regenerations == 0
    assert len(once.epochs) == len(twice.epochs) == 2


def test_cache_writes_only_into_the_state_dir(make_run, tmp_path):
    run = make_run(name="s9999-032-x")
    state = tmp_path / "state"
    before = {p: p.stat().st_mtime for p in run.rglob("*") if p.is_file()}
    cached_run_model(run, state)
    after = {p: p.stat().st_mtime for p in run.rglob("*") if p.is_file()}
    assert before == after
    assert state.is_dir() and any(state.rglob("*"))
