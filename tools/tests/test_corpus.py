# tools/tests/test_corpus.py
from __future__ import annotations

from pathlib import Path

import pytest

from gpsr_ui.corpus import Attempt, list_tiers, split_dir_name


def test_split_dir_name_separates_entry_from_attempt_slug():
    assert split_dir_name("s2026-002-countPrsInRoom") == (
        "s2026-002-countPrsInRoom", None)
    assert split_dir_name("s2026-002-countPrsInRoom.attempt7-rpp") == (
        "s2026-002-countPrsInRoom", "attempt7-rpp")
    assert split_dir_name("s2026-002-countPrsInRoom.attempt10-nav-fixed") == (
        "s2026-002-countPrsInRoom", "attempt10-nav-fixed")


def test_attempts_group_under_their_entry_with_current_first(make_run):
    run = make_run(name="s9999-000-testEntry", verdict="PASS")
    make_run(name="s9999-000-testEntry.attempt2-foo", verdict="FAIL")
    make_run(name="s9999-000-testEntry.attempt3-bar", verdict="TIMEOUT")
    bench_root = run.parents[2]

    tiers = list_tiers(bench_root)
    assert [t.name for t in tiers] == ["t9-test"]
    entries = tiers[0].entries
    assert len(entries) == 1, "three dirs, one corpus entry"

    entry = entries[0]
    assert entry.entry_id == "s9999-000-testEntry"
    assert len(entry.attempts) == 3
    assert entry.attempts[0].is_current is True
    assert entry.attempts[0].slug is None
    assert {a.slug for a in entry.attempts[1:]} == {
        "attempt2-foo", "attempt3-bar"}
    assert all(a.is_current is False for a in entry.attempts[1:])


def test_verdict_comes_from_run_json_not_run_finished(make_run):
    """run.finished.status is 'incomplete' even on PASS runs."""
    run = make_run(name="s9999-001-x", verdict="PASS")
    tiers = list_tiers(run.parents[2])
    assert tiers[0].entries[0].attempts[0].verdict == "PASS"


def test_missing_run_json_yields_none_verdict_not_a_crash(make_run):
    run = make_run(name="s9999-002-x", verdict=None)
    tiers = list_tiers(run.parents[2])
    attempt = tiers[0].entries[0].attempts[0]
    assert attempt.verdict is None
    assert attempt.seconds is None


def test_empty_bench_root_yields_no_tiers(tmp_path):
    assert list_tiers(tmp_path) == []


def test_vanished_run_dir_is_skipped_not_fatal(make_run, monkeypatch):
    """A battery archiving/renaming a run dir between iterdir() and stat()
    must not take down discovery for every other tier and entry."""
    alive = make_run(name="s9999-003-alive", verdict="PASS")
    make_run(name="s9999-004-vanishing", verdict="PASS")
    bench_root = alive.parents[2]

    real_stat = Path.stat
    calls = {"n": 0}

    def flaky_stat(self, *args, **kwargs):
        if self.name == "s9999-004-vanishing":
            calls["n"] += 1
            if calls["n"] > 1:
                # Simulate the directory vanishing after it passed the
                # is_dir() filter (1st stat) but before _attempt()'s
                # explicit mtime stat (2nd).
                raise FileNotFoundError(self)
        return real_stat(self, *args, **kwargs)

    monkeypatch.setattr(Path, "stat", flaky_stat)

    tiers = list_tiers(bench_root)
    entry_ids = {e.entry_id for t in tiers for e in t.entries}
    assert "s9999-003-alive" in entry_ids
    assert "s9999-004-vanishing" not in entry_ids


def test_run_json_bare_list_yields_none_verdict_not_a_crash(make_run):
    """A truncated/garbage run.json can parse as a non-dict JSON value."""
    run = make_run(name="s9999-006-x", verdict=None)
    (run / "run.json").write_text("[1, 2, 3]")
    tiers = list_tiers(run.parents[2])
    attempt = tiers[0].entries[0].attempts[0]
    assert attempt.verdict is None


@pytest.mark.corpus
def test_real_corpus_groups_the_known_attempt_history(corpus_root):
    tiers = {t.name: t for t in list_tiers(corpus_root)}
    assert "t2-2026" in tiers
    entries = {e.entry_id: e for e in tiers["t2-2026"].entries}
    counts = entries["s2026-002-countPrsInRoom"]
    # The battery may add more attempts; assert the floor, not equality.
    assert len(counts.attempts) >= 11
    assert counts.attempts[0].is_current is True
