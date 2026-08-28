# tools/tests/test_read_only.py
from __future__ import annotations

from pathlib import Path

import pytest

from gpsr_ui.cache import cached_run_model, snapshot_mtimes
from gpsr_ui.corpus import list_tiers


def _run_dir_of(path: Path, run_dirs: set[Path]) -> Path | None:
    """Which of `run_dirs` (if any) `path` lives under."""
    for parent in path.parents:
        if parent in run_dirs:
            return parent
    return None


def _in_flight_run_dirs(run_dirs: set[Path], snapshot: dict[Path, float]) -> set[Path]:
    """Which of `run_dirs` are currently in flight, per one mtime snapshot.

    `run.json` is written last, after a run finishes (see conftest.make_run's
    comment: "run.json is written LAST by the bench; its absence means
    in-flight") -- so a run directory with no `run.json` yet is unfinished
    and every file under it is fair game for the battery to still be
    writing.

    Presence of `run.json` is NOT enough on its own, though: the bench
    (gpsr_bench.py's run_tier2) re-runs a corpus entry by reusing its
    existing "current" run directory in place -- `run_dir.mkdir(exist_ok=
    True)`, not a fresh directory -- and does not touch `run.json` again
    until the new run completes. So a re-run's directory keeps the
    PREVIOUS run's `run.json` sitting there, present and unchanged, for
    the entire duration of the new run, while orchestrator.log is actively
    being appended to. This was seen directly: s2026-005-greetNameInRm's
    orchestrator.log was being rewritten while its run.json (from the
    prior completed run) had not budged.

    The obvious next idea -- "in flight if ANYTHING in the tree is newer
    than run.json" -- was tried and is WRONG: bench/tier2.py's `run_tier2`
    calls `_write_run_json` and only THEN (`if sheet_cmd: ...`) generates
    `sheet.jpg`/`judge-sheet.jpg` as a post-processing step. Those files
    are, by design, always newer than `run.json` on every completed run
    that has one -- confirmed against this corpus: 67 of 71 runs with a
    run.json have a sheet.jpg newer than it. That heuristic would have
    excluded nearly the entire corpus from the check, silently gutting
    the guarantee rather than adapting it.

    `orchestrator.log` is the one file that does not have this problem:
    `_run_orchestrator` opens it in append mode for the orchestrator
    subprocess's entire lifetime and returns -- ending that subprocess and
    closing the file -- strictly before `_write_run_json` is ever called.
    So for a genuinely finished run, orchestrator.log must be no newer
    than run.json (confirmed: 0 of 68 runs in this corpus with both files
    violate that). The only way orchestrator.log ends up newer is a fresh
    run actively appending to it right now. That is exactly, and only,
    the in-flight signal we want.
    """
    out: set[Path] = set()
    for run_dir in run_dirs:
        run_json_mtime = snapshot.get(run_dir / "run.json")
        if run_json_mtime is None:
            out.add(run_dir)  # never finished (or never started reporting)
            continue
        orchestrator_mtime = snapshot.get(run_dir / "orchestrator.log")
        if orchestrator_mtime is not None and orchestrator_mtime > run_json_mtime:
            out.add(run_dir)  # mid re-run: log outpacing the stale run.json
    return out


def _under_any(path: Path, run_dirs: set[Path]) -> bool:
    return any(run_dir in path.parents for run_dir in run_dirs)


def _attempt_dirs(corpus_root: Path) -> set[Path]:
    return {
        attempt.path
        for tier in list_tiers(corpus_root)
        for entry in tier.entries
        for attempt in entry.attempts
    }


@pytest.mark.corpus
def test_indexing_the_real_corpus_mutates_nothing(corpus_root, tmp_path):
    """The read-only guarantee, enforced rather than asserted in prose.

    This is the real-world smoke test: it runs against the actual,
    concurrently-written bench corpus, where a live battery is legitimately
    adding, appending to and completing run directories the whole time we
    are indexing. It cannot demand full file-set equality (see the
    hermetic companion test below for why) -- but "a live battery may
    write" is not a blanket excuse either. We know precisely which files
    the battery may still be writing: any file under a run directory that
    is in flight -- see `_in_flight_run_dirs` -- at the START or the END of
    this test is excluded from the modification/removal check. Checking
    both snapshots matters -- a run can go from in-flight to finished mid-
    test, and such a run's files were still fair game for the battery to
    touch while in flight, so it must be excluded on the strength of
    either snapshot, not just one. Every other file -- anything not under
    an in-flight run at either time -- is a file only this tool's
    indexing pass could plausibly have touched, and for those the
    guarantee is exactly as strict as ever: no pre-existing file may be
    modified or removed (additions are still tolerated, since the battery
    may create whole new run directories).
    """
    before = snapshot_mtimes(corpus_root)
    before_run_dirs = _attempt_dirs(corpus_root)
    before_in_flight = _in_flight_run_dirs(before_run_dirs, before)

    tiers = list_tiers(corpus_root)
    assert tiers, "corpus should contain at least one tier"
    scanned = 0
    for tier in tiers:
        for entry in tier.entries:
            for attempt in entry.attempts:
                cached_run_model(attempt.path, tmp_path / "state")
                scanned += 1
                if scanned >= 12:
                    break
            if scanned >= 12:
                break
        if scanned >= 12:
            break

    after = snapshot_mtimes(corpus_root)
    after_run_dirs = _attempt_dirs(corpus_root)
    after_in_flight = _in_flight_run_dirs(after_run_dirs, after)
    excluded_run_dirs = before_in_flight | after_in_flight

    # A live battery may ADD files, and may freely rewrite anything inside
    # a run that was in flight at either snapshot; it must never be us.
    # Assert that no OTHER pre-existing file was modified or removed.
    for path, mtime in before.items():
        if _under_any(path, excluded_run_dirs):
            continue
        assert path in after, f"{path} disappeared during indexing"
        assert after[path] == mtime, f"{path} was modified during indexing"


def test_indexing_a_synthetic_corpus_mutates_nothing_at_all(
    make_run, tmp_path, tmp_path_factory
):
    """Hermetic counterpart to test_indexing_the_real_corpus_mutates_nothing.

    That test tolerates a live battery ADDING files concurrently, so it can
    only assert non-modification/non-removal of files seen beforehand -- it
    cannot catch a bug in the indexing path itself (list_tiers or
    cached_run_model) that CREATES a new file inside the bench root, because
    such a file would be indistinguishable from a legitimate battery
    addition. Here nothing else is writing: the corpus is entirely synthetic
    and built by this test, so full dict equality -- additions included --
    is available and is the strict check the live test cannot do. The two
    tests are complementary, not redundant: this one closes the one gap the
    other identifies; the other one is what proves the guarantee holds
    against the real, concurrently-written corpus this tool exists to serve.
    """
    # Multiple entries, and multiple attempts (current + one archived) on
    # one of them, all under one synthetic tier -- make_run pins the tier
    # name ("t9-test") to this test's own tmp_path, so bench_root is
    # tmp_path itself.
    make_run(name="s9999-040-x", epochs=[["a"]])
    make_run(name="s9999-040-x.attempt1-old", epochs=[["a"]])
    make_run(name="s9999-041-y", epochs=[["a"], ["a", "b"]])

    bench_root = tmp_path
    # The cache's state dir must live outside the synthetic corpus, exactly
    # as GPSR_UI_STATE_DIR must live outside the real one (config.py
    # enforces that for the real deployment) -- tmp_path_factory hands back
    # a sibling temp dir, never a child of tmp_path/bench_root.
    state_dir = tmp_path_factory.mktemp("state")

    before = snapshot_mtimes(bench_root)

    tiers = list_tiers(bench_root)
    assert tiers, "synthetic corpus should contain at least one tier"
    scanned = 0
    for tier in tiers:
        for entry in tier.entries:
            for attempt in entry.attempts:
                cached_run_model(attempt.path, state_dir)
                scanned += 1
    assert scanned == 3, "expected to index every synthetic attempt built above"

    after = snapshot_mtimes(bench_root)
    # Hermetic: no concurrent writer exists, so equality must be exact --
    # additions, modifications and removals are all disqualifying.
    assert before == after, "indexing must not add, modify or remove any file"
