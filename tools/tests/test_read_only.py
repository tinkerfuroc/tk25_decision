# tools/tests/test_read_only.py
from __future__ import annotations

import pytest

from gpsr_ui.cache import cached_run_model, snapshot_mtimes
from gpsr_ui.corpus import list_tiers


@pytest.mark.corpus
def test_indexing_the_real_corpus_mutates_nothing(corpus_root, tmp_path):
    """The read-only guarantee, enforced rather than asserted in prose."""
    before = snapshot_mtimes(corpus_root)

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
    # A live battery may ADD files; it must never be us. Assert that no
    # file we saw beforehand was modified or removed.
    for path, mtime in before.items():
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
