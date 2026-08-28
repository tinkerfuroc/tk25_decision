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
