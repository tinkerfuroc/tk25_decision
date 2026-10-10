"""Sim-feasible corpus generation: excludes templates/followups the sim can't execute today.

The task-7 brief names the write/read helpers `write_corpus`/`load_corpus`, but the module's
actual helpers are `write_jsonl`/`read_jsonl` (see corpus.py; gpsr_bench.py's `gen`/tier0/tier1
already use those names). This test extends those real helpers instead of introducing new
ones -- the behaviour contract (a `{"_`-prefixed first line is header metadata, skipped by
the reader) is what's tested, not the specific names.
"""
from pathlib import Path

from behavior_tree.GPSR.bench.corpus import (
    SIM_INFEASIBLE, FEASIBILITY, generate_sim_corpus, read_jsonl, write_jsonl,
)

CONSTANTS = Path(__file__).resolve().parents[1] / "behavior_tree/GPSR/constants.rcw2026.json"


def test_infeasible_names_are_known():
    assert SIM_INFEASIBLE <= set(FEASIBILITY)


def test_sim_corpus_excludes_infeasible_and_hits_count():
    entries, skipped = generate_sim_corpus(CONSTANTS, seed=2026, count=40)
    assert len(entries) == 40
    for e in entries:
        assert e.template not in SIM_INFEASIBLE
        assert not (set(e.followups) & SIM_INFEASIBLE)
    assert isinstance(skipped, dict)


def test_sim_corpus_deterministic():
    a, _ = generate_sim_corpus(CONSTANTS, seed=7, count=10)
    b, _ = generate_sim_corpus(CONSTANTS, seed=7, count=10)
    assert [e.text for e in a] == [e.text for e in b]


def test_sim_corpus_entry_ids_carry_seed_index_and_template():
    entries, _ = generate_sim_corpus(CONSTANTS, seed=2026, count=5)
    for i, e in enumerate(entries):
        assert e.id == f"s2026-{i:03d}-{e.template}"


def test_sim_corpus_respects_templates_filter():
    entries, _ = generate_sim_corpus(CONSTANTS, seed=1, count=6, templates=["goToLoc"])
    assert entries
    assert all(e.template == "goToLoc" for e in entries)


def test_header_line_roundtrip(tmp_path):
    entries, skipped = generate_sim_corpus(CONSTANTS, seed=7, count=5)
    out = tmp_path / "c.jsonl"
    write_jsonl(entries, out, header={"_skipped": skipped, "_seed": 7, "_mode": "sim-feasible"})
    loaded = read_jsonl(out)
    assert [e.id for e in loaded] == [e.id for e in entries]


def test_read_jsonl_without_header_still_works(tmp_path):
    entries, _ = generate_sim_corpus(CONSTANTS, seed=3, count=3)
    out = tmp_path / "plain.jsonl"
    write_jsonl(entries, out)
    assert read_jsonl(out) == entries
