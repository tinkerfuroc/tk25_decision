import json
import os
os.environ.setdefault("BT_MOCK_MODE", "true")
from pathlib import Path

from behavior_tree.GPSR import gpsr_bench
from behavior_tree.GPSR.bench.report import BenchResult

CONSTANTS = Path(__file__).resolve().parents[1] / "behavior_tree" / "GPSR" / "constants.rcw2026.json"


def test_gen_writes_a_corpus_with_every_template(tmp_path):
    out = tmp_path / "corpus.jsonl"
    assert gpsr_bench.main(["gen", "--seed", "1", "--per-template", "1", "--constants", str(CONSTANTS),
                            "--edge", "--out", str(out)]) == 0
    lines = [json.loads(l) for l in out.read_text().splitlines()]
    assert len({l["template"] for l in lines if l["template"] != "edge"}) == 22
    assert any(l["template"] == "edge" for l in lines)


def test_tier0_uses_injected_runner_and_writes_report(tmp_path, monkeypatch):
    corpus = tmp_path / "corpus.jsonl"
    gpsr_bench.main(["gen", "--seed", "1", "--per-template", "1", "--constants", str(CONSTANTS), "--out", str(corpus)])

    def fake_run(entries, planner, **kwargs):
        return [BenchResult(e.id, e.template, e.feasibility, 0, "PASS") for e in entries]

    monkeypatch.setattr(gpsr_bench, "run_tier0", fake_run)
    monkeypatch.setattr(gpsr_bench, "_make_planner", lambda: object())
    out = tmp_path / "t0"
    assert gpsr_bench.main(["tier0", "--corpus", str(corpus), "--out", str(out), "--constants", str(CONSTANTS)]) == 0
    assert (out / "SUMMARY.md").exists() and (out / "report.json").exists()


def test_knowledge_includes_start_location_aliases():
    known_actions, known_locations = gpsr_bench._knowledge(CONSTANTS)
    assert "start_position" in known_locations
    assert "goto" in known_actions


def test_only_class_filters_entries(tmp_path, monkeypatch):
    corpus = tmp_path / "corpus.jsonl"
    gpsr_bench.main(["gen", "--seed", "1", "--per-template", "1", "--constants", str(CONSTANTS), "--out", str(corpus)])
    seen = []

    def fake_run(entries, planner, **kwargs):
        seen.extend(entries)
        return [BenchResult(e.id, e.template, e.feasibility, 0, "FAIL", "x") for e in entries]

    monkeypatch.setattr(gpsr_bench, "run_tier0", fake_run)
    monkeypatch.setattr(gpsr_bench, "_make_planner", lambda: object())
    rc = gpsr_bench.main(["tier0", "--corpus", str(corpus), "--out", str(tmp_path / "o"),
                          "--constants", str(CONSTANTS), "--only-class", "A"])
    assert rc == 1
    assert seen and all(e.feasibility == "A" for e in seen)
