import json

from behavior_tree.GPSR.bench.report import BenchResult, class_totals, matrix, write_report


def _r(eid, template, tier, verdict, feas="A"):
    return BenchResult(entry_id=eid, template=template, feasibility=feas, tier=tier,
                       verdict=verdict, detail="", seconds=1.0, plan=["goto"])


def test_matrix_counts_pass_over_total_per_template_class_and_tier():
    m = matrix([_r("a", "goToLoc", 0, "PASS"), _r("b", "goToLoc", 0, "FAIL"),
                _r("c", "goToLoc", 1, "PASS"), _r("d", "countObjOnPlcmt", 0, "TIMEOUT", feas="C")])
    assert m[("goToLoc", "A")][0] == (1, 2)
    assert m[("goToLoc", "A")][1] == (1, 1)
    assert m[("countObjOnPlcmt", "C")][0] == (0, 1)


def test_matrix_splits_a_template_with_mixed_class_entries_into_separate_rows():
    """A template's feasibility is per ENTRY (worst of template + follow-ups); a template with
    both class-A and class-C entries must not collapse into one arbitrarily-labelled row."""
    m = matrix([_r("a", "goToLoc", 0, "PASS", feas="A"), _r("b", "goToLoc", 0, "FAIL", feas="C")])
    assert set(m) == {("goToLoc", "A"), ("goToLoc", "C")}
    assert m[("goToLoc", "A")][0] == (1, 1)
    assert m[("goToLoc", "C")][0] == (0, 1)


def test_class_totals_counts_pass_over_total_per_class_and_tier():
    ct = class_totals([_r("a", "goToLoc", 0, "PASS", feas="A"), _r("b", "goToLoc", 0, "FAIL", feas="A"),
                       _r("c", "takeObjFromPlcmt", 0, "PASS", feas="B"),
                       _r("d", "greetClothDscInRm", 0, "FAIL", feas="C")])
    assert ct[0]["A"] == (1, 2)
    assert ct[0]["B"] == (1, 1)
    assert ct[0]["C"] == (0, 1)


def test_write_report_emits_json_and_markdown(tmp_path):
    results = [_r("a", "goToLoc", 0, "PASS"), _r("b", "takeObjFromPlcmt", 0, "FAIL", feas="B")]
    summary = write_report(results, tmp_path)
    assert summary.name == "SUMMARY.md"
    text = summary.read_text()
    assert "| goToLoc | A |" in text and "1/1" in text
    assert "| takeObjFromPlcmt | B |" in text and "0/1" in text
    data = json.loads((tmp_path / "report.json").read_text())
    assert data["results"][1]["verdict"] == "FAIL"
    assert data["totals"]["0"] == {"PASS": 1, "FAIL": 1}
    assert "timestamp" in data["meta"]


def test_write_report_totals_by_class_and_ab_pass_rate(tmp_path):
    results = [_r("a", "goToLoc", 0, "PASS", feas="A"), _r("b", "goToLoc", 0, "FAIL", feas="A"),
              _r("c", "takeObjFromPlcmt", 0, "PASS", feas="B"),
              _r("d", "greetClothDscInRm", 0, "FAIL", feas="C")]
    summary = write_report(results, tmp_path)
    text = summary.read_text()
    assert "Totals by class" in text
    assert "class A: 1/2" in text
    assert "class B: 1/1" in text
    assert "class C: 0/1" in text
    assert "Class A+B pass rate: 2/3 (67 %)" in text


def test_write_report_stores_run_metadata(tmp_path):
    results = [_r("a", "goToLoc", 0, "PASS")]
    meta = {"tier": 0, "timeout_s": 180.0, "seed": 42, "group_size": None}
    summary = write_report(results, tmp_path, corpus_path="corpus-42.jsonl", meta=meta)
    data = json.loads((tmp_path / "report.json").read_text())
    assert data["meta"]["tier"] == 0
    assert data["meta"]["timeout_s"] == 180.0
    assert data["meta"]["seed"] == 42
    assert "timestamp" in data["meta"] and "commit" in data["meta"]
    assert data["meta"]["corpus"] == "corpus-42.jsonl"
    text = summary.read_text()
    assert "Run:" in text and "seed=42" in text
