import json

from behavior_tree.GPSR.bench.report import BenchResult, matrix, write_report


def _r(eid, template, tier, verdict, feas="A"):
    return BenchResult(entry_id=eid, template=template, feasibility=feas, tier=tier,
                       verdict=verdict, detail="", seconds=1.0, plan=["goto"])


def test_matrix_counts_pass_over_total_per_template_and_tier():
    m = matrix([_r("a", "goToLoc", 0, "PASS"), _r("b", "goToLoc", 0, "FAIL"),
                _r("c", "goToLoc", 1, "PASS"), _r("d", "countObjOnPlcmt", 0, "TIMEOUT")])
    assert m["goToLoc"][0] == (1, 2)
    assert m["goToLoc"][1] == (1, 1)
    assert m["countObjOnPlcmt"][0] == (0, 1)


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
