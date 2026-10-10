import json

from behavior_tree.GPSR.bench.campaign_checkpoint import build_checkpoint, _failure_stage


def _batch(root, name, result=None):
    batch = root / name
    batch.mkdir()
    (batch / "progress.json").write_text(json.dumps({"status": "complete"}))
    (batch / "battery.log").write_text("gpsr-stack up rc=0\nBATTERY DONE up_rc=0\n")
    if result is not None:
        (batch / "report.json").write_text(json.dumps({"results": [result]}))


def _result(entry, planner, physical, detail=""):
    return {
        "entry_id": entry,
        "verdict": "PASS" if planner == physical == "PASS" else "FAIL",
        "planner_verdict": planner,
        "physical_verdict": physical,
        "detail": detail,
    }


def test_checkpoint_groups_completed_launches_into_five_launch_windows(tmp_path):
    names = [f"run{i}" for i in range(1, 8)]
    for i, name in enumerate(names, 1):
        _batch(tmp_path, name, _result(f"entry{i}", "PASS", "PASS" if i in (1, 6) else "FAIL",
                                       "robot ended 2.0 m from person" if i not in (1, 6) else ""))
    (tmp_path / "campaign.json").write_text(json.dumps({"name": "test", "target": 20, "batches": names}))

    data = build_checkpoint(tmp_path, "campaign.json", 7)

    assert data["completed_launches"] == 7
    assert data["strict_pass_pass"] == 2
    assert data["verdict_pairs"]["PASS/PASS"] == 2
    assert len(data["windows"]) == 2
    assert data["windows"][0]["complete"] is True
    assert data["windows"][0]["launches"] == 5
    assert data["windows"][0]["strict_pass_pass"] == 1
    assert data["windows"][1]["complete"] is False
    assert data["windows"][1]["failure_stages"] == {"navigation": 1}


def test_checkpoint_keeps_completed_launch_with_no_report(tmp_path):
    names = [f"run{i}" for i in range(1, 6)]
    for name in names[:-1]:
        _batch(tmp_path, name, _result(name, "PASS", "PASS"))
    _batch(tmp_path, names[-1])
    (tmp_path / "campaign.json").write_text(json.dumps({"batches": names}))

    data = build_checkpoint(tmp_path, "campaign.json", 5)

    assert data["trial_count"] == 5
    assert data["verdict_pairs"]["UNKNOWN/UNKNOWN"] == 1
    assert data["windows"][0]["failure_stages"] == {"unspecified": 1}
    assert data["windows"][0]["failures"][-1]["batch"] == names[-1]


def test_failure_stage_prioritizes_specific_execution_failures():
    assert _failure_stage({"detail": "spawn failed: timed out after 120s"}) == "startup"
    assert _failure_stage({"detail": "Graspnet found no grasp pose"}) == "grasp"
    assert _failure_stage({"detail": "object still in the robot's hand"}) == "delivery"
    assert _failure_stage({"detail": "robot ended 2.1 m from person"}) == "navigation"
    assert _failure_stage({"detail": "counted 4, expected 3"}) == "count"
