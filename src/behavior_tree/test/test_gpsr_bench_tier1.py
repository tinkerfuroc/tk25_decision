import json
import os
import stat
import sys
import textwrap
from pathlib import Path

from behavior_tree.GPSR.bench.corpus import CorpusEntry
from behavior_tree.GPSR.bench.tier1 import bench_env, run_tier1


def _entry(i, text):
    return CorpusEntry(id=f"c{i}", seed=1, template="goToLoc", followups=(), category="people",
                       text=text, feasibility="A")


def _fake_orchestrator(tmp_path: Path, behaviour: str) -> list[str]:
    """A stand-in for `ros2 run behavior_tree gpsr-orchestrator` that writes telemetry then idles."""
    script = tmp_path / "fake_orch.py"
    script.write_text(textwrap.dedent(f"""
        import json, os, time, sys
        cmds = os.environ["BT_GPSR_CMD"].split("|")
        d = os.path.join(os.environ["BT_GPSR_PLAN_DIR"], "debug", "traj-1"); os.makedirs(d, exist_ok=True)
        f = open(os.path.join(d, "events.jsonl"), "a", buffering=1)
        def ev(t, slot, payload): f.write(json.dumps({{"event_type": t, "task_id": f"traj-1/task-{{slot + 1}}", "payload": payload, "occurred_at": "x"}}) + "\\n")
        for slot, c in enumerate(cmds):
            if "{behaviour}" == "hang" and slot == 1:
                time.sleep(60)
            if "{behaviour}" == "crash" and slot == 1:
                sys.exit(3)
            ev("step.finished", slot, {{"action": "goto", "outcome": "succeeded"}})
            ev("task.finished", slot, {{"status": "failed" if "fail" in c else "succeeded", "reason": "r"}})
        while True:
            time.sleep(1)
    """))
    return [sys.executable, str(script)]


def test_bench_env_sets_the_orchestrator_switches(tmp_path):
    env = bench_env(mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
                    plan_dir=tmp_path / "runs", commands=["a", "b"], live_llm=True)
    assert env["BT_GPSR_CMD"] == "a|b"
    assert env["BT_MOCK_MODE"] == "true"
    assert env["GPSR_OFFLINE_PLANNER"] == "0"
    assert env["BT_MOCK_CONFIG"].endswith("m.json")
    assert env["GPSR_CONSTANTS_PATH"].endswith("c.json")
    assert env["BT_GPSR_PLAN_DIR"].endswith("runs")
    assert os.environ.get("PATH", "") == env.get("PATH", "")  # inherits the shell env


def test_tier1_scores_each_slot_and_stops_the_idle_process(tmp_path):
    entries = [_entry(0, "go to the sofa"), _entry(1, "please fail"), _entry(2, "go to the shelf")]
    results = run_tier1(entries, group_size=3, timeout_s=20, mock_config=tmp_path / "m.json",
                        constants=tmp_path / "c.json", plan_dir=tmp_path / "runs",
                        launcher=_fake_orchestrator(tmp_path, "ok"))
    assert [r.verdict for r in results] == ["PASS", "FAIL", "PASS"]
    assert results[1].detail == "r"
    assert results[0].plan == ["goto"]
    assert all(r.tier == 1 for r in results)


def test_tier1_times_out_a_hung_group_and_continues(tmp_path):
    entries = [_entry(i, f"cmd {i}") for i in range(4)]
    results = run_tier1(entries, group_size=2, timeout_s=3, mock_config=tmp_path / "m.json",
                        constants=tmp_path / "c.json", plan_dir=tmp_path / "runs",
                        launcher=_fake_orchestrator(tmp_path, "hang"))
    assert [r.verdict for r in results] == ["PASS", "TIMEOUT", "PASS", "TIMEOUT"]


def test_tier1_reports_a_crashed_process(tmp_path):
    entries = [_entry(0, "cmd 0"), _entry(1, "cmd 1")]
    results = run_tier1(entries, group_size=2, timeout_s=10, mock_config=tmp_path / "m.json",
                        constants=tmp_path / "c.json", plan_dir=tmp_path / "runs",
                        launcher=_fake_orchestrator(tmp_path, "crash"))
    assert [r.verdict for r in results] == ["PASS", "ERROR"]
    assert "exit code 3" in results[1].detail
