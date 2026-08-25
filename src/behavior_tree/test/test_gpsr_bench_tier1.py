import json
import os
import stat
import sys
import textwrap
import time
from pathlib import Path

import pytest

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
            if "{behaviour}" == "stall" and slot == 1:
                # Report SOME telemetry (so the slot's per-task clock starts) then hang
                # forever without ever reaching a terminal status or the next slot.
                ev("step.finished", slot, {{"action": "goto", "outcome": "succeeded"}})
                time.sleep(60)
            ev("step.finished", slot, {{"action": "goto", "outcome": "succeeded"}})
            ev("task.finished", slot, {{"status": "failed" if "fail" in c else "succeeded", "reason": "r"}})
        while True:
            time.sleep(1)
    """))
    return [sys.executable, str(script)]


def _fake_wrapper_orchestrator(tmp_path: Path, pidfile: Path) -> list[str]:
    """A stand-in for `ros2 run ...`: a wrapper that Popen's the real orchestrator as a
    grandchild, mirroring how the real launcher forks `gpsr-orchestrator` beneath itself."""
    inner = _fake_orchestrator(tmp_path, "ok")
    wrapper = tmp_path / "fake_wrapper.py"
    wrapper.write_text(textwrap.dedent(f"""
        import subprocess, sys
        p = subprocess.Popen({inner!r})
        with open({str(pidfile)!r}, "w") as f:
            f.write(str(p.pid))
        p.wait()
    """))
    return [sys.executable, str(wrapper)]


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


def test_bench_env_resolves_a_relative_plan_dir_to_absolute():
    # run_group launches the orchestrator with cwd=plan_dir; a relative BT_GPSR_PLAN_DIR would
    # have the orchestrator re-resolve it against that same cwd and nest its telemetry under
    # plan_dir/plan_dir/debug/... instead of plan_dir/debug/..., so _events_file() never finds
    # it and the group silently times out no matter what it actually did.
    env = bench_env(mock_config=Path("m.json"), constants=Path("c.json"),
                    plan_dir=Path("relative/runs/group-000"), commands=["a"], live_llm=True)
    assert Path(env["BT_GPSR_PLAN_DIR"]).is_absolute()
    assert env["BT_GPSR_PLAN_DIR"].endswith("relative/runs/group-000")


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


def test_tier1_gives_a_per_task_budget_and_reports_unreached_slots(tmp_path):
    """F2: timeout_s is a PER-TASK budget, not a per-group one. Slot 0 finishes; slot 1
    reports some telemetry (its clock starts) then stalls, so it alone times out at
    ~timeout_s past its own first_seen rather than the group cap (group_size * timeout_s
    = 9s here); slot 2, never reached because the group is stopped on slot 1, is a
    different fact (ERROR "not reached") from a slot that actually ran and hung."""
    entries = [_entry(0, "go to the sofa"), _entry(1, "cmd 1"), _entry(2, "cmd 2")]
    started = time.monotonic()
    results = run_tier1(entries, group_size=3, timeout_s=3, mock_config=tmp_path / "m.json",
                        constants=tmp_path / "c.json", plan_dir=tmp_path / "runs",
                        launcher=_fake_orchestrator(tmp_path, "stall"))
    elapsed = time.monotonic() - started
    assert [r.verdict for r in results] == ["PASS", "TIMEOUT", "ERROR"]
    assert "not reached: slot 1 timed out" in results[2].detail
    assert elapsed < 7.0  # well under the 9s group cap -- stopped on slot 1's own budget


def test_tier1_reports_a_crashed_process(tmp_path):
    entries = [_entry(0, "cmd 0"), _entry(1, "cmd 1")]
    results = run_tier1(entries, group_size=2, timeout_s=10, mock_config=tmp_path / "m.json",
                        constants=tmp_path / "c.json", plan_dir=tmp_path / "runs",
                        launcher=_fake_orchestrator(tmp_path, "crash"))
    assert [r.verdict for r in results] == ["PASS", "ERROR"]
    assert "exit code 3" in results[1].detail


def test_tier1_stops_the_whole_process_group(tmp_path):
    """`ros2 run ...` is a wrapper that Popen's the real orchestrator as a grandchild; stopping
    the group must reach that grandchild too, not just the direct child we launched."""
    pidfile = tmp_path / "grandchild.pid"
    entries = [_entry(0, "go to the sofa")]
    results = run_tier1(entries, group_size=1, timeout_s=20, mock_config=tmp_path / "m.json",
                        constants=tmp_path / "c.json", plan_dir=tmp_path / "runs",
                        launcher=_fake_wrapper_orchestrator(tmp_path, pidfile))
    assert results[0].verdict == "PASS"
    grandchild_pid = int(pidfile.read_text().strip())

    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        try:
            os.kill(grandchild_pid, 0)
        except ProcessLookupError:
            break
        time.sleep(0.1)
    else:
        pytest.fail(f"grandchild pid {grandchild_pid} was still alive after the group stopped")

    with pytest.raises(ProcessLookupError):
        os.kill(grandchild_pid, 0)
