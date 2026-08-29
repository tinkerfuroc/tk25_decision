"""tier2's --spawn-cmd/--clear-cmd hooks and the GPSR_SIM_IDENTITY_RELAXED
env flag (command-driven-scene-and-sim-identity-design.md, 2026-08-28).

Mirrors test_gpsr_bench_tier2.py's fake-launcher/fake-command style: a
`sh -c` script appends to an order log so we can assert ordering without
a real orchestrator or a real gpsr_spawn.py.

Run with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 (ROS pytest plugins break
collection).
"""
from __future__ import annotations

import json
import sys
import textwrap
from pathlib import Path

import pytest

from behavior_tree.GPSR.bench import tier1, tier2
from behavior_tree.GPSR.bench.corpus import CorpusEntry
from behavior_tree.GPSR.bench.tier2 import run_tier2


@pytest.fixture(autouse=True)
def _fake_llm_preflight_ok(monkeypatch):
    monkeypatch.setattr(tier2, "llm_preflight", lambda env: (True, ""))


def _entry(i, text):
    return CorpusEntry(id=f"c{i}", seed=7, template="goToLoc", followups=(), category="objects",
                       text=text, feasibility="A")


def _fake_orchestrator(tmp_path: Path, marker: Path | None = None) -> list[str]:
    script = tmp_path / "fake_orch_spawn.py"
    marker_line = f"open({str(marker)!r}, 'w').close()" if marker is not None else "pass"
    script.write_text(textwrap.dedent(f"""
        import json, os, time
        {marker_line}
        d = os.path.join(os.environ["BT_GPSR_PLAN_DIR"], "debug", "traj-1"); os.makedirs(d, exist_ok=True)
        f = open(os.path.join(d, "events.jsonl"), "a", buffering=1)
        def ev(t, payload):
            f.write(json.dumps({{"event_type": t, "task_id": "traj-1/task-1", "payload": payload, "occurred_at": "x"}}) + "\\n")
        ev("step.finished", {{"action": "goto", "outcome": "succeeded"}})
        ev("task.finished", {{"status": "succeeded", "reason": "r"}})
        while True:
            time.sleep(1)
    """))
    return [sys.executable, str(script)]


def test_bench_env_carries_the_sim_identity_relaxation_flag(tmp_path):
    env = tier1.bench_env(mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
                          plan_dir=tmp_path / "plan", commands=["go to the sofa"], live_llm=False)
    assert env["GPSR_SIM_IDENTITY_RELAXED"] == "1"


def test_run_tier2_spawn_cmd_runs_before_the_recorder_and_writes_scene(tmp_path):
    order_log = tmp_path / "order.log"
    reset_cmd = ["sh", "-c", "echo reset >> " + str(order_log)]
    recorder_cmd = ["sh", "-c",
                    "echo recorder-start >> " + str(order_log) + "; "
                    "trap 'echo recorder-stop >> " + str(order_log) + "; exit 0' INT TERM; "
                    "while true; do sleep 1; done"]
    spawn_cmd = ["sh", "-c", "echo spawn >> " + str(order_log) + "; touch {plan} {manifest}"]
    clear_cmd = ["sh", "-c", "echo clear >> " + str(order_log)]

    entries = [_entry(0, "go to the sofa")]
    results = run_tier2(entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
                        out_dir=tmp_path / "out", timeout_s=20,
                        launcher=_fake_orchestrator(tmp_path),
                        reset_cmd=reset_cmd, recorder_cmd=recorder_cmd,
                        spawn_cmd=spawn_cmd, clear_cmd=clear_cmd, settle_s=0)

    assert results[0].verdict == "PASS"
    order = order_log.read_text().splitlines()
    assert order.index("reset") < order.index("spawn") < order.index("recorder-start")
    assert "clear" in order

    run_json = json.loads((tmp_path / "out" / "runs" / "c0" / "run.json").read_text())
    assert run_json["scene"] == {"plan": "scene-plan.json", "spawned": "spawned.json"}


def test_run_tier2_spawn_failure_scores_error_and_skips_the_run(tmp_path):
    marker = tmp_path / "orchestrator-marker"
    entries = [_entry(0, "go to the sofa")]
    spawn_cmd = ["sh", "-c", "echo boom 1>&2; exit 1"]
    results = run_tier2(entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
                        out_dir=tmp_path / "out", timeout_s=20,
                        launcher=_fake_orchestrator(tmp_path, marker=marker),
                        reset_cmd=["true"], spawn_cmd=spawn_cmd, settle_s=0)
    assert results[0].verdict == "ERROR"
    assert "spawn failed" in results[0].detail
    assert not marker.exists()


def test_run_tier2_clear_failure_is_appended_to_detail_without_changing_verdict(tmp_path):
    entries = [_entry(0, "go to the sofa")]
    clear_cmd = ["sh", "-c", "echo boom 1>&2; exit 1"]
    results = run_tier2(entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
                        out_dir=tmp_path / "out", timeout_s=20,
                        launcher=_fake_orchestrator(tmp_path),
                        reset_cmd=["true"], clear_cmd=clear_cmd, settle_s=0)
    assert results[0].verdict == "PASS"
    assert "clear failed" in results[0].detail


def test_run_tier2_without_spawn_cmd_writes_no_scene_key(tmp_path):
    entries = [_entry(0, "go to the sofa")]
    results = run_tier2(entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
                        out_dir=tmp_path / "out", timeout_s=20,
                        launcher=_fake_orchestrator(tmp_path),
                        reset_cmd=["true"], settle_s=0)
    assert results[0].verdict == "PASS"
    run_json = json.loads((tmp_path / "out" / "runs" / "c0" / "run.json").read_text())
    assert "scene" not in run_json
