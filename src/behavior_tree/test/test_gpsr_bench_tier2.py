import json
import sys
import textwrap
import time
from pathlib import Path

from behavior_tree.GPSR.bench.corpus import CorpusEntry
from behavior_tree.GPSR.bench.tier2 import run_tier2


def _entry(i, text):
    return CorpusEntry(id=f"c{i}", seed=1, template="goToLoc", followups=(), category="people",
                       text=text, feasibility="A")


def _fake_orchestrator(tmp_path: Path, marker: Path | None = None) -> list[str]:
    """A stand-in for `ros2 run behavior_tree gpsr-orchestrator` for a SINGLE-command tier2 run:
    writes task-1 telemetry unless the command text says to hang, then idles. Behaviour is
    keyed off BT_GPSR_CMD's text (like tier1's fake) so one script serves every entry."""
    script = tmp_path / "fake_orch2.py"
    marker_line = f"open({str(marker)!r}, 'w').close()" if marker is not None else "pass"
    script.write_text(textwrap.dedent(f"""
        import json, os, time, sys
        {marker_line}
        cmd = os.environ["BT_GPSR_CMD"]
        d = os.path.join(os.environ["BT_GPSR_PLAN_DIR"], "debug", "traj-1"); os.makedirs(d, exist_ok=True)
        f = open(os.path.join(d, "events.jsonl"), "a", buffering=1)
        def ev(t, payload):
            f.write(json.dumps({{"event_type": t, "task_id": "traj-1/task-1", "payload": payload, "occurred_at": "x"}}) + "\\n")
        if "hang" in cmd:
            while True:
                time.sleep(1)
        ev("step.finished", {{"action": "goto", "outcome": "succeeded"}})
        ev("task.finished", {{"status": "failed" if "fail" in cmd else "succeeded", "reason": "r"}})
        while True:
            time.sleep(1)
    """))
    return [sys.executable, str(script)]


def test_tier2_pass_run_orders_reset_recorder_sheet_and_writes_run_json(tmp_path):
    order_log = tmp_path / "order.log"
    reset_cmd = ["sh", "-c", f"echo reset >> {order_log}"]
    recorder_cmd = ["sh", "-c",
                    f"echo recorder-start >> {order_log}; "
                    "trap 'echo recorder-stop >> " + str(order_log) + "; exit 0' INT TERM; "
                    "while true; do sleep 1; done"]
    sheet_cmd = ["sh", "-c", f"echo sheet >> {order_log}; touch " + "{out}"]

    entries = [_entry(0, "go to the sofa")]
    results = run_tier2(entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
                        out_dir=tmp_path / "out", timeout_s=20,
                        launcher=_fake_orchestrator(tmp_path),
                        reset_cmd=reset_cmd, recorder_cmd=recorder_cmd, sheet_cmd=sheet_cmd,
                        settle_s=0)

    assert results[0].verdict == "PASS"
    assert "sheet=runs/c0/sheet.jpg" in results[0].detail

    order = order_log.read_text().splitlines()
    assert order.index("reset") < order.index("recorder-start") < order.index("sheet")

    run_json = json.loads((tmp_path / "out" / "runs" / "c0" / "run.json").read_text())
    assert run_json["id"] == "c0"
    assert run_json["verdict"] == "PASS"
    assert run_json["text"] == "go to the sofa"


def test_tier2_reset_failure_scores_error_without_launching_orchestrator(tmp_path):
    marker = tmp_path / "orchestrator-marker"
    entries = [_entry(0, "go to the sofa")]
    results = run_tier2(entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
                        out_dir=tmp_path / "out", timeout_s=20,
                        launcher=_fake_orchestrator(tmp_path, marker=marker),
                        reset_cmd=["false"], settle_s=0)
    assert results[0].verdict == "ERROR"
    assert "reset failed" in results[0].detail
    assert not marker.exists()


def test_tier2_timeout_stops_the_recorder(tmp_path):
    stop_marker = tmp_path / "recorder-stopped"
    recorder_cmd = ["sh", "-c",
                    f"trap 'touch {stop_marker}; exit 0' INT TERM; while true; do sleep 1; done"]
    entries = [_entry(0, "please hang forever")]
    results = run_tier2(entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
                        out_dir=tmp_path / "out", timeout_s=3,
                        launcher=_fake_orchestrator(tmp_path),
                        reset_cmd=["true"], recorder_cmd=recorder_cmd, settle_s=0)
    assert results[0].verdict == "TIMEOUT"

    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and not stop_marker.exists():
        time.sleep(0.1)
    assert stop_marker.exists()


def test_tier2_halts_after_three_consecutive_errors(tmp_path):
    entries = [_entry(i, "go to the sofa") for i in range(5)]
    results = run_tier2(entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
                        out_dir=tmp_path / "out", timeout_s=5,
                        launcher=_fake_orchestrator(tmp_path),
                        reset_cmd=["false"], settle_s=0)
    assert [r.verdict for r in results] == ["ERROR", "ERROR", "ERROR"]
    assert (tmp_path / "out" / "HALTED").exists()


def test_tier2_substitutes_run_dir_as_an_absolute_path(tmp_path):
    captured = tmp_path / "captured_path.txt"
    recorder_cmd = ["sh", "-c", "echo {run_dir} > " + str(captured) + "; while true; do sleep 1; done"]
    entries = [_entry(0, "go to the sofa")]
    results = run_tier2(entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
                        out_dir=tmp_path / "out", timeout_s=20,
                        launcher=_fake_orchestrator(tmp_path),
                        reset_cmd=["true"], recorder_cmd=recorder_cmd, settle_s=0)
    assert results[0].verdict == "PASS"

    written = captured.read_text().strip()
    assert Path(written).is_absolute()
    assert written == str((tmp_path / "out" / "runs" / "c0").resolve())


def test_tier2_unexecutable_launcher_stops_recorder_and_scores_error(tmp_path):
    stop_marker = tmp_path / "recorder-stopped"
    recorder_cmd = ["sh", "-c",
                    f"trap 'touch {stop_marker}; exit 0' INT TERM; while true; do sleep 1; done"]
    entries = [_entry(0, "go to the sofa")]
    # Launcher path is unexecutable, will raise OSError in Popen inside _run_orchestrator.
    results = run_tier2(entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
                        out_dir=tmp_path / "out", timeout_s=20,
                        launcher=["/nonexistent/path/to/orchestrator"],
                        reset_cmd=["true"], recorder_cmd=recorder_cmd, settle_s=0)
    assert results[0].verdict == "ERROR"
    assert "exception:" in results[0].detail

    # Verify recorder was stopped despite the exception.
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and not stop_marker.exists():
        time.sleep(0.1)
    assert stop_marker.exists()


def test_tier2_sheet_failure_doesnt_change_verdict(tmp_path):
    entries = [_entry(0, "go to the sofa")]
    # sheet_cmd exits with code 1, which should leave verdict unchanged and append "; sheet failed".
    sheet_cmd = ["sh", "-c", "exit 1"]
    results = run_tier2(entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
                        out_dir=tmp_path / "out", timeout_s=20,
                        launcher=_fake_orchestrator(tmp_path),
                        reset_cmd=["true"], sheet_cmd=sheet_cmd, settle_s=0)
    assert results[0].verdict == "PASS"
    # The detail should NOT contain the " | sheet=" suffix for failed sheet, only "; sheet failed".
    assert "; sheet failed" in results[0].detail
    assert " | sheet=" not in results[0].detail
