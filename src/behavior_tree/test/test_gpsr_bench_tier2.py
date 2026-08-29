import json
import sys
import textwrap
import time
from pathlib import Path
from unittest import mock

import pytest

from behavior_tree.GPSR.bench.corpus import CorpusEntry
from behavior_tree.GPSR.bench import tier2
from behavior_tree.GPSR.bench.tier2 import run_tier2


@pytest.fixture(autouse=True)
def _fake_llm_preflight_ok(monkeypatch):
    """Every test in this file drives a fake orchestrator launcher and must never touch the
    network; stub the LLM preflight to "ok" so run_tier2's default llm_check=True doesn't
    attempt a real OpenRouter call. Tests that specifically exercise the preflight (or its
    --skip-llm-check bypass) override this patch themselves."""
    monkeypatch.setattr(tier2, "llm_preflight", lambda env: (True, ""))


def _entry(i, text):
    return CorpusEntry(id=f"c{i}", seed=1, template="goToLoc", followups=(), category="people",
                       text=text, feasibility="A")


def _fake_orchestrator_with_log_lines(tmp_path: Path, log_lines: list[str] = (),
                                      status: str = "succeeded") -> list[str]:
    """A stand-in orchestrator that prints `log_lines` to stdout -- captured verbatim into
    orchestrator.log by tier2._run_orchestrator -- before writing task-1 telemetry with the
    given status, then idles. Used to synthesize planner-fallback evidence for the post-run
    verdict guard without needing a real (or previously-recorded) orchestrator run."""
    script = tmp_path / "fake_orch_guard.py"
    script.write_text(textwrap.dedent(f"""
        import json, os, time
        for line in {list(log_lines)!r}:
            print(line, flush=True)
        d = os.path.join(os.environ["BT_GPSR_PLAN_DIR"], "debug", "traj-1"); os.makedirs(d, exist_ok=True)
        f = open(os.path.join(d, "events.jsonl"), "a", buffering=1)
        def ev(t, payload):
            f.write(json.dumps({{"event_type": t, "task_id": "traj-1/task-1", "payload": payload, "occurred_at": "x"}}) + "\\n")
        ev("step.finished", {{"action": "announce", "outcome": "succeeded"}})
        ev("task.finished", {{"status": {status!r}, "reason": "r"}})
        while True:
            time.sleep(1)
    """))
    return [sys.executable, str(script)]


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
    entries = [_entry(0, "go to the sofa")]

    # Monkeypatch tier2._stop to record which Popen objects were stopped and call the real implementation.
    stopped_procs = []
    real_stop = tier2._stop

    def patched_stop(proc):
        stopped_procs.append(proc)
        real_stop(proc)

    # Launcher path is unexecutable, will raise OSError in Popen.
    with mock.patch.object(tier2, "_stop", side_effect=patched_stop):
        results = run_tier2(entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
                            out_dir=tmp_path / "out", timeout_s=20,
                            launcher=["/nonexistent/path/to/orchestrator"],
                            reset_cmd=["true"], recorder_cmd=["sh", "-c", "while true; do sleep 1; done"],
                            settle_s=0)

    # Assert (a) the recorder's Popen is among the stopped processes
    assert len(stopped_procs) == 1, f"Expected 1 process stopped, got {len(stopped_procs)}"
    recorder_proc = stopped_procs[0]

    # Assert (b) the recorder process is no longer alive after run_tier2 returns
    assert recorder_proc.poll() is not None, "Recorder process should not be alive after _stop"

    # Assert (c) the entry's verdict is ERROR with detail starting "exception:"
    assert results[0].verdict == "ERROR", f"Expected verdict ERROR, got {results[0].verdict}"
    assert results[0].detail.startswith("exception:"), f"Expected detail to start with 'exception:', got {results[0].detail}"


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


# -- post-run verdict guard: reject hollow PASSes from an exhausted planner --------------

_EXHAUSTED_LOG_LINES = [
    "[plan:0:0] attempt 1/4 -> LLM call error: PermissionDeniedError(\"Error code: 403\")",
    "[plan:0:0] attempt 2/4 -> LLM call error: PermissionDeniedError(\"Error code: 403\")",
    "[plan:0:0] attempt 3/4 -> LLM call error: PermissionDeniedError(\"Error code: 403\")",
    "[plan:0:0] attempt 4/4 -> LLM call error: PermissionDeniedError(\"Error code: 403\")",
    "[plan:0:0] all 4 attempts failed (last reason: LLM call error: PermissionDeniedError) "
    "-> fallback acknowledgement plan",
]

_TRANSIENT_LOG_LINES = [
    "[plan:0:0] attempt 1/4 -> LLM call error: APIConnectionError(\"timeout\")",
    "[plan:0:0] accepted on attempt 2: ['goto target']",
]


def test_tier2_pass_overridden_to_fail_when_planner_exhausted_attempts(tmp_path):
    entries = [_entry(0, "bring me a spam from the laundry desk")]
    results = run_tier2(
        entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
        out_dir=tmp_path / "out", timeout_s=20,
        launcher=_fake_orchestrator_with_log_lines(tmp_path, _EXHAUSTED_LOG_LINES),
        reset_cmd=["true"], settle_s=0)

    assert results[0].verdict == "FAIL"
    assert results[0].detail == "planner exhausted attempts (fallback plan executed)"

    run_json = json.loads((tmp_path / "out" / "runs" / "c0" / "run.json").read_text())
    assert run_json["verdict"] == "FAIL"


def test_tier2_pass_overridden_to_fail_on_fallback_plan_announce_text_alone(tmp_path):
    """Even without the `all N attempts failed` line making it into the (possibly
    truncated/rotated) log, the fallback plan's own announce text is independently
    sufficient evidence that a real plan never ran."""
    entries = [_entry(0, "bring me a spam from the laundry desk")]
    log_lines = ["step 0: announce({'text': 'I heard your command but could not work out a "
                "complete plan for it. I will skip it for now.'})"]
    results = run_tier2(
        entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
        out_dir=tmp_path / "out", timeout_s=20,
        launcher=_fake_orchestrator_with_log_lines(tmp_path, log_lines),
        reset_cmd=["true"], settle_s=0)

    assert results[0].verdict == "FAIL"
    assert results[0].detail == "planner exhausted attempts (fallback plan executed)"


def test_tier2_pass_survives_transient_llm_errors_that_recovered(tmp_path):
    entries = [_entry(0, "go to the sofa")]
    results = run_tier2(
        entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
        out_dir=tmp_path / "out", timeout_s=20,
        launcher=_fake_orchestrator_with_log_lines(tmp_path, _TRANSIENT_LOG_LINES),
        reset_cmd=["true"], settle_s=0)

    assert results[0].verdict == "PASS"
    assert results[0].detail == ""


# -- H2 (round-2 review, sim run 003, 2026-08-29): the exhausted-attempts override must only
# apply when the fallback was the run's FINAL outcome, not merely evidence that it happened
# somewhere in the log -- a later replan/escape that goes on to complete every real target
# (003: goto, search_object, grasp, place, all AFTER one earlier fallback) is a genuine PASS.
# ------------------------------------------------------------------------------------------

def _fake_orchestrator_with_exhaustion_then_recovery(tmp_path: Path) -> list[str]:
    """Prints the exhausted-planner log lines (same as `_EXHAUSTED_LOG_LINES`), THEN emits a
    hollow fallback `announce` step succeeding, THEN emits real recovered steps (goto,
    search_object, grasp, place, all succeeding) before `task.finished(succeeded)` -- mirrors
    sim run 003: an earlier planner exhaustion followed by genuine recovery."""
    script = tmp_path / "fake_orch_recovery.py"
    script.write_text(textwrap.dedent(f"""
        import json, os, time
        for line in {list(_EXHAUSTED_LOG_LINES)!r}:
            print(line, flush=True)
        d = os.path.join(os.environ["BT_GPSR_PLAN_DIR"], "debug", "traj-1"); os.makedirs(d, exist_ok=True)
        f = open(os.path.join(d, "events.jsonl"), "a", buffering=1)
        def ev(t, payload):
            f.write(json.dumps({{"event_type": t, "task_id": "traj-1/task-1", "payload": payload, "occurred_at": "x"}}) + "\\n")
        ev("step.finished", {{"action": "announce", "outcome": "succeeded"}})
        for action in ("goto", "search_object", "grasp", "place"):
            ev("step.finished", {{"action": action, "outcome": "succeeded"}})
        ev("task.finished", {{"status": "succeeded", "reason": "r"}})
        while True:
            time.sleep(1)
    """))
    return [sys.executable, str(script)]


def test_tier2_pass_stays_pass_when_exhaustion_is_followed_by_real_recovery(tmp_path):
    entries = [_entry(0, "bring me a spam from the laundry desk")]
    results = run_tier2(
        entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
        out_dir=tmp_path / "out", timeout_s=20,
        launcher=_fake_orchestrator_with_exhaustion_then_recovery(tmp_path),
        reset_cmd=["true"], settle_s=0)

    assert results[0].verdict == "PASS"
    assert results[0].detail == ""


def test_scan_planner_exhaustion_drops_override_when_events_show_later_real_success(tmp_path):
    log_path = tmp_path / "orchestrator.log"
    log_path.write_text("\n".join(_EXHAUSTED_LOG_LINES) + "\n")
    events_path = tmp_path / "events.jsonl"
    events = [
        {"event_type": "step.finished", "payload": {"action": "announce", "outcome": "succeeded"}},
        {"event_type": "step.finished", "payload": {"action": "goto", "outcome": "succeeded"}},
        {"event_type": "step.finished", "payload": {"action": "place", "outcome": "succeeded"}},
    ]
    events_path.write_text("\n".join(json.dumps(e) for e in events) + "\n")

    exhausted, split_fell_back = tier2._scan_planner_exhaustion(log_path, events_path)
    assert exhausted is False
    assert split_fell_back is False


def test_scan_planner_exhaustion_keeps_override_when_fallback_is_the_final_event(tmp_path):
    log_path = tmp_path / "orchestrator.log"
    log_path.write_text("\n".join(_EXHAUSTED_LOG_LINES) + "\n")
    events_path = tmp_path / "events.jsonl"
    events = [
        {"event_type": "step.finished", "payload": {"action": "goto", "outcome": "succeeded"}},
        {"event_type": "step.finished", "payload": {"action": "announce", "outcome": "succeeded"}},
    ]
    events_path.write_text("\n".join(json.dumps(e) for e in events) + "\n")

    exhausted, split_fell_back = tier2._scan_planner_exhaustion(log_path, events_path)
    assert exhausted is True
    assert split_fell_back is False


def test_scan_planner_exhaustion_keeps_override_when_no_events_file_is_given(tmp_path):
    # Backward compatible: an omitted/None events_path (e.g. a direct caller that never
    # located one) leaves the pre-H2 behaviour unchanged -- any exhaustion evidence still
    # flips the verdict, since there is nothing to prove a later recovery happened.
    log_path = tmp_path / "orchestrator.log"
    log_path.write_text("\n".join(_EXHAUSTED_LOG_LINES) + "\n")
    exhausted, split_fell_back = tier2._scan_planner_exhaustion(log_path)
    assert exhausted is True
    assert split_fell_back is False


def test_tier2_split_stage_fallback_alone_annotates_detail_but_stays_pass(tmp_path):
    """A deterministic split fallback (`[split] all N attempts failed`) does NOT by itself
    mean planning failed -- the deterministic split can still be planned normally -- so it
    must not flip the verdict, only annotate the detail."""
    entries = [_entry(0, "go to the sofa")]
    log_lines = ["[split] all 4 attempts failed -> deterministic fallback split"]
    results = run_tier2(
        entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
        out_dir=tmp_path / "out", timeout_s=20,
        launcher=_fake_orchestrator_with_log_lines(tmp_path, log_lines),
        reset_cmd=["true"], settle_s=0)

    assert results[0].verdict == "PASS"
    assert "split fell back" in results[0].detail


def test_tier2_non_pass_verdict_is_not_touched_by_the_guard(tmp_path):
    """A FAIL from the orchestrator itself is scored on its own merits; the guard only
    ever downgrades a PASS, never re-labels an already-failing run's detail."""
    entries = [_entry(0, "please fail this run")]
    results = run_tier2(
        entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
        out_dir=tmp_path / "out", timeout_s=20,
        launcher=_fake_orchestrator_with_log_lines(tmp_path, _EXHAUSTED_LOG_LINES, status="failed"),
        reset_cmd=["true"], settle_s=0)

    assert results[0].verdict == "FAIL"
    assert results[0].detail == "r"


# -- LLM preflight ------------------------------------------------------------------------

def test_tier2_llm_preflight_failure_halts_before_any_entry_runs(tmp_path, monkeypatch):
    marker = tmp_path / "orchestrator-marker"
    monkeypatch.setattr(tier2, "llm_preflight",
                        lambda env: (False, "PermissionDeniedError('Key limit exceeded')"))
    entries = [_entry(0, "go to the sofa")]

    results = run_tier2(
        entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
        out_dir=tmp_path / "out", timeout_s=20,
        launcher=_fake_orchestrator(tmp_path, marker=marker),
        reset_cmd=["true"], settle_s=0)

    assert results == []
    assert not marker.exists()
    halted = tmp_path / "out" / "HALTED"
    assert halted.exists()
    assert "PermissionDeniedError" in halted.read_text()
    assert not (tmp_path / "out" / "runs").exists()


def test_tier2_skip_llm_check_bypasses_the_preflight(tmp_path, monkeypatch):
    def _boom(env):
        raise AssertionError("llm_preflight must not be called when llm_check=False")

    monkeypatch.setattr(tier2, "llm_preflight", _boom)
    entries = [_entry(0, "go to the sofa")]

    results = run_tier2(
        entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
        out_dir=tmp_path / "out", timeout_s=20,
        launcher=_fake_orchestrator(tmp_path),
        reset_cmd=["true"], settle_s=0, llm_check=False)

    assert results[0].verdict == "PASS"
    assert not (tmp_path / "out" / "HALTED").exists()


def test_tier2_offline_planner_skips_the_preflight_even_with_llm_check_true(tmp_path, monkeypatch):
    """live_llm=False (--offline-planner) means no LLM is ever called, so the preflight
    would be pure overhead (or a false negative if the key really is dead); llm_check's
    default of True must not force a probe when live_llm is False."""
    def _boom(env):
        raise AssertionError("llm_preflight must not be called when live_llm=False")

    monkeypatch.setattr(tier2, "llm_preflight", _boom)
    entries = [_entry(0, "go to the sofa")]

    results = run_tier2(
        entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
        out_dir=tmp_path / "out", timeout_s=20,
        launcher=_fake_orchestrator(tmp_path),
        reset_cmd=["true"], settle_s=0, live_llm=False)

    assert results[0].verdict == "PASS"
