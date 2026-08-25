"""Tier 2: run one GPSR command per orchestrator process against a live sim.

Per entry: reset sim -> start frame recorder -> fresh orchestrator with a single
BT_GPSR_CMD -> verdict from telemetry (task id 1) -> stop recorder -> build a contact
sheet. Reuses tier1's launcher/env/telemetry machinery (``bench_env``, ``_stop``,
``_events_file``) rather than duplicating it.
"""
from __future__ import annotations

import json
import os
import subprocess
import time
from pathlib import Path
from typing import Sequence

from behavior_tree.GPSR.bench.corpus import CorpusEntry
from behavior_tree.GPSR.bench.events import parse_events
from behavior_tree.GPSR.bench.report import BenchResult
from behavior_tree.GPSR.bench.tier1 import DEFAULT_LAUNCHER, _events_file, _stop, bench_env

DEFAULT_RESET_CMD = ["ros2", "service", "call", "/reset_simulation",
                     "simulation_interfaces/srv/ResetSimulation", "{}"]

TIER = 2


def _substitute(cmd: Sequence[str], mapping: dict[str, str]) -> list[str]:
    out = []
    for arg in cmd:
        for key, val in mapping.items():
            arg = arg.replace("{" + key + "}", val)
        out.append(arg)
    return out


def _reset(reset_cmd: Sequence[str]) -> str | None:
    """Run the reset command; return an error detail string, or None on success."""
    try:
        result = subprocess.run(list(reset_cmd), timeout=60, capture_output=True, text=True)
    except subprocess.TimeoutExpired:
        return "reset failed: timed out after 60s"
    except OSError as exc:
        return f"reset failed: {exc}"
    if result.returncode != 0:
        stderr = (result.stderr or "").strip()
        suffix = f": {stderr}" if stderr else ""
        return f"reset failed: exit code {result.returncode}{suffix}"
    return None


def _run_orchestrator(*, env: dict[str, str], run_dir: Path, launcher: Sequence[str],
                      timeout_s: float, poll_s: float = 1.0) -> tuple[str, str, float, list[str]]:
    """Launch the orchestrator for a single command and score task id 1.

    This is tier1's ``run_group`` per-task clock semantics specialised to n=1: slot 0's
    clock starts at launch (same as ``run_group``'s ``slot_clock_start[0] = started_mono``),
    so ``timeout_s`` is a plain wall-clock budget here.
    """
    run_dir = Path(run_dir).resolve()
    tasks: dict[int, object] = {}
    exit_code = None
    timed_out = False
    with (run_dir / "orchestrator.log").open("a", encoding="utf-8") as log:
        started = time.time()
        started_mono = time.monotonic()
        proc = subprocess.Popen(list(launcher), env=env, stdout=log, stderr=subprocess.STDOUT,
                                cwd=str(run_dir), start_new_session=True)
        try:
            deadline = started_mono + timeout_s
            while time.monotonic() < deadline:
                events = _events_file(run_dir, started - 1)
                if events:
                    tasks = parse_events(events)
                    task = tasks.get(1)
                    if task is not None and task.status:
                        break
                exit_code = proc.poll()
                if exit_code is not None:
                    break
                time.sleep(poll_s)
            else:
                timed_out = True
        finally:
            _stop(proc)
        # The orchestrator may flush its final events during the SIGINT/SIGKILL window
        # _stop() just waited through; re-parse once more (mirrors run_group).
        events = _events_file(run_dir, started - 1)
        if events:
            tasks = parse_events(events)

    task = tasks.get(1)
    plan = [a for a, _ in task.steps] if task else []
    seconds = time.time() - started
    if task and task.status == "succeeded":
        verdict, detail = "PASS", ""
    elif task and task.status:
        verdict, detail = "FAIL", str(task.reason or task.status)
    elif exit_code is not None:
        verdict, detail = "ERROR", f"orchestrator exited with exit code {exit_code}"
    elif timed_out:
        verdict, detail = "TIMEOUT", f"timed out after {timeout_s:.0f}s"
    else:
        # Defensive fallback; not a normal path (mirrors run_group's own fallback branch).
        verdict, detail = "TIMEOUT", f"timed out after {timeout_s:.0f}s"
    return verdict, detail, seconds, plan


def _write_run_json(run_dir: Path, entry: CorpusEntry, tier_label: str, result: BenchResult) -> None:
    data = {
        "id": entry.id, "text": entry.text, "template": entry.template,
        "feasibility": entry.feasibility, "tier": tier_label, "verdict": result.verdict,
        "detail": result.detail, "seconds": result.seconds,
    }
    (run_dir / "run.json").write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def _halted(results: list[BenchResult], out_dir: Path, halt_after_errors: int) -> bool:
    if halt_after_errors <= 0 or len(results) < halt_after_errors:
        return False
    tail = results[-halt_after_errors:]
    if not all(r.verdict == "ERROR" for r in tail):
        return False
    reason = f"{halt_after_errors} consecutive ERROR results (last: {tail[-1].entry_id}: {tail[-1].detail})"
    (Path(out_dir) / "HALTED").write_text(reason + "\n", encoding="utf-8")
    return True


def run_tier2(entries: Sequence[CorpusEntry], *, mock_config: Path, constants: Path, out_dir: Path,
              timeout_s: float, tier_label: str = "T2",
              launcher: Sequence[str] = DEFAULT_LAUNCHER, reset_cmd: Sequence[str] = DEFAULT_RESET_CMD,
              recorder_cmd: list[str] | None = None,
              sheet_cmd: list[str] | None = None,
              settle_s: float = 10.0, halt_after_errors: int = 3,
              live_llm: bool = True) -> list[BenchResult]:
    out_dir = Path(out_dir)
    runs_root = out_dir / "runs"
    results: list[BenchResult] = []

    for entry in entries:
        run_dir = (runs_root / entry.id).resolve()
        run_dir.mkdir(parents=True, exist_ok=True)

        reset_error = _reset(reset_cmd)
        if reset_error is not None:
            result = BenchResult(entry.id, entry.template, entry.feasibility, TIER, "ERROR", reset_error)
            _write_run_json(run_dir, entry, tier_label, result)
            results.append(result)
            if _halted(results, out_dir, halt_after_errors):
                break
            continue

        if settle_s > 0:
            time.sleep(settle_s)

        recorder_proc = None
        recorder_log = None
        try:
            if recorder_cmd:
                cmd = _substitute(recorder_cmd, {"run_dir": str(run_dir)})
                recorder_log = (run_dir / "recorder.log").open("a", encoding="utf-8")
                recorder_proc = subprocess.Popen(cmd, stdout=recorder_log, stderr=subprocess.STDOUT,
                                                 start_new_session=True)

            env = bench_env(mock_config=mock_config, constants=constants, plan_dir=run_dir,
                            commands=[entry.text], live_llm=live_llm)
            verdict, detail, seconds, plan = _run_orchestrator(
                env=env, run_dir=run_dir, launcher=launcher, timeout_s=timeout_s)
        except Exception as exc:
            # Unexpected exception in a single run: score as ERROR with detail, continue batch.
            # This exception source is typically Popen(launcher) raising OSError for unexecutable
            # binary (which occurs outside _run_orchestrator's own try/finally), but guards all
            # unexpected exceptions in this span.
            verdict, detail, seconds, plan = "ERROR", f"exception: {exc!r}", 0.0, []
        finally:
            if recorder_proc is not None:
                _stop(recorder_proc)
            if recorder_log is not None:
                recorder_log.close()

        result = BenchResult(entry.id, entry.template, entry.feasibility, TIER, verdict, detail,
                             seconds, plan)
        _write_run_json(run_dir, entry, tier_label, result)

        if sheet_cmd:
            run_json_path = run_dir / "run.json"
            sheet_out = run_dir / "sheet.jpg"
            cmd = _substitute(sheet_cmd, {"run_dir": str(run_dir), "run_json": str(run_json_path),
                                          "out": str(sheet_out)})
            try:
                sheet_result = subprocess.run(cmd, timeout=120, capture_output=True, text=True)
                ok = sheet_result.returncode == 0
            except (subprocess.TimeoutExpired, OSError):
                ok = False
            if ok:
                rel = os.path.relpath(sheet_out, out_dir)
                result.detail = (result.detail + " | " if result.detail else "") + f"sheet={rel}"
            else:
                result.detail = result.detail + "; sheet failed"

        results.append(result)
        if _halted(results, out_dir, halt_after_errors):
            break

    return results
