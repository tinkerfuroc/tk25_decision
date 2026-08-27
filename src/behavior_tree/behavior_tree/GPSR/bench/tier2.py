"""Tier 2: run one GPSR command per orchestrator process against a live sim.

Per entry: reset sim -> start frame recorder -> fresh orchestrator with a single
BT_GPSR_CMD -> verdict from telemetry (task id 1) -> stop recorder -> build a contact
sheet. Reuses tier1's launcher/env/telemetry machinery (``bench_env``, ``_stop``,
``_events_file``) rather than duplicating it.
"""
from __future__ import annotations

import json
import os
import re
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

# Post-run verdict guard: evidence that the planner exhausted its retries and the
# orchestrator executed (or announced) the acknowledgement-only fallback plan instead of a
# real one. A `[plan:...] all N attempts failed` line, or the fallback plan's own announce
# text, means the run's task.finished "succeeded" was really just the fallback plan
# succeeding trivially -- a hollow PASS. A `[split] all N attempts failed` line alone does
# NOT mean this: the deterministic split fallback can still be planned normally, so it is
# recorded as a detail annotation only.
_PLAN_ATTEMPTS_EXHAUSTED_RE = re.compile(r"all \d+ attempts failed")
_FALLBACK_PLAN_MARKER = "could not work out a complete plan"
PLANNER_EXHAUSTED_DETAIL = "planner exhausted attempts (fallback plan executed)"


def _scan_planner_exhaustion(log_path: Path, max_bytes: int = 5_000_000) -> tuple[bool, bool]:
    """Bounded scan of an orchestrator.log for planner-fallback evidence.

    Returns ``(exhausted, split_fell_back)``. Reads at most the last ``max_bytes`` of the
    file (a stuck/looping run's log can't blow up scoring). Missing/unreadable files score
    as ``(False, False)`` -- absence of evidence is not evidence of exhaustion.
    """
    if not log_path.is_file():
        return False, False
    try:
        size = log_path.stat().st_size
        with log_path.open("r", encoding="utf-8", errors="replace") as f:
            if size > max_bytes:
                f.seek(size - max_bytes)
            text = f.read()
    except OSError:
        return False, False

    exhausted = _FALLBACK_PLAN_MARKER in text
    split_fell_back = False
    for line in text.splitlines():
        if not _PLAN_ATTEMPTS_EXHAUSTED_RE.search(line):
            continue
        if "[split]" in line:
            split_fell_back = True
        elif "[plan:" in line:
            exhausted = True
    return exhausted, split_fell_back


def llm_preflight(env: dict[str, str]) -> tuple[bool, str]:
    """One minimal chat completion through the SAME client construction the planner uses.

    Catches an exhausted/invalid OpenRouter key BEFORE a whole tier-2 battery burns through
    every entry as fallback-plan runs that would otherwise each score a hollow PASS.

    ``env`` is the environment the orchestrator subprocesses will run under (normally
    ``dict(os.environ)``); an ``OPENROUTER_API_KEY``/``OPENAI_API_KEY`` there overrides the
    planner's own resolved key, mirroring what the subprocess would actually see.

    Never raises (catches everything -- a preflight probe must not itself crash the
    battery) and never returns/logs the key itself, only the provider's error text with the
    key value scrubbed out if it somehow appears there.
    """
    try:
        import openai
        from behavior_tree.GPSR.planner import OPENAI_API_KEY as _default_key, OPENAI_MODEL
    except Exception as exc:  # pragma: no cover - import machinery
        return False, f"preflight import error: {exc!r}"

    api_key = env.get("OPENROUTER_API_KEY") or env.get("OPENAI_API_KEY") or _default_key
    try:
        client = openai.OpenAI(api_key=api_key, base_url="https://openrouter.ai/api/v1", timeout=30.0)
        client.chat.completions.create(
            model=OPENAI_MODEL,
            messages=[{"role": "user", "content": "hi"}],
            max_tokens=1,
        )
    except Exception as exc:  # noqa: BLE001 - catch everything, never crash the battery
        msg = repr(exc)
        if api_key and api_key in msg:
            msg = msg.replace(api_key, "***")
        return False, msg
    return True, ""


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
        result = subprocess.run(list(reset_cmd), timeout=180, capture_output=True, text=True)
    except subprocess.TimeoutExpired:
        return "reset failed: timed out after 180s"
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

    _write_announcements(run_dir)

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


_ANNOUNCE_RE = re.compile(r"Finished announcing (.+?)(?:\.\s*)?$")


def _write_announcements(run_dir: Path) -> None:
    """Extract everything the robot announced into ``announcements.txt``.

    The user-facing record of Tinker's spoken responses for human
    verification: one line per announcement, in log order, deduplicated
    only when the same text repeats consecutively (the BT re-logs a
    finished announce on several ticks). Sourced from orchestrator.log;
    best-effort — a missing/unreadable log writes nothing.
    """
    log = run_dir / "orchestrator.log"
    if not log.is_file():
        return
    lines: list[str] = []
    last = None
    try:
        with log.open(encoding="utf-8", errors="ignore") as fh:
            for raw in fh:
                m = _ANNOUNCE_RE.search(raw.rstrip())
                if not m:
                    continue
                text = m.group(1).strip()
                if text and text != last:
                    lines.append(text)
                    last = text
    except OSError:
        return
    if lines:
        (run_dir / "announcements.txt").write_text(
            "\n".join(lines) + "\n", encoding="utf-8")


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
              live_llm: bool = True, llm_check: bool = True) -> list[BenchResult]:
    out_dir = Path(out_dir)
    runs_root = out_dir / "runs"
    results: list[BenchResult] = []

    if llm_check and live_llm:
        ok, preflight_detail = llm_preflight(dict(os.environ))
        if not ok:
            out_dir.mkdir(parents=True, exist_ok=True)
            (out_dir / "HALTED").write_text(
                f"LLM preflight failed: {preflight_detail}\n", encoding="utf-8")
            return []

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

        if verdict == "PASS":
            exhausted, split_fell_back = _scan_planner_exhaustion(run_dir / "orchestrator.log")
            if exhausted:
                verdict, detail = "FAIL", PLANNER_EXHAUSTED_DETAIL
            if split_fell_back:
                detail = detail + " | split fell back"

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
