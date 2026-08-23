"""Tier 1: run corpus commands through the real orchestrator process with every ROS boundary mocked."""
from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Sequence

from behavior_tree.GPSR.bench.corpus import CorpusEntry
from behavior_tree.GPSR.bench.events import parse_events
from behavior_tree.GPSR.bench.report import BenchResult

DEFAULT_LAUNCHER = ["ros2", "run", "behavior_tree", "gpsr-orchestrator"]


def bench_env(*, mock_config: Path, constants: Path, plan_dir: Path, commands: Sequence[str],
              live_llm: bool) -> dict[str, str]:
    env = dict(os.environ)
    env.update({
        "BT_GPSR_CMD": "|".join(commands),
        "BT_GPSR_NUM_COMMANDS": str(len(commands)),
        "BT_MOCK_MODE": "true",
        "BT_MOCK_CONFIG": str(mock_config),
        "GPSR_OFFLINE_PLANNER": "0" if live_llm else "1",
        "GPSR_CONSTANTS_PATH": str(constants),
        "BT_GPSR_PLAN_DIR": str(plan_dir),
        "GPSR_DEBUG_TELEMETRY": "1",
        "BT_LISTEN_MOCK_TYPED": "0",
    })
    return env


def _events_file(plan_dir: Path, since: float) -> Path | None:
    debug = Path(plan_dir) / "debug"
    if not debug.is_dir():
        return None
    candidates = [p / "events.jsonl" for p in debug.iterdir() if (p / "events.jsonl").is_file()
                  and (p / "events.jsonl").stat().st_mtime >= since]
    return max(candidates, key=lambda p: p.stat().st_mtime) if candidates else None


def _stop(proc: subprocess.Popen) -> None:
    """Stop the whole process group, not just the direct child.

    DEFAULT_LAUNCHER (`ros2 run ...`) is a wrapper that Popen's the real `gpsr-orchestrator`
    as a grandchild; signalling only `proc` would orphan it. `run_group` launches with
    `start_new_session=True` so `proc`'s pid is also its process group id, letting us signal
    the whole group here.
    """
    if proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait()


def run_group(entries: Sequence[CorpusEntry], *, env: dict[str, str], plan_dir: Path,
              launcher: Sequence[str], timeout_s: float, poll_s: float = 1.0) -> list[BenchResult]:
    plan_dir = Path(plan_dir)
    plan_dir.mkdir(parents=True, exist_ok=True)
    tasks = {}
    exit_code = None
    with (plan_dir / "bench-orchestrator.log").open("a", encoding="utf-8") as log:
        started = time.time()
        proc = subprocess.Popen(list(launcher), env=env, stdout=log, stderr=subprocess.STDOUT,
                                cwd=str(plan_dir), start_new_session=True)
        try:
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                events = _events_file(plan_dir, started - 1)
                if events:
                    tasks = parse_events(events)
                    # Task ids are 1-based (orchestrator.py:2584 telemetry.task_id(slot + 1); see
                    # orchestrator.py _task_identity), so slot N's result lives at tasks[N + 1].
                    if all(tasks.get(i + 1) and tasks[i + 1].status for i in range(len(entries))):
                        break
                exit_code = proc.poll()
                if exit_code is not None:
                    break
                time.sleep(poll_s)
        finally:
            _stop(proc)

    results: list[BenchResult] = []
    timed_out_at = None
    for slot, entry in enumerate(entries):
        task = tasks.get(slot + 1)
        plan = [a for a, _ in task.steps] if task else []
        seconds = time.time() - started
        if task and task.status == "succeeded":
            verdict, detail = "PASS", ""
        elif task and task.status:
            verdict, detail = "FAIL", str(task.reason or task.status)
        elif exit_code is not None:
            verdict, detail = "ERROR", f"orchestrator exited with exit code {exit_code} before slot {slot} finished"
        else:
            if timed_out_at is None:
                timed_out_at = slot
            verdict, detail = "TIMEOUT", f"group timed out at slot {timed_out_at}"
        results.append(BenchResult(entry.id, entry.template, entry.feasibility, 1, verdict, detail, seconds, plan))
    return results


def run_tier1(entries: Sequence[CorpusEntry], *, group_size: int, timeout_s: float, mock_config: Path,
              constants: Path, plan_dir: Path, launcher: Sequence[str] = DEFAULT_LAUNCHER,
              live_llm: bool = True) -> list[BenchResult]:
    results: list[BenchResult] = []
    for start in range(0, len(entries), group_size):
        group = list(entries[start:start + group_size])
        group_dir = Path(plan_dir) / f"group-{start // group_size:03d}"
        env = bench_env(mock_config=mock_config, constants=constants, plan_dir=group_dir,
                        commands=[e.text for e in group], live_llm=live_llm)
        results.extend(run_group(group, env=env, plan_dir=group_dir, launcher=launcher, timeout_s=timeout_s))
    return results
