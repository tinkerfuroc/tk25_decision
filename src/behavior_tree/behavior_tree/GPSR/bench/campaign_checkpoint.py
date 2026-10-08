"""Write cumulative GPSR campaign checkpoints from completed batch artifacts."""

from __future__ import annotations

import argparse
import json
import re
from collections import Counter
from pathlib import Path
from typing import Any


DEFAULT_BENCH = Path(__file__).resolve().parents[1] / "gpsr_runs" / "bench"
STAGE_RE = re.compile(r"\bstage:\s*([^,|]+)", re.IGNORECASE)


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _completed_batch(bench: Path, name: str) -> bool:
    batch = bench / name
    progress = _read_json(batch / "progress.json")
    if progress.get("status") != "complete":
        return False
    battery = batch / "battery.log"
    if not battery.is_file():
        return False
    try:
        text = battery.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return False
    return "gpsr-stack up rc=0" in text and "BATTERY DONE up_rc=0" in text


def _failure_reason(result: dict[str, Any]) -> str:
    detail = str(result.get("physical_detail") or result.get("detail") or "").strip()
    if not detail:
        return "no failure detail"
    return detail.split(" | ", 1)[0]


def _failure_stage(result: dict[str, Any]) -> str:
    """Classify a failed result using its evidence text.

    The stage names are deliberately small and stable because they are used for
    checkpoint comparisons between five-launch windows.  Check the specific
    execution failures before broad words such as ``delivery`` that often occur
    in a postcondition emitted by a failed grasp.
    """
    text = " ".join(
        str(value)
        for value in (
            result.get("physical_detail"),
            result.get("detail"),
            *(check.get("reason", "") for check in result.get("physical_checks", [])
              if isinstance(check, dict)),
        )
        if value
    )
    lowered = text.lower()
    match = STAGE_RE.search(text)
    if match and not match.group(1).strip().isdigit():
        return match.group(1).strip()
    if any(token in lowered for token in (
        "spawn failed", "startup failed", "simulator startup", "stack up",
        "safety starvation", "failed to start", "unable to start",
    )):
        return "startup"
    if "timed out" in lowered or "timeout" in lowered:
        return "timeout"
    if "grasp" in lowered:
        return "grasp"
    if "count" in lowered or "announced" in lowered:
        return "count"
    if any(token in lowered for token in (
        "never handed", "nothing to deliver", "still in the robot's hand",
        "held(", "gripper", "handoff",
    )):
        return "delivery"
    if any(token in lowered for token in (
        "navigation", "navigate", "approach", "robot ended", "distance",
        "go to", "gotoloc", "deliver",
    )):
        return "navigation"
    return "unspecified"


def _trial_rows(bench: Path, batches: list[str]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for batch in batches:
        report = _read_json(bench / batch / "report.json")
        results = report.get("results", [])
        if not isinstance(results, list):
            results = []
        # A normally torn-down launch can still lack a command report (for
        # example, a simulator startup failure).  Keep that launch visible in
        # the checkpoint instead of silently dropping it from the denominator.
        if not results:
            result = {
                "entry_id": f"{batch}:launch",
                "template": "<launch>",
                "verdict": "ERROR",
                "detail": "launch produced no command report",
                "planner_verdict": None,
                "physical_verdict": None,
            }
            results = [result]
        for result in results:
            if not isinstance(result, dict):
                continue
            row = dict(result)
            row["batch"] = batch
            row["failure_reason"] = _failure_reason(result)
            row["failure_stage"] = _failure_stage(result)
            rows.append(row)
    return rows


def _counter(rows: list[dict[str, Any]]) -> dict[str, int]:
    """Return deterministic planner/physical pair counts for a set of rows."""
    return dict(sorted(Counter(
        f"{row.get('planner_verdict') or 'UNKNOWN'}/"
        f"{row.get('physical_verdict') or 'UNKNOWN'}"
        for row in rows
    ).items()))


def _window(index: int, batches: list[str], rows: list[dict[str, Any]], *, size: int) -> dict[str, Any]:
    failures = [
        row for row in rows
        if not (row.get("planner_verdict") == "PASS" and row.get("physical_verdict") == "PASS")
    ]
    return {
        "index": index,
        "start_launch": (index - 1) * size + 1,
        "end_launch": (index - 1) * size + len(batches),
        "complete": len(batches) == size,
        "batches": batches,
        "launches": len(batches),
        "trial_count": len(rows),
        "strict_pass_pass": sum(
            row.get("planner_verdict") == "PASS" and row.get("physical_verdict") == "PASS"
            for row in rows
        ),
        "verdict_pairs": _counter(rows),
        "failure_stages": dict(Counter(
            row["failure_stage"] for row in failures
        ).most_common()),
        # Keep the row IDs and concise evidence in each checkpoint so a human
        # can investigate a window without opening every report directory.
        "failures": [
            {
                "batch": row["batch"],
                "entry_id": row.get("entry_id"),
                "stage": row["failure_stage"],
                "reason": row["failure_reason"],
            }
            for row in failures
        ],
        "successes": [
            {"batch": row["batch"], "entry_id": row.get("entry_id")}
            for row in rows
            if row.get("planner_verdict") == "PASS"
            and row.get("physical_verdict") == "PASS"
        ],
    }


def build_checkpoint(bench: Path, campaign_name: str, completed: int, *, step: int = 5) -> dict[str, Any]:
    bench = Path(bench)
    if step <= 0:
        raise ValueError("checkpoint step must be positive")
    if completed < 0:
        raise ValueError("completed launches must be non-negative")
    campaign = _read_json(bench / campaign_name)
    names = [str(name) for name in campaign.get("batches", [])]
    completed_batches = [name for name in names if _completed_batch(bench, name)]
    if completed > len(completed_batches):
        raise ValueError(f"requested checkpoint {completed}, only {len(completed_batches)} completed launches")
    selected_batches = completed_batches[:completed]
    trials = _trial_rows(bench, selected_batches)
    pair_counts = Counter(_counter(trials))
    failures = [
        row for row in trials
        if not (row.get("planner_verdict") == "PASS" and row.get("physical_verdict") == "PASS")
    ]
    reasons = Counter(row["failure_reason"] for row in failures)
    stages = Counter(row["failure_stage"] for row in failures)
    windows: list[dict[str, Any]] = []
    for offset in range(0, len(selected_batches), step):
        window_batches = selected_batches[offset:offset + step]
        # Batches are unique in a campaign, but retaining report order here
        # makes the output deterministic even if a hand-edited campaign repeats
        # a name.
        window_rows = [
            row for batch in window_batches for row in trials if row["batch"] == batch
        ]
        windows.append(_window(offset // step + 1, window_batches, window_rows, size=step))

    return {
        "schema_version": 2,
        "campaign": campaign.get("name", campaign_name),
        "target": int(campaign.get("target", 20) or 20),
        "window_size": step,
        "completed_launches": completed,
        "batches": selected_batches,
        "trial_count": len(trials),
        "strict_pass_pass": sum(
            row.get("planner_verdict") == "PASS" and row.get("physical_verdict") == "PASS"
            for row in trials
        ),
        "verdict_pairs": dict(sorted(pair_counts.items())),
        "failure_stages": dict(stages.most_common()),
        "failure_reasons": dict(reasons.most_common()),
        "windows": windows,
        "trials": trials,
    }


def _markdown(data: dict[str, Any]) -> str:
    lines = [
        f"# GPSR Campaign Checkpoint ({data['completed_launches']} launches)",
        "",
        f"- Strict planner/physical PASS: **{data['strict_pass_pass']} / {data['trial_count']}**",
        f"- Campaign target: **{data['target']}** launches",
        "",
        f"- Window size: **{data['window_size']} launches**",
        "",
        "## Verdict Pairs",
        "",
        "| Planner / physical | Count |",
        "| --- | ---: |",
    ]
    lines.extend(f"| {key} | {value} |" for key, value in data["verdict_pairs"].items())
    lines.extend(["", "## Failure Stages", "", "| Stage | Count |", "| --- | ---: |"])
    lines.extend(f"| {key} | {value} |" for key, value in data["failure_stages"].items())
    lines.extend(["", "## Failure Reasons", "", "| Reason | Count |", "| --- | ---: |"])
    lines.extend(f"| {key} | {value} |" for key, value in data["failure_reasons"].items())
    lines.extend(["", "## Five-Launch Windows", "", "| Window | Launches | Strict PASS/PASS | Verdict pairs | Failure stages |",
                   "| ---: | ---: | ---: | --- | --- |"])
    for window in data["windows"]:
        pairs = ", ".join(f"{key}: {value}" for key, value in window["verdict_pairs"].items()) or "-"
        stages = ", ".join(f"{key}: {value}" for key, value in window["failure_stages"].items()) or "-"
        complete = "" if window["complete"] else " (partial)"
        lines.append(f"| {window['index']}{complete} | {window['launches']} | "
                     f"{window['strict_pass_pass']}/{window['trial_count']} | {pairs} | {stages} |")
    lines.extend(["", "## Batches", "", "| # | Batch |", "| ---: | --- |"])
    lines.extend(f"| {index} | `{batch}` |" for index, batch in enumerate(data["batches"], 1))
    lines.append("")
    return "\n".join(lines)


def write_due_checkpoints(bench: Path = DEFAULT_BENCH, *, campaign_name: str = "t2-20-live-campaign.json", step: int = 5) -> list[Path]:
    """Write every newly due multiple-of-``step`` checkpoint and return JSON paths."""
    if step <= 0:
        raise ValueError("checkpoint step must be positive")
    campaign = _read_json(bench / campaign_name)
    completed = sum(_completed_batch(bench, str(name)) for name in campaign.get("batches", []))
    written: list[Path] = []
    for threshold in range(step, completed + 1, step):
        stem = bench / f"campaign-checkpoint-{threshold:03d}"
        json_path = stem.with_suffix(".json")
        markdown_path = stem.with_suffix(".md")
        # A checkpoint is immutable evidence for the five launches it names.
        # Avoid rewriting it on every later launch, while still returning paths
        # for checkpoints created in this invocation.
        if json_path.exists() and markdown_path.exists():
            existing = _read_json(json_path)
            if existing.get("schema_version") == 2 and existing.get("window_size") == step:
                continue
        data = build_checkpoint(bench, campaign_name, threshold, step=step)
        json_path.write_text(json.dumps(data, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
        markdown_path.write_text(_markdown(data), encoding="utf-8")
        written.append(json_path)
    return written


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bench", type=Path, default=DEFAULT_BENCH)
    parser.add_argument("--campaign", default="t2-20-live-campaign.json")
    parser.add_argument("--step", type=int, default=5)
    args = parser.parse_args()
    for path in write_due_checkpoints(args.bench.resolve(), campaign_name=args.campaign, step=args.step):
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
