"""Serve a read-only local browser for GPSR tier-2 run artifacts."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import mimetypes
import re
import shutil
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, quote, unquote, urlparse


DEFAULT_BENCH_DIR = Path(__file__).resolve().parents[1] / "gpsr_runs" / "bench"
BATCH_RE = re.compile(
    r"^(?:t2-20-live|t2-15-distinct|t2-diverse15|t2-five-diverse-gpu1-\d{8})(?:-rerun\d+)?$"
)
MEDIA_SUFFIXES = {".jpg", ".jpeg", ".png", ".webp", ".db3", ".zstd", ".yaml", ".json", ".txt", ".log"}
# The active qualification target is the seeded diverse-command campaign. The
# older count-heavy campaigns remain exposed in ``campaigns`` for context.
CAMPAIGN_FILE = "t2-diverse15-campaign.json"
CAMPAIGN_FILES = (
    "t2-20-live-campaign.json",
    "t2-15-distinct-campaign.json",
    "t2-diverse15-campaign.json",
)
FIVE_BATCH_PREFIX = "t2-five-diverse-gpu1-"


def read_json(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}
    except (OSError, json.JSONDecodeError):
        return {}


def read_corpus(path: Path) -> dict[str, dict]:
    entries: dict[str, dict] = {}
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            try:
                entry = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(entry, dict) and isinstance(entry.get("id"), str):
                entries[entry["id"]] = entry
    except OSError:
        pass
    return entries


def batch_sort_key(path: Path) -> tuple[int, str]:
    match = re.search(r"rerun(\d+)$", path.name)
    return (int(match.group(1)) if match else 0, path.name)


def batch_trials(batch_dir: Path) -> tuple[dict, list[dict]]:
    progress = read_json(batch_dir / "progress.json")
    report = read_json(batch_dir / "report.json")
    corpus = read_corpus(batch_dir / "corpus.jsonl")
    progress_results = progress.get("results") if isinstance(progress.get("results"), list) else []
    report_results = report.get("results") if isinstance(report.get("results"), list) else []
    result_by_id: dict[str, dict] = {}
    # A completed report is rescored and authoritative.  Progress is useful
    # while a run is live, but may retain the pre-rescore verdict afterward.
    for result in [*progress_results, *report_results]:
        if isinstance(result, dict):
            entry_id = result.get("entry_id", result.get("id"))
            if isinstance(entry_id, str):
                result_by_id[entry_id] = result

    run_dirs = batch_dir / "runs"
    if run_dirs.is_dir():
        for run_dir in run_dirs.iterdir():
            if not run_dir.is_dir():
                continue
            run_meta = read_json(run_dir / "run.json")
            truth = read_json(run_dir / "run_truth.json")
            entry_id = run_meta.get("id") or run_dir.name
            if entry_id not in result_by_id:
                result_by_id[entry_id] = {}
            # The report/progress result is the authoritative scored view.  A
            # raw run.json can contain the pre-rescore verdict (the historical
            # T2 runs did), so let it provide defaults while preserving any
            # planner/physical verdicts and evidence already regenerated in
            # report.json.
            merged = dict(run_meta)
            merged.update(result_by_id[entry_id])
            if truth:
                merged.setdefault("physical_verdict", truth.get("verdict"))
                merged.setdefault("physical_detail", truth.get("detail", ""))
            result_by_id[entry_id] = merged

    trials: list[dict] = []
    for entry_id, result in result_by_id.items():
        source = corpus.get(entry_id, {})
        run_id = source.get("id", entry_id)
        if not isinstance(run_id, str) or Path(run_id).name != run_id or run_id in {".", ".."}:
            continue
        run_dir = batch_dir / "runs" / run_id
        result = dict(result)
        result["entry_id"] = entry_id
        result["text"] = result.get("text") or source.get("text", "")
        result["seed"] = source.get("seed")
        result["followups"] = source.get("followups", [])
        result["batch"] = batch_dir.name
        result["has_run_dir"] = run_dir.is_dir()
        # Keep planner and physical evidence separate in the API.  Older
        # reports use ``detail`` for the planner gate and newer reports add
        # ``physical_detail``; never let the viewer display the latter as
        # planner evidence.
        result["planner_verdict"] = result.get("planner_verdict") or (
            result.get("verdict") if result.get("verdict") in {"PASS", "FAIL", "TIMEOUT", "ERROR"} else None
        )
        # New reports carry an explicit planner_detail, including the empty
        # string for a clean planner pass/timeout.  Only legacy artifacts that
        # lack that field should fall back to their combined ``detail`` text;
        # otherwise physical evidence leaks into the planner column.
        if "planner_detail" not in result:
            result["planner_detail"] = result.get("detail") or ""
        else:
            result["planner_detail"] = result.get("planner_detail") or ""
        result["planner_plan"] = result.get("planner_plan") or result.get("plan") or []
        result["physical_detail"] = result.get("physical_detail") or ""
        trials.append(result)

    trials.sort(key=lambda item: (item.get("entry_id", ""), item.get("batch", "")))
    info = {
        "name": batch_dir.name,
        "status": progress.get("status") or ("complete" if report else "unknown"),
        "completed": progress.get("completed", len(report_results) or len(progress_results)),
        "expected": progress.get("expected"),
        "updated": progress.get("updated") or report.get("meta", {}).get("timestamp"),
        "streak": progress.get("streak") or report.get("streak", {}),
    }
    battery_log = batch_dir / "battery.log"
    battery_text = battery_log.read_text(encoding="utf-8", errors="replace") if battery_log.is_file() else ""
    info["stack_started"] = "gpsr-stack up rc=0" in battery_text
    info["launch_success"] = (
        info["stack_started"]
        and "BATTERY DONE up_rc=0" in battery_text
        and info["status"] == "complete"
    )
    return info, trials


def discover_batches(bench_dir: Path) -> list[Path]:
    if not bench_dir.is_dir():
        return []
    batches = [path for path in bench_dir.iterdir() if path.is_dir() and BATCH_RE.fullmatch(path.name)]
    return sorted(batches, key=batch_sort_key)


def trial_detail(bench_dir: Path, batch: str, entry_id: str) -> dict | None:
    if not BATCH_RE.fullmatch(batch) or Path(entry_id).name != entry_id or entry_id in {".", ".."}:
        return None
    batch_dir = bench_dir / batch
    run_dir = batch_dir / "runs" / entry_id
    try:
        run_dir.resolve().relative_to(batch_dir.resolve())
    except ValueError:
        return None
    if not run_dir.is_dir():
        return None

    _, trials = batch_trials(batch_dir)
    trial = next((item for item in trials if item["entry_id"] == entry_id), {})
    frame_meta, recording_start, recording_end = _frame_metadata(run_dir, batch, entry_id)
    # ``cameras`` remains a list of URLs for compatibility with older viewers/API
    # clients. ``camera_frames`` adds timestamps needed for timeline scrubbing.
    cameras = {name: [frame["url"] for frame in frames] for name, frames in frame_meta.items()}
    files = {}
    for name in ("sheet.jpg", "judge-sheet.jpg", "run.json", "run_truth.json", "scene-plan.json", "spawned.json", "announcements.txt", "orchestrator.log", "recorder.log", "rosbag.log"):
        if (run_dir / name).is_file():
            files[name] = media_url(batch, entry_id, name, download=name.startswith("rosbag"))
    bag_dir = run_dir / "rosbag"
    if bag_dir.is_dir():
        files["rosbag"] = [
            {"name": path.relative_to(run_dir).as_posix(), "url": media_url(batch, entry_id, path.relative_to(run_dir).as_posix(), download=True)}
            for path in sorted(bag_dir.rglob("*")) if path.is_file()
        ]
    logs_dir = run_dir / "logs"
    if logs_dir.is_dir():
        files["command_logs"] = [
            {"name": path.name, "url": media_url(batch, entry_id, path.relative_to(run_dir).as_posix())}
            for path in sorted(logs_dir.glob("*.log"))
        ]
    recording = {
        "start_s": recording_start,
        "end_s": recording_end,
        "duration_s": round(max(0.0, recording_end - recording_start), 3) if recording_start is not None and recording_end is not None else None,
        "timeline": _execution_timeline(run_dir, recording_start, recording_end),
    }
    return {"trial": trial, "cameras": cameras, "camera_frames": frame_meta, "recording": recording, "files": files}


def media_url(batch: str, entry_id: str, relative: str, download: bool = False) -> str:
    suffix = "&download=1" if download else ""
    return f"/media?batch={quote(batch)}&entry={quote(entry_id)}&file={quote(relative, safe='')}{suffix}"


def _event_timestamp(value: object) -> float | None:
    """Convert telemetry ISO timestamps to epoch seconds for frame alignment."""
    if not isinstance(value, str) or not value:
        return None
    try:
        return datetime.fromisoformat(value.replace("Z", "+00:00")).timestamp()
    except ValueError:
        return None


def _frame_metadata(run_dir: Path, batch: str, entry_id: str) -> tuple[dict[str, list[dict]], float | None, float | None]:
    """Read recorder frame timestamps while retaining the old camera URL shape."""
    frames_dir = run_dir / "frames"
    metadata: dict[str, list[dict]] = {}
    stamps: list[float] = []
    wall_stamps: list[float] = []
    index_path = frames_dir / "index.jsonl"
    if index_path.is_file():
        try:
            lines = index_path.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            lines = []
        for line in lines:
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                continue
            label = item.get("label")
            relative = item.get("file")
            stamp = item.get("stamp_s")
            if not isinstance(label, str) or not isinstance(relative, str):
                continue
            try:
                stamp = float(stamp)
            except (TypeError, ValueError):
                stamp = None
            wall_s = _event_timestamp(item.get("wall"))
            if stamp is not None:
                stamps.append(stamp)
            if wall_s is not None:
                wall_stamps.append(wall_s)
            metadata.setdefault(label, []).append({
                "url": media_url(batch, entry_id, relative),
                "stamp_s": stamp,
                "wall": item.get("wall"),
                "wall_s": wall_s,
            })
    # Old runs can have frames but no index. Keep them visible with unknown time.
    if not metadata and frames_dir.is_dir():
        for camera_dir in sorted(path for path in frames_dir.iterdir() if path.is_dir()):
            metadata[camera_dir.name] = [
                {"url": media_url(batch, entry_id, path.relative_to(run_dir).as_posix()), "stamp_s": None, "wall": None}
                for path in sorted(camera_dir.iterdir())
                if path.is_file() and path.suffix.lower() in {".jpg", ".jpeg", ".png", ".webp"}
            ]
    # Recorder files contain both a simulator/ROS stamp and a wall-clock ISO
    # timestamp. The telemetry stream uses wall-clock timestamps, so prefer
    # that shared clock whenever it is available. The simulator stamp can run
    # at a different rate and otherwise collapses execution markers at the
    # end of the recording.
    use_wall = bool(wall_stamps)
    time_stamps = wall_stamps if use_wall else stamps
    start_s = min(time_stamps) if time_stamps else None
    end_s = max(time_stamps) if time_stamps else None
    for values in metadata.values():
        for index, frame in enumerate(values):
            frame["index"] = index
            frame_time = frame.get("wall_s") if use_wall else frame.get("stamp_s")
            frame["time_s"] = frame_time - start_s if start_s is not None and frame_time is not None else None
    return metadata, start_s, end_s


def _debug_events(run_dir: Path) -> list[dict]:
    """Read the newest structured telemetry stream for a trial."""
    debug_dir = run_dir / "debug"
    candidates = [path / "events.jsonl" for path in debug_dir.iterdir() if (path / "events.jsonl").is_file()] if debug_dir.is_dir() else []
    if not candidates:
        return []
    path = max(candidates, key=lambda item: item.stat().st_mtime)
    events: list[dict] = []
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return []
    for line in lines:
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(event, dict) and _event_timestamp(event.get("occurred_at")) is not None:
            event["_time_s"] = _event_timestamp(event.get("occurred_at"))
            events.append(event)
    return events


def _execution_timeline(run_dir: Path, recording_start: float | None, recording_end: float | None) -> list[dict]:
    """Fold plan/step/gate/tree telemetry into scrubber-friendly execution markers."""
    events = _debug_events(run_dir)
    if not events:
        return []
    start = recording_start
    if start is None:
        start = min(event["_time_s"] for event in events)
    end = recording_end if recording_end is not None else max(event["_time_s"] for event in events)
    node_names: dict[str, str] = {}
    node_states: dict[str, dict] = {}
    markers: list[dict] = []
    last_active: tuple[str, ...] | None = None

    def add_marker(event: dict, kind: str, label: str, *, status: str | None = None, detail: str = "", **extra: object) -> None:
        raw_time = float(event["_time_s"])
        # Markers outside the camera recording are retained at the nearest edge;
        # this makes the first/last frame explainable even when telemetry starts earlier.
        relative = max(0.0, min(raw_time - start, max(0.0, end - start)))
        marker = {"time_s": round(relative, 3), "at": event.get("occurred_at"), "kind": kind, "label": label}
        if status:
            marker["status"] = status
        if detail:
            marker["detail"] = detail
        marker.update({key: value for key, value in extra.items() if value is not None})
        markers.append(marker)

    for event in events:
        kind = event.get("event_type")
        payload = event.get("payload")
        if not isinstance(payload, dict):
            payload = {}
        if kind == "tree.generated":
            for node in payload.get("nodes", []):
                if not isinstance(node, dict):
                    continue
                node_id = node.get("id") or node.get("node_id")
                name = node.get("name")
                if node_id and name:
                    node_names[str(node_id)] = str(name)
            continue
        if kind == "plan.materialized":
            target_index = payload.get("target_index")
            for index, step in enumerate(payload.get("steps") or []):
                if not isinstance(step, dict):
                    continue
                action = str(step.get("action") or "step")
                params = step.get("params") or {}
                detail = ", ".join(f"{key}={value}" for key, value in params.items())
                add_marker(event, "planned", f"Plan: {action}", detail=detail, action=action, target_index=target_index, step_index=index)
        elif kind == "step.finished":
            action = str(payload.get("action") or "step")
            params = payload.get("params") or {}
            detail = ", ".join(f"{key}={value}" for key, value in params.items())
            feedback = payload.get("feedback")
            if feedback:
                detail = f"{detail}; {feedback}" if detail else str(feedback)
            add_marker(event, "step", f"{action}: {payload.get('outcome', 'finished')}", status=str(payload.get("outcome") or ""), detail=detail, action=action, step_index=payload.get("step_index"), target_index=payload.get("target_index"), method=payload.get("method"))
        elif kind == "gate.verified":
            fact = str(payload.get("fact") or "gate")
            verdict = str(payload.get("verdict") or "UNKNOWN")
            add_marker(event, "gate", f"Gate: {fact} ({verdict})", status=verdict, detail=str(payload.get("reason") or ""), target_index=payload.get("target_index"), phase=payload.get("phase"))
        elif kind == "run.started":
            add_marker(event, "run", "Run started")
        elif kind == "run.finished":
            add_marker(event, "run", f"Run {payload.get('status', 'finished')}", status=str(payload.get("status") or ""))
        elif kind == "tree.node_states_changed":
            for node in payload.get("nodes", []):
                if not isinstance(node, dict):
                    continue
                node_id = str(node.get("id") or node.get("node_id") or "")
                if node_id:
                    node_states[node_id] = node
            active = tuple(sorted(node_id for node_id, node in node_states.items() if node.get("status") == "RUNNING"))
            if active != last_active:
                last_active = active
                # Deepest running leaf is the most useful description at a scrub position.
                chosen = max(active, key=lambda node_id: node_id.count("/"), default=None)
                if chosen:
                    node = node_states[chosen]
                    add_marker(event, "behavior_tree", node_names.get(chosen, chosen), status="RUNNING", detail=str(node.get("feedback") or ""), node_id=chosen)

    markers.sort(key=lambda item: (item["time_s"], item["kind"] == "behavior_tree"))
    # Adjacent telemetry updates often describe the same node. Keep the API compact.
    compact: list[dict] = []
    for marker in markers:
        if compact and marker["time_s"] == compact[-1]["time_s"] and marker["label"] == compact[-1]["label"] and marker.get("status") == compact[-1].get("status"):
            continue
        compact.append(marker)
    return compact


def create_handler(bench_dir: Path, html_path: Path):
    class Handler(BaseHTTPRequestHandler):
        server_version = "GPSRTrialViewer/1.0"

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            query = parse_qs(parsed.query)
            if parsed.path in {"/", "/index.html"}:
                self._send_file(html_path, inline=True)
            elif parsed.path == "/api/trials":
                self._send_json(self._all_trials())
            elif parsed.path == "/api/trial":
                batch = query.get("batch", [""])[0]
                entry_id = query.get("entry", [""])[0]
                detail = trial_detail(bench_dir, batch, entry_id)
                self._send_json(detail or {"error": "trial not found"}, status=200 if detail else 404)
            elif parsed.path == "/media":
                self._send_media(query)
            else:
                self.send_error(404)

        def _all_trials(self) -> dict:
            batches = []
            trials = []
            for batch_dir in discover_batches(bench_dir):
                info, items = batch_trials(batch_dir)
                batches.append(info)
                trials.extend(items)
            trials.sort(key=lambda item: (batch_sort_key(bench_dir / item.get("batch", "")), item.get("entry_id", "")))
            campaign_summaries = []
            for campaign_file in CAMPAIGN_FILES:
                campaign = read_json(bench_dir / campaign_file)
                if not campaign and not (bench_dir / campaign_file).is_file():
                    # The distinct runner creates its campaign manifest only
                    # after the first clean batch. Expose its live batches as
                    # soon as the corpus directory exists so the viewer can
                    # show progress while the first run is still underway.
                    if campaign_file == "t2-15-distinct-campaign.json" and (bench_dir / "t2-15-distinct").is_dir():
                        campaign = {
                            "name": "gpsr-distinct15",
                            "target": 15,
                            "batches": [path.name for path in discover_batches(bench_dir) if path.name.startswith("t2-15-distinct")],
                        }
                    elif campaign_file == "t2-diverse15-campaign.json" and (bench_dir / "t2-diverse15").is_dir():
                        campaign = {
                            "name": "gpsr-diverse15",
                            "target": 15,
                            "batches": [path.name for path in discover_batches(bench_dir) if path.name.startswith("t2-diverse15")],
                        }
                    else:
                        continue
                campaign_batches = campaign.get("batches") if isinstance(campaign.get("batches"), list) else []
                selected = set(str(name) for name in campaign_batches)
                campaign_items = [batch for batch in batches if batch.get("name") in selected]
                launch_successes = sum(1 for batch in campaign_items if batch.get("launch_success"))
                selected_trials = [trial for trial in trials if trial.get("batch") in selected]
                passed_trials = [
                    trial for trial in trials
                    if trial.get("batch") in selected
                    and trial.get("planner_verdict") == "PASS"
                    and trial.get("physical_verdict") == "PASS"
                ]
                unique_pass_ids = {str(trial.get("entry_id")) for trial in passed_trials if trial.get("entry_id")}
                # The distinct-command campaign has one target per corpus ID;
                # retries must not inflate its progress. Preserve the original
                # launch-count semantics for the historical 20-launch campaign.
                is_distinct = "15-distinct" in campaign_file
                is_diverse = "diverse15" in campaign_file
                # Both qualification campaigns require distinct command IDs.
                # A retry may add another PASS row for the same command, but it
                # must not inflate the 15-command progress counter.
                clean_passes = len(unique_pass_ids) if (is_distinct or is_diverse) else len(passed_trials)
                recorded_ids = {
                    str(trial.get("entry_id")) for trial in selected_trials
                    if trial.get("entry_id") and (
                        trial.get("planner_verdict") is not None
                        or trial.get("physical_verdict") is not None
                    )
                }
                target = int(campaign.get("target", 15 if "15-distinct" in campaign_file else 20) or (15 if "15-distinct" in campaign_file else 20))
                template_stats = {}
                for trial in selected_trials:
                    template = str(trial.get("template") or "unknown")
                    stats = template_stats.setdefault(template, {"total": 0, "planner_passes": 0, "execution_passes": 0, "strict_passes": 0})
                    stats["total"] += 1
                    if trial.get("planner_verdict") == "PASS":
                        stats["planner_passes"] += 1
                    if trial.get("verdict") == "PASS":
                        stats["execution_passes"] += 1
                    if trial.get("planner_verdict") == "PASS" and trial.get("physical_verdict") == "PASS":
                        stats["strict_passes"] += 1
                campaign_summaries.append({
                    "file": campaign_file,
                    "name": campaign.get("name", "gpsr-15-distinct-commands" if "15-distinct" in campaign_file else "gpsr-20-launches"),
                    "target": target,
                    "batches": campaign_batches,
                    "launch_successes": launch_successes,
                    "recorded_commands": len(recorded_ids),
                    "scored_commands": len({str(trial.get("entry_id")) for trial in selected_trials if trial.get("entry_id")}),
                    "remaining": max(0, target - clean_passes),
                    "clean_passes": clean_passes,
                    "unique_clean_passes": len(unique_pass_ids),
                    "distinct_commands": is_distinct,
                    "diverse_commands": is_diverse,
                    "planner_passes": sum(1 for trial in selected_trials if trial.get("planner_verdict") == "PASS"),
                    "execution_passes": sum(1 for trial in selected_trials if trial.get("verdict") == "PASS"),
                    "physical_passes": sum(1 for trial in selected_trials if trial.get("physical_verdict") == "PASS"),
                    "template_stats": template_stats,
                    "met": clean_passes >= target,
                })
            # The short GPU-1 verification run is intentionally exposed as its
            # own campaign. It has no manifest because it was a one-off batch,
            # but it is the newest experiment and should be the viewer's main
            # dataset instead of being mixed into historical runs.
            five_candidates = [
                batch for batch in batches
                if str(batch.get("name", "")).startswith(FIVE_BATCH_PREFIX)
            ]
            five_batches = []
            if five_candidates:
                five_batches = [max(
                    five_candidates,
                    key=lambda item: (str(item.get("updated") or ""), str(item.get("name", ""))),
                )]
            if five_batches:
                five_names = {str(batch.get("name")) for batch in five_batches}
                selected_trials = [trial for trial in trials if trial.get("batch") in five_names]
                passed_trials = [
                    trial for trial in selected_trials
                    if trial.get("planner_verdict") == "PASS"
                    and trial.get("physical_verdict") == "PASS"
                ]
                unique_pass_ids = {str(trial.get("entry_id")) for trial in passed_trials if trial.get("entry_id")}
                recorded_ids = {
                    str(trial.get("entry_id")) for trial in selected_trials
                    if trial.get("entry_id") and (
                        trial.get("planner_verdict") is not None
                        or trial.get("physical_verdict") is not None
                    )
                }
                campaign_summaries.append({
                    "file": None,
                    "name": "gpsr-five-diverse",
                    "target": 5,
                    "batches": sorted(five_names),
                    "launch_successes": sum(1 for batch in five_batches if batch.get("launch_success")),
                    "recorded_commands": len(recorded_ids),
                    "scored_commands": len({str(trial.get("entry_id")) for trial in selected_trials if trial.get("entry_id")}),
                    "remaining": max(0, 5 - len(unique_pass_ids)),
                    "clean_passes": len(unique_pass_ids),
                    "unique_clean_passes": len(unique_pass_ids),
                    "five_commands": True,
                    "distinct_commands": True,
                    "diverse_commands": True,
                    "planner_passes": sum(1 for trial in selected_trials if trial.get("planner_verdict") == "PASS"),
                    "execution_passes": sum(1 for trial in selected_trials if trial.get("verdict") == "PASS"),
                    "physical_passes": sum(1 for trial in selected_trials if trial.get("physical_verdict") == "PASS"),
                    "template_stats": {},
                    "met": len(unique_pass_ids) >= 5,
                })
            # Annotate every row so clients can separate the active distinct
            # command qualification from the historical launch campaign.
            memberships: dict[str, list[dict]] = {}
            for item in campaign_summaries:
                membership = {
                    "file": item["file"],
                    "name": item["name"],
                    "distinct_commands": item["distinct_commands"],
                    "diverse_commands": item.get("diverse_commands", False),
                    "five_commands": item.get("five_commands", False),
                    "manifested": True,
                }
                for batch_name in item["batches"]:
                    memberships.setdefault(str(batch_name), []).append(membership)
            for trial in trials:
                batch_name = str(trial.get("batch"))
                trial["campaigns"] = memberships.get(batch_name, [])
                # A live batch is created before the runner records it in the
                # manifest. Keep it visible in the active filter without
                # allowing it to affect the strict campaign count.
                if not trial["campaigns"] and batch_name.startswith("t2-15-distinct-"):
                    trial["campaigns"] = [{
                        "file": "t2-15-distinct-campaign.json",
                        "name": "gpsr-distinct15",
                        "distinct_commands": True,
                        "manifested": False,
                    }]
                if not trial["campaigns"] and batch_name.startswith("t2-diverse15-"):
                    trial["campaigns"] = [{
                        "file": "t2-diverse15-campaign.json",
                        "name": "gpsr-diverse15",
                        "diverse_commands": True,
                        "manifested": False,
                    }]
                if not trial["campaigns"] and batch_name.startswith(FIVE_BATCH_PREFIX):
                    trial["campaigns"] = [{
                        "file": None,
                        "name": "gpsr-five-diverse",
                        "five_commands": True,
                        "distinct_commands": True,
                        "diverse_commands": True,
                        "manifested": False,
                    }]
            # Keep the original singular field for existing clients; expose every
            # configured campaign so the distinct-command run is visible beside it.
            campaign = next((item for item in campaign_summaries if item["file"] == CAMPAIGN_FILE), None)
            if campaign is None and campaign_summaries:
                campaign = campaign_summaries[0]
            if campaign is None:
                campaign = {
                    "file": CAMPAIGN_FILE,
                    "name": "gpsr-diverse15",
                    "target": 15,
                    "batches": [],
                    "launch_successes": 0,
                    "remaining": 15,
                    "clean_passes": 0,
                    "unique_clean_passes": 0,
                    "distinct_commands": True,
                    "diverse_commands": True,
                    "met": False,
                }
            def updated_key(item: dict) -> str:
                value = item.get("updated")
                return str(value) if isinstance(value, str) else ""

            latest_batch = max(
                batches,
                key=lambda item: (updated_key(item), str(item.get("name", ""))),
                default=None,
            )
            latest_batch_name = latest_batch.get("name") if latest_batch else None
            latest_campaign = next(
                (
                    item for item in campaign_summaries
                    if latest_batch_name and latest_batch_name in item.get("batches", [])
                ),
                None,
            )
            five_campaign = next(
                (item for item in campaign_summaries if item.get("five_commands")),
                None,
            )
            latest_trials = [trial for trial in trials if trial.get("batch") == latest_batch_name]
            latest_passes = [
                trial for trial in latest_trials
                if trial.get("planner_verdict") == "PASS"
                and trial.get("physical_verdict") == "PASS"
            ]
            latest_recorded = {
                str(trial.get("entry_id")) for trial in latest_trials
                if trial.get("entry_id") and (
                    trial.get("planner_verdict") is not None
                    or trial.get("physical_verdict") is not None
                )
            }
            latest_experiment = None
            if latest_batch is not None:
                latest_experiment = dict(latest_batch)
                latest_experiment.update({
                    "file": latest_campaign.get("file") if latest_campaign else None,
                    "name": latest_batch_name,
                    "latest": True,
                    "target": int(latest_batch.get("expected") or len(latest_trials) or 0),
                    "batches": [latest_batch_name],
                    "recorded_commands": len(latest_recorded),
                    "scored_commands": len({str(trial.get("entry_id")) for trial in latest_trials if trial.get("entry_id")}),
                    "remaining": max(0, int(latest_batch.get("expected") or len(latest_trials) or 0) - len(latest_passes)),
                    "clean_passes": len(latest_passes),
                    "unique_clean_passes": len({str(trial.get("entry_id")) for trial in latest_passes if trial.get("entry_id")}),
                    "five_commands": bool(latest_campaign and latest_campaign.get("five_commands")),
                    "distinct_commands": bool(latest_campaign and latest_campaign.get("distinct_commands")),
                    "diverse_commands": bool(latest_campaign and latest_campaign.get("diverse_commands")),
                    "planner_passes": sum(1 for trial in latest_trials if trial.get("planner_verdict") == "PASS"),
                    "execution_passes": sum(1 for trial in latest_trials if trial.get("verdict") == "PASS"),
                    "physical_passes": sum(1 for trial in latest_trials if trial.get("physical_verdict") == "PASS"),
                    "template_stats": {},
                    "met": bool(latest_batch.get("expected") and len(latest_passes) >= int(latest_batch["expected"])),
                })
            for item in campaign_summaries:
                item["latest"] = bool(latest_campaign is item)
            return {
                "batches": batches,
                "trials": trials,
                "campaign": campaign,
                "campaigns": campaign_summaries,
                "latest_batch": latest_batch,
                "latest_campaign": latest_campaign,
                "five_campaign": five_campaign,
                "latest_experiment": latest_experiment,
            }

        def _send_json(self, value: dict, status: int = 200) -> None:
            body = json.dumps(value, ensure_ascii=True).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _send_media(self, query: dict[str, list[str]]) -> None:
            batch = query.get("batch", [""])[0]
            entry_id = query.get("entry", [""])[0]
            relative = unquote(query.get("file", [""])[0])
            if not BATCH_RE.fullmatch(batch) or Path(entry_id).name != entry_id or entry_id in {".", ".."}:
                self.send_error(404)
                return
            run_dir = bench_dir / batch / "runs" / entry_id
            target = (run_dir / relative).resolve()
            try:
                target.relative_to(run_dir.resolve())
            except ValueError:
                self.send_error(404)
                return
            if not target.is_file() or target.suffix.lower() not in MEDIA_SUFFIXES:
                self.send_error(404)
                return
            self._send_file(target, inline=query.get("download", [""])[0] != "1")

        def _send_file(self, path: Path, *, inline: bool) -> None:
            try:
                size = path.stat().st_size
                handle = path.open("rb")
            except OSError:
                self.send_error(404)
                return
            content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(size))
            disposition = "inline" if inline else "attachment"
            self.send_header("Content-Disposition", f"{disposition}; filename*=UTF-8''{quote(path.name)}")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            with handle:
                shutil.copyfileobj(handle, self.wfile, length=1024 * 128)

        def log_message(self, fmt: str, *args) -> None:
            print(f"[viewer] {self.address_string()} {fmt % args}")

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bench-dir", type=Path, default=DEFAULT_BENCH_DIR)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    bench_dir = args.bench_dir.resolve()
    html_path = Path(__file__).with_name("trial_viewer.html")
    server = ThreadingHTTPServer((args.host, args.port), create_handler(bench_dir, html_path))
    print(f"GPSR trial viewer: http://{args.host}:{server.server_port}/")
    print(f"Reading trials from {bench_dir}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
