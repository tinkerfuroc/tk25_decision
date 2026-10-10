#!/usr/bin/env python3
"""Seed the GPSR dashboard with the ten-case hardware-free VLM suite."""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import shutil
from typing import Any, Mapping

from behavior_tree.GPSR.supervision.context import FixtureContextProvider
from behavior_tree.GPSR.supervision.scenarios import (
    SCENARIO_CASES,
    build_capture_request,
)
from gpsr_debug_server.store import DebugStore


TRAJECTORY_ID = "gpsr-vlm-ten-case-validation"
BASE_TIME = datetime(2026, 8, 5, 7, 0, tzinfo=timezone.utc)


def _safe_name(value: str) -> str:
    return "".join(
        character if character.isalnum() or character in "-_" else "_"
        for character in value
    )[:100]


def _load_results(path: Path | None) -> dict[str, list[dict[str, Any]]]:
    if path is None or not path.is_file():
        return {}
    document = json.loads(path.read_text(encoding="utf-8"))
    grouped: dict[str, list[dict[str, Any]]] = {}
    for record in document.get("records", []):
        grouped.setdefault(str(record.get("stage_id", "")), []).append(record)
    return grouped


class Replay:
    def __init__(
        self,
        state_dir: Path,
        live_results: Mapping[str, list[dict[str, Any]]],
    ) -> None:
        self.state_dir = state_dir
        self.artifact_dir = state_dir / "artifacts"
        self.artifact_dir.mkdir(parents=True, exist_ok=True)
        self.store = DebugStore(state_dir / "events.sqlite3")
        self.live_results = live_results
        self.sequence = 0
        self.previous_event_id: str | None = None

    def close(self) -> None:
        self.store.close()

    def emit(self, event_type: str, payload: Mapping[str, Any]) -> str:
        self.sequence += 1
        event_id = (
            f"supervisor-suite-{self.sequence:04d}-"
            f"{event_type.replace('.', '-')}"
        )
        event: dict[str, Any] = {
            "schema": "tinker.gpsr.telemetry",
            "schema_version": 1,
            "trajectory_id": TRAJECTORY_ID,
            "sequence": self.sequence,
            "event_id": event_id,
            "type": event_type,
            "phase": (
                "supervision"
                if event_type.startswith("supervisor.")
                else "execution"
            ),
            "occurred_at": (
                BASE_TIME + timedelta(seconds=self.sequence * 2)
            ).isoformat().replace("+00:00", "Z"),
            "payload": dict(payload),
        }
        if self.previous_event_id:
            event["causation_id"] = self.previous_event_id
        self.store.append_event(event)
        self.previous_event_id = event_id
        return event_id

    def checkpoint(self, case, stage) -> None:
        request = build_capture_request(case, stage)
        provider = FixtureContextProvider(
            output_dir=self.artifact_dir,
            clock=lambda: BASE_TIME,
            scenario_id=case.scenario_id,
            require_urdf_renderer=True,
        )
        snapshot = provider.capture(request)
        artifact_records: list[dict[str, Any]] = []
        prefix = _safe_name(stage.stage_id)
        for artifact in snapshot.artifacts:
            record = asdict(artifact)
            if artifact.path and not artifact.missing:
                source = Path(artifact.path)
                target = (
                    self.artifact_dir
                    / f"{prefix}-{artifact.role}{source.suffix.lower()}"
                )
                if source.resolve() != target.resolve():
                    shutil.copyfile(source, target)
                record["path"] = None
                record["url"] = f"/api/v1/artifacts/{target.name}"
            artifact_records.append(record)
        results = self.live_results.get(stage.stage_id, [])
        verify_results = [
            item for item in results if item.get("role") == "verify"
        ]
        planner_results = [
            item for item in results if item.get("role") != "verify"
        ]
        expected = asdict(stage.verification)
        payload = snapshot.to_prompt_dict()
        payload.update(
            {
                "test_case": f"Case {case.number:02d} · {case.title}",
                "case_number": case.number,
                "scenario_id": case.scenario_id,
                "node": dict(request.terminal_node),
                "reported_status": request.terminal_node["reported_status"],
                "risk": stage.risk.value,
                "artifacts": artifact_records,
                "expected_response": expected,
                "expected_planner": (
                    asdict(stage.planner) if stage.planner else None
                ),
                "observed_responses": verify_results,
                "observed_planners": planner_results,
                "live_passed": bool(results)
                and all(item.get("passed") for item in results),
            }
        )
        self.emit("supervisor.checkpoint.created", payload)
        observed = next(
            (
                item.get("observed")
                for item in verify_results
                if item.get("observed")
            ),
            None,
        )
        verdict_payload = dict(observed or expected)
        verdict_payload.update(
            {
                "checkpoint_id": stage.stage_id,
                "expected": expected,
                "samples": verify_results,
            }
        )
        self.emit("supervisor.verdict.received", verdict_payload)
        for result in results:
            self.emit(
                "supervisor.query.completed",
                {
                    "checkpoint_id": stage.stage_id,
                    "role": result.get("role"),
                    "model": "openai/gpt-5.6-luna",
                    "reasoning_effort": (
                        "medium"
                        if result.get("role") == "verify"
                        else "high"
                    ),
                    "latency_ms": result.get("latency_ms"),
                    "attempt": result.get("repetition"),
                    "passed": result.get("passed"),
                    "errors": result.get("errors", []),
                    "response": result.get("observed"),
                },
            )
        if stage.planner:
            event_type = (
                "supervisor.recovery.proposed"
                if stage.planner.role == "local_recovery"
                else "supervisor.global.proposed"
            )
            observed_planner = next(
                (
                    item.get("observed")
                    for item in planner_results
                    if item.get("observed")
                ),
                {},
            )
            action_key = (
                "kind"
                if stage.planner.role == "local_recovery"
                else "action"
            )
            self.emit(
                event_type,
                {
                    "checkpoint_id": stage.stage_id,
                    action_key: (
                        observed_planner.get(action_key)
                        or stage.planner.action
                    ),
                    "expected_action": stage.planner.action,
                    "samples": planner_results,
                    **{
                        key: value
                        for key, value in observed_planner.items()
                        if key != "checkpoint_id"
                    },
                },
            )


def seed(
    state_dir: Path,
    *,
    live_results_path: Path | None = None,
    replace: bool = False,
) -> str:
    state_dir.mkdir(parents=True, exist_ok=True)
    database = state_dir / "events.sqlite3"
    if database.exists() and replace:
        existing = DebugStore(database)
        try:
            existing.delete_trajectory(TRAJECTORY_ID)
        finally:
            existing.close()
    elif database.exists():
        existing = DebugStore(database)
        try:
            if any(
                item["trajectory_id"] == TRAJECTORY_ID
                for item in existing.list_trajectories(limit=500)
            ):
                return TRAJECTORY_ID
        finally:
            existing.close()

    grouped = _load_results(live_results_path)
    all_results = [item for records in grouped.values() for item in records]
    live_complete = len(all_results) == 57
    all_passed = live_complete and all(
        item.get("passed") for item in all_results
    )
    replay = Replay(state_dir, grouped)
    try:
        replay.emit(
            "trajectory.started",
            {
                "name": "GPSR VLM · ten-case hardware-free validation",
                "pinned": True,
                "status": "running",
                "source": (
                    "vision logs + generated camera pairs + arena map + "
                    "scene-only xArm7/Tinker URDF renders"
                ),
                "mode": "hardware-free replay",
                "metadata": {
                    "prompt_version": "gpsr-supervisor-v7",
                    "model": "openai/gpt-5.6-luna",
                    "verify_effort": "medium",
                    "plan_effort": "high",
                    "expected_live_calls": 57,
                },
            },
        )
        replay.emit(
            "supervisor.test.summary",
            {
                "title": "Ten-case GPSR VLM response matrix",
                "status": (
                    "passed"
                    if all_passed
                    else "failed" if live_complete else "not_run"
                ),
                "tests": [
                    {
                        "role": f"Case {case.number:02d}",
                        "status": (
                            "passed"
                            if grouped
                            and all(
                                item.get("passed")
                                for stage in case.stages
                                for item in grouped.get(stage.stage_id, [])
                            )
                            and all(
                                grouped.get(stage.stage_id)
                                for stage in case.stages
                            )
                            else "not_run"
                        ),
                        "coverage": case.title,
                        "model": "openai/gpt-5.6-luna",
                        "effort": "medium verify · high plan",
                        "mode": "3 live repetitions",
                    }
                    for case in SCENARIO_CASES
                ],
                "expected_calls": 57,
                "observed_calls": len(all_results),
                "passed_calls": sum(
                    1 for item in all_results if item.get("passed")
                ),
            },
        )
        for case in SCENARIO_CASES:
            for stage in case.stages:
                replay.checkpoint(case, stage)
        replay.emit(
            "trajectory.completed",
            {
                "status": (
                    "succeeded"
                    if all_passed
                    else "failed" if live_complete else "pending"
                ),
                "completed": live_complete,
                "message": (
                    "All 57 live Luna decisions matched."
                    if all_passed
                    else "Replay seeded; inspect per-case live results."
                ),
            },
        )
    finally:
        replay.close()
    return TRAJECTORY_ID


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--state-dir",
        type=Path,
        default=Path("/tmp/gpsr-supervisor-dashboard"),
        help="Debugger state directory to populate.",
    )
    parser.add_argument(
        "--live-results",
        type=Path,
        help="Optional 57-call JSON report produced by the live pytest.",
    )
    parser.add_argument(
        "--replace",
        action="store_true",
        help="Replace only this seeded trajectory if it already exists.",
    )
    args = parser.parse_args()
    trajectory_id = seed(
        args.state_dir,
        live_results_path=args.live_results,
        replace=args.replace,
    )
    print(f"Seeded {trajectory_id} in {args.state_dir}")


if __name__ == "__main__":
    main()
