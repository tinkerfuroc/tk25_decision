"""Opt-in 57-call Luna regression for the ten hardware-free scenarios."""
from __future__ import annotations

from dataclasses import asdict
import json
import os
from pathlib import Path
import time

import pytest

from behavior_tree.GPSR.config import OPENAI_API_KEY
from behavior_tree.GPSR.supervision.clients import OpenRouterSupervisorClient
from behavior_tree.GPSR.supervision.context import FixtureContextProvider
from behavior_tree.GPSR.supervision.models import (
    SupervisorConfig,
    VerificationDecision,
)
from behavior_tree.GPSR.supervision.prompts import PROMPT_VERSION
from behavior_tree.GPSR.supervision.recovery import (
    validate_global_decision,
    validate_recovery_macro,
)
from behavior_tree.GPSR.supervision.scenarios import (
    build_capture_request,
    iter_stages,
)


pytestmark = pytest.mark.live_openrouter


def _enabled() -> bool:
    return os.environ.get("GPSR_RUN_LIVE_LLM_TESTS", "0").lower() in {
        "1",
        "true",
        "yes",
        "on",
    }


def _expected_decision(stage) -> VerificationDecision:
    item = stage.verification
    return VerificationDecision.from_dict(
        {
            "checkpoint_id": stage.stage_id,
            "verdict": item.verdict,
            "bt_assessment": item.bt_assessment,
            "subtask_status": item.subtask_status,
            "world_change": item.world_change,
            "escalation": item.escalation,
            "failure_category": item.failure_category,
            "evidence": ["scenario expectation supplied to planner tests"],
            "rationale": "Planner isolation uses the canonical verifier decision.",
            "confidence": 1.0,
        }
    )


def _serialise(value):
    return json.loads(json.dumps(asdict(value), default=str))


def _verification_errors(stage, observed: VerificationDecision) -> list[str]:
    expected = stage.verification
    fields = {
        "verdict": observed.verdict.value,
        "bt_assessment": observed.bt_assessment.value,
        "subtask_status": observed.subtask_status.value,
        "world_change": observed.world_change.value,
        "escalation": observed.escalation.value,
    }
    errors = [
        f"{name}: expected {getattr(expected, name)!r}, got {actual!r}"
        for name, actual in fields.items()
        if getattr(expected, name) != actual
    ]
    if (
        expected.failure_category == "sensor_context_mismatch"
        and observed.failure_category != expected.failure_category
    ):
        errors.append(
            "failure_category: expected 'sensor_context_mismatch', got "
            f"{observed.failure_category!r}"
        )
    evidence = " ".join((*observed.evidence, observed.rationale)).lower()
    if expected.evidence_terms and not any(
        term.lower() in evidence for term in expected.evidence_terms
    ):
        errors.append(
            "evidence did not mention any anchor: "
            + ", ".join(expected.evidence_terms)
        )
    return errors


def _known_locations() -> set[str]:
    constants_path = (
        Path(__file__).resolve().parents[1]
        / "behavior_tree/GPSR/constants.json"
    )
    raw = json.loads(constants_path.read_text(encoding="utf-8"))
    return set(raw["possible_poses"]) | {"start_position", "kitchen"}


def _selected_stages():
    requested = {
        item.strip()
        for item in os.environ.get("GPSR_LIVE_STAGE_FILTER", "").split(",")
        if item.strip()
    }
    stages = list(iter_stages())
    if not requested:
        return stages
    selected = [
        (case, stage)
        for case, stage in stages
        if stage.stage_id in requested
    ]
    missing = requested - {stage.stage_id for _, stage in selected}
    if missing:
        raise ValueError(
            "unknown GPSR_LIVE_STAGE_FILTER values: " + ", ".join(sorted(missing))
        )
    return selected


@pytest.mark.skipif(
    not _enabled(),
    reason="set GPSR_RUN_LIVE_LLM_TESTS=1",
)
def test_luna_three_run_ten_case_matrix(tmp_path: Path) -> None:
    key = (
        os.environ.get("OPENROUTER_API_KEY")
        or os.environ.get("OPENAI_API_KEY")
        or OPENAI_API_KEY
    )
    if not key:
        pytest.skip("OpenRouter credential is not configured")
    config = SupervisorConfig(
        model="openai/gpt-5.6-luna",
        verify_effort="medium",
        plan_effort="high",
        verify_timeout_s=120.0,
        plan_timeout_s=180.0,
        run_live_tests=True,
    )
    client = OpenRouterSupervisorClient(config, api_key=key)
    stages = _selected_stages()
    repetitions = int(os.environ.get("GPSR_LIVE_REPETITIONS", "3"))
    if repetitions < 1:
        raise ValueError("GPSR_LIVE_REPETITIONS must be at least 1")
    snapshots = {}
    for case, stage in stages:
        snapshots[stage.stage_id] = FixtureContextProvider(
            output_dir=tmp_path / "artifacts",
            scenario_id=case.scenario_id,
            require_urdf_renderer=True,
        ).capture(build_capture_request(case, stage))

    records: list[dict] = []
    for repetition in range(1, repetitions + 1):
        for case, stage in stages:
            started = time.monotonic()
            errors: list[str] = []
            observed = None
            try:
                observed = client.verify(snapshots[stage.stage_id])
                errors = _verification_errors(stage, observed)
            except Exception as exc:  # keep the complete matrix observable
                errors = [f"{type(exc).__name__}: {exc}"]
            records.append(
                {
                    "case_number": case.number,
                    "scenario_id": case.scenario_id,
                    "stage_id": stage.stage_id,
                    "role": "verify",
                    "repetition": repetition,
                    "expected": asdict(stage.verification),
                    "observed": _serialise(observed) if observed else None,
                    "errors": errors,
                    "passed": not errors,
                    "latency_ms": round(
                        (time.monotonic() - started) * 1000.0, 1
                    ),
                }
            )

    known_locations = _known_locations()
    for repetition in range(1, repetitions + 1):
        for case, stage in stages:
            expectation = stage.planner
            if expectation is None:
                continue
            started = time.monotonic()
            errors = []
            observed = None
            try:
                verification = _expected_decision(stage)
                snapshot = snapshots[stage.stage_id]
                if expectation.role == "local_recovery":
                    observed = client.plan_local_recovery(
                        snapshot,
                        verification,
                        f"live-{stage.stage_id}",
                    )
                    validate_recovery_macro(observed)
                    if observed.kind.value != expectation.action:
                        errors.append(
                            f"action: expected {expectation.action!r}, "
                            f"got {observed.kind.value!r}"
                        )
                else:
                    observed = client.plan_global_replan(
                        snapshot,
                        verification,
                        expectation.reason,
                    )
                    validate_global_decision(
                        observed,
                        completed_steps=stage.completed_steps,
                        original_instruction=stage.original_instruction,
                        known_actions={
                            "goto",
                            "find_object",
                            "grasp",
                            "deliver",
                            "announce",
                        },
                        known_locations=known_locations,
                    )
                    if observed.action.value != expectation.action:
                        errors.append(
                            f"action: expected {expectation.action!r}, "
                            f"got {observed.action.value!r}"
                        )
                    if (
                        expectation.action == "abort_and_report"
                        and observed.replacement_plan
                    ):
                        errors.append(
                            "abort_and_report must not contain a replacement plan"
                        )
            except Exception as exc:  # keep the complete matrix observable
                errors.append(f"{type(exc).__name__}: {exc}")
            records.append(
                {
                    "case_number": case.number,
                    "scenario_id": case.scenario_id,
                    "stage_id": stage.stage_id,
                    "role": expectation.role,
                    "repetition": repetition,
                    "expected": {"action": expectation.action},
                    "observed": _serialise(observed) if observed else None,
                    "errors": errors,
                    "passed": not errors,
                    "latency_ms": round(
                        (time.monotonic() - started) * 1000.0, 1
                    ),
                }
            )

    planner_stage_count = sum(1 for _, stage in stages if stage.planner)
    expected_call_count = repetitions * (len(stages) + planner_stage_count)
    report = {
        "suite": "gpsr-vlm-ten-case",
        "model": config.model,
        "verify_effort": config.verify_effort,
        "plan_effort": config.plan_effort,
        "prompt_version": PROMPT_VERSION,
        "stage_filter": [stage.stage_id for _, stage in stages],
        "repetitions": repetitions,
        "expected_call_count": expected_call_count,
        "actual_call_count": len(records),
        "passed": sum(1 for item in records if item["passed"]),
        "failed": sum(1 for item in records if not item["passed"]),
        "records": records,
    }
    report_path = Path(
        os.environ.get("GPSR_LIVE_REPORT", str(tmp_path / "live-results.json"))
    )
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    assert len(records) == expected_call_count
    failures = [
        f"{item['stage_id']} {item['role']} run {item['repetition']}: "
        + "; ".join(item["errors"])
        for item in records
        if not item["passed"]
    ]
    assert not failures, "\n".join(failures)
