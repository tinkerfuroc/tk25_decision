"""Asynchronous state machine for GPSR verification and recovery."""
from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import asdict, dataclass, field, replace
import threading
import time
from typing import Any, Mapping

from .clients import SupervisorClient
from .context import ContextProvider
from .models import (
    BtAssessment,
    CaptureRequest,
    EffectRisk,
    Escalation,
    GlobalPlanDecision,
    NodeContract,
    RecoveryProposal,
    ReportedStatus,
    SnapshotBundle,
    SuccessMode,
    SupervisionMode,
    SupervisorConfig,
    Verdict,
    VerificationDecision,
    WorldChange,
    issue_identity,
)
from .recovery import RecoveryLedger


@dataclass(frozen=True)
class SupervisorIntervention:
    kind: str
    checkpoint_id: str
    reason: str
    payload: RecoveryProposal | GlobalPlanDecision | None = None


@dataclass
class CheckpointRecord:
    snapshot: SnapshotBundle
    contract: NodeContract
    reported_status: ReportedStatus
    stage: str = "captured"
    future: Future | None = None
    verification: VerificationDecision | None = None
    issue_id: str | None = None
    resolution: str | None = None
    error: str | None = None
    unverified: bool = False
    metadata: dict[str, Any] = field(default_factory=dict)


class MissionSupervisor:
    """Own query concurrency; expose decisions only at BT-safe boundaries."""

    def __init__(
        self,
        config: SupervisorConfig,
        context_provider: ContextProvider,
        client: SupervisorClient,
        *,
        ledger: RecoveryLedger | None = None,
        executor: ThreadPoolExecutor | None = None,
        telemetry: Any = None,
    ) -> None:
        self.config = config
        self.context_provider = context_provider
        self.client = client
        self.ledger = ledger or RecoveryLedger(config.max_recoveries)
        self._executor = executor or ThreadPoolExecutor(
            max_workers=2, thread_name_prefix="gpsr-supervisor"
        )
        self._owns_executor = executor is None
        self._telemetry = telemetry
        self._records: dict[str, CheckpointRecord] = {}
        self._interventions: list[SupervisorIntervention] = []
        self._issue_checkpoint: dict[str, str] = {}
        self._lock = threading.RLock()

    def submit(
        self,
        request: CaptureRequest,
        contract: NodeContract,
        reported_status: ReportedStatus,
    ) -> str:
        """Capture and enqueue verification exactly once per checkpoint."""
        with self._lock:
            if request.checkpoint_id in self._records:
                return request.checkpoint_id
        snapshot = self.context_provider.capture(request)
        record = CheckpointRecord(
            snapshot=snapshot,
            contract=contract,
            reported_status=reported_status,
        )
        with self._lock:
            if request.checkpoint_id in self._records:
                return request.checkpoint_id
            self._records[request.checkpoint_id] = record
            if self.config.mode is SupervisionMode.OFF:
                record.stage = "complete"
                record.resolution = reported_status.value.lower()
            else:
                record.stage = "verifying"
                record.future = self._executor.submit(self.client.verify, snapshot)
        self._emit(
            "supervisor.checkpoint.created",
            {
                "checkpoint_id": request.checkpoint_id,
                "task_id": request.task_id,
                "subtask_id": request.subtask_id,
                "original_instruction": request.original_instruction,
                "subtask_goal": request.subtask_goal,
                "node": dict(request.terminal_node),
                "next_node": dict(request.next_node) if request.next_node else None,
                "subtask_tree": dict(request.subtask_tree),
                "blackboard": dict(request.blackboard),
                "execution_history": list(request.execution_history),
                "recovery_ledger": list(request.recovery_ledger),
                "reported_status": reported_status.value,
                "risk": contract.risk.value,
                "artifact_roles": [artifact.role for artifact in snapshot.artifacts],
                "artifacts": [asdict(artifact) for artifact in snapshot.artifacts],
            },
        )
        return request.checkpoint_id

    def poll(self) -> None:
        """Advance completed queries without blocking the BT tick thread."""
        with self._lock:
            ready = [
                (checkpoint_id, record)
                for checkpoint_id, record in self._records.items()
                if record.future is not None and record.future.done()
            ]
            for _, record in ready:
                future, record.future = record.future, None
                try:
                    result = future.result()
                except Exception as exc:
                    self._handle_query_error(record, exc)
                    continue
                if record.stage == "verifying":
                    self._handle_verification(record, result)
                elif record.stage == "planning_local":
                    self._handle_local_recovery(record, result)
                elif record.stage == "planning_global":
                    self._handle_global_replan(record, result)

    def can_start_effect(
        self,
        next_risk: EffectRisk,
        *,
        subtask_id: str | None = None,
    ) -> bool:
        self.poll()
        if self.config.mode in {SupervisionMode.OFF, SupervisionMode.SHADOW}:
            return True
        with self._lock:
            if self._interventions:
                return False
            pending = [
                record
                for record in self._records.values()
                if _query_or_barrier_pending(record)
                and (subtask_id is None or record.snapshot.request.subtask_id == subtask_id)
            ]
            if any(record.reported_status is ReportedStatus.FAILURE for record in pending):
                return False
            if self.config.success_mode is SuccessMode.OPTIMISTIC:
                return not any(record.error for record in pending)
            if next_risk is not EffectRisk.OBSERVATION:
                return not pending
            return not any(record.contract.risk is EffectRisk.IRREVERSIBLE for record in pending)

    def can_finish_subtask(self, subtask_id: str) -> bool:
        self.poll()
        if self.config.mode in {SupervisionMode.OFF, SupervisionMode.SHADOW}:
            return True
        with self._lock:
            if self._interventions:
                return False
            if self.config.success_mode is SuccessMode.OPTIMISTIC:
                return True
            return not any(
                record.snapshot.request.subtask_id == subtask_id
                and _query_or_barrier_pending(record)
                for record in self._records.values()
            )

    def resolution(self, checkpoint_id: str) -> str | None:
        self.poll()
        with self._lock:
            record = self._records.get(checkpoint_id)
            return record.resolution if record else None

    def consume_intervention(
        self, *, subtask_id: str | None = None
    ) -> SupervisorIntervention | None:
        self.poll()
        with self._lock:
            for index, intervention in enumerate(self._interventions):
                record = self._records.get(intervention.checkpoint_id)
                if record is None:
                    continue
                if (
                    subtask_id is None
                    or record.snapshot.request.subtask_id == subtask_id
                ):
                    return self._interventions.pop(index)
        return None

    def recovery_started(self, proposal: RecoveryProposal) -> None:
        self.ledger.mark_executed(proposal.issue_id, proposal.strategy_id)
        with self._lock:
            record = self._records.get(proposal.checkpoint_id)
            if record is not None:
                record.stage = "recovery_executing"
        self._emit(
            "supervisor.recovery.started",
            {
                "checkpoint_id": proposal.checkpoint_id,
                "issue_id": proposal.issue_id,
                "strategy_id": proposal.strategy_id,
                "kind": proposal.kind.value,
            },
        )

    def recovery_finished(self, proposal: RecoveryProposal, *, succeeded: bool) -> None:
        self.ledger.mark_result(
            proposal.issue_id, proposal.strategy_id, succeeded=succeeded
        )
        with self._lock:
            record = self._records.get(proposal.checkpoint_id)
            if record is not None:
                record.stage = "complete"
                record.resolution = (
                    "recovery_succeeded" if succeeded else "recovery_failed"
                )
        self._emit(
            "supervisor.recovery.finished",
            {
                "checkpoint_id": proposal.checkpoint_id,
                "issue_id": proposal.issue_id,
                "strategy_id": proposal.strategy_id,
                "succeeded": succeeded,
                "failed_distinct": self.ledger.failed_count(proposal.issue_id),
            },
        )
        if succeeded or not self.ledger.exhausted(proposal.issue_id):
            return
        with self._lock:
            checkpoint_id = self._issue_checkpoint.get(proposal.issue_id)
            record = self._records.get(checkpoint_id or "")
            if record is not None and record.verification is not None:
                self._start_global(record, "three_distinct_recoveries_failed")

    def record(self, checkpoint_id: str) -> CheckpointRecord | None:
        with self._lock:
            return self._records.get(checkpoint_id)

    def records(self) -> tuple[CheckpointRecord, ...]:
        with self._lock:
            return tuple(self._records.values())

    def wait_for_idle(self, timeout_s: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.poll()
            with self._lock:
                if not any(record.future is not None for record in self._records.values()):
                    return True
            time.sleep(0.01)
        return False

    def close(self) -> None:
        if self._owns_executor:
            self._executor.shutdown(wait=False, cancel_futures=True)

    def _handle_verification(
        self, record: CheckpointRecord, decision: VerificationDecision
    ) -> None:
        expected = record.snapshot.request.checkpoint_id
        if decision.checkpoint_id != expected:
            self._handle_query_error(
                record,
                ValueError(
                    f"stale verifier checkpoint {decision.checkpoint_id}; expected {expected}"
                ),
            )
            return
        record.verification = decision
        self._emit(
            "supervisor.verdict.received",
            {
                "checkpoint_id": expected,
                "verdict": decision.verdict.value,
                "bt_assessment": decision.bt_assessment.value,
                "subtask_status": decision.subtask_status.value,
                "world_change": decision.world_change.value,
                "escalation": decision.escalation.value,
                "failure_category": decision.failure_category,
                "evidence": list(decision.evidence),
                "rationale": decision.rationale,
                "confidence": decision.confidence,
            },
        )
        if self.config.mode is SupervisionMode.SHADOW:
            record.stage = "shadow_complete"
            record.resolution = record.reported_status.value.lower()
            return
        if decision.bt_assessment is BtAssessment.FALSE_FAILURE:
            record.stage = "complete"
            record.resolution = "success"
            return

        force_global = (
            decision.world_change is WorldChange.DESTRUCTIVE
            or decision.verdict is Verdict.UNRECOVERABLE
            or decision.escalation is Escalation.GLOBAL_REPLAN
        )
        needs_local = (
            decision.bt_assessment is BtAssessment.FALSE_SUCCESS
            or decision.verdict in {Verdict.RECOVERABLE, Verdict.UNCERTAIN}
            or decision.escalation is Escalation.LOCAL_RECOVERY
        )
        if force_global:
            self._start_global(record, "verifier_global_escalation")
        elif needs_local and record.contract.allow_local_recovery:
            self._start_local(record)
        elif needs_local:
            self._start_global(record, "local_recovery_not_permitted")
        elif (
            decision.verdict is Verdict.ALL_CLEAR
            and record.reported_status is ReportedStatus.SUCCESS
        ):
            record.stage = "complete"
            record.resolution = "success"
        else:
            record.stage = "complete"
            record.resolution = "stop"
            self._interventions.append(
                SupervisorIntervention(
                    kind="stop",
                    checkpoint_id=expected,
                    reason="inconsistent_or_failed_verification",
                )
            )

    def _start_local(self, record: CheckpointRecord) -> None:
        assert record.verification is not None
        request = record.snapshot.request
        target = request.blackboard.get("gpsr/target_object_name")
        location = request.blackboard.get("gpsr/target_location")
        issue_id = issue_identity(
            subtask_goal=request.subtask_goal,
            effect=record.contract.effect,
            failure_category=record.verification.failure_category,
            target=target,
            location=location,
        )
        if self.ledger.exhausted(issue_id):
            self._start_global(record, "recovery_budget_exhausted")
            return
        record.issue_id = issue_id
        self._issue_checkpoint[issue_id] = request.checkpoint_id
        fresh_request = replace(
            request,
            recovery_ledger=tuple(self.ledger.snapshot(issue_id)),
        )
        record.snapshot = replace(record.snapshot, request=fresh_request)
        record.stage = "planning_local"
        record.future = self._executor.submit(
            self.client.plan_local_recovery,
            record.snapshot,
            record.verification,
            issue_id,
        )

    def _start_global(self, record: CheckpointRecord, reason: str) -> None:
        if record.future is not None or record.stage == "planning_global":
            return
        if record.verification is None:
            return
        record.stage = "planning_global"
        record.future = self._executor.submit(
            self.client.plan_global_replan,
            record.snapshot,
            record.verification,
            reason,
        )

    def _handle_local_recovery(
        self, record: CheckpointRecord, proposal: RecoveryProposal
    ) -> None:
        expected = record.snapshot.request.checkpoint_id
        if proposal.checkpoint_id != expected or proposal.issue_id != record.issue_id:
            self._handle_query_error(record, ValueError("stale local recovery proposal"))
            return
        try:
            self.ledger.register(proposal)
        except Exception as exc:
            self._handle_query_error(record, exc)
            return
        record.stage = "awaiting_recovery"
        record.resolution = "recovery"
        self._interventions.append(
            SupervisorIntervention(
                kind="local_recovery",
                checkpoint_id=expected,
                reason=record.verification.failure_category if record.verification else "",
                payload=proposal,
            )
        )
        self._emit(
            "supervisor.recovery.proposed",
            {
                "checkpoint_id": expected,
                "issue_id": proposal.issue_id,
                "strategy_id": proposal.strategy_id,
                "kind": proposal.kind.value,
                "arguments": dict(proposal.arguments),
                "rationale": proposal.rationale,
                "expected_evidence": list(proposal.expected_evidence),
                "stop_conditions": list(proposal.stop_conditions),
            },
        )

    def _handle_global_replan(
        self, record: CheckpointRecord, decision: GlobalPlanDecision
    ) -> None:
        expected = record.snapshot.request.checkpoint_id
        if decision.checkpoint_id != expected:
            self._handle_query_error(record, ValueError("stale global replan decision"))
            return
        record.stage = "awaiting_global"
        record.resolution = "global"
        self._interventions.append(
            SupervisorIntervention(
                kind="global_replan",
                checkpoint_id=expected,
                reason=decision.rationale,
                payload=decision,
            )
        )
        self._emit(
            "supervisor.global.proposed",
            {
                "checkpoint_id": expected,
                "action": decision.action.value,
                "replacement_plan": list(decision.replacement_plan),
                "preserved_completed_steps": decision.preserved_completed_steps,
                "relaxed_constraints": list(decision.relaxed_constraints),
                "rationale": decision.rationale,
                "operator_message": decision.operator_message,
            },
        )

    def _handle_query_error(self, record: CheckpointRecord, exc: Exception) -> None:
        record.error = f"{type(exc).__name__}: {exc}"
        request = record.snapshot.request
        low_risk_success = (
            record.reported_status is ReportedStatus.SUCCESS
            and record.contract.risk is EffectRisk.OBSERVATION
        )
        if self.config.mode is SupervisionMode.SHADOW or low_risk_success:
            record.stage = "complete" if self.config.mode is SupervisionMode.ACTIVE else "shadow_complete"
            record.resolution = "success"
            record.unverified = True
        else:
            record.stage = "complete"
            record.resolution = "stop"
            self._interventions.append(
                SupervisorIntervention(
                    kind="stop",
                    checkpoint_id=request.checkpoint_id,
                    reason="supervisor_unavailable",
                )
            )
        self._emit(
            "supervisor.unavailable",
            {
                "checkpoint_id": request.checkpoint_id,
                "error_type": type(exc).__name__,
                "continued_unverified": record.unverified,
            },
        )

    def _emit(self, event: str, payload: Mapping[str, Any]) -> None:
        if self._telemetry is None:
            return
        try:
            if callable(self._telemetry):
                self._telemetry(event, payload)
            else:
                self._telemetry.emit(
                    event,
                    dict(payload),
                    task_id=payload.get("task_id"),
                    phase="supervision",
                )
        except Exception:
            pass


__all__ = [
    "CheckpointRecord",
    "MissionSupervisor",
    "SupervisorIntervention",
]


def _query_or_barrier_pending(record: CheckpointRecord) -> bool:
    return record.stage in {
        "captured",
        "verifying",
        "planning_local",
        "planning_global",
    }
