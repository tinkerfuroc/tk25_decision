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
    SubtaskStatus,
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
        # O4 (unavailability never stops a mission): count consecutive
        # supervisor.unavailable results; once the run of failures reaches
        # config.max_consecutive_errors, degrade supervision to off for the
        # rest of the run rather than keep issuing queries a broken
        # verifier will just keep failing. Reset on any successful verdict.
        self._consecutive_errors = 0
        # Q4 (task-Q, round-6 supervision-economics fix): TOTAL query
        # failures over the whole run, never reset (unlike the consecutive
        # counter above) -- a verifier flaky enough to error every third
        # call never trips the consecutive path but still costs the
        # mission real time on every failed query.
        self._total_errors = 0
        # Q4: cumulative overhead budget -- applied-intervention deferral
        # age (reported by the slot, see `record_deferral_overhead`) plus
        # recovery-cycle span (`recovery_started` -> `recovery_finished`,
        # tracked in `_recovery_started_at` below), accumulated over the
        # whole run.
        self._overhead_s = 0.0
        self._recovery_started_at: dict[tuple[str, str], float] = {}
        self._degraded = False
        # R-1 (task-Q, round-6 review fix): subtask_ids whose owning slot
        # has already hard-stopped (SupervisedSubtaskSlot._finish_hard_stop
        # calls `mark_subtask_dead` at the same point the Y-1 drain runs).
        # A dead slot never ticks its intervention-processing block again,
        # so ANY intervention that would enqueue for one of these
        # subtask_ids afterward -- from whichever checkpoint, whenever its
        # async future happens to resolve -- can never be consumed; every
        # enqueue site routes through `_enqueue_intervention`, which drops
        # it instead. This is the class fix for the X-2/Y-1/R-1 orphaned-
        # intervention family: it does not matter which checkpoint or
        # which of the three enqueue call sites produces the intervention,
        # or how long its async future takes to resolve -- once the owning
        # subtask is dead, nothing enqueued against it can ever wedge the
        # mission again.
        self._dead_subtasks: set[str] = set()

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
            # O4: once degraded (ACTIVE mode only), supervision behaves like
            # OFF -- resolve straight from reported_status and never enqueue
            # another query; SHADOW mode keeps querying regardless, since it
            # never intervenes and a broken verifier costs it nothing.
            skip_query = self.config.mode is SupervisionMode.OFF or (
                self.config.mode is SupervisionMode.ACTIVE and self._degraded
            )
            if skip_query:
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

    def has_queued_intervention(self, checkpoint_id: str) -> bool:
        """Structural predicate: is an intervention for THIS checkpoint
        still queued, waiting for its owning slot to consume it?

        Z-1 (task-O review, CRITICAL): every call site that queues a
        ``SupervisorIntervention`` (the ``Escalation.STOP`` branch and the
        else-fallback in ``_handle_verification``, ``_handle_local_recovery``,
        ``_handle_global_replan``) does so for exactly this checkpoint_id in
        the same call that sets ``record.resolution``. This lets a caller
        (``SupervisedEffect.tick()``) defer to the slot-level consumption
        machinery *structurally* -- by asking the controller directly,
        rather than by pattern-matching which resolution strings happen to
        mean "an intervention is coming" -- so a future intervention-queuing
        code path is recognized automatically, with nothing to remember to
        patch in the caller.
        """
        self.poll()
        with self._lock:
            return any(
                intervention.checkpoint_id == checkpoint_id
                for intervention in self._interventions
            )

    def mark_subtask_dead(self, subtask_id: str) -> None:
        """R-1 (task-Q, round-6 review fix): a slot has hard-stopped for
        this subtask_id and will never tick its intervention-processing
        block again. Every future call to `_enqueue_intervention` for this
        subtask_id -- regardless of which checkpoint or which of the three
        enqueue sites produces it -- drops the intervention instead of
        queuing it. Called by `SupervisedSubtaskSlot._finish_hard_stop`, at
        the same point the Y-1 drain runs (belt and braces: Y-1 still
        drains whatever is ALREADY queued at that instant; this covers
        whatever enqueues LATER, from an async future that resolves after
        the hard stop).
        """
        with self._lock:
            self._dead_subtasks.add(subtask_id)

    def _enqueue_intervention(
        self, intervention: SupervisorIntervention, *, record: CheckpointRecord
    ) -> None:
        """Single choke point for every intervention-enqueue site (the
        ``Escalation.STOP`` branch and the else-fallback in
        ``_handle_verification``, ``_handle_local_recovery``, and
        ``_handle_global_replan``). R-1: if `record`'s subtask is already
        dead (`mark_subtask_dead`), resolve the record immediately instead
        of appending -- an intervention queued against a dead subtask can
        never be consumed and would otherwise sit in `_interventions`
        forever, and `can_start_effect`/`can_finish_subtask`'s
        `if self._interventions: return False` gate is unscoped by
        subtask_id, so ONE such orphan blocks every later subtask for the
        rest of the mission. This is the structural fix: a future
        intervention-queuing code path only has to route through here to
        be covered automatically, with nothing to remember to check at the
        call site.
        """
        subtask_id = record.snapshot.request.subtask_id
        with self._lock:
            dead = subtask_id in self._dead_subtasks
            if dead:
                record.stage = "complete"
                record.resolution = "dropped_dead_subtask"
            else:
                self._interventions.append(intervention)
        if dead:
            self._emit(
                "supervisor.intervention.dropped",
                {
                    "checkpoint_id": intervention.checkpoint_id,
                    "subtask_id": subtask_id,
                    "kind": intervention.kind,
                    "reason": intervention.reason,
                },
            )

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

    def peek_intervention(
        self, *, subtask_id: str | None = None
    ) -> SupervisorIntervention | None:
        """Look at the next matching intervention without dequeuing it.

        N1c (round-5 rerun fix): an ACTIVE-mode slot must be able to inspect
        a pending intervention (to decide whether it is premature -- see
        ``SupervisedSubtaskSlot._may_apply_intervention_now``) without
        removing it from the queue, since a deferred intervention is "not
        consumed, not dropped": the next qualifying tick must still be able
        to find and apply it. Mirrors ``consume_intervention``'s matching
        exactly, just without the ``pop``.
        """
        self.poll()
        with self._lock:
            for intervention in self._interventions:
                record = self._records.get(intervention.checkpoint_id)
                if record is None:
                    continue
                if (
                    subtask_id is None
                    or record.snapshot.request.subtask_id == subtask_id
                ):
                    return intervention
        return None

    def emit_telemetry(self, event: str, payload: Mapping[str, Any]) -> None:
        """Public wrapper so BT nodes outside this module can emit events
        through the same fail-open ``_emit`` path (e.g.
        ``BtNode_RecoveryDirective`` reporting ``recovery.handler_missing``)."""
        self._emit(event, payload)

    def recovery_started(self, proposal: RecoveryProposal) -> None:
        self.ledger.mark_executed(proposal.issue_id, proposal.strategy_id)
        with self._lock:
            record = self._records.get(proposal.checkpoint_id)
            if record is not None:
                record.stage = "recovery_executing"
            # Q4 (task-Q, round-6): remember when this recovery cycle
            # started so `recovery_finished` can add its span to the
            # cumulative overhead budget. Keyed by (issue_id, strategy_id)
            # -- validate_recovery_macro/RecoveryLedger.register already
            # reject a duplicate fingerprint for the same issue, so this
            # pair is always a fresh key at this point.
            self._recovery_started_at[
                (proposal.issue_id, proposal.strategy_id)
            ] = time.monotonic()
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
            started_at = self._recovery_started_at.pop(
                (proposal.issue_id, proposal.strategy_id), None
            )
        if started_at is not None:
            # Q4: (ii) each recovery cycle span, accumulated regardless of
            # outcome -- a recovery that ultimately failed still cost the
            # mission the time it ran.
            self._accumulate_overhead(max(0.0, time.monotonic() - started_at))
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

    def record_deferral_overhead(self, age_s: float) -> None:
        """Q4 (task-Q, round-6): (i) each applied intervention's deferral
        age contributes to the cumulative overhead budget. Called by
        ``SupervisedSubtaskSlot._note_deferral`` with the SAME ``age_s`` it
        already computes for the ``intervention.deferred`` telemetry event
        -- one contribution per intervention (``_note_deferral`` itself
        only calls this once per checkpoint_id, on the first tick it
        notices the deferral), not accumulated again on every later tick
        the same intervention stays deferred.

        Chose slot-side reporting (over reconstructing the deferral
        contribution controller-side from checkpoint timestamps) because
        ``_note_deferral`` already computes the exact age value the
        ruling asks to reuse, at the exact point it becomes known -- a
        controller-side reconstruction would need to duplicate that same
        "first tick a pending intervention was noticed against this
        attempt's start time" logic with no additional accuracy to show
        for it.
        """
        self._accumulate_overhead(max(0.0, age_s))

    def _accumulate_overhead(self, amount_s: float) -> None:
        if amount_s <= 0:
            return
        with self._lock:
            self._overhead_s += amount_s
            total = self._overhead_s
        if total >= self.config.overhead_budget_s:
            self._maybe_degrade("overhead_budget", extra={"overhead_s": total})

    def _maybe_degrade(self, reason: str, *, extra: Mapping[str, Any] | None = None) -> bool:
        """Set `_degraded` at most once; emit `supervisor.degraded` only on
        the call that actually flips it. Shared by every degrade trigger
        (consecutive errors, total errors, overhead budget) so exactly one
        reason -- whichever trips first -- is ever reported, matching the
        existing "fires exactly once" contract.
        """
        with self._lock:
            if self._degraded:
                return False
            self._degraded = True
        payload: dict[str, Any] = {"reason": reason}
        if extra:
            payload.update(extra)
        self._emit("supervisor.degraded", payload)
        return True

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
        # O4: any successful verdict (the query itself succeeded, whatever
        # its content) resets the consecutive-unavailable counter.
        self._consecutive_errors = 0
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
        # O2 (uncertainty is not failure): a verdict of uncertain, or a
        # subtask_status the verifier itself admits it does not know, is not
        # a positive finding of anything wrong -- it is the verifier saying
        # it cannot tell. Never turn that into an intervention (regardless
        # of what escalation the free-text prompt asked for, e.g. the old
        # sensor_context_mismatch=stop instruction): mark the checkpoint
        # unverified and let the mission proceed. This must precede the
        # Escalation.STOP branch below, which remains reserved for a
        # positive failure finding (verdict recoverable/unrecoverable).
        if (
            decision.verdict is Verdict.UNCERTAIN
            or decision.subtask_status is SubtaskStatus.UNKNOWN
        ):
            record.stage = "complete"
            record.resolution = "unverified"
            self._emit(
                "supervisor.verdict.downgraded",
                {
                    "checkpoint_id": expected,
                    "failure_category": decision.failure_category,
                    "escalation_requested": decision.escalation.value,
                },
            )
            return
        if decision.escalation is Escalation.STOP:
            record.stage = "complete"
            record.resolution = "stop"
            self._enqueue_intervention(
                SupervisorIntervention(
                    kind="stop",
                    checkpoint_id=expected,
                    reason=decision.failure_category or "verifier_stop",
                ),
                record=record,
            )
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
            or decision.verdict is Verdict.RECOVERABLE
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
            self._enqueue_intervention(
                SupervisorIntervention(
                    kind="stop",
                    checkpoint_id=expected,
                    reason="inconsistent_or_failed_verification",
                ),
                record=record,
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
        self._enqueue_intervention(
            SupervisorIntervention(
                kind="local_recovery",
                checkpoint_id=expected,
                reason=record.verification.failure_category if record.verification else "",
                payload=proposal,
            ),
            record=record,
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
        self._enqueue_intervention(
            SupervisorIntervention(
                kind="global_replan",
                checkpoint_id=expected,
                reason=decision.rationale,
                payload=decision,
            ),
            record=record,
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
        # O4 (unavailability never stops a mission): a query error -- a
        # transport failure, a malformed/empty structured response, a stale
        # checkpoint id, anything that reaches here -- is a supervisor
        # problem, not evidence the mission failed. It ALWAYS resolves as
        # continue-unverified now; the old stop-intervention branch is gone.
        record.error = f"{type(exc).__name__}: {exc}"
        request = record.snapshot.request
        record.stage = (
            "complete" if self.config.mode is SupervisionMode.ACTIVE else "shadow_complete"
        )
        record.resolution = "unavailable"
        record.unverified = True
        self._consecutive_errors += 1
        # Q4 (task-Q, round-6): TOTAL query failures, never reset within
        # the run -- alongside the existing consecutive-run counter above.
        self._total_errors += 1
        self._emit(
            "supervisor.unavailable",
            {
                "checkpoint_id": request.checkpoint_id,
                "error_type": type(exc).__name__,
                "continued_unverified": True,
            },
        )
        if self._consecutive_errors >= self.config.max_consecutive_errors:
            self._maybe_degrade(
                "consecutive_errors",
                extra={
                    "checkpoint_id": request.checkpoint_id,
                    "consecutive_errors": self._consecutive_errors,
                },
            )
        elif self._total_errors >= self.config.max_total_errors:
            self._maybe_degrade(
                "total_errors",
                extra={
                    "checkpoint_id": request.checkpoint_id,
                    "total_errors": self._total_errors,
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
