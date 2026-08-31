"""py_trees integration for the GPSR mission supervisor."""
from __future__ import annotations

import json
import os
import threading
import time
from typing import Any, Callable, Mapping

import py_trees
from py_trees.common import Status

from .clients import OpenRouterSupervisorClient, SupervisorClient
from .context import (
    ContextProvider,
    FixtureContextProvider,
    blackboard_keys_from_tree,
    next_unticked_node,
    snapshot_blackboard,
)
from .contracts import NodeContractRegistry, default_node_contracts
from .controller import MissionSupervisor, SupervisorIntervention
from .models import (
    CaptureRequest,
    GlobalAction,
    GlobalPlanDecision,
    NodeContract,
    RecoveryKind,
    RecoveryProposal,
    ReportedStatus,
    SchemaError,
    SupervisionMode,
    SupervisorConfig,
)
from .recovery import (
    RecoveryMacroCompiler,
    validate_global_decision,
)


_DEFAULT_SUPERVISOR: MissionSupervisor | None = None
_DEFAULT_LOCK = threading.RLock()


def set_default_supervisor(supervisor: MissionSupervisor | None) -> None:
    global _DEFAULT_SUPERVISOR
    with _DEFAULT_LOCK:
        _DEFAULT_SUPERVISOR = supervisor


def get_default_supervisor() -> MissionSupervisor | None:
    with _DEFAULT_LOCK:
        return _DEFAULT_SUPERVISOR


def configure_default_supervisor(
    *,
    context_provider: ContextProvider | None = None,
    client: SupervisorClient | None = None,
    telemetry: Any = None,
    config: SupervisorConfig | None = None,
) -> MissionSupervisor | None:
    """Build the process supervisor.

    Fixture context is automatic only in full-mock mode or when explicitly
    requested.  A real run must install its production ContextProvider.
    """
    config = config or SupervisorConfig.from_env()
    if config.mode is SupervisionMode.OFF:
        set_default_supervisor(None)
        return None
    if context_provider is None:
        use_fixture = os.environ.get("GPSR_SUPERVISOR_CONTEXT", "").lower() == "fixture"
        try:
            from behavior_tree.config import is_full_mock_mode

            use_fixture = use_fixture or is_full_mock_mode()
        except Exception:
            pass
        if not use_fixture:
            raise RuntimeError(
                "active GPSR supervision requires a production ContextProvider; "
                "set GPSR_SUPERVISOR_CONTEXT=fixture only for hardware-free runs"
            )
        context_provider = FixtureContextProvider()
    if client is None:
        from behavior_tree.GPSR.config import OPENAI_API_KEY

        client = OpenRouterSupervisorClient(
            config,
            api_key=OPENAI_API_KEY,
            telemetry=_client_telemetry(telemetry),
        )
    supervisor = MissionSupervisor(
        config,
        context_provider,
        client,
        telemetry=telemetry,
    )
    set_default_supervisor(supervisor)
    return supervisor


class BtNode_RecoveryDirective(py_trees.behaviour.Behaviour):
    """Trusted typed recovery hook.

    Hardware-free runs acknowledge the directive and then rematerialise the
    subtask. Production deployments register handlers that perform the typed
    operation; the LLM can never supply code.
    """

    def __init__(
        self,
        proposal: RecoveryProposal,
        handlers: Mapping[RecoveryKind, Callable[[RecoveryProposal], bool]] | None = None,
        *,
        telemetry: Callable[[str, Mapping[str, Any]], None] | None = None,
    ) -> None:
        super().__init__(f"recovery:{proposal.kind.value}:{proposal.strategy_id}")
        self.proposal = proposal
        self.handlers = dict(handlers or {})
        self._telemetry = telemetry
        self._done = False

    def initialise(self) -> None:
        self._done = False

    def update(self) -> Status:
        if self._done:
            return Status.SUCCESS
        handler = self.handlers.get(self.proposal.kind)
        if handler is not None:
            try:
                succeeded = bool(handler(self.proposal))
            except Exception as exc:
                self.feedback_message = f"recovery handler failed: {type(exc).__name__}"
                return Status.FAILURE
            self._done = succeeded
            return Status.SUCCESS if succeeded else Status.FAILURE
        try:
            from behavior_tree.config import is_full_mock_mode

            mock_mode = is_full_mock_mode()
        except Exception:
            mock_mode = False
        if not mock_mode:
            # N1b (round-5 rerun fix): zero production RecoveryKind handlers
            # are registered repo-wide today -- returning FAILURE here used
            # to fail the whole `recover+retry` Sequence before it ever
            # reached the fresh-subtask retry, which is the one thing that
            # actually recovers when there is no preparatory handler. A
            # missing handler is the expected/normal case, not an error:
            # warn, tell telemetry, and let the Sequence proceed to the
            # retry instead of dooming it.
            self.feedback_message = (
                f"no production handler installed for {self.proposal.kind.value}; "
                "proceeding directly to the fresh-subtask retry"
            )
            self.logger.warning(self.feedback_message)
            if self._telemetry is not None:
                try:
                    self._telemetry(
                        "recovery.handler_missing", {"kind": self.proposal.kind.value}
                    )
                except Exception:
                    pass
            self._done = True
            return Status.SUCCESS
        self._done = True
        self.feedback_message = (
            f"MOCK recovery directive executed: {self.proposal.kind.value} "
            f"{dict(self.proposal.arguments)}"
        )
        return Status.SUCCESS


def default_recovery_compiler(
    handlers: Mapping[RecoveryKind, Callable[[RecoveryProposal], bool]] | None = None,
    *,
    telemetry: Callable[[str, Mapping[str, Any]], None] | None = None,
) -> RecoveryMacroCompiler:
    return RecoveryMacroCompiler(
        {
            kind: (
                lambda proposal, _handlers=handlers, _telemetry=telemetry: BtNode_RecoveryDirective(
                    proposal, _handlers, telemetry=_telemetry
                )
            )
            for kind in RecoveryKind
        }
    )


class SupervisedEffect(py_trees.decorators.Decorator):
    """Capture one checkpoint for each terminal activation of an effect leaf."""

    def __init__(
        self,
        *,
        child: py_trees.behaviour.Behaviour,
        contract: NodeContract,
        supervisor: MissionSupervisor,
        effect_node_id: str,
        slot: "SupervisedSubtaskSlot",
        in_recovery_retry: bool = False,
    ) -> None:
        super().__init__(name=f"supervise:{child.name}", child=child)
        self.contract = contract
        self.supervisor = supervisor
        self.effect_node_id = effect_node_id
        self.slot = slot
        # Q1 (task-Q, round-6 supervision-economics fix): True only for
        # effects built INSIDE a recovery-built `recover+retry` composite
        # (see `SupervisedSubtaskSlot._apply_intervention`'s local_recovery
        # branch, which calls `_materialize_subtask(in_recovery_retry=True)`
        # for the fresh retry subtree). A retry attempt that creates its own
        # checkpoint re-enters the full verify/recovery pipeline -- the
        # measured root cause of the 10x120s recovery-churn stall (every
        # retry re-supervises itself, multiplying supervisor.query queries
        # and re-arming the full stall window against the fresh subtree).
        # An effect marked here skips checkpoint creation entirely (see
        # `tick()` below) and reports its own terminal status straight
        # through -- exactly like OFF mode -- for THIS activation only; a
        # later, differently-constructed slot (a genuinely new subtask) is
        # never marked and supervises normally (see `_materialize_subtask`'s
        # default `in_recovery_retry=False` for every non-retry build).
        self.in_recovery_retry = in_recovery_retry
        self.activation_counter = 0
        self._started = False
        self._terminal_status: Status | None = None
        self._checkpoint_id: str | None = None
        self._gpsr_effect_contract = {
            "effect": contract.effect,
            "risk": contract.risk.value,
            "expected_postcondition": contract.expected_postcondition,
            "evidence_modalities": list(contract.evidence_modalities),
        }
        self._gpsr_effect_node_id = effect_node_id

    def initialise(self) -> None:
        self._started = False
        self._terminal_status = None
        self._checkpoint_id = None

    def tick(self):
        self.logger.debug("%s.tick()" % self.__class__.__name__)
        if self.status != Status.RUNNING:
            self.initialise()

        if not self._started:
            if not self.supervisor.can_start_effect(
                self.contract.risk,
                subtask_id=self.slot.current_subtask_id(),
            ):
                self.status = Status.RUNNING
                yield self
                return
            self._started = True
            self.activation_counter += 1

        if self._terminal_status is None:
            for node in self.decorated.tick():
                yield node
            child_status = self.decorated.status
            if child_status not in {Status.SUCCESS, Status.FAILURE}:
                self.status = child_status
                yield self
                return
            self._terminal_status = child_status
            # Q1 (task-Q, round-6): a recovery-retry effect never creates a
            # checkpoint at all -- no `supervisor.submit()`, no verify/
            # recovery pipeline, exactly one verify cycle per ORIGINAL
            # attempt. `self._checkpoint_id` stays None (its `terminal_status`
            # property, used by `SupervisedSubtaskSlot._has_failed_pending_
            # effect`, still reports the real reported status either way).
            if not self.in_recovery_retry:
                request = self.slot.build_capture_request(
                    effect=self,
                    terminal_status=child_status,
                )
                self._checkpoint_id = self.supervisor.submit(
                    request,
                    self.contract,
                    ReportedStatus(child_status.name),
                )

        if self.in_recovery_retry:
            # Bypass the checkpoint/mode/resolution machinery entirely --
            # there is no checkpoint to resolve. Report the real terminal
            # status straight through, exactly like OFF mode, regardless of
            # `self.supervisor.config.mode`.
            new_status = self._terminal_status
        elif (
            self.supervisor.config.mode is not SupervisionMode.ACTIVE
            or self._terminal_status is Status.SUCCESS
        ):
            new_status = self._terminal_status
        else:
            # Z-1 (task-O review, CRITICAL): this used to be an open
            # allowlist of "known terminal resolution strings"
            # (_UNVERIFIED_RESOLUTIONS = {"unverified", "unavailable"}) that
            # every future terminal resolution had to remember to join --
            # and one already didn't: O4's own skip_query path (once
            # degraded) resolves a genuinely FAILED checkpoint to the bare
            # string "failure", which matched neither "success" nor that
            # allowlist and fell into the `else: RUNNING` branch, hanging
            # the effect (and its slot) forever. Fixed at the class, not
            # the instance: invert the check to be closed over "still
            # pending" (nothing terminal has happened yet -- resolution is
            # unset -- or an intervention IS queued for this checkpoint and
            # the owning SupervisedSubtaskSlot has not consumed it yet, see
            # has_queued_intervention()) rather than open over "known
            # terminal". Any resolution that is not "success" and is not
            # still pending unmasks to the real, already-reported terminal
            # status -- the resolution *string* only ever selects SUCCESS
            # vs FAILURE surfacing, so a brand new future resolution value
            # is safe by construction and cannot reintroduce this hang.
            checkpoint_id = self._checkpoint_id or ""
            resolution = self.supervisor.resolution(checkpoint_id)
            record = self.supervisor.record(checkpoint_id)
            stage_complete = record is not None and record.stage in {
                "complete",
                "shadow_complete",
            }
            intervention_pending = self.supervisor.has_queued_intervention(
                checkpoint_id
            )
            if stage_complete and resolution is None and not intervention_pending:
                # Structural invariant guard, not an expected runtime path:
                # every controller code path that marks a record
                # complete/shadow_complete also sets `resolution` in the
                # same call (verified against every `record.resolution =`
                # assignment in controller.py). If that invariant is ever
                # violated by a future change, fail loud with a real
                # terminal FAILURE and a feedback message naming the
                # anomaly instead of silently holding RUNNING forever.
                self.feedback_message = (
                    f"supervisor invariant violated: checkpoint {checkpoint_id!r} "
                    f"is stage={record.stage if record else None!r} with no "
                    "resolution and no queued intervention -- surfacing FAILURE"
                )
                self.logger.error(
                    f"{self.__class__.__name__}.tick(): {self.feedback_message}"
                )
                new_status = self._terminal_status
            elif resolution is None or intervention_pending:
                new_status = Status.RUNNING
            elif resolution == "success":
                new_status = Status.SUCCESS
            else:
                new_status = self._terminal_status

        if new_status != Status.RUNNING:
            self.stop(new_status)
        self.status = new_status
        yield self

    def update(self) -> Status:  # pragma: no cover - custom tick owns status
        return self.status

    @property
    def terminal_status(self) -> Status | None:
        """The leaf's own reported terminal status, if it has terminated.

        N1c (round-5 rerun fix): in ACTIVE mode, ``tick()`` above maps a
        terminated-but-unresolved FAILURE into an outward ``Status.RUNNING``
        (line ~246) while the async verifier/recovery pipeline runs -- so a
        ``SupervisedSubtaskSlot`` cannot tell "genuinely still executing"
        apart from "already failed, waiting on a verdict" by reading
        ``.status`` alone. This exposes the masked internal state so the
        slot's premature-intervention gate can treat "child already failed"
        (apply immediately -- that is the real recovery case) differently
        from "child never terminated" (defer until the stall threshold).
        """
        return self._terminal_status


class SupervisedSubtaskSlot(py_trees.decorators.Decorator):
    """Stable action slot that applies validated recovery at tick barriers."""

    def __init__(
        self,
        *,
        action_name: str,
        factory: Callable[[], py_trees.behaviour.Behaviour],
        supervisor: MissionSupervisor,
        registry: NodeContractRegistry | None = None,
        recovery_compiler: RecoveryMacroCompiler | None = None,
        target_slot: int | None = None,
        target_index: int | None = None,
        step_index: int | None = None,
        time_source: Callable[[], float] | None = None,
    ) -> None:
        self.action_name = action_name
        self.factory = factory
        self.supervisor = supervisor
        self.registry = registry or default_node_contracts()
        self.recovery_compiler = recovery_compiler or default_recovery_compiler(
            telemetry=lambda event, payload: supervisor.emit_telemetry(event, payload)
        )
        self.tree_revision = 1
        self._setup_kwargs: dict[str, Any] | None = None
        self._child_terminal: Status | None = None
        self._active_recovery: RecoveryProposal | None = None
        # N1c (round-5 rerun fix): "an attempt has been running longer than
        # a stall threshold" needs a clock and a per-attempt start time.
        # Injectable so tests can drive a fake clock deterministically
        # instead of racing real wall-clock sleeps; production callers get
        # `time.monotonic`. Read once at construction, per the ruling --
        # this is a stable-for-the-life-of-the-slot setting, not something
        # that should shift mid-mission if the env var changes.
        self._time_source: Callable[[], float] = time_source or time.monotonic
        self._stall_s = float(os.environ.get("GPSR_SUPERVISION_STALL_S", "120.0"))
        # Q2 (task-Q, round-6): once a subtask has been through >=1 applied
        # intervention (see `_apply_intervention`'s local_recovery branch),
        # a FURTHER intervention against a still-RUNNING retry only has to
        # wait this long, not the full `_stall_s` -- a step already in
        # recovery has forfeited the presumption of nominality. Read once at
        # construction, same rationale as `_stall_s` above.
        self._retry_stall_s = float(
            os.environ.get("GPSR_SUPERVISION_RETRY_STALL_S", "20.0")
        )
        self._has_retried = False
        self._attempt_started_at: float = self._time_source()
        self._deferred_intervention_checkpoint: str | None = None
        self._hard_stop_reason: str | None = None
        self._release_after_global = False
        # M2 (task-M, round-4 battery fix): the builder that pre-builds one
        # slot PER PLAN STEP (``GPSRPlanner.build_target_subtree``) can have
        # several ``SupervisedSubtaskSlot`` instances live in the tree at
        # once (this target's own later steps, plus a pipelined next
        # target's whole tree, pre-built ahead of the swap). Deriving this
        # slot's identity from ``gpsr/plan_index`` (a single, LIVE,
        # process-global blackboard value) at call time means two DIFFERENT
        # slot instances -- e.g. the same action name at the same relative
        # step position in two different targets, since ``DynamicExecutor``
        # does not bump ``gpsr/plan_revision`` on an ordinary target-to-
        # target swap, only on a supervisor replan -- can compute the exact
        # same id string. When the caller knows which (target_slot,
        # target_index, step_index) this slot was built for (the two-layer
        # ``build_target_subtree`` path always does), pass it here so the id
        # is FIXED at construction and scoped to this slot alone, never
        # recomputed from shared global state. Callers that construct a slot
        # directly (unit tests, the legacy flat dispatcher in
        # ``create_dispatcher``, which genuinely does reuse one slot across
        # every occurrence of an action) omit these and keep the previous,
        # live-plan_index-derived id -- see ``current_subtask_id`` below.
        self.target_slot = target_slot
        self.target_index = target_index
        self.step_index = step_index
        child = self._materialize_subtask()
        super().__init__(name=f"adaptive:{action_name}", child=child)
        self._adaptive_slot = True

    def setup(self, **kwargs) -> None:
        self._setup_kwargs = dict(kwargs)

    def current_subtask_id(self) -> str:
        task_id = str(_bb_get("gpsr/task_id", "task"))
        plan_revision = int(_bb_get("gpsr/plan_revision", 1) or 1)
        if self.step_index is not None:
            step = self.step_index
        else:
            step = max(0, int(_bb_get("gpsr/plan_index", 1) or 1) - 1)
        if self.target_slot is not None and self.target_index is not None:
            return (
                f"{task_id}/target-{self.target_slot}-{self.target_index}"
                f"/plan-r{plan_revision}/step-{step:04d}:{self.action_name}"
            )
        return f"{task_id}/plan-r{plan_revision}/step-{step:04d}:{self.action_name}"

    def build_capture_request(
        self,
        *,
        effect: SupervisedEffect,
        terminal_status: Status,
    ) -> CaptureRequest:
        from behavior_tree.GPSR.tree_serialization import serialize_tree

        subtask_id = self.current_subtask_id()
        kind = f"executed:{subtask_id}:r{self.tree_revision}"
        tree = serialize_tree(self.decorated, kind=kind, label=self.action_name)
        keys = set(blackboard_keys_from_tree(tree))
        keys.update(
            {
                "gpsr/command",
                "gpsr/task_id",
                "gpsr/plan",
                "gpsr/plan_index",
                "gpsr/plan_revision",
                "gpsr/current_action",
                "gpsr/current_params",
                "gpsr/state_log",
                "gpsr/target_location",
                # Q5 (task-Q, round-6): the ACTUAL navigation goal the
                # goto dispatch drives to (bb_keys.TARGET_POSE, a
                # PoseStamped set by the orchestrator right before a goto
                # dispatches) -- was never captured at all, so
                # FixtureContextProvider.capture() had no choice but to
                # re-derive a goal marker from gpsr/target_location's NAME
                # via a separate, fallible lookup. See
                # supervision/context.py's _goal_pose_from_blackboard.
                "gpsr/target_pose",
                "gpsr/target_object_name",
                "gpsr/target_object_prompt",
                "gpsr/target_person_prompt",
                "gpsr/last_capture",
                "gpsr/start_pose",
                "gpsr/arm_navigating",
            }
        )
        blackboard = snapshot_blackboard(tuple(sorted(keys)), _bb_required)
        checkpoint_id = (
            f"{subtask_id}/tree-r{self.tree_revision}/"
            f"{effect.effect_node_id}/activation-{effect.activation_counter:04d}"
        )
        terminal_node = {
            "node_id": effect.effect_node_id,
            "name": effect.decorated.name,
            "class_name": type(effect.decorated).__name__,
            "reported_status": terminal_status.name,
            "feedback": getattr(effect.decorated, "feedback_message", ""),
            "effect": effect.contract.effect,
            "risk": effect.contract.risk.value,
            "expected_postcondition": effect.contract.expected_postcondition,
        }
        next_node = _next_effect(tree, effect.effect_node_id)
        history = [
            {"entry": str(entry)}
            for entry in (_bb_get("gpsr/state_log", []) or [])
        ]
        params = _bb_get("gpsr/current_params", {}) or {}
        return CaptureRequest(
            checkpoint_id=checkpoint_id,
            task_id=str(_bb_get("gpsr/task_id", "task")),
            subtask_id=subtask_id,
            tree_revision=str(self.tree_revision),
            plan_revision=int(_bb_get("gpsr/plan_revision", 1) or 1),
            original_instruction=str(_bb_get("gpsr/command", "")),
            subtask_goal=f"{self.action_name}({json.dumps(params, default=str, sort_keys=True)})",
            terminal_node=terminal_node,
            next_node=next_node,
            subtask_tree=tree,
            blackboard=blackboard,
            execution_history=tuple(history),
            recovery_ledger=tuple(self.supervisor.ledger.snapshot()),
            robot_pose=_robot_pose(),
            arm_joints=_arm_joints(),
        )

    def tick(self):
        self.logger.debug("%s.tick()" % self.__class__.__name__)
        # M2b (task-M, round-4 battery fix): captured BEFORE `initialise()`
        # (which never touches `self.status`) -- True only when this slot
        # was already RUNNING as of the end of its OWN previous tick, i.e.
        # this is not its first ever activation. The slot's own status is
        # the one piece of state it owns outright; using it (rather than
        # any global/shared blackboard value) to gate consumption means an
        # idle/not-yet-reached slot -- which the tree has never ticked, so
        # never reaches this method at all -- and a slot on its very first
        # tick -- which cannot yet have any checkpoint/intervention of its
        # own, since one requires this slot's child to have already ticked
        # to a terminal status once -- both correctly never consume.
        is_active = self.status == Status.RUNNING
        if self.status != Status.RUNNING:
            self.initialise()
        self.supervisor.poll()
        current_subtask = self.current_subtask_id()
        if is_active:
            # N1c (round-5 rerun fix): peek, not consume. An intervention
            # against a genuinely still-running child (not merely one whose
            # failure is masked as RUNNING pending a verdict, see
            # `_may_apply_intervention_now`) may be a hallucinated/premature
            # verifier call -- applying it immediately would yank a
            # perfectly healthy in-flight action. Only pop it off the
            # supervisor's queue once we have actually decided to apply it;
            # otherwise it stays queued (peeked again next tick) and
            # `can_start_effect`/`can_finish_subtask` keep seeing it as
            # pending, same as before this fix.
            pending = self.supervisor.peek_intervention(subtask_id=current_subtask)
            if pending is not None:
                if self._may_apply_intervention_now(pending):
                    intervention = self.supervisor.consume_intervention(
                        subtask_id=current_subtask
                    )
                    if intervention is not None:
                        self._deferred_intervention_checkpoint = None
                        self._apply_intervention(intervention)
                else:
                    self._note_deferral(pending)

        if self._hard_stop_reason is not None:
            yield from self._finish_hard_stop()
            return
        if self._release_after_global:
            self._release_after_global = False
            self.status = Status.SUCCESS
            yield self
            return

        if self._child_terminal is None:
            for node in self.decorated.tick():
                yield node
            if self.decorated.status in {Status.SUCCESS, Status.FAILURE}:
                self._child_terminal = self.decorated.status

        if self._child_terminal is Status.SUCCESS:
            if not self.supervisor.can_finish_subtask(current_subtask):
                self.status = Status.RUNNING
                yield self
                return
            if self._active_recovery is not None:
                self.supervisor.recovery_finished(self._active_recovery, succeeded=True)
                self._active_recovery = None
            self.status = Status.SUCCESS
            yield self
            return

        if self._child_terminal is Status.FAILURE:
            # Q1 (task-Q, round-6): under the old architecture a retry's own
            # FAILURE re-entered the verify/recovery pipeline via its own
            # checkpoint, and THAT is where a failed retry's outcome used to
            # reach the ledger (the top of `_apply_intervention`, when the
            # NEXT intervention arrived). Retry internals no longer create
            # checkpoints (see `SupervisedEffect.in_recovery_retry`), so
            # that path is gone -- record the retry's outcome here instead,
            # directly from the composite's own terminal result, exactly as
            # the ruling requires ("the retry's outcome still reaches the
            # ledger via the composite result").
            if self._active_recovery is not None:
                self.supervisor.recovery_finished(self._active_recovery, succeeded=False)
                self._active_recovery = None
            # An unregistered/internal failure cannot be adjudicated leaf-wise.
            _set_supervisor_outcome(
                "failed",
                "uncontracted_subtask_failure",
                subtask_id=current_subtask,
            )
            self._hard_stop_reason = "uncontracted_subtask_failure"
            yield from self._finish_hard_stop()
            return

        self.status = Status.RUNNING
        yield self

    def update(self) -> Status:  # pragma: no cover - custom tick owns status
        return self.status

    def _finish_hard_stop(self):
        # N1a (round-5 rerun fix): a hard stop used to latch `self.status =
        # Status.RUNNING` forever -- the slot never re-ticks its child (that
        # is correct and preserved below), but it also never told the tree
        # it was done, so the tree's own failure machinery (target-failure
        # path, replan/skip, bench exhaustion) never got a chance to run;
        # the only consumer of `gpsr/task_outcome`, `BtNode_FinalizeTask`,
        # sat downstream and unreachable forever. Release the slot via a
        # real terminal FAILURE instead -- exactly once, via `Decorator.stop`
        # (which itself stops a still-RUNNING child with Status.INVALID, so
        # the child is never re-ticked either) -- and every later call just
        # replays that same FAILURE without touching the child again.
        if self.status != Status.FAILURE:
            self._drain_own_interventions()
            self.feedback_message = f"hard stop: {self._hard_stop_reason}"
            self.stop(Status.FAILURE)
        self.status = Status.FAILURE
        yield self

    def _drain_own_interventions(self) -> None:
        # Y-1 (round-5 review fix): N1c's peek-then-defer window can leave a
        # still-queued, still-matching-subtask_id intervention behind when
        # THIS tick's own child independently produces a raw, un-instrumented
        # FAILURE (no SupervisedEffect involved) -- that hard-stops the slot
        # via the branch below, in the SAME tick, strictly AFTER the
        # intervention-processing block at the top of `tick()` already ran
        # and decided to defer. Once hard-stopped the slot never re-enters
        # that block again (`is_active` is permanently False from here on),
        # so a deferred intervention would otherwise sit in
        # `supervisor._interventions` forever -- and `can_start_effect`/
        # `can_finish_subtask`'s `if self._interventions: return False` gate
        # is unscoped by subtask id, so ONE orphaned entry silently blocks
        # EVERY later subtask for the rest of the mission (the X-2 orphaned-
        # intervention shape, reopened through a new door). A dead slot can
        # never apply an intervention against it, so drain every remaining
        # match for this subtask before finalizing -- nothing is left behind
        # for `can_start_effect`/`can_finish_subtask` to misread as pending.
        current_subtask = self.current_subtask_id()
        while self.supervisor.consume_intervention(subtask_id=current_subtask) is not None:
            pass

    def _may_apply_intervention_now(
        self, intervention: SupervisorIntervention
    ) -> bool:
        # N1c (round-5 rerun fix): "the real recovery case" -- the
        # contract-covered effect already reported FAILURE and is merely
        # waiting (masked as RUNNING, see `SupervisedEffect.terminal_status`)
        # on the async verifier/recovery pipeline -- always applies
        # immediately, no matter how young the attempt is. Anything else
        # (the child never terminated at all, e.g. a nominal in-flight goto)
        # only applies once the attempt has run longer than the stall
        # threshold; a hallucinated "recoverable" verdict against a healthy
        # action should not be able to yank it mid-flight.
        if self._child_terminal is not None:
            return True
        if self._has_failed_pending_effect():
            return True
        age_s = self._time_source() - self._attempt_started_at
        # Q2 (task-Q, round-6): the full stall window protects a subtask's
        # FIRST attempt from a premature intervention. Once this subtask
        # has been through >=1 applied intervention (`_has_retried`, set at
        # the end of `_apply_intervention`'s local_recovery branch), it has
        # already forfeited that presumption of nominality -- a FURTHER
        # intervention against the still-RUNNING retry only has to wait the
        # much shorter retry-stall window.
        stall_s = self._retry_stall_s if self._has_retried else self._stall_s
        return age_s >= stall_s

    def _has_failed_pending_effect(self) -> bool:
        for node in self.decorated.iterate():
            if (
                isinstance(node, SupervisedEffect)
                and node.terminal_status is Status.FAILURE
            ):
                return True
        return False

    def _note_deferral(self, intervention: SupervisorIntervention) -> None:
        if self._deferred_intervention_checkpoint == intervention.checkpoint_id:
            return
        self._deferred_intervention_checkpoint = intervention.checkpoint_id
        age_s = self._time_source() - self._attempt_started_at
        self.supervisor.emit_telemetry(
            "intervention.deferred",
            {"checkpoint_id": intervention.checkpoint_id, "age_s": age_s},
        )
        # Q4 (task-Q, round-6): (i) each applied intervention's deferral
        # age contributes to the cumulative overhead budget -- the SAME
        # age_s just reported above, reused rather than recomputed, so a
        # future change to either can never let them silently diverge.
        self.supervisor.record_deferral_overhead(age_s)

    def _apply_intervention(self, intervention: SupervisorIntervention) -> None:
        if self._active_recovery is not None:
            self.supervisor.recovery_finished(self._active_recovery, succeeded=False)
            self._active_recovery = None
        if intervention.kind == "local_recovery":
            proposal = intervention.payload
            assert isinstance(proposal, RecoveryProposal)
            # M1b (task-M, round-4 battery fix): detach-first. The crash
            # this guards against: `Sequence(children=[macro, fresh_subtask])`
            # calls `add_child` -> `Composite.add_child`, which raises
            # RuntimeError if the behaviour it is given already has a
            # `.parent` -- and the slot's OWN currently-attached child
            # (`self.decorated`) still has `.parent is self` at this point in
            # the old code, since nothing had unlinked it yet. `_materialize_
            # subtask` is now asserted to always return a parentless subtree
            # (see M1a), which already closes the one way `fresh_subtask`
            # could BE that live child -- this ordering is the second,
            # independent half of the fix: detach the slot's current child
            # BEFORE building anything that might get added to a composite,
            # mirroring `DynamicExecutor._swap_in`'s build-fully-unattached-
            # then-swap-in pattern, so a bug in any future composite built
            # here cannot resurrect this crash.
            self._detach_child()
            macro = self.recovery_compiler.compile(proposal)
            # Q1 (task-Q, round-6): the fresh retry subtree is the ONE place
            # `in_recovery_retry=True` is ever passed -- see `_materialize_
            # subtask` and `SupervisedEffect` for what that suppresses.
            fresh_subtask = self._materialize_subtask(in_recovery_retry=True)
            sequence = py_trees.composites.Sequence(
                name=f"recover+retry:{proposal.strategy_id}",
                memory=True,
            )
            sequence.add_child(macro)
            sequence.add_child(fresh_subtask)
            self._attach_child(sequence)
            self.tree_revision += 1
            self._child_terminal = None
            self._active_recovery = proposal
            # N1c (round-5 rerun fix): a fresh attempt starts now -- reset
            # the stall clock so the NEXT intervention against this retry is
            # judged against its own age, not however long the PRIOR failed
            # attempt had been running.
            self._attempt_started_at = self._time_source()
            # Q2 (task-Q, round-6): this subtask has now been through >=1
            # applied intervention -- it has forfeited the presumption of
            # nominality `GPSR_SUPERVISION_STALL_S` protects. Any FURTHER
            # intervention against a still-RUNNING retry (e.g. a
            # ledger-exhaustion global replan resolving late) only needs to
            # wait `GPSR_SUPERVISION_RETRY_STALL_S`, not the full stall --
            # see `_may_apply_intervention_now`.
            self._has_retried = True
            self.supervisor.recovery_started(proposal)
            return
        if intervention.kind == "global_replan":
            decision = intervention.payload
            assert isinstance(decision, GlobalPlanDecision)
            self._apply_global_decision(decision)
            return
        _set_supervisor_outcome(
            "stopped",
            intervention.reason,
            subtask_id=self.current_subtask_id(),
        )
        self._hard_stop_reason = intervention.reason

    def _apply_global_decision(self, decision: GlobalPlanDecision) -> None:
        # J7 (round-3 adversarial review, H3, ACTIVE mode): ``len(state_log)``
        # counts every logged line for the WHOLE task (target-transition
        # lines included, see ``DynamicExecutor._log`` / ``BtNode_LogStepResult``
        # both appending to the same ``gpsr/state_log`` key) -- but the
        # two-layer executor (``orchestrator._on_target_failure``) consumes
        # ``preserved_completed_steps`` as a prefix COUNT into the CURRENT
        # TARGET's own action plan (``get_action_plan(slot, index)[:preserved]``).
        # The two counts disagree, so the strict equality check in
        # ``validate_global_decision`` raised ``SchemaError`` for a
        # perfectly-formed decision. ``gpsr/plan_index`` (already on the
        # blackboard, and already used exactly this way by
        # ``current_subtask_id`` above) is the per-target step count:
        # ``BtNode_MaterialiseStep`` sets it to ``step_index + 1`` for the
        # CURRENT target's own plan, so ``plan_index - 1`` is the number of
        # that target's steps completed before the one now in flight.
        completed = max(0, int(_bb_get("gpsr/plan_index", 1) or 1) - 1)
        command = str(_bb_get("gpsr/command", ""))
        from behavior_tree.GPSR.small_trees import ACTION_FACTORIES

        try:
            validate_global_decision(
                decision,
                completed_steps=completed,
                original_instruction=command,
                known_actions=ACTION_FACTORIES,
            )
        except SchemaError as exc:
            # J7: never let a malformed/mismatched global decision crash the
            # tree from inside tick() -- treat it as a stop intervention,
            # same as any other unrecognised intervention kind below.
            _set_supervisor_outcome(
                "stopped",
                str(exc),
                subtask_id=self.current_subtask_id(),
            )
            self._hard_stop_reason = str(exc)
            return
        if decision.action is GlobalAction.ABORT_AND_REPORT:
            _bb_set("gpsr/plan", [])
            _bb_set("gpsr/plan_index", 0)
            _set_supervisor_outcome(
                "aborted",
                decision.rationale,
                operator_message=decision.operator_message,
                subtask_id=self.current_subtask_id(),
            )
        else:
            _bb_set("gpsr/plan", [dict(step) for step in decision.replacement_plan])
            _bb_set("gpsr/plan_index", 0)
            revision = int(_bb_get("gpsr/plan_revision", 1) or 1) + 1
            _bb_set("gpsr/plan_revision", revision)
            _bb_set("gpsr/last_failure", decision.rationale)
            _bb_set(
                "gpsr/supervisor_step_disposition",
                {
                    "kind": decision.action.value,
                    "operator_message": decision.operator_message,
                    "relaxed_constraints": list(decision.relaxed_constraints),
                },
            )
        _bb_set(
            "gpsr/replan_request",
            {
                "level": "supervisor",
                "action": decision.action.value,
                "reason": decision.rationale,
                "replacement_plan": [
                    dict(step) for step in decision.replacement_plan
                ],
                "operator_message": decision.operator_message,
                "preserved_completed_steps": decision.preserved_completed_steps,
                "relaxed_constraints": list(decision.relaxed_constraints),
            },
        )
        self._release_after_global = True

    def _materialize_subtask(
        self, *, in_recovery_retry: bool = False
    ) -> py_trees.behaviour.Behaviour:
        root = self.factory()
        subtree = instrument_effect_nodes(
            root,
            supervisor=self.supervisor,
            slot=self,
            registry=self.registry,
            path=f"{self.action_name}/root",
            in_recovery_retry=in_recovery_retry,
        )
        # M1a (task-M, round-4 battery fix): `self.factory` MUST be a
        # builder closure invoked fresh per call, never a captured/cached
        # subtree instance. The verified root cause of the crash this
        # guards: `GPSRPlanner.build_target_subtree` used to build one
        # small_tree object per plan step and hand `SupervisedSubtaskSlot`
        # `lambda: small_tree` (closing over that ONE already-built
        # instance) as its "factory" -- so every call here, including the
        # very first one made from `__init__`, returned the SAME object.
        # By the time a `local_recovery` intervention called this a second
        # time, that object was already `self.decorated` (parent ==
        # this slot), and building `Sequence(children=[..., fresh_subtask])`
        # crashed inside py_trees `add_child` with "already has parent" --
        # a factory-wiring regression surfacing as a confusing library
        # exception several layers away from its cause. Assert here so it
        # fails loudly, AT the site of the regression, instead: a fresh
        # small_tree object is never anyone else's child yet.
        assert subtree.parent is None, (
            f"SupervisedSubtaskSlot({self.action_name!r})._materialize_subtask "
            f"returned a subtree that already has a parent "
            f"({subtree.name!r} parent={getattr(subtree.parent, 'name', None)!r}); "
            "`factory` must build a fresh, unparented subtree on every call, "
            "never return a cached/closure-captured instance"
        )
        return subtree

    def _detach_child(self) -> None:
        if self.decorated.status == Status.RUNNING:
            self.decorated.stop(Status.INVALID)
        self.decorated.parent = None

    def _attach_child(self, child: py_trees.behaviour.Behaviour) -> None:
        self.children[0] = child
        self.decorated = child
        child.parent = self
        if self._setup_kwargs is not None:
            for node in child.iterate():
                node.setup(**self._setup_kwargs)


def instrument_effect_nodes(
    root: py_trees.behaviour.Behaviour,
    *,
    supervisor: MissionSupervisor,
    slot: SupervisedSubtaskSlot,
    registry: NodeContractRegistry,
    path: str,
    in_recovery_retry: bool = False,
) -> py_trees.behaviour.Behaviour:
    contract = registry.contract_for(root)
    if contract is not None and not isinstance(root, SupervisedEffect):
        return SupervisedEffect(
            child=root,
            contract=contract,
            supervisor=supervisor,
            effect_node_id=path,
            slot=slot,
            in_recovery_retry=in_recovery_retry,
        )
    for index, child in enumerate(list(root.children)):
        replacement = instrument_effect_nodes(
            child,
            supervisor=supervisor,
            slot=slot,
            registry=registry,
            path=f"{path}/{index}",
            in_recovery_retry=in_recovery_retry,
        )
        if replacement is child:
            continue
        if isinstance(root, py_trees.decorators.Decorator):
            root.children[0] = replacement
            root.decorated = replacement
            replacement.parent = root
            child.parent = None
        else:
            root.replace_child(child, replacement)
    return root


def wrap_action_factory(
    action_name: str,
    factory: Callable[[], py_trees.behaviour.Behaviour],
    supervisor: MissionSupervisor | None,
    *,
    target_slot: int | None = None,
    target_index: int | None = None,
    step_index: int | None = None,
) -> py_trees.behaviour.Behaviour:
    if supervisor is None or supervisor.config.mode is SupervisionMode.OFF:
        return factory()
    return SupervisedSubtaskSlot(
        action_name=action_name,
        factory=factory,
        supervisor=supervisor,
        target_slot=target_slot,
        target_index=target_index,
        step_index=step_index,
    )


def _next_effect(
    tree: Mapping[str, Any], effect_node_id: str
) -> Mapping[str, Any] | None:
    nodes = list(tree.get("nodes", []) or [])
    index = next(
        (
            position
            for position, node in enumerate(nodes)
            if node.get("effect_node_id") == effect_node_id
        ),
        -1,
    )
    for node in nodes[index + 1 :]:
        if node.get("effect_contract") and node.get("status") == "INVALID":
            return node
    return next_unticked_node(tree, "")


def _bb_required(key: str) -> Any:
    return py_trees.blackboard.Blackboard.get(key)


def _bb_get(key: str, default: Any) -> Any:
    try:
        return py_trees.blackboard.Blackboard.get(key)
    except (KeyError, AttributeError):
        return default


def _bb_set(key: str, value: Any) -> None:
    py_trees.blackboard.Blackboard.set(key, value)


def _robot_pose() -> tuple[float, float, float]:
    pose = _bb_get("gpsr/last_capture", None) or _bb_get("gpsr/start_pose", None)
    nested = getattr(pose, "pose", pose)
    position = getattr(nested, "position", None)
    orientation = getattr(nested, "orientation", None)
    if position is None:
        return (0.0, 0.0, 0.0)
    yaw = 0.0
    if orientation is not None:
        siny = 2.0 * (
            float(getattr(orientation, "w", 1.0)) * float(getattr(orientation, "z", 0.0))
            + float(getattr(orientation, "x", 0.0)) * float(getattr(orientation, "y", 0.0))
        )
        cosy = 1.0 - 2.0 * (
            float(getattr(orientation, "y", 0.0)) ** 2
            + float(getattr(orientation, "z", 0.0)) ** 2
        )
        import math

        yaw = math.atan2(siny, cosy)
    return (
        float(getattr(position, "x", 0.0)),
        float(getattr(position, "y", 0.0)),
        yaw,
    )


def _arm_joints() -> tuple[float, ...]:
    value = _bb_get("gpsr/arm_navigating", None)
    positions = getattr(value, "position", value)
    try:
        result = tuple(float(item) for item in positions)
    except (TypeError, ValueError):
        result = ()
    return (result + (0.0,) * 7)[:7]


def _set_supervisor_outcome(
    status: str,
    reason: str,
    **extra: Any,
) -> None:
    _bb_set(
        "gpsr/task_outcome",
        {
            "status": status,
            "reason": reason,
            "source": "llm_supervisor",
            **extra,
        },
    )


def _client_telemetry(telemetry: Any):
    if telemetry is None:
        return None

    def emit(event: str, payload: Mapping[str, Any]) -> None:
        telemetry.emit(event, dict(payload), phase="supervision")

    return emit


__all__ = [
    "BtNode_RecoveryDirective",
    "SupervisedEffect",
    "SupervisedSubtaskSlot",
    "configure_default_supervisor",
    "default_recovery_compiler",
    "get_default_supervisor",
    "instrument_effect_nodes",
    "set_default_supervisor",
    "wrap_action_factory",
]
