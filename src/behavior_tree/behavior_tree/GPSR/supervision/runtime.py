"""py_trees integration for the GPSR mission supervisor."""
from __future__ import annotations

import json
import os
import threading
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
    ) -> None:
        super().__init__(f"recovery:{proposal.kind.value}:{proposal.strategy_id}")
        self.proposal = proposal
        self.handlers = dict(handlers or {})
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
            self.feedback_message = (
                f"no production handler installed for {self.proposal.kind.value}"
            )
            return Status.FAILURE
        self._done = True
        self.feedback_message = (
            f"MOCK recovery directive executed: {self.proposal.kind.value} "
            f"{dict(self.proposal.arguments)}"
        )
        return Status.SUCCESS


def default_recovery_compiler(
    handlers: Mapping[RecoveryKind, Callable[[RecoveryProposal], bool]] | None = None,
) -> RecoveryMacroCompiler:
    return RecoveryMacroCompiler(
        {
            kind: (
                lambda proposal, _handlers=handlers: BtNode_RecoveryDirective(
                    proposal, _handlers
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
    ) -> None:
        super().__init__(name=f"supervise:{child.name}", child=child)
        self.contract = contract
        self.supervisor = supervisor
        self.effect_node_id = effect_node_id
        self.slot = slot
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
            request = self.slot.build_capture_request(
                effect=self,
                terminal_status=child_status,
            )
            self._checkpoint_id = self.supervisor.submit(
                request,
                self.contract,
                ReportedStatus(child_status.name),
            )

        if (
            self.supervisor.config.mode is not SupervisionMode.ACTIVE
            or self._terminal_status is Status.SUCCESS
        ):
            new_status = self._terminal_status
        else:
            resolution = self.supervisor.resolution(self._checkpoint_id or "")
            new_status = Status.SUCCESS if resolution == "success" else Status.RUNNING

        if new_status != Status.RUNNING:
            self.stop(new_status)
        self.status = new_status
        yield self

    def update(self) -> Status:  # pragma: no cover - custom tick owns status
        return self.status


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
    ) -> None:
        self.action_name = action_name
        self.factory = factory
        self.supervisor = supervisor
        self.registry = registry or default_node_contracts()
        self.recovery_compiler = recovery_compiler or default_recovery_compiler()
        self.tree_revision = 1
        self._setup_kwargs: dict[str, Any] | None = None
        self._child_terminal: Status | None = None
        self._active_recovery: RecoveryProposal | None = None
        self._hard_stop_reason: str | None = None
        self._release_after_global = False
        child = self._materialize_subtask()
        super().__init__(name=f"adaptive:{action_name}", child=child)
        self._adaptive_slot = True

    def setup(self, **kwargs) -> None:
        self._setup_kwargs = dict(kwargs)

    def current_subtask_id(self) -> str:
        task_id = str(_bb_get("gpsr/task_id", "task"))
        plan_revision = int(_bb_get("gpsr/plan_revision", 1) or 1)
        plan_index = max(0, int(_bb_get("gpsr/plan_index", 1) or 1) - 1)
        return f"{task_id}/plan-r{plan_revision}/step-{plan_index:04d}:{self.action_name}"

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
        if self.status != Status.RUNNING:
            self.initialise()
        self.supervisor.poll()
        current_subtask = self.current_subtask_id()
        intervention = self.supervisor.consume_intervention(subtask_id=current_subtask)
        if intervention is not None:
            self._apply_intervention(intervention)

        if self._hard_stop_reason is not None:
            self.status = Status.RUNNING
            yield self
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
            # An unregistered/internal failure cannot be adjudicated leaf-wise.
            _set_supervisor_outcome(
                "failed",
                "uncontracted_subtask_failure",
                subtask_id=current_subtask,
            )
            self._hard_stop_reason = "uncontracted_subtask_failure"
            self.status = Status.RUNNING
            yield self
            return

        self.status = Status.RUNNING
        yield self

    def update(self) -> Status:  # pragma: no cover - custom tick owns status
        return self.status

    def _apply_intervention(self, intervention: SupervisorIntervention) -> None:
        if self._active_recovery is not None:
            self.supervisor.recovery_finished(self._active_recovery, succeeded=False)
            self._active_recovery = None
        if intervention.kind == "local_recovery":
            proposal = intervention.payload
            assert isinstance(proposal, RecoveryProposal)
            macro = self.recovery_compiler.compile(proposal)
            fresh_subtask = self._materialize_subtask()
            sequence = py_trees.composites.Sequence(
                name=f"recover+retry:{proposal.strategy_id}",
                memory=True,
                children=[macro, fresh_subtask],
            )
            self._replace_child(sequence)
            self.tree_revision += 1
            self._child_terminal = None
            self._active_recovery = proposal
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

    def _materialize_subtask(self) -> py_trees.behaviour.Behaviour:
        root = self.factory()
        return instrument_effect_nodes(
            root,
            supervisor=self.supervisor,
            slot=self,
            registry=self.registry,
            path=f"{self.action_name}/root",
        )

    def _replace_child(self, child: py_trees.behaviour.Behaviour) -> None:
        if self.decorated.status == Status.RUNNING:
            self.decorated.stop(Status.INVALID)
        self.decorated.parent = None
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
) -> py_trees.behaviour.Behaviour:
    contract = registry.contract_for(root)
    if contract is not None and not isinstance(root, SupervisedEffect):
        return SupervisedEffect(
            child=root,
            contract=contract,
            supervisor=supervisor,
            effect_node_id=path,
            slot=slot,
        )
    for index, child in enumerate(list(root.children)):
        replacement = instrument_effect_nodes(
            child,
            supervisor=supervisor,
            slot=slot,
            registry=registry,
            path=f"{path}/{index}",
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
) -> py_trees.behaviour.Behaviour:
    if supervisor is None or supervisor.config.mode is SupervisionMode.OFF:
        return factory()
    return SupervisedSubtaskSlot(
        action_name=action_name,
        factory=factory,
        supervisor=supervisor,
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
