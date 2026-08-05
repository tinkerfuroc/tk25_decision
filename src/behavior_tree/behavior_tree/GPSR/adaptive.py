"""Runtime primitives for future adaptive GPSR behavior trees.

The current competition tree may continue to use its dispatcher while these
primitives are introduced.  New adaptive implementations should put their
mutable task subtree behind :class:`AdaptiveExecutionSlot` and submit
declarative :class:`gpsr_trace.patch.BehaviorTreeProposal` objects to the
coordinator.  Agents never receive a live ``py_trees`` object.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import Any, Callable, Iterable, Optional

try:
    from gpsr_trace.ir import BehaviorTree, NodeSpec, NodeTypeRegistry, stable_node_id
    from gpsr_trace.patch import BehaviorTreeProposal, TreePatch, apply_patch
    from gpsr_trace.metadata import ProposalMetadata
    from gpsr_trace.events import ValidationError
except ImportError:  # pragma: no cover - allows old installs to import GPSR
    BehaviorTree = NodeSpec = NodeTypeRegistry = BehaviorTreeProposal = TreePatch = ProposalMetadata = None
    stable_node_id = None
    apply_patch = None

    class ValidationError(ValueError):
        pass


@dataclass(frozen=True)
class ProposalResult:
    proposal_id: str
    status: str
    base_version: int | None
    resulting_version: int | None = None
    reason: str = ""


def plan_to_tree_ir(
    plan: Iterable[dict[str, Any]],
    *,
    namespace: str = "gpsr",
    revision: int = 1,
    label: str = "GPSR plan",
) -> Any:
    """Build the versioned declarative IR for a GPSR action plan.

    The current orchestrator still materialises ``ACTION_FACTORIES`` into
    py_trees.  This adapter gives it a stable, serialisable source-of-truth
    document for the adaptive runtime: step ids depend only on the task
    namespace and logical step index, never on Python object identity.  A
    future planner can therefore propose typed patches against this document
    while the live executor remains behind :class:`AdaptiveExecutionSlot`.
    """
    if BehaviorTree is None or NodeSpec is None or stable_node_id is None:
        raise RuntimeError("gpsr_trace is required for declarative GPSR tree IR")
    if not isinstance(namespace, str) or not namespace.strip():
        raise ValidationError("namespace must be a non-empty string")
    if not isinstance(revision, int) or revision < 0:
        raise ValidationError("revision must be a non-negative integer")
    root_id = stable_node_id(namespace, "root")
    nodes: dict[str, Any] = {}
    child_ids: list[str] = []
    for index, raw_step in enumerate(plan):
        if not isinstance(raw_step, dict):
            raise ValidationError(f"plan step {index} must be a mapping")
        action = raw_step.get("action")
        if not isinstance(action, str) or not action.strip():
            raise ValidationError(f"plan step {index} has no action")
        node_id = stable_node_id(namespace, "step", str(index))
        params = raw_step.get("params", {})
        if not isinstance(params, dict):
            raise ValidationError(f"plan step {index} params must be a mapping")
        nodes[node_id] = NodeSpec(
            node_id=node_id,
            node_type="gpsr.action",
            params={"action": action, "params": params, "index": index},
            children=(),
        )
        child_ids.append(node_id)
    nodes[root_id] = NodeSpec(
        node_id=root_id,
        node_type="gpsr.sequence",
        params={"label": label, "namespace": namespace},
        children=tuple(child_ids),
    )
    return BehaviorTree(root_id=root_id, nodes=nodes, version=revision)


class TreeRevisionCoordinator:
    """Serialize autonomous proposals and produce immutable tree revisions.

    ``materialize`` is called only by the owner at a tick barrier.  It must
    construct and set up a candidate subtree without mutating the active one;
    exceptions preserve the old revision and are returned as a rejected result.
    """

    def __init__(
        self,
        initial_tree: Any,
        *,
        registry: Any = None,
        materialize: Callable[[Any], Any] | None = None,
        on_result: Callable[[ProposalResult, Any], None] | None = None,
    ) -> None:
        if BehaviorTree is not None and not isinstance(initial_tree, BehaviorTree):
            raise ValidationError("initial_tree must be gpsr_trace.ir.BehaviorTree")
        self._tree = initial_tree
        self._registry = registry
        self._materialize = materialize
        self._on_result = on_result
        self._pending: list[BehaviorTreeProposal] = []
        self._lock = threading.RLock()
        self._paused = False

    @property
    def tree(self):
        with self._lock:
            return self._tree

    @property
    def version(self) -> int:
        return int(getattr(self.tree, "version", 0))

    @property
    def paused(self) -> bool:
        with self._lock:
            return self._paused

    def set_paused(self, paused: bool) -> None:
        with self._lock:
            self._paused = bool(paused)

    def submit(self, proposal: Any) -> ProposalResult:
        if BehaviorTreeProposal is not None and not isinstance(proposal, BehaviorTreeProposal):
            raise ValidationError("proposal must be BehaviorTreeProposal")
        with self._lock:
            base = getattr(proposal.patch, "base_version", None)
            if base is not None and base != self.version:
                result = ProposalResult(
                    proposal_id=proposal.metadata.proposal_id,
                    status="rejected",
                    base_version=base,
                    resulting_version=self.version,
                    reason=f"stale base version {base}; active version is {self.version}",
                )
                self._notify(result, None)
                return result
            self._pending.append(proposal)
            result = ProposalResult(
                proposal_id=proposal.metadata.proposal_id,
                status="queued",
                base_version=base,
                resulting_version=self.version,
            )
            self._notify(result, None)
            return result

    # Explicit name used by background-agent integrations.  Keeping ``submit``
    # preserves the small API used by existing GPSR code.
    submit_proposal = submit

    def apply_pending(self, *, force: bool = False) -> list[ProposalResult]:
        """Apply queued proposals at the caller's tick barrier.

        Proposals are processed FIFO.  The active IR is changed only after the
        candidate has passed structural/schema validation and optional
        materialization.  ``force`` is reserved for an operator command that
        has already passed the external controller lease.
        """
        results: list[ProposalResult] = []
        with self._lock:
            if self._paused and not force:
                return results
            pending = list(self._pending)
            self._pending.clear()
        for proposal in pending:
            result = self._apply_one(proposal)
            results.append(result)
        return results

    def _apply_one(self, proposal: Any) -> ProposalResult:
        proposal_id = proposal.metadata.proposal_id
        with self._lock:
            base = getattr(proposal.patch, "base_version", None)
            if base is not None and base != self.version:
                result = ProposalResult(proposal_id, "rejected", base, self.version, "stale proposal")
                self._notify(result, None)
                return result
            try:
                candidate = apply_patch(self._tree, proposal.patch, self._registry)
                materialized = self._materialize(candidate) if self._materialize else None
            except Exception as exc:  # candidate remains isolated from active tree
                result = ProposalResult(proposal_id, "rejected", base, self.version, f"candidate rejected: {exc}")
                self._notify(result, None)
                return result
            self._tree = candidate
            result = ProposalResult(proposal_id, "committed", base, candidate.version)
            self._notify(result, materialized)
            return result

    def _notify(self, result: ProposalResult, materialized: Any) -> None:
        if self._on_result:
            try:
                self._on_result(result, materialized)
            except Exception:
                # Telemetry/control callbacks must not break the coordinator.
                pass


class AdaptiveExecutionSlot:
    """A stable py_trees composite whose single child can be replaced safely."""

    def __new__(cls, name: str = "adaptive execution slot", child: Any = None):
        try:
            import py_trees
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("py_trees is required for AdaptiveExecutionSlot") from exc
        sequence = py_trees.composites.Sequence(name=name, memory=True)
        sequence._adaptive_slot = True
        sequence._adaptive_child = None
        if child is not None:
            sequence.add_child(child)
            sequence._adaptive_child = child
        return sequence

    @staticmethod
    def replace(slot: Any, child: Any) -> None:
        """Replace the slot child; callers must be at a tick barrier."""
        if not getattr(slot, "_adaptive_slot", False):
            raise ValidationError("target is not an AdaptiveExecutionSlot")
        current = getattr(slot, "_adaptive_child", None)
        if current is not None:
            slot.replace_child(current, child)
        else:
            slot.add_child(child)
        slot._adaptive_child = child


class DebugControlGateway:
    """Typed command router shared by the ROS gateway and debugger tests."""

    COMMANDS = frozenset({
        "pause", "resume", "cancel", "retry_step", "skip_step",
        "capture_checkpoint", "apply_patch", "rollback", "edit_state",
        "agent_directive", "activate_proposal",
    })

    def __init__(self, coordinator: TreeRevisionCoordinator, callbacks: dict[str, Callable[[dict], Any]] | None = None):
        self.coordinator = coordinator
        self.callbacks = callbacks or {}

    def handle(self, command: dict[str, Any]) -> dict[str, Any]:
        kind = command.get("command")
        if kind not in self.COMMANDS:
            return {"status": "rejected", "reason": f"unsupported command: {kind!r}"}
        expected = command.get("expected_revision")
        if expected is not None and str(expected) != str(self.coordinator.version):
            return {"status": "conflict", "reason": "tree revision changed", "active_revision": self.coordinator.version}
        if kind == "pause":
            self.coordinator.set_paused(True)
            return {"status": "applied", "paused": True}
        if kind == "resume":
            self.coordinator.set_paused(False)
            return {"status": "applied", "paused": False}
        if kind in {"apply_patch", "activate_proposal"}:
            try:
                proposal = _proposal_from_command(command)
                queued = self.coordinator.submit(proposal)
                if queued.status == "rejected":
                    return {"status": "rejected", "proposal": queued.__dict__}
                if kind == "activate_proposal":
                    applied = self.coordinator.apply_pending(force=True)
                    return {
                        "status": "applied" if any(item.status == "committed" for item in applied) else "rejected",
                        "queued": queued.__dict__,
                        "results": [item.__dict__ for item in applied],
                    }
                return {"status": "queued", "proposal": queued.__dict__}
            except Exception as exc:
                return {"status": "rejected", "reason": f"invalid declarative proposal: {exc}"}
        callback = self.callbacks.get(kind)
        if callback is None:
            return {"status": "unavailable", "reason": f"gateway callback not configured for {kind}"}
        try:
            result = callback(command)
            return result if isinstance(result, dict) else {"status": "applied", "result": result}
        except Exception as exc:
            return {"status": "failed", "reason": f"{type(exc).__name__}: {exc}"}


def _proposal_from_command(command: dict[str, Any]) -> Any:
    """Parse the only mutation shape accepted by the runtime gateway.

    The web server may carry arbitrary JSON in a request, but the execution
    process accepts only a versioned ``BehaviorTreeProposal``/``TreePatch``;
    there is deliberately no eval, shell, or Python callback path here.
    """
    if BehaviorTreeProposal is None or TreePatch is None or ProposalMetadata is None:
        raise ValidationError("gpsr_trace is required for declarative proposals")
    raw = command.get("payload")
    if not isinstance(raw, dict):
        raise ValidationError("proposal payload must be an object")
    proposal_value = raw.get("proposal")
    if isinstance(proposal_value, dict):
        metadata_value = proposal_value.get("metadata")
        patch_value = proposal_value.get("patch")
    else:
        metadata_value = raw.get("metadata")
        patch_value = raw.get("patch")
    if not isinstance(metadata_value, dict) or not isinstance(patch_value, dict):
        raise ValidationError("payload must include proposal.metadata and proposal.patch")
    return BehaviorTreeProposal(
        metadata=ProposalMetadata.from_dict(metadata_value),
        patch=TreePatch.from_dict(patch_value),
    )
