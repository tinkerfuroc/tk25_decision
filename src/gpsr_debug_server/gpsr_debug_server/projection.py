"""Pure, forward-compatible projections for GPSR debugger event streams.

The debugger stores the complete event envelope.  This module deliberately
does not know about ROS, SQLite, or FastAPI: a projection can be rebuilt from
an exported JSONL file just as it can from the local event store.  Unknown
event types are retained in ``unknown_events`` instead of being discarded so
that a newer producer never makes an older debugger lose audit information.
"""
from __future__ import annotations

from copy import deepcopy
from typing import Any, Iterable, Mapping


JsonObject = dict[str, Any]
PROJECTION_VERSION = 4


def empty_projection(trajectory_id: str) -> JsonObject:
    """Return the JSON-serialisable initial state for *trajectory_id*."""

    return {
        "projection_version": PROJECTION_VERSION,
        "trajectory_id": trajectory_id,
        "sequence": 0,
        "status": "active",
        "started_at": None,
        "finished_at": None,
        "metadata": {},
        "command": None,
        "tasks": {},
        "agents": {},
        "proposals": [],
        "plans": {"active_revision": None, "revisions": {}},
        "planning_attempts": {},
        "trees": {"active_revision": None, "revisions": {}},
        "outcomes": [],
        "interventions": [],
        "state_history": [],
        "warnings": [],
        "unknown_events": [],
        "last_event": None,
    }


def is_terminal_status(value: Any) -> bool:
    """Whether a status value represents a completed trajectory."""

    if not isinstance(value, str):
        return False
    return value.lower() in {
        "complete",
        "completed",
        "done",
        "finished",
        "success",
        "succeeded",
        "failure",
        "failed",
        "cancelled",
        "canceled",
        "aborted",
        "terminated",
        "incomplete",
    }


def lifecycle_for_event(event: Mapping[str, Any]) -> tuple[bool, bool, str | None]:
    """Return ``(started, completed, status)`` inferred from an event.

    Only trajectory/mission/run lifecycle events complete a trajectory.  A
    ``task.finished`` or ``step.finished`` event is still projected as an
    outcome, but does not make the surrounding trajectory eligible for
    retention.
    """

    event_type = str(event.get("type") or event.get("event_type") or "").lower()
    payload = _payload(event)
    status = _first_string(payload, "status", "outcome", "result")
    started = event_type in {
        "trajectory.started",
        "trajectory.created",
        "mission.started",
        "run.started",
    }
    completed = event_type in {
        "trajectory.finished",
        "trajectory.completed",
        "trajectory.failed",
        "trajectory.cancelled",
        "trajectory.canceled",
        "trajectory.aborted",
        "trajectory.outcome",
        "mission.finished",
        "mission.completed",
        "mission.failed",
        "mission.cancelled",
        "mission.canceled",
        "run.finished",
        "run.completed",
        "run.failed",
        "run.cancelled",
        "run.canceled",
    } or payload.get("completed") is True
    lifecycle_event = started or completed or event_type.startswith(("trajectory.", "mission.", "run."))
    if completed and status is None:
        if event_type.endswith("failed"):
            status = "failed"
        elif event_type.endswith(("cancelled", "canceled")):
            status = "cancelled"
        else:
            status = "completed"
    if started and status is None:
        status = "running"
    if not lifecycle_event:
        status = None
    return started, completed, status


def reduce_events(
    trajectory_id: str,
    events: Iterable[Mapping[str, Any]],
    *,
    initial: Mapping[str, Any] | None = None,
) -> JsonObject:
    """Apply ordered events to an optional checkpoint and return a snapshot."""

    state = deepcopy(dict(initial)) if initial is not None else empty_projection(trajectory_id)
    state["trajectory_id"] = trajectory_id
    _ensure_shape(state)
    for event in events:
        state = apply_event(state, event)
    return state


def apply_event(state: Mapping[str, Any], event: Mapping[str, Any]) -> JsonObject:
    """Apply one event without mutating either input.

    The reducer accepts both the compact debugger event names and the GPSR
    telemetry names (for example ``tree.generated`` and
    ``tree.node_states_changed``).  Producers may add fields freely; fields
    not understood by a specialised reducer remain available in the event
    store and are noted in ``unknown_events``.
    """

    next_state = deepcopy(dict(state))
    trajectory_id = str(event.get("trajectory_id") or event.get("run_id") or event.get("trace_id") or next_state.get("trajectory_id") or "")
    if not trajectory_id:
        raise ValueError("projection event has no trajectory_id")
    next_state["trajectory_id"] = trajectory_id
    _ensure_shape(next_state)

    event_copy = deepcopy(dict(event))
    payload = _payload(event_copy)
    event_type = str(event_copy.get("type") or event_copy.get("event_type") or "unknown")
    lowered = event_type.lower()
    event_copy.setdefault("type", event_type)
    if "occurred_at" not in event_copy and "timestamp" in event_copy:
        event_copy["occurred_at"] = event_copy["timestamp"]

    # A producer may emit a full state checkpoint.  It is useful for exported
    # JSONL replay too, even though the SQLite store also writes its own
    # transparent checkpoints.
    if _is_checkpoint(lowered):
        checkpoint = _checkpoint_payload(payload)
        if checkpoint is not None:
            next_state = deepcopy(checkpoint)
            next_state["trajectory_id"] = trajectory_id
            _ensure_shape(next_state)

    _touch(next_state, event_copy)
    started, completed, lifecycle_status = lifecycle_for_event(event_copy)
    if started:
        next_state["started_at"] = next_state.get("started_at") or event_copy.get("occurred_at") or event_copy.get("timestamp")
    if lifecycle_status is not None:
        next_state["status"] = lifecycle_status
    if completed:
        next_state["finished_at"] = event_copy.get("occurred_at") or event_copy.get("timestamp") or next_state.get("finished_at")

    handled = _is_checkpoint(lowered) or started or completed
    if _is_metadata_event(lowered):
        _apply_metadata(next_state, payload)
        handled = True
    if _is_agent_event(lowered):
        _apply_agent(next_state, event_copy, payload)
        handled = True
    if _is_task_event(lowered):
        _apply_task(next_state, event_copy, payload)
        handled = True
    if _is_proposal_event(lowered):
        _apply_proposal(next_state, event_copy, payload)
        handled = True
    if _is_planning_event(lowered):
        _apply_planning_attempt(next_state, event_copy, payload)
        handled = True
    if _is_plan_event(lowered):
        _apply_plan(next_state, event_copy, payload)
        handled = True
    if _is_tree_event(lowered):
        _apply_tree(next_state, event_copy, payload)
        handled = True
    if _is_tree_tick_event(lowered):
        _apply_tree_tick(next_state, event_copy, payload)
        handled = True
    if _is_node_event(lowered):
        _apply_nodes(next_state, event_copy, payload)
        handled = True
    if _is_step_event(lowered):
        _apply_step(next_state, event_copy, payload)
        handled = True
    if _is_outcome_event(lowered):
        _apply_outcome(next_state, event_copy, payload)
        handled = True
    if _is_intervention_event(lowered):
        _apply_intervention(next_state, event_copy, payload)
        handled = True
    if _is_state_event(lowered):
        _apply_state(next_state, event_copy, payload)
        handled = True
    if lowered.endswith((".warning", ".warn")):
        _append_limited(next_state["warnings"], _event_summary(event_copy), 100)
        handled = True
    if not handled:
        _append_limited(next_state["unknown_events"], _event_summary(event_copy), 100)
    return next_state


def _ensure_shape(state: JsonObject) -> None:
    state["projection_version"] = PROJECTION_VERSION
    state.setdefault("sequence", 0)
    state.setdefault("status", "active")
    state.setdefault("started_at", None)
    state.setdefault("finished_at", None)
    state.setdefault("metadata", {})
    state.setdefault("command", None)
    state.setdefault("tasks", {})
    state.setdefault("agents", {})
    state.setdefault("proposals", [])
    state.setdefault("plans", {"active_revision": None, "revisions": {}})
    state["plans"].setdefault("active_revision", None)
    state["plans"].setdefault("revisions", {})
    state.setdefault("planning_attempts", {})
    state.setdefault("trees", {"active_revision": None, "revisions": {}})
    state["trees"].setdefault("active_revision", None)
    state["trees"].setdefault("revisions", {})
    state.setdefault("outcomes", [])
    state.setdefault("interventions", [])
    state.setdefault("state_history", [])
    state.setdefault("warnings", [])
    state.setdefault("unknown_events", [])
    state.setdefault("last_event", None)


def _payload(event: Mapping[str, Any]) -> JsonObject:
    value = event.get("payload", {})
    return dict(value) if isinstance(value, Mapping) else {"value": deepcopy(value)}


def _touch(state: JsonObject, event: Mapping[str, Any]) -> None:
    sequence = event.get("sequence")
    if isinstance(sequence, int) and not isinstance(sequence, bool):
        state["sequence"] = max(int(state.get("sequence", 0)), sequence)
    state["last_event"] = _event_summary(event)


def _event_summary(event: Mapping[str, Any]) -> JsonObject:
    return {
        "event_id": event.get("event_id"),
        "sequence": event.get("sequence"),
        "type": event.get("type") or event.get("event_type"),
        "occurred_at": event.get("occurred_at") or event.get("timestamp"),
    }


def _first_string(source: Mapping[str, Any], *keys: str) -> str | None:
    for key in keys:
        value = source.get(key)
        if isinstance(value, str) and value:
            return value
    return None


def _is_checkpoint(event_type: str) -> bool:
    return event_type in {"checkpoint", "checkpoint.created", "trajectory.checkpoint", "state.checkpoint"}


def _checkpoint_payload(payload: Mapping[str, Any]) -> JsonObject | None:
    for key in ("snapshot", "state", "projection", "checkpoint"):
        value = payload.get(key)
        if isinstance(value, Mapping):
            return dict(value)
    return None


def _is_metadata_event(event_type: str) -> bool:
    return event_type.startswith("trajectory.") or event_type.startswith("mission.") or event_type.startswith("run.")


def _apply_metadata(state: JsonObject, payload: Mapping[str, Any]) -> None:
    metadata = state["metadata"]
    incoming = payload.get("metadata")
    if isinstance(incoming, Mapping):
        _deep_merge(metadata, incoming)
    for key in ("name", "title", "description", "mode", "source", "tags", "pinned"):
        if key in payload:
            metadata[key] = deepcopy(payload[key])


def _is_agent_event(event_type: str) -> bool:
    return event_type.startswith("agent.") or event_type in {"agents.updated", "agents.snapshot"}


def _apply_agent(state: JsonObject, event: Mapping[str, Any], payload: Mapping[str, Any]) -> None:
    candidates: list[Mapping[str, Any]] = []
    raw_agents = payload.get("agents")
    if isinstance(raw_agents, Mapping):
        for agent_id, agent in raw_agents.items():
            if isinstance(agent, Mapping):
                candidates.append({"agent_id": str(agent_id), **dict(agent)})
    elif isinstance(raw_agents, list):
        candidates.extend(item for item in raw_agents if isinstance(item, Mapping))
    else:
        embedded = payload.get("agent")
        candidates.append(embedded if isinstance(embedded, Mapping) else payload)
    for candidate in candidates:
        agent_id = _first_string(candidate, "agent_id", "id", "name")
        if agent_id is None:
            continue
        current = state["agents"].setdefault(agent_id, {"agent_id": agent_id})
        _deep_merge(current, candidate)
        current["agent_id"] = agent_id
        current["updated_sequence"] = event.get("sequence")
        current["updated_at"] = event.get("occurred_at")


def _is_task_event(event_type: str) -> bool:
    return event_type.startswith("task.")


def _apply_task(state: JsonObject, event: Mapping[str, Any], payload: Mapping[str, Any]) -> None:
    task_id = _first_string(payload, "task_id", "id") or _first_string(event, "task_id")
    if task_id is None:
        task_id = "task-default"
    task = state["tasks"].setdefault(
        task_id,
        {"task_id": task_id, "status": "pending", "steps": {}},
    )
    event_type = str(event.get("type") or event.get("event_type") or "")
    command = _first_string(payload, "command", "instruction", "goal")
    if command is not None:
        task["command"] = command
        state["command"] = command
    if event_type == "task.command_received":
        task["status"] = "planning"
    elif event_type == "task.execution_started":
        task["status"] = "running"
    elif event_type.endswith(".finished"):
        task["status"] = _first_string(payload, "status", "outcome", "result") or "completed"
    _deep_merge(
        task,
        {
            key: value
            for key, value in payload.items()
            if key not in {"steps"}
        },
    )
    task["task_id"] = task_id
    task["last_type"] = event_type
    task["updated_sequence"] = event.get("sequence")
    task["updated_at"] = event.get("occurred_at") or event.get("timestamp")


def _is_planning_event(event_type: str) -> bool:
    return event_type.startswith("planner.") or event_type.startswith("planning.")


def _is_proposal_event(event_type: str) -> bool:
    return event_type.startswith(("proposal.", "vote.")) or event_type in {
        "tree.patch_requested", "tree.patch_validated", "tree.patch_applied",
    }


def _apply_proposal(state: JsonObject, event: Mapping[str, Any], payload: Mapping[str, Any]) -> None:
    proposal_id = _first_string(payload, "proposal_id", "request_id", "id") or str(event.get("event_id", ""))
    incoming: JsonObject = {
        "proposal_id": proposal_id,
        "type": event.get("type") or event.get("event_type"),
        "sequence": event.get("sequence"),
        "occurred_at": event.get("occurred_at") or event.get("timestamp"),
    }
    _deep_merge(incoming, payload)
    incoming_type = str(incoming.get("type", "")).lower()
    if incoming_type == "proposal.created" and "agent_id" in incoming:
        incoming.setdefault("proposer_agent_id", incoming["agent_id"])
    for index, existing in enumerate(state["proposals"]):
        if existing.get("proposal_id") == proposal_id:
            record = deepcopy(existing)
            if incoming_type.startswith("vote."):
                record.setdefault("votes", []).append(incoming)
                record["last_type"] = incoming.get("type")
                record["updated_sequence"] = incoming.get("sequence")
                record["updated_at"] = incoming.get("occurred_at")
            else:
                _deep_merge(record, incoming)
            state["proposals"][index] = record
            return
    if incoming_type.startswith("vote."):
        incoming["votes"] = [deepcopy(incoming)]
    state["proposals"].append(incoming)


def _apply_planning_attempt(state: JsonObject, event: Mapping[str, Any], payload: Mapping[str, Any]) -> None:
    attempt_id = _first_string(payload, "attempt_id", "request_id", "id") or str(event.get("event_id", ""))
    if not attempt_id:
        return
    current = state["planning_attempts"].setdefault(attempt_id, {"attempt_id": attempt_id})
    _deep_merge(current, payload)
    current["attempt_id"] = attempt_id
    current["last_type"] = event.get("type")
    current["updated_sequence"] = event.get("sequence")
    current["updated_at"] = event.get("occurred_at")


def _is_plan_event(event_type: str) -> bool:
    return event_type.startswith("plan.") or event_type in {"planner.plan_committed", "planner.plan_validated"}


def _plan_revision(payload: Mapping[str, Any], event: Mapping[str, Any]) -> str | None:
    for source in (payload, event):
        value = source.get("revision", source.get("plan_revision", source.get("plan_id")))
        if value is not None and str(value):
            return str(value)
    return None


def _apply_plan(state: JsonObject, event: Mapping[str, Any], payload: Mapping[str, Any]) -> None:
    event_type = str(event.get("type", "")).lower()
    revisions = state["plans"]["revisions"]
    revision = _plan_revision(payload, event)

    if event_type == "plan.superseded":
        old = payload.get("old_revision", payload.get("old_plan_id"))
        new = payload.get("new_revision", payload.get("new_plan_id", revision))
        if old is not None:
            old_key = str(old)
            record = revisions.setdefault(old_key, {"revision": old_key})
            record["status"] = "superseded"
            record["superseded_by"] = None if new is None else str(new)
            record["updated_sequence"] = event.get("sequence")
        if new is not None:
            revision = str(new)
            record = revisions.setdefault(revision, {"revision": revision})
            record["supersedes"] = None if old is None else str(old)
            state["plans"]["active_revision"] = revision
    if revision is None:
        return
    record = revisions.setdefault(revision, {"revision": revision, "created_at": event.get("occurred_at")})
    plan_data = payload.get("plan")
    if isinstance(plan_data, Mapping):
        _deep_merge(record, plan_data)
    elif isinstance(plan_data, list):
        # The GPSR orchestrator emits its large-step breakdown directly as
        # ``payload.plan``. Normalize that wire shape to the projection's
        # stable ``steps`` field so old and new producers render identically.
        record["steps"] = deepcopy(plan_data)
    for key in ("plan_id", "steps", "status", "validation", "accepted", "reason", "fallback", "offline_mock"):
        if key in payload:
            record[key] = deepcopy(payload[key])
    record["revision"] = revision
    record["last_type"] = event.get("type")
    record["updated_sequence"] = event.get("sequence")
    record["updated_at"] = event.get("occurred_at")
    if event_type not in {"plan.validated", "planner.plan_validated"} and record.get("status") != "superseded":
        state["plans"]["active_revision"] = revision


def _is_tree_event(event_type: str) -> bool:
    return event_type.startswith("tree.") and "node" not in event_type


def _is_tree_tick_event(event_type: str) -> bool:
    return event_type == "tree.tick_observed"


def _is_node_event(event_type: str) -> bool:
    return "node" in event_type and (event_type.startswith("tree.") or event_type.startswith("node."))


def _tree_revision(payload: Mapping[str, Any], event: Mapping[str, Any], state: Mapping[str, Any]) -> str:
    for source in (payload, event):
        value = source.get(
            "revision",
            source.get("tree_revision", source.get("resulting_tree_version")),
        )
        if value is not None and str(value):
            return str(value)
    nested = payload.get("tree", payload.get("document"))
    if isinstance(nested, Mapping):
        value = nested.get("revision", nested.get("tree_revision"))
        if value is not None and str(value):
            return str(value)
    active = state["trees"].get("active_revision")
    return str(active) if active is not None else "default"


def _apply_tree(state: JsonObject, event: Mapping[str, Any], payload: Mapping[str, Any]) -> None:
    revision = _tree_revision(payload, event, state)
    revisions = state["trees"]["revisions"]
    tree = revisions.setdefault(revision, {"revision": revision, "nodes": {}, "edges": []})
    tree["revision"] = revision
    document = payload.get("tree", payload.get("document"))
    if isinstance(document, Mapping):
        _deep_merge(tree, {key: value for key, value in document.items() if key not in {"nodes", "edges"}})
    for key in ("tree_id", "kind", "plan_id", "plan_revision", "root_id", "edges", "metadata"):
        if key in payload:
            tree[key] = deepcopy(payload[key])
    if "nodes" in payload:
        _merge_topology_nodes(tree, payload["nodes"])
    elif isinstance(document, Mapping) and "nodes" in document:
        _merge_topology_nodes(tree, document["nodes"])
    if "edges" not in payload and isinstance(document, Mapping) and "edges" in document:
        tree["edges"] = deepcopy(document["edges"])
    tree["updated_sequence"] = event.get("sequence")
    tree["updated_at"] = event.get("occurred_at")
    state["trees"]["active_revision"] = revision


def _merge_topology_nodes(tree: JsonObject, raw_nodes: Any) -> None:
    nodes = tree.get("nodes")
    if not isinstance(nodes, dict):
        nodes = {}
        tree["nodes"] = nodes
    if isinstance(raw_nodes, Mapping):
        entries = []
        for node_id, value in raw_nodes.items():
            if isinstance(value, Mapping):
                entries.append({"node_id": str(node_id), **dict(value)})
    elif isinstance(raw_nodes, list):
        entries = [value for value in raw_nodes if isinstance(value, Mapping)]
    else:
        return
    for entry in entries:
        node_id = _first_string(entry, "node_id", "id", "uid", "name")
        if node_id is None:
            continue
        current = nodes.setdefault(node_id, {"node_id": node_id})
        _deep_merge(current, entry)
        current["node_id"] = node_id


def _apply_nodes(state: JsonObject, event: Mapping[str, Any], payload: Mapping[str, Any]) -> None:
    revision = _tree_revision(payload, event, state)
    tree = state["trees"]["revisions"].setdefault(revision, {"revision": revision, "nodes": {}, "edges": []})
    updates = payload.get(
        "changed",
        payload.get("changed_nodes", payload.get("changes", payload.get("node_states", payload.get("nodes")))),
    )
    if updates is None and any(key in payload for key in ("node_id", "id", "uid")):
        updates = [payload]
    _merge_topology_nodes(tree, updates)
    # Mark only the nodes supplied by this delta as updated.
    supplied = _normalise_nodes(updates)
    for node_id, update in supplied.items():
        node = tree["nodes"].get(node_id)
        if node is not None:
            node["updated_sequence"] = event.get("sequence")
            node["updated_at"] = event.get("occurred_at")
            if "state" in node and "status" not in node:
                node["status"] = node["state"]
    tree["updated_sequence"] = event.get("sequence")
    tree["updated_at"] = event.get("occurred_at")
    state["trees"]["active_revision"] = revision


def _apply_tree_tick(state: JsonObject, event: Mapping[str, Any], payload: Mapping[str, Any]) -> None:
    """Project the latest actual visitation while retaining one prior tick.

    ``tree.node_states_changed`` remains supported for older producers. A
    ``tree.tick_observed`` event adds the stronger assertion that a node was
    visited in that exact tick, preserving deterministic visitor order and
    control-flow counter deltas for the debugger.
    """

    revision = _tree_revision(payload, event, state)
    tree = state["trees"]["revisions"].setdefault(revision, {"revision": revision, "nodes": {}, "edges": []})
    raw_nodes = payload.get("visited_nodes", payload.get("runtime_nodes", payload.get("nodes", [])))
    normalised = _normalise_nodes(raw_nodes)
    _merge_topology_nodes(tree, raw_nodes)
    runtime_nodes: JsonObject = {}
    for node_id, raw in normalised.items():
        node_runtime = deepcopy(dict(raw))
        node_runtime["node_id"] = node_id
        if "state" in node_runtime and "status" not in node_runtime:
            node_runtime["status"] = node_runtime["state"]
        runtime_nodes[node_id] = node_runtime
        node = tree["nodes"].get(node_id)
        if node is not None:
            node["updated_sequence"] = event.get("sequence")
            node["updated_at"] = event.get("occurred_at")

    supplied_order = payload.get("visit_order")
    visit_order = (
        [str(node_id) for node_id in supplied_order if str(node_id) in runtime_nodes]
        if isinstance(supplied_order, list)
        else [node_id for node_id, _node in sorted(runtime_nodes.items(), key=lambda item: item[1].get("visit_order", 0))]
    )
    raw_deltas = payload.get("counter_deltas", payload.get("retry_repeat_deltas", []))
    deltas = [deepcopy(item) for item in raw_deltas if isinstance(item, Mapping)] if isinstance(raw_deltas, list) else []
    current_tick: JsonObject = {
        "tick": payload.get("tick"),
        "visited_node_ids": visit_order,
        "nodes": runtime_nodes,
        "counter_deltas": deltas,
        # Keep the early wire name as a stable alias for old clients/tests.
        "retry_repeat_deltas": deepcopy(deltas),
    }
    if isinstance(payload.get("active_action_context"), Mapping):
        current_tick["active_action_context"] = deepcopy(dict(payload["active_action_context"]))
    runtime = tree.setdefault("runtime", {})
    previous_tick = runtime.get("current_tick")
    runtime["previous_tick"] = deepcopy(previous_tick) if isinstance(previous_tick, Mapping) else None
    runtime["current_tick"] = current_tick
    tree["last_tick"] = payload.get("tick")
    tree["updated_sequence"] = event.get("sequence")
    tree["updated_at"] = event.get("occurred_at")
    state["trees"]["active_revision"] = revision


def _normalise_nodes(raw_nodes: Any) -> dict[str, Mapping[str, Any]]:
    result: dict[str, Mapping[str, Any]] = {}
    if isinstance(raw_nodes, Mapping):
        if any(key in raw_nodes for key in ("node_id", "id", "uid", "name")):
            entries = [raw_nodes]
        else:
            entries = [{"node_id": str(key), **dict(value)} for key, value in raw_nodes.items() if isinstance(value, Mapping)]
    elif isinstance(raw_nodes, list):
        entries = [value for value in raw_nodes if isinstance(value, Mapping)]
    else:
        entries = []
    for entry in entries:
        node_id = _first_string(entry, "node_id", "id", "uid", "name")
        if node_id is not None:
            result[node_id] = entry
    return result


def _is_outcome_event(event_type: str) -> bool:
    return (
        event_type.startswith("outcome.")
        or event_type.endswith(".finished")
        or event_type.endswith(".failed")
        or event_type.endswith(".succeeded")
        or event_type.endswith(".cancelled")
        or event_type.endswith(".canceled")
    )


def _is_step_event(event_type: str) -> bool:
    return event_type.startswith("step.")


def _apply_step(state: JsonObject, event: Mapping[str, Any], payload: Mapping[str, Any]) -> None:
    task_id = _first_string(payload, "task_id") or _first_string(event, "task_id") or "task-default"
    task = state["tasks"].setdefault(
        task_id,
        {"task_id": task_id, "status": "running", "steps": {}},
    )
    steps = task.setdefault("steps", {})
    step_id = _first_string(payload, "step_id", "id") or str(event.get("event_id", ""))
    record = steps.setdefault(step_id, {"step_id": step_id})
    _deep_merge(record, payload)
    event_type = str(event.get("type") or event.get("event_type") or "")
    record["last_type"] = event_type
    record["status"] = (
        _first_string(payload, "status", "outcome", "result")
        or ("running" if event_type == "step.started" else record.get("status", "unknown"))
    )
    record["updated_sequence"] = event.get("sequence")
    record["updated_at"] = event.get("occurred_at") or event.get("timestamp")
    task["status"] = "running"
    task["updated_sequence"] = event.get("sequence")
    task["updated_at"] = event.get("occurred_at") or event.get("timestamp")

    plan_revision = payload.get("plan_revision")
    if plan_revision is None and step_id.startswith("plan-r") and "/" in step_id:
        plan_revision = step_id.split("/", 1)[0].removeprefix("plan-r")
    plan = state["plans"]["revisions"].get(str(plan_revision)) if plan_revision is not None else None
    if not isinstance(plan, dict) or not isinstance(plan.get("steps"), list):
        return
    index = payload.get("step_index")
    if not isinstance(index, int):
        try:
            index = int(step_id.rsplit("-", 1)[-1])
        except (TypeError, ValueError):
            return
    if 0 <= index < len(plan["steps"]) and isinstance(plan["steps"][index], dict):
        plan["steps"][index]["status"] = record["status"]
        plan["steps"][index]["step_id"] = step_id


def _apply_outcome(state: JsonObject, event: Mapping[str, Any], payload: Mapping[str, Any]) -> None:
    outcome_id = _first_string(payload, "outcome_id", "task_id", "step_id", "id") or str(event.get("event_id", ""))
    record: JsonObject = {
        "outcome_id": outcome_id,
        "type": event.get("type"),
        "sequence": event.get("sequence"),
        "occurred_at": event.get("occurred_at"),
    }
    _deep_merge(record, payload)
    for index, existing in enumerate(state["outcomes"]):
        if existing.get("outcome_id") == outcome_id:
            state["outcomes"][index] = record
            return
    state["outcomes"].append(record)


def _is_intervention_event(event_type: str) -> bool:
    return event_type.startswith(("intervention.", "control.")) or event_type in {
        "trajectory.patch_applied",
        "trajectory.rollback",
    }


def _is_state_event(event_type: str) -> bool:
    return event_type.startswith(("state.", "blackboard.", "shared_memory."))


def _apply_state(state: JsonObject, event: Mapping[str, Any], payload: Mapping[str, Any]) -> None:
    record: JsonObject = {
        "event_id": event.get("event_id"),
        "type": event.get("type") or event.get("event_type"),
        "sequence": event.get("sequence"),
        "occurred_at": event.get("occurred_at") or event.get("timestamp"),
    }
    record["payload"] = deepcopy(dict(payload))
    _append_limited(state["state_history"], record, 500)


def _apply_intervention(state: JsonObject, event: Mapping[str, Any], payload: Mapping[str, Any]) -> None:
    intervention_id = _first_string(payload, "intervention_id", "request_id", "command_id", "id") or str(event.get("event_id", ""))
    record: JsonObject = {
        "intervention_id": intervention_id,
        "type": event.get("type"),
        "sequence": event.get("sequence"),
        "occurred_at": event.get("occurred_at"),
    }
    _deep_merge(record, payload)
    for index, existing in enumerate(state["interventions"]):
        if existing.get("intervention_id") == intervention_id:
            state["interventions"][index] = record
            return
    state["interventions"].append(record)


def _deep_merge(target: JsonObject, source: Mapping[str, Any]) -> None:
    for key, value in source.items():
        if isinstance(value, Mapping) and isinstance(target.get(key), dict):
            _deep_merge(target[key], value)
        else:
            target[key] = deepcopy(value)


def _append_limited(values: list[Any], value: Any, limit: int) -> None:
    values.append(value)
    if len(values) > limit:
        del values[:-limit]
