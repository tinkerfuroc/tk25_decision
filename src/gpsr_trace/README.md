# gpsr_trace

`gpsr_trace` is a small `ament_python` package for observable GPSR planning
and behavior-tree changes. Its runtime uses only the Python standard library.

## Trace events

`TraceEvent` is a strict, versioned JSON envelope. It includes a UUID event
id, trace id, source id, monotonic source sequence, timezone-aware timestamp,
parent event id, and explicit causation ids. `to_json()` and `from_json()`
validate the complete wire representation.

`TraceClient` owns one default `trace_id` for its producer session (pass the
GPSR trajectory id explicitly when multiple processes share a mission) and
accepts events without waiting on network I/O. A daemon thread
drains a bounded producer queue into batched NDJSON writes over a Unix-domain
socket. When no collector is available it writes whole records to an atomic,
bounded local spool; a later connection replays that spool before newer data.
Queue overflow is visible through `False` from `emit()` and `stats.dropped`.

```python
from gpsr_trace import TraceClient

with TraceClient("/run/gpsr/trace.sock", source_id="gpsr-planner") as trace:
    trace.emit("plan.started", {"goal": "serve water", "token": "never sent"})
```

Payloads are redacted before delivery (default sensitive key names include
`token`, `password`, and `authorization`). `ContentPolicy` bounds inline JSON
content and can pass oversized canonical bytes to an `artifact_hook`; the
event contains an `artifact_ref` instead of the inline value.

## Immutable behavior-tree IR

`NodeSpec` and `BehaviorTree` are frozen declarations. Use `stable_node_id()`
or `NodeSpec.create()` to derive ids from a stable namespace and logical path.
The tree enforces one root, exactly one parent for every other node, valid
children, reachability, and acyclicity. `NodeTypeRegistry` additionally checks
registered node types, parameter names/types, and child limits.

```python
from gpsr_trace import BehaviorTree, NodeSpec, NodeTypeRegistry, NodeTypeSchema, ParamSpec

root = NodeSpec("root", "sequence", {}, ("announce",))
announce = NodeSpec("announce", "say", {"text": "Hello"})
tree = BehaviorTree("root", {"root": root, "announce": announce})
registry = NodeTypeRegistry({
    "sequence": NodeTypeSchema("sequence", max_children=None),
    "say": NodeTypeSchema("say", {"text": ParamSpec(str, required=True)}, max_children=0),
})
tree.validate(registry)
```

`TreePatch` carries typed `AddNode`, `RemoveNode`, `ReplaceNode`, `MoveNode`,
and `UpdateNode` edits. `apply_patch()` is atomic: it returns a new incremented
tree only after structural and optional schema validation succeeds.

`AgentMetadata`, `ProposalMetadata`, and `BehaviorTreeProposal` provide
immutable agent identity and review/audit metadata for a proposed patch.

## Tests

From this package directory:

```bash
python3 -m pytest
```
