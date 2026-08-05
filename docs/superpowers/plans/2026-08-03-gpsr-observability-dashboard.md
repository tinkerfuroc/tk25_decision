# GPSR End-to-End Observability Dashboard Plan

**Date:** 2026-08-03  
**Scope:** `tk25_decision` GPSR telemetry producer, versioned declarative
behavior-tree IR/patches, and a separate developer-only `gpsr_debug_server`  
**Primary goal:** Track a GPSR mission from command intake through every LLM
attempt, accepted plan, behavior-tree artifact, live execution, correction, and
terminal outcome without coupling the web server to GPSR's current Python
classes or blackboard layout.

## 1. Current implementation baseline

The canonical command is `ros2 run behavior_tree gpsr`. It enters the arena,
goes to the command point, captures the operator pose, collects and plans all
commands, and then executes those saved plans in sequence.

For each command, `BtNode_PlanActions`:

1. Builds a system prompt and a user prompt containing the command, known
   locations/objects, completed steps, and the previous failure when replanning.
2. Calls OpenRouter through the OpenAI-compatible chat-completions API.
3. Extracts JSON, cleans unknown actions, and validates the plan.
4. Retries up to four times, feeding the rejection reason into the next prompt.
5. Stores only the accepted/fallback action list on the blackboard.

The accepted plan is a list of `{action, params}` records. Each action selects a
pre-built small tree from `ACTION_FACTORIES`. At runtime the live tree is a
fixed dispatcher containing every registered action branch; it is not the same
object as the plan-shaped tree rendered by `plan_viz.py`. The latter is a
structural reconstruction used for Graphviz and generated replay modules.

Existing observability consists of:

- a tkinter post-tick view of the full live tree and blackboard;
- per-command text logs derived from blackboard changes;
- generated Python plan modules; and
- optional DOT/PNG/SVG structural tree artifacts.

Important gaps:

- Raw LLM requests, raw responses, provider metadata, timing, token usage,
  retries, and parsed reasoning are discarded.
- Commands are used as logger identity, so repeated identical commands cannot
  be correlated reliably.
- There are no stable mission, task, planning-attempt, plan-revision, tree, or
  execution-step identifiers.
- Plan exhaustion and genuine execution aborts are both hidden behind
  `FailureIsSuccess`; a dashboard cannot infer a truthful terminal outcome from
  the root status.
- The generated plan tree and live dispatcher tree are related artifacts but
  are not currently linked by an explicit contract.
- Human log parsing would couple a server to names, strings, and blackboard keys
  that are likely to change.

## 2. Architectural decision

Keep the debugger separate from the standard runtime/operations dashboard.
GPSR's future execution model is expected to retain the large-step planner →
tree generation → tree execution shape while adding live tree edits and
background agents. A dedicated service avoids coupling the debugger to either
the current `py_trees` object graph or an operations UI release cadence.

The decision-side telemetry adapter communicates through a versioned JSON event
stream and a local NDJSON socket:

```text
GPSR stage boundaries
    -> typed in-process telemetry API
    -> JSONL audit sink + ROS String publisher
    -> /gpsr/debug/events (ROS String, optional)
    -> gpsr_debug_server RosBridge / Unix ingest socket
    -> SQLite WAL event store + pure projection
    -> REST snapshots + bounded WebSocket broker
    -> standalone GPSR Mission Debugger page
```

Reasons:

- The debugger owns loopback binding, session/origin checks, ROS spinning,
  controller leases, WebSockets, and static UI delivery without changing the
  operations dashboard.
- JSON with an explicit schema version is easier to evolve than a large ROS
  message definition while GPSR internals and agent processes are changing.
- The server consumes domain events, not GPSR imports or blackboard keys.
- JSONL gives a recoverable audit trail when the dashboard is not running.
- SQLite (stdlib, WAL mode) gives fast run history and projections without a
  new service dependency.

The debugger is started manually by a developer and is loopback-only (use SSH
port forwarding for remote viewing). It supports many viewers and one expiring
controller lease. Guarded interventions are declarative and audited; they can
pause/resume/cancel/retry/skip, edit state, or submit a typed tree proposal, but
never execute arbitrary Python, shell, or model-generated code. Autonomous
background-agent proposals may commit after schema/structural validation at a
tick barrier; human approval is not required for that path.

Mixed-process producers (threads, subprocesses, and ROS nodes) use the same
`gpsr_trace.TraceClient` UDS protocol with bounded queues and a disk spool.
Historical replay reconstructs the debugger projection exactly at any sequence
without re-running the robot or an LLM. Retention defaults to seven days and
10 GiB, but only unnamed, unpinned completed trajectories are disposable;
named trajectories are retained.

## 3. Stable telemetry contract

Every event uses this envelope:

```json
{
  "schema": "tinker.gpsr.telemetry",
  "schema_version": 1,
  "event_id": "uuid",
  "trajectory_id": "gpsr-20260803T102000Z-a1b2c3d4",
  "sequence": 42,
  "occurred_at": "2026-08-03T10:20:30.123456Z",
  "monotonic_ns": 1234567890,
  "task_id": ".../task-01",
  "event_type": "planner.response",
  "phase": "planning",
  "source_id": "gpsr-orchestrator:1234",
  "payload": {}
}
```

`task_id` is nullable for mission-level events. `sequence` is strictly
increasing per run. Consumers must preserve unknown event types and ignore
unknown fields. A future incompatible envelope becomes `schema_version: 2`;
payload additions remain version 1.

Required event types:

| Phase | Event | Required payload |
|---|---|---|
| Mission | `run.started` | mode, expected task count, configuration summary |
| Intake | `task.command_received` | slot, command text, source (`voice`, `injected`) |
| Planning | `planner.request` | attempt ID/index, plan revision, model, provider, messages, temperature, token cap, correction context |
| Planning | `planner.response` | attempt ID, raw content, raw reasoning if separate, finish reason, usage, latency, provider request ID |
| Planning | `planner.error` | attempt ID, error class/message, latency |
| Validation | `plan.validated` | raw plan, cleaned plan, dropped actions, accepted flag, rejection reason |
| Planning | `plan.committed` | plan ID/revision, steps, fallback/offline-mock flags |
| Tree | `tree.generated` | tree ID/revision, kind, node/edge topology, linked plan ID |
| Execution | `task.execution_started` | task slot, active plan/tree revision |
| Execution | `step.started` | stable step ID/index, action, params |
| Execution | `tree.node_states_changed` | tree ID, tick, changed node states/feedback |
| Execution | `step.finished` | step ID, outcome, duration, deepest failure details |
| Correction | `correction.started` | failed step, correction number, failure context |
| Correction | `plan.superseded` | old/new plan IDs and reason |
| Terminal | `task.finished` | `succeeded`, `failed`, `cancelled`, or `incomplete`; reason and summary |
| Terminal | `run.finished` | aggregate status and per-task outcomes |
| Runtime | `run.heartbeat` | current phase/task/step and last sequence |

LLM messages are captured before the provider call and the response before it is
parsed, so rejected and malformed attempts remain inspectable. Secrets and HTTP
headers are never included. Payloads have explicit size limits; future
multimodal inputs store image metadata and an artifact reference rather than
embedding image bytes in ROS JSON.

## 4. Identity and outcome prerequisites

Add these runtime concepts before building the UI:

- `run_id`: created once in `gpsr_orchestrator.main`.
- `task_id`: based on run ID and intake slot, never command text.
- `attempt_id`: unique for every provider call, including a retry without
  `seed`.
- `plan_id` and integer `plan_revision`: revision 1 is the initial plan;
  corrections create later revisions.
- `step_id`: `<plan_id>/step-<zero-padded-index>`.
- `tree_id`: unique per generated topology/revision.

Add explicit task outcome state. Plan exhaustion is success only when every
step in the active plan completed. Correction-limit exhaustion, max-step
exhaustion, shutdown, and uncaught execution failures must become distinct
terminal reasons. This can initially be observational (preserving robot
control flow) but it must no longer be inferred from the decorated root status.

The task/run outcome should be written to telemetry and blackboard state so
other robot behaviors can use it. Do not have the dashboard derive mission
truth from node colors.

## 5. Producer design in `tk25_decision`

Add a small `behavior_tree/GPSR/telemetry/` package:

- `events.py`: envelope builder, event names, validation, safe JSON conversion.
- `context.py`: run/task/attempt/plan/tree identities.
- `sink.py`: `NoOpSink`, `CompositeSink`, bounded asynchronous `JsonlSink`, and
  attachable `RosStringSink`.
- `tree.py`: serialize a `py_trees` topology with stable structural IDs and
  compute status/feedback diffs.
- `observer.py`: stage-level API used by GPSR and a post-tick execution observer.

All telemetry calls are non-blocking and fail-open. Sink errors emit one
rate-limited ROS/logger warning and can never fail planning or tick execution.
The JSONL writer uses a bounded queue and flushes terminal events on shutdown.

Instrumentation belongs only at stable boundaries:

- Command intake completion.
- Immediately before and after each `_call_llm`.
- After cleaning/validation and when a plan is committed.
- When a plan-shaped tree artifact is serialized.
- When `PopNextAction`, step completion, and correction change domain state.
- In one post-tick observer for live node-state diffs and heartbeat.
- At explicit task/run finalization.

Do not instrument every small-tree leaf manually. The generic tree observer
already sees leaf status and feedback. Small trees may optionally add semantic
metadata later.

Create a run artifact directory:

```text
gpsr_runs/debug/<run_id>/
  events.jsonl
  tasks/task-01/
    plan-r001.json
    plan-r001.py
    planned-tree-r001.json
    planned-tree-r001.svg        # optional compatibility artifact
  tasks/task-02/
  logs/
```

Keep the legacy flat plan/log filenames for one release if operators still use
them, but make the run manifest the source of truth.

### Tree representation

Publish two explicit tree kinds:

1. `planned`: the plan-shaped composition built from the accepted action list
   and small-tree factories. It answers “what behavior was generated?”
2. `executor`: the exact live orchestrator/dispatcher topology. It answers
   “what is actually ticking?”

Link both to the active plan and step. In the planned view, top-level step
status comes from `step.started`/`step.finished`. In the executor view, status
comes directly from post-tick snapshots. The UI must label these views rather
than imply they are the same object.

When a future GPSR implementation executes the generated tree directly, it may
emit a single tree with both roles; no server/API change is required.

## 6. Standalone debugger backend in `gpsr_debug_server`

Implemented modules:

- `store.py`: SQLite WAL append-only storage, immutable sequence/event-id
  checks, causal links, checkpoint-plus-delta replay, and trajectory naming.
- `projection.py`: pure reducer for agents, planning attempts, plan/tree
  revisions, node-state deltas, interventions, outcomes, and unknown events.
- `retention.py`: conservative seven-day/10-GiB policy that protects active,
  named, and pinned trajectories.
- `ros_bridge.py` and `main.py`: optional ROS String bridge plus mixed-process
  NDJSON Unix socket ingestion.
- `web.py`: authenticated REST/WebSocket API, loopback enforcement, and a
  many-viewer/one-controller lease.

The server accepts both the GPSR envelope (`event_type`, `occurred_at`) and the
stdlib `gpsr_trace` envelope (`event_type`, `timestamp`, `trace_id`), normalising
them at the storage boundary. Invalid records are rejected without stopping
the ROS callback or Unix collector.

SQLite tables:

- `gpsr_runs(run_id, started_at, finished_at, status, last_sequence, summary_json)`
- `gpsr_tasks(task_id, run_id, slot, command, status, summary_json)`
- `gpsr_events(run_id, sequence, event_id, type, occurred_at, task_id, payload_json)`

The event table is append-only with `(run_id, sequence)` and `event_id` unique.
Run/task rows are disposable projections and can be rebuilt from events.

REST API:

- `GET /api/v1/trajectories?limit=&cursor=`
- `GET /api/v1/trajectories/{trajectory_id}` (current or `?at=sequence`)
- `GET /api/v1/trajectories/{trajectory_id}/events?after=&limit=`
- `GET /api/v1/trajectories/{trajectory_id}/trees/{revision}`
- `PATCH /api/v1/trajectories/{trajectory_id}` (name/pin retention metadata)
- `POST/DELETE /api/v1/control/lease`
- `POST /api/v1/trajectories/{trajectory_id}/commands`

The `/api/v1/stream` WebSocket publishes `gpsr.event` records and heartbeats.
Clients load a REST snapshot first; bounded-broker gaps are repaired from the
immutable events endpoint. The server does not run an LLM or robot replay.

## 7. Web UI

The standalone debugger serves a dedicated **GPSR Trace** page; the standard
operations dashboard remains unchanged.

Layout:

- Header: connection state, run ID, live/historical badge, elapsed time,
  aggregate outcome.
- Task tabs: command text, slot, status, current phase, correction count.
- Stage rail: Intake → LLM Planning → Validation → Plan → Tree → Execution →
  Outcome, with retries/corrections shown as branches.
- Plan panel: ordered large-step cards with action, params, plan revision,
  validation result, and per-step duration/outcome.
- Tree panel: nested DOM/SVG hierarchy with status colors, collapse/expand,
  “focus active,” and a `Planned` / `Live executor` toggle.
- LLM inspector: request messages, raw response, reasoning, parsed JSON,
  model/provider, latency, tokens, finish reason, and rejection/error. Each
  attempt is selectable.
- Timeline: chronological events, failures, corrections, feedback, and artifact
  links.
- History drawer: recent missions, command search, status filter.

Render all prompt/response text with `textContent`; never inject model output as
HTML. Large prompts are collapsed by default and virtualized or lazily rendered.
Tree layout should use the hierarchy already present in event topology rather
than Graphviz on every browser update. Unchanged nodes are not re-rendered.

Split GPSR UI code into its own `gpsr.js`/`gpsr.css` module instead of enlarging
the existing monolithic `app.js`. The page consumes projection DTOs only and
contains no knowledge of blackboard keys or Python class names.

Status colors:

- `RUNNING`: amber
- `SUCCESS`: green
- `FAILURE`: red
- `INVALID`/not visited: gray
- `CANCELLED`: purple
- corrected/superseded plan: blue outline

## 8. Implementation status and next increments

### Phase 0 — Contract and truthful lifecycle (implemented)

1. Write the JSON schema and representative fixtures.
2. Add run/task/attempt/plan/tree/step identities.
3. Add explicit task and mission outcomes, including max-step and
   correction-limit failure.
4. Pin behavior with unit tests before changing logs or UI.

**Exit:** A full-mock three-command run has unique IDs and unambiguous terminal
outcomes, including repeated identical commands.

### Phase 1 — Decision-side telemetry (implemented for current GPSR)

1. Implement sinks and stage observer.
2. Capture every LLM attempt and validation result.
3. Serialize planned and executor tree topologies.
4. Emit changed node states after ticks and periodic heartbeat.
5. Produce run-scoped JSONL/artifacts.
6. Keep existing GUI/logger via the current combined post-tick handler.

**Exit:** Replaying `events.jsonl` reconstructs command, LLM attempts, committed
plans, trees, step results, corrections, and final outcome without parsing text
logs.

### Phase 2 — Debugger ingestion and API (implemented)

1. Extend `RosBridge` subscription.
2. Add SQLite store and pure projection reducer.
3. Add REST routes and WebSocket forwarding.
4. Add retention and malformed/out-of-order event handling.

**Exit:** API tests can ingest fixtures, restart the server, and return the same
projection.

### Phase 3 — GPSR Trace UI (implemented baseline)

1. Add live/historical run selector and stage rail.
2. Add plan and LLM inspectors.
3. Add planned/live tree views and status diffs.
4. Add timeline, corrections, outcome, and sequence-gap recovery.

**Exit:** One browser session can follow a full-mock mission live and reopen it
from history after both processes restart.

### Phase 4 — Hardening

1. Load/backpressure tests at 2 Hz ticks with a large dispatcher tree.
2. Truncated JSONL and SQLite recovery tests.
3. XSS and oversized model-output tests.
4. Real-robot smoke test through initial plan and one safe navigation action.
5. Document contract evolution and operator runbook.

## 9. Test plan

Decision-side unit tests:

- Event envelope validation and monotonic sequencing.
- Every retry records the exact request and raw response/error.
- Provider retry without `seed` retains the same `attempt_id` and records a
  sub-attempt.
- Cleaning/validator rejection and fallback/offline-mock flags are preserved.
- Duplicate command text still yields different task IDs and files.
- Sink exceptions and full queues do not change behavior-tree status.
- Stable tree IDs for one topology; new IDs for a new plan revision.
- Only changed node states are emitted.
- Plan exhaustion, max steps, correction limit, shutdown, and success yield
  distinct terminal outcomes.

Dashboard unit/API tests:

- Schema version acceptance/rejection.
- Idempotent event ingestion.
- Out-of-order and missing sequence handling.
- Projection rebuild from raw events.
- Active-run recovery after server restart.
- WebSocket notification plus REST gap recovery.
- Retention leaves active runs untouched.
- Model output is rendered as inert text.

End-to-end fixture:

1. Start `gpsr-debug-server` manually (and optionally an SSH tunnel).
2. Run GPSR in full mock with three injected commands, including two identical
   commands.
3. Assert all three tasks appear separately.
4. Assert request/response absence is explicitly labeled `offline mock`, not
   presented as missing telemetry.
5. Use a deterministic fake planner run to exercise rejected attempt,
   correction, replanning, and failure.
6. Reopen both runs from history.

## 10. Maintainability rules

- The web server knows the telemetry schema only; it never imports
  `behavior_tree.GPSR`.
- GPSR emits semantic stage events; the dashboard never scrapes console logs,
  generated Python, node names, or blackboard storage.
- Optional stages are capabilities. Missing `planner.response` or
  `tree.generated` produces an “unavailable/not emitted” panel, not a server
  failure.
- Unknown events are persisted for forward compatibility.
- All event/projection code is covered by fixtures shared as versioned JSON.
- UI DTOs are separate from raw event payloads.
- Tree topology and node-state updates are separate, preventing tick traffic
  from resending large trees.
- Telemetry remains fail-open and cannot block robot control.
- The current tkinter visualizer and human logs remain usable during rollout;
  remove them only after operators approve the web view.

## 11. Non-goals

- Executing arbitrary Python, shell, or model-generated code from the browser.
- Replacing ROS diagnostics or the operations/process dashboard.
- Streaming full camera feeds.
- Parsing old flat GPSR logs into perfect historical traces. A one-time
  best-effort importer may show command/plan/step text but must label inferred
  fields.

## 12. Acceptance criteria

- A live task appears in the browser within one tick period.
- Every online planning attempt exposes exact model input, raw response,
  validation result, latency, and usage when provided.
- Initial and corrected plans remain independently inspectable.
- The planned behavior tree is visible before execution.
- The exact live executor tree exposes node status/feedback with at most
  one-second display latency.
- Repeated identical commands are distinct tasks.
- The final status is truthful and never inferred from `FailureIsSuccess`.
- Restarting the dashboard does not lose completed history; running GPSR
  retains its JSONL audit if the dashboard was offline.
- Disabling or breaking telemetry does not change GPSR task behavior.
- A future GPSR that preserves intake → plan → tree → execute, adds background
  agents, or mutates trees on the fly needs only a new `gpsr_trace` producer or
  typed gateway adapter; its server routes, database, WebSocket protocol, and
  main UI remain unchanged.
