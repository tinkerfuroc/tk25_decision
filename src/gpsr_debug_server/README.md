# GPSR debug server

`gpsr-debug-server` is a developer-only, loopback-bound mission debugger. It
stores causal GPSR events and exposes a separate UI for adaptive behavior-tree
revisions, background-agent activity, historical replay, and guarded controls.

Run it manually on the robot host:

```bash
ros2 run gpsr_debug_server gpsr-debug-server --port 8766
```

The ROS package keeps FastAPI/Uvicorn optional for offline event-store use.
The command above therefore expects the package entry point to use a Python
environment containing the web extra. Install it in the decision environment:

```bash
python3 -m pip install -e 'src/gpsr_debug_server[web]'
```

For this workspace, `.venv_basic` already contains FastAPI/Uvicorn and can run
the installed ROS package without changing system Python:

```bash
source /opt/ros/humble/setup.bash
source /home/tinker/tk25_ws/install/local_setup.bash
ROS2_PTH_WARNED=1 \
  /home/tinker/tk25_ws/src/tk25_basic/.venv_basic/bin/python \
  -m gpsr_debug_server.main --port 8766
```

Open `http://127.0.0.1:8766/`. The browser obtains the ephemeral loopback
session automatically. State defaults to
`~/.local/state/gpsr_debug_server`; set `GPSR_DEBUG_STATE_DIR` or pass
`--state-dir` to place SQLite, the session token, ROS logs, and
`ingest.sock` elsewhere.

The debugger is organized around seven stable investigation surfaces:

- Overview: mission phases, projection health, filtered causal timeline, and
  event envelope/payload/ancestor/descendant inspection.
- LLM Supervisor: post-effect BT checkpoints with front/wrist camera evidence,
  rendered map and arm state, relevant tree/blackboard context, verifier
  rationale, bounded local recovery, and global replan/stop decisions.
- Planning & LLM: complete model attempts and committed large-step revisions.
- Behavior tree: planned/executor revisions rendered as left-to-right semantic
  Cytoscape graphs, with per-tick activation evidence, node search, stable ids,
  blackboard access, and explicit sequence/selector/parallel/decorator rules.
- Agents & adaptation: concurrent agent lanes plus proposal, vote, and
  resulting-tree relationships.
- State: checkpoint/delta inspection and sequence-based historical replay.
- Controls: lease-gated, revision-checked, audited interventions that are
  disabled while inspecting history.

The web UI deliberately has no build step. `webui/ui_model.js` contains the
pure normalization, filtering, URL-state, and tree-visibility logic and is
covered by Node's built-in test runner. `webui/app.js` is the DOM/API adapter;
`webui/style.css` owns the compact neutral-dark design system. Cytoscape is
pinned and vendored under `webui/vendor/` so the debugger remains usable on
robots without internet access. New event producers should extend the pure
projection and UI model rather than teach the HTTP server about GPSR Python
classes.

### Behavior-tree debugging contract

Current GPSR producers use `tree_document_version: 2`. Each topology node
keeps its concrete Python `type`, plus a stable semantic shape:

```json
{
  "node_class": "composite",
  "semantics": {
    "kind": "parallel",
    "parallel_policy": "all",
    "synchronise": true
  },
  "blackboard_access": {"read": [], "write": [], "exclusive": []},
  "action_context": {"action": "goto", "step_index": 1, "boundary": true}
}
```

The serializer also describes memory sequences/selectors, selected-child
parallel policies, retry/repeat limits, timeout/condition/one-shot/status-map
decorators, and inherited action boundaries. The browser accepts legacy
`type`-only documents too, so adding semantic fields does not require a
coordinated dashboard deployment.

The executor attaches a real `py_trees.visitors.SnapshotVisitor` and emits one
`tree.tick_observed` event per tick. Projection retains the selected and
preceding ticks, visit order, node statuses, retry/repeat counter deltas, and
active action context. The graph derives these evidence labels without
guessing unrecorded work:

- **Ticked**: visited on the selected tick.
- **Resumed**: visited on both selected and preceding ticks.
- **Skipped**: a recorded sequence, selector, memory, or synchronized-parallel
  rule explains why the child was not visited.
- **Blocked**: an ancestor or earlier running prerequisite was not eligible.
- **Not recorded**: the trace lacks enough evidence to make a stronger claim.

`Explain` is the default semantic skeleton and groups hidden branches.
`Runtime` follows all nodes visited on the selected tick and their immediate
context. `Full` exposes the complete revision. Arrow labels include child order
and the parent gate (for example, “after previous succeeds” or “if earlier
branches fail”). Selecting a node shows its activation reason, control rule,
configuration, runtime counters, and blackboard reads/writes. Previous/next
tick controls reuse the debugger's normal historical projection, so every
panel remains causally consistent at the chosen sequence.

Use an SSH tunnel for a browser on another machine:

```bash
ssh -L 8766:127.0.0.1:8766 robot-host
```

### Hardware-free supervisor replay

Populate a disposable state directory with the ten committed camera scenarios,
rendered navigation map/scene-only arm poses, and the optional 57-call live
Luna report:

```bash
PYTHONPATH=src/gpsr_debug_server:src/behavior_tree \
  python3 src/gpsr_debug_server/tools/seed_supervisor_demo.py \
  --state-dir /tmp/gpsr-supervisor-dashboard \
  --live-results /path/to/live-results-v7.json --replace

PYTHONPATH=src/gpsr_debug_server \
  python3 -m gpsr_debug_server.main \
  --state-dir /tmp/gpsr-supervisor-dashboard \
  --port 8766 --no-ros --no-ingest
```

Then open
`http://127.0.0.1:8766/?trajectory=gpsr-vlm-ten-case-validation&view=supervisor`.
`--no-ingest` is intended for immutable replays; omit it when live trace
producers need the Unix ingest socket. Checkpoint images are served only from
the debugger-owned `artifacts/` directory and require the ephemeral browser
session token.

The server has no dependency on the operations dashboard and never executes
arbitrary model output, Python, or shell commands. Mutations are submitted to
the GPSR-side typed debug gateway with a trajectory/revision precondition.

The canonical GPSR orchestrator publishes JSON envelopes on
`/gpsr/debug/events`. Background agents and separate planning processes can
use the stdlib-only `gpsr_trace.TraceClient` against
`<state-dir>/ingest.sock`; failed delivery is bounded and spooled for automatic
replay after the debugger restarts.

## Durable backend API

`gpsr_debug_server.store.DebugStore` is the ROS- and FastAPI-free persistence
boundary used by the web adapter. It uses the standard-library `sqlite3`
driver in WAL mode and persists whole JSON event envelopes. Its stable methods
are:

```python
from gpsr_debug_server.store import DebugStore

store = DebugStore("/var/lib/gpsr-debug/events.sqlite3")
accepted = store.append_event(event)  # False only for an exact redelivery
recent = store.list_trajectories(limit=50, cursor=None)
snapshot = store.trajectory_snapshot("trajectory-42", at_sequence=None)
tail = store.events("trajectory-42", after=0, limit=1000)
tree = store.tree_document("trajectory-42", "planned-r3")
store.set_trajectory_name("trajectory-42", "operator replay")
store.delete_trajectory("trajectory-42")
report = store.retention()
```

Every event needs `trajectory_id`, a non-negative `sequence`, `event_id`,
`type`, and usually a JSON-object `payload`. `(trajectory_id, sequence)` and
global `event_id` are immutable keys. Exact redelivery is
idempotent; reusing either key with changed content raises
`EventConflictError`. Event ids can reference a cause with `causation_id`,
`parent_event_id`, or their plural forms; `causal_events()` exposes the linked
ancestors/descendants when a client needs a focused explanation.

Snapshots replay the newest local checkpoint plus ordered event deltas. The
pure reducer in `projection.py` handles agents, planning attempts and plan
revisions, tree topology and node-state deltas, outcomes, and interventions.
Unknown event types remain in the snapshot's `unknown_events` audit list and
are never dropped from SQLite.

Retention is intentionally conservative: it removes only trajectories that
are all of unnamed, unpinned, and completed. It first removes those older than
seven days, then deletes the oldest remaining eligible trajectories until their
stored event JSON is at most 10 GB. Active, named, and pinned trajectories are
always preserved. The return value is a JSON-safe deletion audit report.
