# Follow-person behaviour tree (reacq-aware announcements + dummy nav) — design

**Date:** 2026-06-10
**Package:** `behavior_tree` (`src/tk25_decision`)
**Status:** approved (user, 2026-06-10)

A py_trees behaviour tree that follows a person via the finalized
`/track_person` action and reacts to the tracker's reacquisition state with voice
announcements: while PASSIVE-reacquiring it asks the person to slow down, and once
the tracker escalates to NEEDS_HELP it asks them to raise their hand. Navigation
is a **dummy topic stub** — the BT publishes the follow target; a small standalone
node subscribes and logs.

## Decisions (operator, 2026-06-10)
- **Dummy nav = topic stub.** The BT publishes the follow target to a topic; a
  standalone `dummy_nav` node subscribes and logs. No real navigation.
- **Announce cadence = on transition + ~5 s throttled repeat.** Announce once when
  entering PASSIVE / NEEDS_HELP, then re-announce every ~5 s while still in that
  state; reset on return to TRACKING.
- **Announcer is non-overlapping + latest-wins (coalescing).** A new announcement
  does NOT start until the current one finishes; if newer announcements are queued
  while one is speaking, only the NEWEST is spoken next (older queued ones are
  dropped). Implemented as a small single-slot coalescing speaker.
- **Real tracker only.** The BT wires to the real `/track_person` action + the real
  `TextToSpeech` service (no mock track server). Nodes still honour the package's
  mock-mode config for unit testing without hardware.
- **Launch dir.** Create `behavior_tree/launch/` (none exists today) with a
  `follow_process.launch.py` that starts the dummy nav node + the BT runner; wire
  `setup.py` `data_files` so it installs.

## Architecture / data flow
```
person_track_server ──/track_person feedback──▶ BtNode_TrackPersonAction
 (real)                (reacquisition_state,        (writes blackboard:
                        target_position,             track/person_position,
                        target_lost)                 track/target_lost,
                                                     track/reacquisition_state)
                                                          │ each tick
            ┌─────────────────────────────────────────────┘
            ▼
  Sequence(BtNode_PublishFollowGoal, BtNode_ReacqAnnounce)
            │                                  │
   /follow_target (PointStamped)         TextToSpeech srv (async, coalesced)
            ▼                                  ▼
       dummy_nav (logs)                    robot speaker
```

Tree root = `py_trees.composites.Parallel` (policy `SuccessOnAll`,
`synchronise=False`):
- **child A** `BtNode_TrackPersonAction` — keeps the `/track_person` goal alive,
  returns RUNNING continuously, writes the blackboard. If the action terminates
  (permanent loss / abort) it returns FAILURE → the Parallel returns FAILURE →
  the follow process ends.
- **child B** `Sequence(memory=False)`: `BtNode_PublishFollowGoal` →
  `BtNode_ReacqAnnounce`, both return SUCCESS each tick.

Children tick in order each cycle, so A refreshes the blackboard before B reads
it. Runner: `behavior_tree.runtime.run_tree(create_follow_person_tree,
period_ms=200, title="Follow Person")`.

## Components

### 1. Extend `FeedbackBuffer` + `BtNode_TrackPersonAction` (`TemplateNodes/TrackPersonAction.py`)
`FeedbackBuffer` currently captures `target_lost`, `target_track_id`,
`is_transformation_successful`, `target_position` — NOT `reacquisition_state`. Add
`_reacquisition_state` (default `0` = TRACKING), set it in `update()` from
`feedback.reacquisition_state`, return it from `get_state()`, clear it in
`clear()`. In `BtNode_TrackPersonAction`, write a new blackboard key
`track/reacquisition_state` (configurable, default that name) alongside the
existing writes. No other behavioural change.

### 2. `CoalescingTTS` (`FollowPerson/coalescing_tts.py`) — the no-overlap + latest-wins speaker
Pure, hardware-free, unit-testable. Constructed with two injected callables so it
is testable with fakes:
- `start(text) -> handle` — begins speaking `text`, returns an opaque handle
  (in production: `client.call_async(TextToSpeech.Request(text=text))` → a Future;
  the handle is the Future).
- `is_done(handle) -> bool` — True when that utterance finished (Future `.done()`).

State: `_active` (handle currently speaking, or None), `_pending` (latest queued
text, or None).
- `submit(text)`: if `_active is None` → `_active = start(text)`; else
  `_pending = text` (OVERWRITE — drops any older pending → latest-wins).
- `poll()`: if `_active is not None and is_done(_active)` → `_active = None`; then
  if `_active is None and _pending is not None` → `_active = start(_pending)`,
  `_pending = None`. Call every tick to advance the queue.

Invariant: at most one utterance active at a time (no overlap); at most one
pending, always the newest (stale dropped).

### 3. `BtNode_ReacqAnnounce` (`FollowPerson/nodes.py`)
A `py_trees.behaviour.Behaviour` (NOT a blocking ServiceHandler) so speech never
stalls the follow loop. In `setup()` it obtains the shared rclpy node (same way
ActionBase/ServiceHandler do) and creates a `TextToSpeech` service client, then a
`CoalescingTTS(start=lambda t: client.call_async(req(t)), is_done=lambda f: f.done())`.
Params (constructor): `passive_text` (default *"Please slow down so I can keep
up."*), `needs_help_text` (default *"I've lost you. Please raise your hand."*),
`throttle_s` (default `5.0`), blackboard key (default `track/reacquisition_state`),
clock (default `time.monotonic`, injectable for tests).

`update()` each tick:
1. `self._tts.poll()` — advance the coalescing queue.
2. Read `reacquisition_state` from the blackboard (default 0 if unset).
3. Determine the desired text: PASSIVE(1) → `passive_text`; NEEDS_HELP(2) →
   `needs_help_text`; TRACKING(0) → none.
4. Fire when **(state changed since last announce)** OR **(state still announced
   AND now − last_announce_time ≥ throttle_s)**: `self._tts.submit(text)` and
   record `(state, now)`.
5. On TRACKING reset the last-announced state so re-entering a help state
   re-announces immediately.
6. Always return `Status.SUCCESS` (non-blocking, never gates the tree).

Mock mode (`is_node_mocked`): `start` logs the phrase via the node logger and
returns an immediately-done sentinel handle (so unit tests + dry runs need no TTS
server).

### 4. `BtNode_PublishFollowGoal` (`FollowPerson/nodes.py`)
A `py_trees.behaviour.Behaviour`. In `setup()` create a `PointStamped` publisher
on the follow-target topic (default `/follow_target`). `update()`: read
`track/target_lost` and `track/person_position` from the blackboard; if not lost
and a position exists, publish it (the BT "commanding" navigation) and return
SUCCESS; if lost / no position, publish nothing and return SUCCESS (does not gate
the tree). Topic + blackboard keys are constructor params.

### 5. `dummy_nav_node` (`behavior_tree/dummy_nav_node.py`) — standalone node
A plain `rclpy` node (own process, `main()`): subscribes `/follow_target`
(`PointStamped`), logs the target throttled (~1 Hz) — e.g. `"[dummy_nav] would
navigate toward (x, y, z) in frame '<frame>'"`. Pure stub; no motion, no action.
Topic is a ROS param (`follow_target_topic`, default `/follow_target`).

### 6. Tree + runner (`FollowPerson/follow_person.py`, `FollowPerson/cli.py`)
`create_follow_person_tree()` builds the Parallel root described above and returns
it (signature compatible with `run_tree`). `cli.py:main()` calls
`run_tree(create_follow_person_tree, period_ms=200.0, title="Follow Person")`.

### 7. Launch dir (`behavior_tree/launch/follow_process.launch.py`) + `setup.py`
New `launch/` dir. `follow_process.launch.py` starts: (a) `dummy_nav` node,
(b) the `follow-person` BT runner. Documents that the real `person_track_server`
and `TextToSpeech` service must already be running (real-tracker-only). `setup.py`:
add console-script entry points `dummy-nav = behavior_tree.dummy_nav_node:main`
and `follow-person = behavior_tree.FollowPerson.cli:main`; add a `data_files`
glob installing `launch/*.launch.py` to `share/behavior_tree/launch`.

## Blackboard keys
| Key | Type | Writer | Reader |
|---|---|---|---|
| `track/person_position` | PointStamped | TrackPersonAction (existing) | PublishFollowGoal |
| `track/target_lost` | bool | TrackPersonAction (existing) | PublishFollowGoal |
| `track/reacquisition_state` | int (0/1/2) | TrackPersonAction (NEW) | ReacqAnnounce |

## Testing (mock mode, no hardware)
- `test_coalescing_tts.py`: with fake `start`/`is_done` + manual completion —
  idle submit speaks immediately; submit-while-active queues; a 2nd
  submit-while-active OVERWRITES (latest-wins, old dropped); `poll()` after
  completion speaks the pending; never two active at once.
- `test_reacq_announce.py`: injected clock + a fake CoalescingTTS (records
  `submit` calls). Drive blackboard states TRACKING→PASSIVE→(advance <5 s: no new
  submit)→(advance ≥5 s: re-submit passive)→NEEDS_HELP (submit needs-help on
  transition)→TRACKING (reset) →PASSIVE (re-announces immediately). Assert the
  exact phrases + cadence.
- `test_publish_follow_goal.py`: blackboard not-lost + position → publishes (capture
  via a fake publisher); lost → no publish; both return SUCCESS.
- `test_feedback_buffer_reacq.py`: `FeedbackBuffer.update(fb with
  reacquisition_state=2)` → `get_state()` returns 2; `clear()` resets to 0.
- All run under the decision venv via `pytest src/behavior_tree/test/` with no ROS
  graph (mock mode / pure logic). No new flake8/pep257 on touched lines.

## Files
- Modify: `behavior_tree/TemplateNodes/TrackPersonAction.py`, `setup.py`,
  `README.md` (changelog).
- Create: `behavior_tree/FollowPerson/{__init__,coalescing_tts,nodes,follow_person,cli}.py`,
  `behavior_tree/dummy_nav_node.py`, `behavior_tree/launch/follow_process.launch.py`,
  `test/test_{coalescing_tts,reacq_announce,publish_follow_goal,feedback_buffer_reacq}.py`.

## Invariants / risks
- Announcer never blocks the tick loop (async submit + poll); no overlapping
  speech; latest-wins coalescing.
- The follow loop runs until the `/track_person` action terminates (permanent
  loss) → Parallel FAILURE → clean end.
- Dummy nav is intentionally a no-op stub; swapping in a real follow/nav node
  later means replacing the `dummy_nav` subscriber (or repointing
  `BtNode_PublishFollowGoal` at a real nav action) — the BT structure is unchanged.
- Mock mode keeps all nodes importable + unit-testable with no ROS graph.
