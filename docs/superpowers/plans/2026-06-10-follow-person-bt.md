# Follow-person BT — implementation plan

> REQUIRED SUB-SKILL: superpowers:subagent-driven-development. TDD. Phase = one commit.

**Spec:** `docs/superpowers/specs/2026-06-10-follow-person-bt-design.md`
**Repo:** `src/tk25_decision` (its own git repo). **Package:** `behavior_tree`.
**Build:** `cd /home/tinker/tk25_ws/src/tk25_decision && colcon build --packages-select behavior_tree`
**Test:** decision venv —
`/home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/python -m pytest src/behavior_tree/test/<file> -q`
(source `/opt/ros/humble/setup.bash` + the repo `install/setup.bash` first; the
mock config auto-loads).
**Patterns to model (read first):** `TemplateNodes/TrackPersonAction.py`
(FeedbackBuffer + node + blackboard writes + mock), `TemplateNodes/Audio.py`
`BtNode_Announce` (TextToSpeech client + mock), `TemplateNodes/BaseBehaviors.py`
`ServiceHandler.setup` (how a node gets the rclpy node + creates a client),
`HelpMeCarry/cli.py` + `runtime.run_tree`, `config.is_node_mocked`,
`HelpMeCarry/test/mock_track_server.py`.
**Invariants:** announcer never blocks the tick (async submit + poll); no
overlapping speech; latest-wins coalescing; all nodes importable + unit-testable
in mock mode with no ROS graph; no new flake8/pep257 on touched lines.

---

## Phase A — TrackPerson feedback exposes `reacquisition_state` (one commit)
**Files:** `behavior_tree/TemplateNodes/TrackPersonAction.py`; test `test/test_feedback_buffer_reacq.py` (new).

- [ ] **Failing test** `test/test_feedback_buffer_reacq.py` (Apache header — copy from any `test/test_*.py`): construct `FeedbackBuffer`; feed a `SimpleNamespace` feedback with `target_lost=False, target_track_id=7, is_transformation_successful=True, target_position=None, reacquisition_state=2`; assert `get_state()` returns the reacq value (extend the unpack); assert default before any update is `0`; assert `clear()` resets to `0`.
- [ ] **Run → FAIL** (buffer has no reacq field).
- [ ] **Implement:** in `FeedbackBuffer` add `self._reacquisition_state = 0` (init + `clear`); in `update()` set `self._reacquisition_state = int(getattr(feedback, "reacquisition_state", 0))`; append it to the `get_state()` tuple (and update its docstring + every caller that unpacks `get_state()` — grep within the file). In `BtNode_TrackPersonAction`: add a configurable blackboard key (default `track/reacquisition_state`), register it for WRITE in `setup()` alongside the others, and write it from the unpacked state on each `update()`.
- [ ] **Run → PASS** + `pytest test/test_feedback_buffer_reacq.py`.
- [ ] **Commit** (explicit paths): `feat(behavior_tree): TrackPerson feedback exposes reacquisition_state on the blackboard`

---

## Phase B — `CoalescingTTS` (no-overlap + latest-wins speaker) (one commit)
**Files:** `behavior_tree/FollowPerson/__init__.py` (new, empty), `behavior_tree/FollowPerson/coalescing_tts.py` (new); test `test/test_coalescing_tts.py` (new).

- [ ] **Failing test** `test/test_coalescing_tts.py`: build `CoalescingTTS(start, is_done)` with fakes — `start(text)` appends a handle dict `{"text": text, "done": False}` to a `started` list and returns it; `is_done(h)` returns `h["done"]`. Assert: (1) `submit("a")` → started==["a"], no pending; (2) `submit("b")` while "a" active → started still ["a"], pending=="b"; (3) `submit("c")` while "a" active → pending=="c" (b dropped); (4) mark "a" done, `poll()` → starts "c" (started==["a","c"]), pending cleared; (5) never two not-done handles at once. Apache header.
- [ ] **Run → FAIL** (module missing).
- [ ] **Implement** `coalescing_tts.py` exactly per spec §2 (`__init__(start, is_done)`, `_active`/`_pending`, `submit`, `poll`). Pure; no ROS imports.
- [ ] **Run → PASS**.
- [ ] **Commit:** `feat(behavior_tree): CoalescingTTS — non-overlapping latest-wins speaker`

---

## Phase C — `BtNode_ReacqAnnounce` + `BtNode_PublishFollowGoal` (one commit)
**Files:** `behavior_tree/FollowPerson/nodes.py` (new); tests `test/test_reacq_announce.py`, `test/test_publish_follow_goal.py` (new).

- [ ] **Failing tests.**
  - `test_reacq_announce.py`: construct `BtNode_ReacqAnnounce` with an injected `clock` (a list-backed fake returning a mutable `now`) and a fake `CoalescingTTS` (records `submit(text)` calls; `poll()` no-op) — inject via constructor or by setting `node._tts` after a mock-mode `setup()`. Use a real py_trees blackboard client to set `track/reacquisition_state`. Drive: TRACKING→PASSIVE (assert submit(passive_text) once) → advance clock <5 s, tick (no new submit) → advance ≥5 s, tick (submit passive_text again) → NEEDS_HELP (submit needs_help_text on transition) → TRACKING (no submit; resets) → PASSIVE (submit immediately). `update()` returns SUCCESS throughout.
  - `test_publish_follow_goal.py`: construct `BtNode_PublishFollowGoal`; inject a fake publisher (records `publish(msg)`); set blackboard `track/target_lost=False` + `track/person_position=<PointStamped>` → tick → published once, SUCCESS; set `track/target_lost=True` → tick → no publish, SUCCESS.
  - Both Apache-headered; both must run in mock mode without a ROS graph (construct the node, call a minimal `setup` path or bypass it by injecting the client/publisher directly — prefer injection so no rclpy node is needed).
- [ ] **Run → FAIL.**
- [ ] **Implement** `nodes.py` per spec §3 + §4. `BtNode_ReacqAnnounce`: `py_trees.behaviour.Behaviour`; constructor params `(name, passive_text=..., needs_help_text=..., throttle_s=5.0, bb_key="track/reacquisition_state", clock=time.monotonic)`; `setup(**kwargs)` grabs the rclpy node (model on `ServiceHandler.setup`) + creates a `TextToSpeech` client + builds `self._tts = CoalescingTTS(...)`; in mock mode (`is_node_mocked`) `start` logs + returns an already-done sentinel. `update()` per spec §3 steps 1–6. `BtNode_PublishFollowGoal`: constructor `(name, topic="/follow_target", pos_key="track/person_position", lost_key="track/target_lost")`; `setup` creates the `PointStamped` publisher; `update()` per spec §4. Both register their blackboard keys for READ. Keep the node-acquisition + client/publisher creation injectable enough that the tests can substitute fakes without a live node.
- [ ] **Run → PASS** both.
- [ ] **Commit:** `feat(behavior_tree): reacq-announce + publish-follow-goal BT nodes`

---

## Phase D — dummy nav node + tree + cli + launch + setup wiring (one commit)
**Files:** `behavior_tree/dummy_nav_node.py`, `behavior_tree/FollowPerson/follow_person.py`, `behavior_tree/FollowPerson/cli.py`, `behavior_tree/launch/follow_process.launch.py` (all new); modify `setup.py`. Test `test/test_follow_tree_build.py` (new).

- [ ] **Failing test** `test/test_follow_tree_build.py`: import `create_follow_person_tree` and assert it returns a `py_trees.composites.Parallel` whose children are the TrackPerson action node and a `Sequence` of [PublishFollowGoal, ReacqAnnounce] (assert by type/name; do NOT call `setup()` — no ROS graph). Confirms assembly wiring.
- [ ] **Run → FAIL.**
- [ ] **Implement:**
  - `dummy_nav_node.py` per spec §5 (rclpy node, subscribes `follow_target_topic` param default `/follow_target` `PointStamped`, logs throttled ~1 Hz; `main()` with `rclpy.init/spin/shutdown`).
  - `FollowPerson/follow_person.py` `create_follow_person_tree()` per spec §6 (Parallel `SuccessOnAll(synchronise=False)`, child A `BtNode_TrackPersonAction`, child B `Sequence(memory=False, [BtNode_PublishFollowGoal, BtNode_ReacqAnnounce])`). Reuse the existing `BtNode_TrackPersonAction` constructor args (read its signature).
  - `FollowPerson/cli.py` `main()` → `run_tree(create_follow_person_tree, period_ms=200.0, title="Follow Person")`.
  - `launch/follow_process.launch.py`: `launch.LaunchDescription` with two `launch_ros.actions.Node`s — `package="behavior_tree", executable="dummy-nav"` and `executable="follow-person"`; a header comment that the real `person_track_server` + `TextToSpeech` service must already be running.
  - `setup.py`: add to `console_scripts` — `"dummy-nav = behavior_tree.dummy_nav_node:main"`, `"follow-person = behavior_tree.FollowPerson.cli:main"`; add a `data_files` entry installing the launch glob: `("share/behavior_tree/launch", glob("launch/*.launch.py"))` (add `from glob import glob` import).
- [ ] **Run → PASS** the build test; `colcon build --packages-select behavior_tree` succeeds; `ros2 pkg executables behavior_tree | grep -E "dummy-nav|follow-person"` shows both.
- [ ] **Commit:** `feat(behavior_tree): follow-person tree + dummy nav node + launch + entry points`

---

## Phase E — README + changelog + final verification (one commit)
- [ ] Update `behavior_tree/README.md`: add a "Follow person" section (what it does, the reacq-driven announcements, the dummy nav stub) + an append-only Changelog entry. Document the run sequence: real tracker (`ros2 run vision_track person_track_server`) + audio TTS service running, then `ros2 launch behavior_tree follow_process.launch.py` (or `ros2 run behavior_tree dummy-nav` + `ros2 run behavior_tree follow-person` in two terminals).
- [ ] Full verify: `colcon build --packages-select behavior_tree`; `pytest src/behavior_tree/test/test_coalescing_tts.py src/behavior_tree/test/test_reacq_announce.py src/behavior_tree/test/test_publish_follow_goal.py src/behavior_tree/test/test_feedback_buffer_reacq.py src/behavior_tree/test/test_follow_tree_build.py -q` all green; `python -c "import behavior_tree.FollowPerson.follow_person, behavior_tree.dummy_nav_node"` OK; no new flake8/pep257 on touched files.
- [ ] **Commit:** `docs(behavior_tree): README + changelog for follow-person BT`
