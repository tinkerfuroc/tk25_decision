# Inspection Behavior Tree — Design Spec

- **Date:** 2026-07-01
- **Status:** DRAFT v1 (awaiting user review)
- **Repo touched:** `tk25_decision` (behavior_tree), branch `dev`
- **Author:** Claude (Opus 4.8), with cindy
- **Related:** `behavior_tree/Inspection/inspection.py`, `Inspection/customNodes.py`, `Inspection/constants.json`, `behavior_tree/runtime.py`; RoboCup@Home Inspection task

---

## 1. Goal

Rework the `Inspection` behavior tree so a single `ros2 run behavior_tree inspection` run performs exactly this competition flow:

1. **Wait for the door to open**, then **announce "door open"**.
2. **Navigate to InspectionPoint** (a `map`-frame `PoseStamped`).
3. **Wait for the operator to press Enter** on the keyboard.
4. **Navigate to ExitPoint** (a `map`-frame `PoseStamped`).

The existing tree already implements ~90% of this; the change is a **trim + one insertion + one bugfix**, not a rewrite. Per the user's directives: keep the arm "navigating" pre-pose, replace the long self-introduction with a single line `"Dear referees, I am Tinker."`, and insert the currently-missing `"door open"` announcement.

### 1.1 In scope
- Restructure `createInspection()` in `Inspection/inspection.py` to the 9-node sequence in §3.
- Insert one `BtNode_Announce(message="door open")` immediately after door detection.
- Collapse the two self-intro announcements into one: `"Dear referees, I am Tinker."`.
- **Fix** `BtNode_PressEnterToSucceed` so it is a robust, true "press-Enter" detector (§4).
- Remove now-dead flow: the commented-out `createQandA()` helper and its dead imports/keys, left tidy.

### 1.2 Out of scope (explicitly not done — user directives / YAGNI)
- The Q&A / listen-and-answer subtree (`createQandA`, `BtNode_ListenAction`) — dropped, not stubbed.
- De-hardcoding the absolute `constants.json` path (`inspection.py:24`) — flagged as an optional portability cleanup, **not** included this pass.
- Any change to `cli.py`, `setup.py`, `runtime.py`, or `constants.json` **values**. Poses stay as they are.
- Named-location lookup service — none exists in-repo; not introduced.

### 1.3 Success criteria
- `createInspection()` constructs, `setup()`s, and ticks the 9-node sequence in order.
- Under `BT_MOCK_MODE=true`: door detection mock-opens, announcements mock-print, both navigations mock-succeed, and the Enter-wait blocks until a real Enter is pressed (interactive checkpoint is intentionally **not** mocked — see §5).
- After the fix, `BtNode_PressEnterToSucceed` returns `SUCCESS` **only** on a deliberate Enter, is not tripped by stray keystrokes buffered during the door-wait / navigation, and consumes the input it acts on.
- Announcement order and wording match §3 exactly.
- No other task (GPSR, Receptionist, etc.) is touched; the `inspection` entry point is unchanged.

---

## 2. Background — current state (source-verified)

### 2.1 The existing tree (`createInspection()`, `inspection.py:91`)
A 9-step `Sequence(memory=True)`:
1. `createConstantWriter()` — a `Parallel(SuccessOnAll)` writing `inspection_pose`, `exit_pose`, `arm_navigating` to the blackboard from `constants.json` (path hardcoded absolute at `:24`).
2. `Retry(3): BtNode_MoveArmSingle(..., arm_navigating)` — tuck arm to a navigating pose (`joint_move_action`).
3. `Retry(999): BtNode_DoorDetection(bb_door_state_key=KEY_DOOR_STATUS)` — poll until door open.
4. `createToIspection()` → `Retry(10): BtNode_GotoAction(KEY_INSPECTION_POSE)`.
5–6. Two `BtNode_Announce`: `"I am Tinker, I am ready for inspection…"` + the long `tinker_description`.
7. `BtNode_PressEnterToSucceed()`.
8. `BtNode_Announce("Heading to the exit.")`.
9. `createToExit()` → `Retry(10): BtNode_GotoAction(KEY_EXIT_POSE)`.

`createQandA()` (a listen→announce loop) is defined but already commented out at `:103`.

### 2.2 The four building blocks (all exist, all reusable)

| Step | Node | File | Contract |
|---|---|---|---|
| Wait for door open | `BtNode_DoorDetection(name, bb_door_state_key, service_name="door_detection_srv")` | `TemplateNodes/Vision.py:1271` | `ServiceHandler`. Calls `/door_detection_srv` (`tinker_vision_msgs_26/srv/DoorDetection`, `camera="orbbec"`). **Returns SUCCESS only when `is_open==1`; FAILURE when closed or on error.** Wrapping in `Retry(999)` makes the decorator stay RUNNING until a real open is seen — that *is* the "wait for open" loop. Orbbec-only. Mock: sets `is_open=1` then keypress. |
| Announce | `BtNode_Announce(name, bb_source, service_name="announce", message=None)` | `TemplateNodes/Audio.py:204` | `ServiceHandler`. Pass a literal via `message=`; `bb_source=None`. Calls `announce` (`tinker_audio_msgs/srv/TextToSpeech`), blocks the tick until TTS returns. Mock: prints + keypress. |
| Navigate | `BtNode_GotoAction(name, key, action_name="navigate_to_pose")` | `TemplateNodes/Navigation.py:82` | `ActionHandler`. Nav2 `NavigateToPose`; reads a `PoseStamped` from blackboard `key`. SUCCESS on `STATUS_SUCCEEDED`, FAILURE on `STATUS_ABORTED`. Mock: instant success. |
| Wait for Enter | `BtNode_PressEnterToSucceed(name)` | `Inspection/customNodes.py:15` | Plain `Behaviour` (no mock hook). Ticks RUNNING until Enter. **Buggy — see §4.** |

### 2.3 The keypress bug (load-bearing — this is why a code fix is needed)
`is_enter_pressed()` (`customNodes.py:13`):
```python
def is_enter_pressed():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])
```
Two defects:
1. It reports True whenever stdin has **any** pending data — not specifically Enter.
2. It **never reads/consumes** the pending byte(s).

In a normal cooked (line-buffered) terminal it *accidentally* satisfies the spec — stdin only becomes select-readable after a full line + Enter — so today it "works." But it is fragile: if the operator taps any key during the (potentially long) door-wait or a navigation leg, that input buffers and, the moment this node initialises, `select` reports ready and it returns SUCCESS **immediately**, skipping the intended human checkpoint. For a scored inspection run that is an unacceptable failure mode.

### 2.4 Runtime / lifecycle (unchanged)
`cli.py` → `run_tree(createInspection, period_ms=500.0, title="Inspection")` (`runtime.py:7`): `rclpy.init` → build root → `py_trees_ros.trees.BehaviourTree` → `tree.setup(timeout=15)` → `tree.tick_tock(period_ms, post_tick_handler)` → `rclpy.spin`. Ticks fire from a ROS timer on the main thread; SIGTERM is translated to `KeyboardInterrupt` for clean shutdown. Task code never touches rclpy directly.

---

## 3. Target design — new `createInspection()`

`Sequence(name="Inspection Root", memory=True)` with children in order:

```
1. createConstantWriter()                                            [reuse]
     writes inspection_pose, exit_pose, arm_navigating to blackboard
2. Retry(3):  BtNode_MoveArmSingle(arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False)
                                                                     [KEEP per user]
3. Retry(999): BtNode_DoorDetection(bb_door_state_key=KEY_DOOR_STATUS)
                                            ── wait until door open   [reuse]
4. BtNode_Announce(name="Announce door open",
                   bb_source=None, message="door open")
                                            ── NEW: the missing announcement
5. createToIspection() → Retry(10): BtNode_GotoAction(KEY_INSPECTION_POSE)
                                            ── navigate to InspectionPoint  [reuse]
6. BtNode_Announce(name="introduce self",
                   bb_source=None, message="Dear referees, I am Tinker.")
                                    ── REPLACES the 2 old self-intro announces
7. BtNode_PressEnterToSucceed()             ── wait for Enter (fixed, §4)  [reuse+fix]
8. BtNode_Announce(name="announce leaving",
                   bb_source=None, message="Heading to the exit.")   [keep]
9. createToExit() → Retry(10): BtNode_GotoAction(KEY_EXIT_POSE)
                                            ── navigate to ExitPoint   [reuse]
```

**Deltas vs. current tree:**
- **Insert** node 4 (`"door open"` announcement) directly after door detection.
- **Replace** current nodes 5–6 (two announces incl. `tinker_description`) with the single node 6 above.
- **Remove** the commented-out `createQandA()` and its now-dead symbols (`BtNode_ListenAction` import, `KEY_LISTEN_RESULT`, `tinker_description` variable read). The `tinker_description` **key stays in `constants.json`** (dormant, zero churn).
- Everything else (constant writer, arm pre-pose, both navigations, retry counts, `"Heading to the exit."`) is **unchanged**.

`createConstantWriter`, `createToIspection`, `createToExit`, and the `KEY_*` constants for poses/arm/door are reused verbatim.

---

## 4. The keypress fix (`BtNode_PressEnterToSucceed`)

Make it a robust, true Enter detector while preserving the exact "press Enter to continue" UX. It remains a plain `Behaviour` (this is a deliberate human-in-the-loop checkpoint, intentionally not mocked — §5). Cooked-terminal assumption is explicit: the tree runs from an interactive `ros2 run` shell.

**`initialise()`** — print the prompt **and drain any stale buffered input** so a keystroke pressed earlier (during door-wait/nav) cannot instantly satisfy the wait:
```python
def initialise(self):
    # Drain stale/buffered stdin lines so a stray earlier keystroke can't
    # instantly satisfy the Enter-wait. Guard EOF (piped/closed stdin) so the
    # loop can't spin — a closed fd reads select-ready but readline() → "".
    while select.select([sys.stdin], [], [], 0)[0]:
        if sys.stdin.readline() == "":   # EOF
            break
    self.logger.info(f"'{self.name}': Press ENTER to continue...")
```

**`update()`** — succeed only when a full line (Enter) is available, and **consume** it:
```python
def update(self):
    if select.select([sys.stdin], [], [], 0)[0]:
        sys.stdin.readline()          # consumes the line incl. the trailing '\n'
        self.feedback_message = "Enter detected"
        return py_trees.common.Status.SUCCESS
    self.feedback_message = "Waiting for user to press Enter..."
    return py_trees.common.Status.RUNNING
```
In cooked mode `select` reports stdin readable only after a complete Enter-terminated line, so `readline()` does not block and returns exactly that line. This (a) waits for a deliberate Enter, (b) consumes it so no stray `'\n'` leaks to any later reader, and (c) drains pre-buffered input on entry. `terminate()` is unchanged.

> This is the minimal robust fix. We deliberately do **not** switch to raw/cbreak mode or to `BtNode_WaitKeyboardPress` (which waits for `'s'`, contradicting the "Enter" spec).

---

## 5. Mock mode & testing

- **Mock building blocks:** `BtNode_DoorDetection`, `BtNode_Announce`, `BtNode_GotoAction`, `BtNode_MoveArmSingle` all carry real mock support (keypress / immediate). Under `BT_MOCK_MODE=true` the whole tree ticks without hardware.
- **The Enter-wait is intentionally never mocked** — it is the human checkpoint the task exists to gate. Confirmed as desired behavior. (Consequence: a fully-unattended mock run will block at node 7 until a human presses Enter — expected.)
- **Smoke test:** `BT_MOCK_MODE=true ros2 run behavior_tree inspection` → observe order: constants → arm mock → door mock-opens → prints "door open" → nav mock → prints "Dear referees, I am Tinker." → blocks for Enter → prints "Heading to the exit." → nav mock → tree SUCCESS.
- **Keypress-fix unit check:** feed buffered bytes to stdin before node 7's `initialise()` and assert it still waits for a *fresh* Enter (drain works); assert `update()` returns RUNNING with no input and SUCCESS after a `'\n'`.
- **Pre-flight (host hazard):** verify `python3 -c "import behavior_tree.messages"` imports cleanly on the target robot before a real run — `messages.py:43` `TTSCnRequest` can fail system-wide on some builds (documented in `tk25_decision/CLAUDE.md`), which would break all BT imports including announce.
- **Real run prerequisites:** Nav2 (`navigate_to_pose`) up with a `map`; `door_detection` node + **Orbbec** camera (RealSense-only ⇒ door service always fails); `announce` TTS service; `joint_move_action` for the arm pre-pose. Poses in `constants.json` must match the arena `map`.

---

## 6. Files touched

| File | Change |
|---|---|
| `behavior_tree/Inspection/inspection.py` | Rewrite `createInspection()` per §3; insert `"door open"` announce; replace self-intro with one line; delete `createQandA()` + dead imports/keys. |
| `behavior_tree/Inspection/customNodes.py` | Fix `BtNode_PressEnterToSucceed` per §4 (drain-on-init + consume-on-success). |
| `behavior_tree/Inspection/constants.json` | **No change** (`tinker_description`/`arm_pos_navigating` keys retained; `tinker_description` now dormant). |
| `cli.py`, `setup.py`, `runtime.py` | **No change.** |

No rebuild is required for the tree logic if the package is installed with `--symlink-install`; confirm on the target and rebuild `behavior_tree` if not symlinked.

---

## 7. Risks & mitigations
- **Door service Orbbec-only / down:** closed *and* error both return FAILURE, so `Retry(999)` keeps polling — the tree will not advance until the door genuinely opens (or ~999 failed attempts). If the Orbbec is not up, the run stalls at node 3 by design; mitigation is the real-run prerequisite check in §5.
- **Nav hard-fail:** `Retry(10)` then the Sequence fails with no recovery — pre-existing behavior, retained. Acceptable for this task.
- **Enter-wait cooked-terminal assumption:** the fix relies on line-buffered stdin (true under `ros2 run` in a terminal). If ever launched with no TTY/piped stdin, `select` may report EOF-ready; out of scope here (documented assumption).

## 8. Non-goals
Named-location registry; path de-hardcoding; Q&A/listen behavior; raw-mode keypress; any change to other tasks or to pose values.
