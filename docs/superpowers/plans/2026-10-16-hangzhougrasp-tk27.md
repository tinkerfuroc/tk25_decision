# hangzhougrasp TK27 manipulation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move the `--grasp` tree of `hangzhougrasp` from legacy `start_grasp` / `joint_move_action` to TK27 `Pick`, `MoveToSavedPose` and a strictly checked gripper open, with Orbbec scan+announce first, then `pick45`, a RealSense yolo26 gate, open, Pick.

**Architecture:** Three task-local action leaves in a new `hangzhougrasp/tk27_nodes.py` (modelled on `HomeServiceHangzhou.ros_nodes.Tk27SavedPose`), three small blackboard leaves in `custom_nodes.py`, then a rewire of `hangzhougrasp.py`, `parts.py` and `config.py`. Mission shape (rooms, holding flag, delivery rules, `--part`) is unchanged.

**Tech Stack:** ROS 2 Humble, py_trees / py_trees_ros, `tk27_manipulation_msgs`, `control_msgs`, pytest.

**Spec:** `docs/superpowers/specs/2026-10-16-hangzhougrasp-tk27-manipulation-design.md` (source: `...-tk27-manipulation-handoff.md`).

## Global Constraints

- Work only in the worktree `/home/tinker/tk25_ws/src/tk25_decision/.claude/worktrees/hangzhougrasp-tk27` (branch `hangzhougrasp-tk27`). Another session edits the main checkout.
- Touch only `src/behavior_tree/behavior_tree/hangzhougrasp/`, `src/behavior_tree/test/test_hangzhougrasp*.py`, and `docs/superpowers/`. Do not edit `TemplateNodes/`, `mock_config.json`, `HomeServiceHangzhou/` or anything outside this package.
- Run tests with `/home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/python .claude/hg_pytest.py <files> -q` from the worktree root (`source` is blocked in this worktree; the helper sets the ROS env). Baseline: `140 passed`.
- Never commit `.claude/` (untracked helper). `git add` explicit paths only; no `-A`, `commit -a`, stash, reset, checkout, amend.
- Endpoints: `/manipulation/pick`, `/manipulation/move_to_saved_pose`, `/xarm_gripper/gripper_action`.
- Pick goal: `prompt` = label from the RealSense scan, `plan_effort` = `"high"`. Saved pose goal: `avoid_collisions = False` always (not configurable).
- Gripper open: `command.position = 0.0`, `command.max_effort = 50.0`; success = ROS SUCCEEDED and `reached_goal` and `|position - 0.0| <= 0.025`.
- Saved poses: table `pick45`, stow `navigation`.
- No fallback: removed config keys (`arm_pos_navigating`, `arm_pos_table`, `arm_action`, `grasp_action`) are rejected as unknown.
- Mock reuse via `mock_config_name`: Pick → `"BtNode_Grasp"`, saved pose → `"BtNode_MoveArmSingle"`, gripper → `"BtNode_GripperAction"`.
- All spoken text stays Mandarin (only the existing announce nodes speak).

## Review Focus

1. **Orbbec miss must not run the stow cleanup.** Nothing seen at a viewpoint → no arm command at all, and a cleanup-stow failure must still fail the run. Test: Task 3 `test_orbbec_miss_moves_no_arm` and `test_cleanup_stow_failure_still_fails_run`.
2. **Stale scan result.** A previous viewpoint's detections must not make the next viewpoint look "seen". Test: Task 3 `test_stale_orbbec_result_is_cleared`.
3. **Failed open blocks Pick, every retry round re-opens.** Test: Task 3 `test_failed_open_never_ticks_pick`, `test_each_retry_round_opens_then_picks`.
4. **Release failure continues the run and clears holding** (so later viewpoints still scan and announce). Test: Task 3 `test_release_failure_logs_continues_and_clears_holding`.
5. **Empty or whitespace label never reaches the Pick server.** Test: Task 1 `test_pick_empty_prompt_sends_no_goal`, Task 2 `test_select_nearest_label_rejects_blank`.

---

### Task 1: TK27 action leaves

**Files:**
- Create: `src/behavior_tree/behavior_tree/hangzhougrasp/tk27_nodes.py`
- Test: `src/behavior_tree/test/test_hangzhougrasp_tk27.py` (new)

**Interfaces:**
- Consumes: `behavior_tree.TemplateNodes.ActionBase.ActionHandler` (same use as `HomeServiceHangzhou/ros_nodes.py:162 Tk27SavedPose`: `super().__init__(name, ActionType, action_name, key, wait_for_server_timeout_sec=-3)`, override `send_goal` calling `self.send_goal_request(goal)`, `process_result` reading `self.result_status` and `self.result_message.result`; in mock mode `send_goal` defers to `super().send_goal()`).
- Produces:
  - `BtNode_Tk27Pick(name: str, prompt_bb_key: str, action_name: str, plan_effort: str)`; class attr `mock_config_name = "BtNode_Grasp"`. Goal prompt read from `self.blackboard.goal` (the key passed as `key`), stripped.
  - `BtNode_Tk27SavedPose(name: str, pose_name: str, action_name: str)`; `mock_config_name = "BtNode_MoveArmSingle"`; `key=None`; goal `pose_name`, `avoid_collisions=False`.
  - `BtNode_Tk27GripperOpen(name: str, action_name: str, open_position: float, open_tolerance: float, max_effort: float)`; `mock_config_name = "BtNode_GripperAction"`; `key=None`.
  - All three: on FAILURE set `feedback_message` with action status, business status and `error_msg` (gripper: position/reached_goal/stalled) and log it with `self.logger.error`.

- [ ] **Step 1: Write the failing tests** in `test_hangzhougrasp_tk27.py` (pattern: `test_hangzhou_live_motion.py:27`, set `leaf.mock_mode = False`, replace `leaf.send_goal_request` with `goals.append`, set `leaf.result_status` / `leaf.result_message = SimpleNamespace(result=...)`):
  - `test_pick_contract`: `leaf.action_type is Pick`, `leaf.action_name == "/manipulation/pick"`; blackboard key holds `" coke "` → one goal, `goal.prompt == "coke"`, `goal.plan_effort == "high"`.
  - `test_pick_success_requires_all_three`: `(4, True, 0)`→SUCCESS; `(6, True, 0)`, `(4, False, 0)`, `(4, True, 2)`→FAILURE, and the FAILURE `feedback_message` contains the result's `error_msg` (e.g. `"PLANNING_FAILED detail"`).
  - `test_pick_empty_prompt_sends_no_goal`: key holds `"  "` (and separately unset) → `goals == []` and `update()` returns FAILURE without a server.
  - `test_saved_pose_contract`: `BtNode_Tk27SavedPose("a", "pick45", "/manipulation/move_to_saved_pose")` → goal `pose_name == "pick45"`, `avoid_collisions is False`; result `(4, True, 0)` SUCCESS; `(4, False, 1)` (FILE_NOT_FOUND) and `(4, False, 9)` FAILURE with `error_msg` in feedback.
  - `test_gripper_open_contract`: goal `command.position == 0.0`, `command.max_effort == 50.0`, type `GripperCommand`.
  - `test_gripper_open_strict_result` parametrized: SUCCEEDED+reached+pos 0.01 → SUCCESS; reached False; stalled True with reached False; pos 0.03; status 6 (aborted); status 5 (canceled) → FAILURE.
  - `test_mock_config_names`: the three `mock_config_name` values above.

- [ ] **Step 2: Run to verify failure**
Run: `.../.venv_decision/bin/python .claude/hg_pytest.py src/behavior_tree/test/test_hangzhougrasp_tk27.py -q`
Expected: FAIL, `ModuleNotFoundError: behavior_tree.hangzhougrasp.tk27_nodes`.

- [ ] **Step 3: Implement the three classes** in `tk27_nodes.py`. Import `GoalStatus` from `action_msgs.msg`; import action types inside `__init__` like `Tk27SavedPose` does. For the empty prompt, `send_goal` must not call `send_goal_request`; make `update()` return FAILURE when no goal was sent (set a flag in `send_goal`; check before `super().update()`).

- [ ] **Step 4: Run to verify pass** (same command). Expected: all pass.

- [ ] **Step 5: Commit**
```bash
git add src/behavior_tree/behavior_tree/hangzhougrasp/tk27_nodes.py src/behavior_tree/test/test_hangzhougrasp_tk27.py
git commit -m "feat(hangzhougrasp): TK27 Pick, MoveToSavedPose and strict gripper-open leaves"
```

### Task 2: Blackboard helper leaves

**Files:**
- Modify: `src/behavior_tree/behavior_tree/hangzhougrasp/custom_nodes.py` (add three classes; keep `BtNode_SelectGraspTarget` until Task 3, because `hangzhougrasp.py` still imports it)
- Test: `src/behavior_tree/test/test_hangzhougrasp_tk27.py` (append)

**Interfaces:**
- Produces:
  - `BtNode_ClearKey(name: str, key: str)`: writes `None` to `key`, SUCCESS.
  - `BtNode_ScanSawObjects(name: str, bb_source: str)`: SUCCESS iff the key exists, value is not None, `value.status == 0` and `value.objects` is nonempty; otherwise FAILURE (never raises).
  - `BtNode_SelectNearestLabel(name: str, bb_source: str, label_key: str)`: first writes `""` to `label_key`; SUCCESS iff the scan saw objects (same rule) and `objects[0].cls.strip()` is nonempty, writing that stripped label; else FAILURE.

- [ ] **Step 1: Write the failing tests** (append): `test_clear_key_sets_none`; `test_scan_saw_objects` parametrized over missing key, `None`, `status=1`, `objects=[]`, one object → only the last SUCCESS; `test_select_nearest_label_takes_first` (`["cup","coke"]` → `"cup"`); `test_select_nearest_label_rejects_blank` (`cls="  "` → FAILURE, label key `""`); `test_select_nearest_label_clears_old_label` (old `"coke"`, then failing input → `""`).
- [ ] **Step 2: Run** the file. Expected: FAIL on import.
- [ ] **Step 3: Implement** the three classes in `custom_nodes.py`.
- [ ] **Step 4: Run** the full hangzhougrasp suite. Expected: all pass (140 + new).
- [ ] **Step 5: Commit** `custom_nodes.py`, `test_hangzhougrasp_tk27.py`: `feat(hangzhougrasp): scan-seen, nearest-label and clear-key leaves`.

### Task 3: Config and tree migration

**Files:**
- Modify: `hangzhougrasp/config.py`, `hangzhougrasp/constants.json`, `hangzhougrasp/hangzhougrasp.py`, `hangzhougrasp/parts.py`, `hangzhougrasp/custom_nodes.py` (delete `BtNode_SelectGraspTarget` and the `copy` import)
- Modify tests: `test_hangzhougrasp.py`, `test_hangzhougrasp_subtrees.py`, `test_hangzhougrasp_parts.py`, `test_hangzhougrasp_scan.py`, `test_hangzhougrasp_poses.py`, `test_hangzhougrasp_cli.py`, `test/hangzhougrasp_test_config.json` (only if it carries removed keys)
- Test: new tree tests in `test_hangzhougrasp_subtrees.py`

**Interfaces:**
- Consumes: Task 1 leaves, Task 2 leaves, existing `BtNode_ScanForYolo26`, `BtNode_AnnounceDetections`, `BtNode_SetHolding`, `BtNode_LogError`.
- Produces:
  - `TaskConfig`: remove `arm_pos_navigating`, `arm_pos_table`, `arm_action`, `grasp_action`; add `pick_action: str`, `saved_pose_action: str`, `gripper_action: str`, `pick_plan_effort: str`, `arm_pose_navigating: str`, `arm_pose_table: str`, `gripper_open_position: float`, `gripper_open_tolerance: float`, `gripper_max_effort: float`. Values in `constants.json` exactly as the spec's Configuration block.
  - `config.py` keys: remove `KEY_ARM_NAVIGATING`, `KEY_ARM_TABLE`, `KEY_GRASP_VISION_RES`, `KEY_GRASP_LABEL`; keep `KEY_SCAN_RESULT` (Orbbec), `KEY_GRASP_SCAN_RESULT` (RealSense), `KEY_TARGET_LABEL` (Pick prompt).
  - `hangzhougrasp.py`:
    - `_savedPoseRetry(name, pose_name, config)` → `Retry(BtNode_Tk27SavedPose(name, pose_name, config.saved_pose_action), config.arm_retry_limit)`; replaces `_moveArmRetry` everywhere.
    - `_scanYolo26Retry(name, config, *, bb_key, use_orbbec, prompt)`: no payload flags (all `return_*` False).
    - `createScanAnnounce(config)` (no `grasp` kwarg): Orbbec scan → `KEY_SCAN_RESULT`, then the existing announce-or-log Selector.
    - `createGrasp(config)`: Sequence `Grasp nearest object` = `_savedPoseRetry("Arm to table pose", config.arm_pose_table)` → `_scanYolo26Retry("Scan table with RealSense", bb_key=KEY_GRASP_SCAN_RESULT, use_orbbec=False, prompt=config.prompt)` → `BtNode_SelectNearestLabel("Select nearest label", KEY_GRASP_SCAN_RESULT, KEY_TARGET_LABEL)` → `Retry("Retry open and pick", Sequence[BtNode_Tk27GripperOpen("Open gripper before pick", ...), BtNode_Tk27Pick("Pick selected object", KEY_TARGET_LABEL, config.pick_action, config.pick_plan_effort)], config.grasp_retry_limit)` → `BtNode_SetHolding("Set holding after grasp", True)` → `_savedPoseRetry("Retract arm with object", config.arm_pose_navigating)`.
    - `createScanGraspWithCleanup(config, *, miss_is_success=False)`: Selector[`createGrasp`, Sequence[`_savedPoseRetry("Retract arm on failure", nav)`, Success/Failure outcome as today]].
    - `createViewpointScanGrasp(config, *, miss_is_success)`: Sequence `Viewpoint scan then grasp` = `BtNode_ClearKey("Clear previous scan result", KEY_SCAN_RESULT)` → Selector `Scan or nothing seen`[`createScanAnnounce(config)`, Success `Nothing seen`] → Selector `Grasp only if seen`[Inverter(`BtNode_ScanSawObjects("Scan saw objects?", KEY_SCAN_RESULT)`), `createScanGraspWithCleanup(config, miss_is_success=miss_is_success)`].
    - `createVisitTable` uses `createViewpointScanGrasp(config, miss_is_success=True)`.
    - `_release`: `_savedPoseRetry("Arm to navigation pose", nav)` then goto.
    - `_open_and_clear(root, where, config)`: Selector[`Retry(BtNode_Tk27GripperOpen(f"Open gripper at {where}"), config.arm_retry_limit)`, `BtNode_LogError(f"Log release failure at {where}", ...)`] then `BtNode_SetHolding(f"Clear holding after release at {where}", False)`.
    - `createHangzhouGrasp`: first move `_savedPoseRetry("Arm to nav before first navigation", nav)`; `createConstantWriter(config)` loses the `enable_grasp` kwarg and the arm writes.
    - Imports of `BtNode_Grasp`, `BtNode_MoveArmSingle`, `BtNode_GripperAction` removed from `hangzhougrasp.py`.
  - `parts.py`: `_writer(config, pose_names)` (no arm flags); `scan-grasp` part body = `createViewpointScanGrasp(config, miss_is_success=False)`; arm-first step uses `task._savedPoseRetry(..., config.arm_pose_navigating)`; `PARTS["scan-grasp"]` text: "Orbbec scan and announce, then pick45, RealSense gate, open gripper and TK27 Pick, with failure cleanup (needs --grasp)".

- [ ] **Step 1: Config tests first** (in `test_hangzhougrasp_poses.py`): `test_tk27_config_defaults` (all nine new values from constants.json); `test_old_arm_keys_rejected` parametrized over the four removed keys via `_write(tmp_path, <key>=...)` → `ValueError` "Unknown"; `test_plan_effort_validated` (`"max"` → ValueError, each of `low/medium/high/auto` passes); `test_gripper_numbers_validated` (tolerance `0`, effort `-1`, position `nan` → ValueError); `test_saved_pose_names_nonempty` (`arm_pose_table=" "` → ValueError).
- [ ] **Step 2: Tree tests first** (in `test_hangzhougrasp_subtrees.py`, rewrite `_stub` to stub `hg.BtNode_Tk27SavedPose`, `hg.BtNode_Tk27Pick`, `hg.BtNode_Tk27GripperOpen`, `hg.BtNode_ScanSawObjects`, `hg.BtNode_SelectNearestLabel`, `hg.BtNode_AnnounceDetections`, `hg._scanYolo26Retry` and `hg._gotoRetry` by node name, keeping the `Leaf`/`overrides` pattern; `_cfg()` gains the new fields). Keep every existing visit-table test, renamed to the new node names (`Pick selected object` replaces `Grasp selected object`). Add:
  - `test_viewpoint_order`: log order is Orbbec scan, announce, `Arm to table pose`, `Scan table with RealSense`, `Select nearest label`, `Open gripper before pick`, `Pick selected object`, `Set holding after grasp`, `Retract arm with object`.
  - `test_orbbec_miss_moves_no_arm`: `Scan saw objects?` FAILURE → viewpoint SUCCESS, no name starting `Arm`/`Retract`/`Pick`/`Open` in log.
  - `test_realsense_miss_is_miss_with_stow`: RealSense scan FAILURE → `Retract arm on failure` logged, no Pick, holding unset, visit-table SUCCESS.
  - `test_failed_open_never_ticks_pick`: `Open gripper before pick` FAILURE → `Pick selected object` not in log, `Retract arm on failure` in log.
  - `test_each_retry_round_opens_then_picks`: `grasp_retry_limit=2`, Pick FAILURE → log contains `[open, pick, open, pick]` in that order.
  - `test_holding_only_after_pick_success`: Pick FAILURE → holding never True.
  - `test_cleanup_stow_failure_still_fails_run` (Pick FAILURE + `Retract arm on failure` FAILURE → visit-table FAILURE).
  - `test_stale_orbbec_result_is_cleared`: use the real `BtNode_ClearKey` and `BtNode_ScanSawObjects` (do not stub those two here); pre-set `/hg_scan_result` to a response with one object and stub the Orbbec scan to FAILURE → no arm move.
  - `test_release_failure_logs_continues_and_clears_holding`: `createDeliver` with every `Open gripper at ...` FAILURE → SUCCESS, `Log release failure at dest` in log, holding False.
  - `test_no_legacy_manipulation_in_grasp_trees` (in `test_hangzhougrasp_parts.py`): for every part with `enable_grasp=True` and the full tree, no node is an instance of `TemplateNodes.Manipulation.BtNode_Grasp`, `BtNode_MoveArmSingle` or `BtNode_GripperAction`, and no node has `action_name` in `{"start_grasp", "joint_move_action"}`.
  - `test_every_pick_preceded_by_open` (parts file): for each `BtNode_Tk27Pick` in the full grasp tree, its parent is a Sequence whose previous child is a `BtNode_Tk27GripperOpen`.
  - Update `test_no_grasp_builds_no_manipulation_nodes` to ban the three TK27 classes plus `BtNode_SetHolding`.
- [ ] **Step 3: Run** all hangzhougrasp tests. Expected: new tests FAIL (missing names / old structure).
- [ ] **Step 4: Implement** config.py (validation per spec), constants.json, hangzhougrasp.py, parts.py as in Interfaces. Then update the remaining old tests: delete the `BtNode_SelectGraspTarget` tests and import in `test_hangzhougrasp.py` (~lines 19, 230-275); `test_hangzhougrasp_scan.py` (no grasp variant; scans request no payload), `test_hangzhougrasp_parts.py` `_tick_stubbed` monkeypatches (replace `BtNode_MoveArmSingle`/`BtNode_Grasp`/`BtNode_GripperAction` stubs with the three TK27 leaves; Pick stub reads `prompt_bb_key`; `RUNTIME_KEYS` drops removed keys and keeps `KEY_SCAN_RESULT`, `KEY_GRASP_SCAN_RESULT`, `KEY_TARGET_LABEL`, `KEY_ANNOUNCEMENT`, `KEY_HOLDING`), writer-name assertions that listed `Write arm navigating`/`Write arm table`, and `arm_key_written`.
- [ ] **Step 5: Run** `.claude/hg_pytest.py src/behavior_tree/test/test_hangzhougrasp*.py -q`. Expected: all pass, count ≥ 140 + new.
- [ ] **Step 6: Print-tree check**: `PYTHONPATH` is set by the helper only for pytest, so verify with a test instead: `test_cli_grasp_print_tree_uses_tk27` in `test_hangzhougrasp_cli.py` (`cli.main(["--config", conf_path, "--grasp", "--print-tree"])` output contains `Pick selected object`, `Arm to table pose`, `Open gripper before pick`, and not `Grasp selected object`).
- [ ] **Step 7: Commit** the five source files and the touched tests: `feat(hangzhougrasp): drive --grasp through TK27 Pick and saved poses`.

### Task 4: Docs

**Files:**
- Modify: `src/behavior_tree/behavior_tree/hangzhougrasp/README.md` (Chinese)
- Modify: `docs/superpowers/specs/2026-10-16-hangzhougrasp-tk27-manipulation-design.md` (Status line → "approved; implemented on branch hangzhougrasp-tk27")

- [ ] **Step 1:** README: replace the grasp tree block with the new order (Orbbec 扫描播报 → pick45 → RealSense 门控 → 开夹爪 → TK27 Pick → 收回 navigation); new config keys table; TK27 bringup (`ros2 launch manipulation_bringup arm.launch.py`, `ros2 launch manipulation_bringup manipulation.launch.py`, `ros2 run object_detection_generalist generalist_node`, `ROBOT_NAME=tinker2`; do not run TK25 `grasp_bringup` alongside); Pick segments via VLM+MobileSAM itself; release failure policy; mock mode never reaches Pick (mock scan returns no objects); live validation order from the spec.
- [ ] **Step 2:** Run the full hangzhougrasp suite once more. Expected: all pass.
- [ ] **Step 3: Commit** README and spec: `docs(hangzhougrasp): TK27 manipulation flow and bringup`.
