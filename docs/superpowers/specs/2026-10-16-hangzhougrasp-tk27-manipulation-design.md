# hangzhougrasp: TK27 manipulation (Pick, MoveToSavedPose, gripper)

Status: approved; implemented on branch `hangzhougrasp-tk27`. Date: 2026-10-16.
Source: `2026-10-16-hangzhougrasp-tk27-manipulation-handoff.md` (the voice
note and the static review). Builds on the multiroom, audit-fixes and
map-switch specs, which still hold except where this one changes them.

## Intent

The `--grasp` tree still drives the arm through the legacy
`tinker_arm_msgs/Grasp` (`start_grasp`) and fixed-joint `JointMove`
(`joint_move_action`). The live stack is TK27, which serves three actions.
The tree must call exactly those, with the goal values the voice note fixes,
and must treat any failure they report as a failure.

## TK27 contracts (verified against the installed interfaces)

| Leaf | Endpoint | Type | Goal |
|---|---|---|---|
| Pick | `/manipulation/pick` | `tk27_manipulation_msgs/action/Pick` | `prompt` = selected label, `plan_effort` = `"high"` |
| Saved pose | `/manipulation/move_to_saved_pose` | `tk27_manipulation_msgs/action/MoveToSavedPose` | `pose_name`, `avoid_collisions` = `false` |
| Gripper | `/xarm_gripper/gripper_action` | `control_msgs/action/GripperCommand` | `command.position` = 0.0 (open), `command.max_effort` = 50.0 |

Success rules (each leaf fails otherwise, with the server's message in its
feedback and an ERROR log line):
- **Pick:** ROS status SUCCEEDED, `result.success`, `result.status == OK (0)`.
  An empty prompt fails the leaf without sending a goal.
- **Saved pose:** ROS status SUCCEEDED, `result.success`,
  `result.status == SUCCESS (0)`.
- **Gripper open:** ROS status SUCCEEDED, `result.reached_goal`, and
  `|result.position - open_position| <= open_tolerance` (0.025). This is the
  same test TK27's own pipeline uses for an open (`pipeline_node.cpp`
  `command_gripper`). A stalled or unreached open is a failure.

`plan_effort`: the `Pick.action` comment says a nonempty value "must match"
the prewarmed instance. The production launcher
(`manipulation_bringup/runtime.py`) prewarms with `auto` and says "Auto is
only the initial warmup; Pick carries its own effort". So `"high"` per
request is the intended use and no special server setting is needed. If the
deployed server rejects it anyway, Pick fails with the server's message; live
validation step 3 checks this.

## What Pick does with the prompt (verified in TK27 source)

Pick's goal carries only text. The server calls `/object_detection_generalist`
with `camera=realsense`, the prompt, `sort_closest=true`, and RGB, depth and
masks returned. The production `runtime.json` sets
`vision_force_vlm_sam: true`, so the detector skips YOLO: a VLM locates the
prompt and MobileSAM segments it. Pick grasps the first (closest) object
with that mask. So:
- The mask Pick grasps comes from its own capture, never from the tree's
  yolo26 scans.
- The RealSense yolo26 scan at `pick45` is the gate (skip Pick when nothing
  is visible from that pose) and the label source (you confirmed this).
- `generalist_node` and its VLM API must be up for every Pick.

## Decisions from you

1. **Scan order, Pick prompt and gating (revised after approval):** at each
   table viewpoint the robot first scans with the Orbbec camera and announces
   in Mandarin, with the arm still in its navigation pose. Only if that scan
   saw something does it move the arm to `pick45`. It then scans again with
   the RealSense camera and starts its Pick. The Pick prompt is the nearest
   label from the RealSense scan.
   - No Orbbec detections: a miss at that viewpoint, and the arm does not
     move.
   - Orbbec saw something but the RealSense scan finds nothing: a miss
     (cleanup stows the arm).
   - The RealSense scan does not announce again, so each viewpoint is
     announced once.
2. **Table pose:** saved pose `pick45`.
3. **Pre-open:** keep a separate, strictly checked gripper open before every
   Pick attempt, as the voice note says, even though TK27's Pick also opens.
   A failed open stops that attempt; Pick is not sent.
4. **Release failure:** a release (at the destination or a drop pose) retries
   the strict open up to `arm_retry_limit` times. If it still fails, the
   robot logs an ERROR and continues the run, so the remaining scans and
   Mandarin announcements still score.
   - Holding is cleared either way (my call). If it stayed set, every later
     viewpoint would be skipped as "already holding", and those scans and
     announcements would be lost.
   - The next Pick's pre-open tries the gripper again. If that open also
     fails, Pick is not sent: the viewpoint is scanned and announced, then
     counts as a miss.

## New grasp tree (per table viewpoint, `--grasp`)

```text
Sequence "Viewpoint scan then grasp"
  Clear hg_scan_result                   # no stale detections from a prior pose
  Selector "Scan or nothing seen"
    Scan and announce                    # Orbbec, arm still in navigation pose;
                                         #   same subtree as ground/--no-grasp
    Success "Nothing seen"
  Selector "Grasp only if seen"
    Inverter(Scan saw objects?)          # nothing seen -> skip, arm never moves
    <Scan and grasp with failure cleanup, below>

Selector "Scan and grasp with failure cleanup"
  Sequence attempt
    SavedPose(pick45)                    # replaces MoveArmSingle(table joints)
    Scan objects with RealSense          # yolo26_sam again, prompt = task prompt,
                                         #   no image/mask payload, no announce
    Select nearest detected label        # objects[0].cls of the RealSense scan
                                         #   -> hg_target_label
    Retry(grasp_retry_limit)
      Sequence
        Open gripper (strict)
        Pick(prompt=hg_target_label, plan_effort=high)
    Set holding after grasp              # only after Pick succeeded
    SavedPose(navigation)                # Pick returns to pick45, so stow
  Sequence cleanup
    SavedPose(navigation)
    Report miss / failure                # unchanged semantics
```

- An Orbbec miss never reaches the cleanup branch, so a failed cleanup stow
  can still stop the run (the audit fix is kept). The "Scan or nothing seen"
  Selector catches only the Orbbec scan.
- Each Retry round sends a fresh open and a fresh Pick goal.
- Everything else in the mission keeps its shape: stop at first grasp,
  holding flag, delivery and drop rules, `--part` independence.
- Every other arm move becomes `SavedPose(navigation)`: the move before the
  first navigation, "Arm to navigation pose" before delivery and drop, and
  the failure cleanup.
- Release at the destination or a drop pose uses the same strict gripper
  open leaf instead of `BtNode_GripperAction`, in
  `Selector[Retry(open, arm_retry_limit), Log release failure]`, followed by
  clearing holding (decision 4).
- Retry rounds re-send only open and Pick, not the scan. Pick makes its own
  fresh observation from the label.
- Cleanup after a failed attempt only stows the arm. If a failed Pick left
  something in the gripper, the next pre-open releases it. A failed cleanup
  stow still fails the run (audit fixes).
- The tree adds no timeouts. Pick and MoveToSavedPose report their own
  `TIMEOUT` status, which is a failure. Server waits follow
  `Tk27SavedPose` (`wait_for_server_timeout_sec=-3`).
- `avoid_collisions=false` applies to every saved-pose move, including the
  stow before the first navigation.
- `--no-grasp` still builds no arm, Pick or gripper leaf.

Removed from the grasp path: `BtNode_Grasp`, `BtNode_MoveArmSingle`,
`BtNode_GripperAction`, `BtNode_SelectGraspTarget`, the RealSense reacquire
scan, and the RGB/depth/mask payload request. Pick does its own fresh
observation; nothing from the yolo26 scan except the label is passed to it.

## Configuration

`constants.json` drops `arm_pos_navigating`, `arm_pos_table`, `arm_action`
and `grasp_action`. With no fallback, these old keys in a `--config` file
are rejected as unknown. New keys:

```json
"pick_action": "/manipulation/pick",
"saved_pose_action": "/manipulation/move_to_saved_pose",
"gripper_action": "/xarm_gripper/gripper_action",
"pick_plan_effort": "high",
"arm_pose_navigating": "navigation",
"arm_pose_table": "pick45",
"gripper_open_position": 0.0,
"gripper_open_tolerance": 0.025,
"gripper_max_effort": 50.0
```

Startup validation, before any setup or motion:
- endpoints and the two pose names are nonempty strings;
- `pick_plan_effort` is one of `low`, `medium`, `high`, `auto`;
- the gripper numbers are finite, the tolerance is positive and the effort
  is positive.

`avoid_collisions=false` is fixed in the saved-pose leaf, not configurable,
because the voice note fixes it.

## My calls (check these)

- The new leaves are task-local (`hangzhougrasp/tk27_nodes.py`), modelled on
  `HomeServiceHangzhou.ros_nodes.Tk27SavedPose`. The shared
  `TemplateNodes.Manipulation.BtNode_GripperAction`, which always reports
  success, is left alone because other tasks use it; fixing it is a
  separate change.
- Pose names are passed to the saved-pose leaf as literals, not through the
  blackboard. The `hg_arm_*` constants and their writers go away, and so do
  the now-unused grasp blackboard keys.
- Mock mode reuses existing mock-config entries (`mock_config_name`):
  Pick → `BtNode_Grasp`, saved pose → `BtNode_MoveArmSingle`,
  gripper → `BtNode_GripperAction`. `mock_config.json` is not edited.
- The Orbbec and RealSense scans write separate blackboard keys
  (`hg_scan_result`, `hg_grasp_scan_result`), so the label Pick receives
  always comes from the RealSense scan made at `pick45`. Neither scan asks
  for images or masks.
- The RealSense scan uses the task prompt (every configured label), not
  only the labels Orbbec announced, so a label Orbbec missed can still be
  picked.
- No startup check that `pick45.yaml` exists. The server's
  `export_directory` may differ from the source tree, so this is a
  live-validation step instead.

## Non-goals

- Changing TK27 servers, saved-pose files or `TemplateNodes`.
- Verifying that the Pick server's observation vocabulary accepts every
  yolo26 label. Labels are sent as the detector names them. A mismatch
  appears as a Pick failure with the server's message.

## Tests (from the handoff)

- Pick leaf: exact action type and endpoint; goal has the label and
  `plan_effort == "high"`; ROS failure, `success=false` or a nonzero status
  each give FAILURE carrying `error_msg`; an empty prompt sends no goal.
- Saved-pose leaf: sends `pick45` / `navigation` with
  `avoid_collisions is False`; `FILE_NOT_FOUND` or `POSTCONDITION_FAILED`
  fails it.
- Gripper leaf: sends position 0.0 and effort 50.0; a rejected, aborted,
  stalled or unreached goal, or a position outside tolerance, fails it.
- Tree: on every path to Pick, a gripper open comes first in the same
  Sequence. A failed open means Pick never ticks. Each retry round ticks a
  fresh open and Pick. Holding is set only after Pick succeeds.
- No `BtNode_Grasp`, `BtNode_MoveArmSingle`, `BtNode_GripperAction`,
  `start_grasp` or `joint_move_action` anywhere in a `--grasp` tree for any
  part.
- `--no-grasp` builds no manipulation or gripper leaf.
- Config: the new keys validate; the old arm keys are rejected.
- Existing suites (parts independence, CLI, subtrees) are updated to the new
  leaves and stay green.

## Live validation (after merge, on the robot)

1. The three actions are listed with the expected types.
2. `navigation.yaml` and `pick45.yaml` exist in the running server's
   `export_directory`.
3. One `Pick` with `plan_effort="high"` is accepted (not `INVALID_GOAL`).
   Bringup: `ros2 launch manipulation_bringup arm.launch.py`,
   `ros2 launch manipulation_bringup manipulation.launch.py`, and
   `ros2 run object_detection_generalist generalist_node`, with
   `ROBOT_NAME=tinker2`.
4. `--grasp --part scan-grasp`, then `--part visit-table`, then the full
   mission.

Tree construction, `--print-tree` and mock success do not prove the live
contracts.
