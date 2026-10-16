# HangzhouGrasp TK27 Manipulation Handoff

Status: static review complete; implementation not started. Date: 2026-10-16.

This handoff covers the manipulation path of the `hangzhougrasp` behavior tree
and the work required to align it with the TK27 manipulation actions described
in the operator's voice note. The current `--grasp` tree still uses legacy
`tinker_arm_msgs/Grasp` and fixed-joint `JointMove` calls. It therefore does not
meet the supplied action contract and is not ready for a TK27-only live stack.

# Original spoken directives translated into English

The first approximately two seconds of the recording were unclear and did not
contain an identifiable technical directive. The remaining directives were:

> The decision layer needs to call three manipulation actions.
>
> The first action is `Pick`. `Pick` grasps an object and lifts it.
>
> The second action is `MoveToSavedPose`. It moves the arm to a previously saved
> position.
>
> The third action is the gripper action. It controls opening and closing the
> gripper.
>
> Before every call to `Pick`, use the gripper action to open the gripper.
> Otherwise, `Pick` will report an error.
>
> `Pick` has two goal fields. The first is `prompt`; provide a prompt describing
> what it should grasp. The second is `plan_effort`; it controls how much effort
> is spent on planning. Use `high` for this field.
>
> `MoveToSavedPose` accepts a field containing the name under which the position
> was saved. It also has a field for whether collision avoidance is enabled.
> Set that field to `false` so collision avoidance is disabled.

The speaker was unsure of the exact name of the second `MoveToSavedPose` field.
The installed interface confirms that the exact fields are `pose_name` and
`avoid_collisions`.

# Verified TK27 action contracts

The current TK27 source exposes these production actions:

| Endpoint | Action type | Goal fields |
| --- | --- | --- |
| `/manipulation/pick` | `tk27_manipulation_msgs/action/Pick` | `prompt`, `plan_effort` |
| `/manipulation/move_to_saved_pose` | `tk27_manipulation_msgs/action/MoveToSavedPose` | `pose_name`, `avoid_collisions` |
| `/xarm_gripper/gripper_action` | `control_msgs/action/GripperCommand` | `command.position`, `command.max_effort` |

`Pick.plan_effort` accepts `low`, `medium`, `high`, or `auto`. The voice note
requires `high`. `MoveToSavedPose.avoid_collisions=false` disables environment,
self, and held-object collision constraints for that request while preserving
joint limits, cancellation, and arrival verification.

There is one material discrepancy between the spoken directive and the current
TK27 server implementation. The voice note requires a separate gripper-open
action immediately before every `Pick`, but the current TK27 coordinator already
opens the gripper at the start of `Pick` in
`tk27_manipulation/src/pick_and_place/src/pipeline_node.cpp`. The implementation
owner must confirm which deployed server version is authoritative. Until that is
confirmed, preserve the explicit pre-open requirement from the voice note and
make it a separately tested tree step.

# Current HangzhouGrasp manipulation flow

The current table-grasp path is:

```text
MoveArmSingle to local table joint angles
-> scan and announce through yolo26_sam
-> choose the nearest detected class
-> reacquire RGB, depth, and mask data
-> send tinker_arm_msgs/Grasp to start_grasp
-> set hg_holding=true
-> MoveArmSingle to local navigation joint angles
```

The relevant implementation is in
`src/behavior_tree/behavior_tree/hangzhougrasp/hangzhougrasp.py`:

- `_moveArmRetry` constructs `BtNode_MoveArmSingle` with a seven-angle
  blackboard value and `add_octomap=False`.
- `createGrasp` constructs `BtNode_Grasp` and passes camera images, masks, and an
  object label.
- `createGrasp` contains no gripper-open node before the grasp call.
- `_open_and_clear` uses the gripper only when releasing an object.

The task configuration in
`src/behavior_tree/behavior_tree/hangzhougrasp/constants.json` still names the
legacy endpoints `start_grasp` and `joint_move_action` and stores seven joint
angles for the table and navigation poses.

# Review findings

## Legacy grasp action blocks TK27 operation

`BtNode_Grasp` uses `tinker_arm_msgs/action/Grasp`, not
`tk27_manipulation_msgs/action/Pick`. Its goal contains a header, RGB image,
depth image, segmentation masks, `use_mesh`, `stay`, and `object_label`. None of
those fields belong to the TK27 `Pick` goal.

The task's detector prompt is currently sent only to `yolo26_sam`; it is never
sent to a TK27 `Pick` action. No code sends `plan_effort="high"`. On a deployment
that starts only the TK27 production actions, behavior-tree setup will wait for
the obsolete `start_grasp` server.

TK27 `Pick` owns target observation, grasp generation, planning, execution,
gripper closure, and return along the grasp trajectory. The behavior tree must
not attempt to pass its existing RGB, depth, mask, or candidate bundle to the new
action.

## Saved arm poses are not used

All table, navigation, retract, and failure-cleanup arm moves use
`BtNode_MoveArmSingle` and the legacy fixed-joint `JointMove` contract. The tree
never constructs `MoveToSavedPose.Goal`, never sends a `pose_name`, and never
sets `avoid_collisions=false`.

TK27 currently ships saved poses including `navigation`, `pick`, and `pick45`,
but the existing HangzhouGrasp joint arrays do not exactly match those files.
The live table-scan pose name must therefore be selected and validated on the
robot rather than inferred from the old angles.

## The explicit gripper-open precondition is absent

There is no `BtNode_GripperAction(open_gripper=True)` before the current grasp
node. This violates the voice-note sequence. The existing
`test_visit_table_part_only_that_table` test explicitly expects the
`visit-table` subtree to contain no gripper node, so that test currently locks in
the old behavior.

## Gripper action failures are reported as success

`BtNode_GripperAction.process_result()` in
`src/behavior_tree/behavior_tree/TemplateNodes/Manipulation.py` returns
`py_trees.common.Status.SUCCESS` unconditionally. An aborted or canceled
gripper action can therefore be followed by `Pick` as if the gripper had opened.
The pre-open directive cannot be enforced reliably until result handling checks
the ROS action status and the returned `reached_goal` or equivalent success
evidence.

## Existing tests validate the legacy architecture

The current HangzhouGrasp test selection passes all 140 tests, but those tests
assert the presence of `BtNode_Grasp` and `BtNode_MoveArmSingle`; they do not
check TK27 action types, endpoints, or goal payloads. A green suite therefore
does not establish compatibility with the spoken directives or the TK27 stack.

# Required implementation changes

## Add TK27 behavior-tree leaves

Add task-local or shared leaves for the following contracts:

1. `Pick`
   - Action type: `tk27_manipulation_msgs.action.Pick`.
   - Default endpoint: `/manipulation/pick`.
   - Goal fields: a nonempty `prompt` and `plan_effort="high"`.
   - Success requires ROS `STATUS_SUCCEEDED`, `result.success=true`, and
     `result.status == Pick.Result.OK`.
   - Preserve `result.error_msg` in behavior-tree feedback on failure.

2. `MoveToSavedPose`
   - Action type: `tk27_manipulation_msgs.action.MoveToSavedPose`.
   - Default endpoint: `/manipulation/move_to_saved_pose`.
   - Goal fields: the configured `pose_name` and `avoid_collisions=false`.
   - Success requires ROS `STATUS_SUCCEEDED`, `result.success=true`, and
     `result.status == MoveToSavedPose.Result.SUCCESS`.
   - Preserve `result.error_msg` in behavior-tree feedback on failure.

3. Gripper open
   - Use `control_msgs.action.GripperCommand` on the deployed gripper endpoint.
   - Do not report success for rejected, canceled, aborted, stalled, or
     unreached goals unless the deployed gripper contract explicitly defines a
     stalled open command as successful.

## Replace the legacy grasp subtree

Keep the existing scan-and-announce step if the task still needs the Mandarin
announcement before grasping, but do not feed that scan payload into TK27
`Pick`. `Pick` performs its own fresh TK26 observation.

The intended sequence for each grasp attempt is:

```text
MoveToSavedPose(table_scan_pose, avoid_collisions=false)
-> optional scan and Mandarin announcement
-> open gripper
-> Pick(prompt=<configured target>, plan_effort="high")
-> set hg_holding=true
-> MoveToSavedPose(navigation, avoid_collisions=false)
```

If the deployed TK27 `Pick` is confirmed to own gripper opening, remove the
separate open command only after updating this handoff decision and adding a
contract test that proves the deployed `Pick` performs and validates that step.

The existing `Pick` implementation returns to the arm joints from which it
started. The final move to `navigation` is still required when the pick starts
from a table-scan pose.

## Migrate HangzhouGrasp configuration

Replace or retire the following legacy configuration:

- `grasp_action: start_grasp`
- `arm_action: joint_move_action`
- `arm_pos_navigating: [seven angles]`
- `arm_pos_table: [seven angles]`

Add explicit configuration for:

```json
{
  "pick_action": "/manipulation/pick",
  "move_to_saved_pose_action": "/manipulation/move_to_saved_pose",
  "gripper_action": "/xarm_gripper/gripper_action",
  "pick_plan_effort": "high",
  "arm_pose_navigating": "navigation",
  "arm_pose_table": "<validated saved pose name>",
  "move_avoid_collisions": false
}
```

Validate the two pose names as nonempty strings during configuration loading.
The server will still return `FILE_NOT_FOUND` if a configured YAML stem is not
present in its active `export_directory`, so deployment validation must check
the live server's directory rather than only the source-tree examples.

# Required tests

Add focused contract and tree-structure tests before changing the full mock
smoke tests:

- A `Pick` leaf sends the exact TK27 action type and endpoint.
- Its goal contains the configured prompt and `plan_effort == "high"`.
- A failed ROS status, `success=false`, or nonzero business status produces
  behavior-tree failure with the original error message.
- Every path that can call `Pick` has a successful gripper-open predecessor if
  the explicit pre-open directive remains in force.
- A failed gripper-open action prevents `Pick` from running.
- `MoveToSavedPose` sends the expected table or navigation `pose_name` and
  `avoid_collisions is False`.
- A missing saved pose or failed arrival verification stops navigation and
  propagates failure.
- No `BtNode_Grasp`, `BtNode_MoveArmSingle`, `start_grasp`, or
  `joint_move_action` remains in a grasp-enabled HangzhouGrasp tree.
- `--no-grasp` continues to construct no manipulation or gripper clients.
- Retry tests confirm that each new `Pick` attempt gets a fresh action request
  and that holding state is set only after confirmed Pick success.

After migration, rerun the existing HangzhouGrasp suite in addition to the new
contract tests. The review baseline was:

```text
140 passed in 0.49s
```

# Live validation sequence

Run live checks from the smallest irreversible scope upward:

1. Confirm `/manipulation/pick`, `/manipulation/move_to_saved_pose`, and the
   gripper action appear with the expected ROS types.
2. Confirm the configured `navigation` and table-scan pose YAML stems exist in
   the active server `export_directory`.
3. Run `MoveToSavedPose` for the table pose with `avoid_collisions=false`.
4. Run the gripper-open leaf and verify the physical opening and action result.
5. Run one `Pick` with the task prompt and `plan_effort="high"`.
6. Run `MoveToSavedPose` for `navigation` while holding the object.
7. Run `--part scan-grasp`, then `--part visit-table`, before the full mission.

Do not treat tree construction, `--print-tree`, mock success, or the legacy
140-test baseline as evidence that the live TK27 action contracts are correct.
