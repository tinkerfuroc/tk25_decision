#!/usr/bin/env bash
# PickAndPlace task bringup. tmux session 'tinker-pick-and-place'.
# Attach: tmux a -t tinker-pick-and-place
# See README.md for the drop_service / arm_cartesian_service caveat.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"

SESSION="tinker-pick-and-place"
DEV="$(pan_tilt_dev)"
[ -z "$DEV" ] && warn_no_pan_tilt pp

bringup_cameras "$SESSION"
bringup_drivers "$SESSION" "$DEV" 1

new_window "$SESSION" "vision"
pane_send "$SESSION:vision" "ros2 run object_detection_new yolo_seg_node"
pane_add "$SESSION:vision" "ros2 run vision_util get_point_cloud"
pane_add "$SESSION:vision" "ros2 run vision_util door_detection"
pane_add "$SESSION:vision" "ros2 run kimi_api grocery_categorize"
retile "$SESSION:vision"

new_window "$SESSION" "manip"
pane_send "$SESSION:manip" "ros2 launch mobile_bringup grasp_bringup.launch.py"
pane_add "$SESSION:manip" "ros2 run pick_and_place pick_and_place"
pane_add "$SESSION:manip" "ros2 run arm_api drop_service"
retile "$SESSION:manip"

bringup_audio "$SESSION" ""
bringup_nav "$SESSION"
bringup_bt "$SESSION" "pick-and-place"

tmux select-window -t "$SESSION:cameras"
echo "[pp] Session '$SESSION' created. Attach: tmux a -t $SESSION"
echo "[pp] Verify graph after ~30s: ros2 run behavior_tree verify-task-endpoints --task pickandplace"
