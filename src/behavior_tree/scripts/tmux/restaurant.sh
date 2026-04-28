#!/usr/bin/env bash
# Restaurant task bringup. tmux session 'tinker-restaurant'.
# Attach: tmux a -t tinker-restaurant
# Override entry point: BT_ENTRY=restaurant-simplified bash restaurant.sh
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"

SESSION="tinker-restaurant"
BT_ENTRY="${BT_ENTRY:-restaurant}"
DEV="$(pan_tilt_dev)"
[ -z "$DEV" ] && warn_no_pan_tilt restaurant

bringup_cameras "$SESSION"
bringup_drivers "$SESSION" "$DEV" 0

new_window "$SESSION" "vision"
pane_send "$SESSION:vision" "ros2 run object_detection_generalist generalist_node"
pane_add "$SESSION:vision" "ros2 run object_detection_new yolo_seg_default_node"
retile "$SESSION:vision"

new_window "$SESSION" "manip"
pane_send "$SESSION:manip" "ros2 launch mobile_bringup grasp_bringup.launch.py"
pane_add "$SESSION:manip" "ros2 run pick_and_place pick_and_place"
retile "$SESSION:manip"

bringup_audio "$SESSION" ""
bringup_nav "$SESSION"
bringup_bt "$SESSION" "$BT_ENTRY"

tmux select-window -t "$SESSION:cameras"
echo "[restaurant] Session '$SESSION' created (BT entry: $BT_ENTRY). Attach: tmux a -t $SESSION"
echo "[restaurant] Verify graph after ~30s: ros2 run behavior_tree verify-task-endpoints --task restaurant"
