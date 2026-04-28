#!/usr/bin/env bash
# HRI task bringup. tmux session 'tinker-hri'. Attach: tmux a -t tinker-hri
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"

SESSION="tinker-hri"
DEV="$(pan_tilt_dev)"
[ -z "$DEV" ] && warn_no_pan_tilt hri

bringup_cameras "$SESSION"
bringup_drivers "$SESSION" "$DEV" 1

new_window "$SESSION" "vision"
pane_send "$SESSION:vision" "ros2 run vision_util door_detection"
pane_add "$SESSION:vision" "ros2 run kimi_api feature_recognition"
pane_add "$SESSION:vision" "ros2 run kimi_api seat_recommend_bbox"
retile "$SESSION:vision"

new_window "$SESSION" "manip"
pane_send "$SESSION:manip" "ros2 launch mobile_bringup grasp_bringup.launch.py"
pane_add "$SESSION:manip" "ros2 run pick_and_place pick_and_place"
retile "$SESSION:manip"

bringup_audio "$SESSION" "get_confirmation_action"
bringup_nav "$SESSION"
bringup_bt "$SESSION" "hri"

tmux select-window -t "$SESSION:cameras"
echo "[hri] Session '$SESSION' created. Attach: tmux a -t $SESSION"
echo "[hri] Verify graph after ~30s: ros2 run behavior_tree verify-task-endpoints --task hri"
