#!/usr/bin/env bash
# hangzhougrasp bringup. tmux session 'tinker-hangzhougrasp'.
# Attach: tmux a -t tinker-hangzhougrasp
#
# Starts every process the task needs EXCEPT the behavior tree; the BT is run
# by hand from the 'bt' window once the arm's GPU workers report ready.
#
# Manipulation is the TK27 stack (manipulation_bringup). Never also start
# mobile_bringup grasp_bringup.launch.py or arm_api: they double-drive the arm.
# arm.launch.py brings up the RealSense and wrist camera itself, so the
# cameras window only starts the Orbbec.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"

SESSION="tinker-hangzhougrasp"
DEV="$(pan_tilt_dev)"
[ -z "$DEV" ] && warn_no_pan_tilt hg

# ROBOT_NAME and ROS_DOMAIN_ID for every pane (robot-env.sh keeps an
# exported ROBOT_NAME, else derives it from the hostname).
RENV="source \"$WS_ROOT/src/tk25_basic/tools/robot-env.sh\""
run() { pane_send "$1" "$RENV && $2"; }
add() { pane_add "$1" "$RENV && $2"; }

new_window "$SESSION" "cameras"
run "$SESSION:cameras" "ros2 launch orbbec_camera femto_bolt.launch.py depth_registration:=true enable_ir:=false enable_frame_sync:=false"

new_window "$SESSION" "drivers"
run "$SESSION:drivers" "ros2 launch navigation_bringup driver.launch.py"
if [ -n "$DEV" ]; then
    add "$SESSION:drivers" "ros2 run pan_tilt controller --ros-args -p device:=$DEV"
fi
add "$SESSION:drivers" "ros2 run pan_tilt state_publisher"
retile "$SESSION:drivers"

new_window "$SESSION" "vision"
# The task's own scan (service object_detection_yolo26_sam).
run "$SESSION:vision" "ros2 run object_detection_generalist yolo26_sam_node"
retile "$SESSION:vision"

new_window "$SESSION" "manip"
run "$SESSION:manip" "ros2 launch manipulation_bringup arm.launch.py"
add "$SESSION:manip" "ros2 launch manipulation_bringup manipulation.launch.py"
# TK26 detector that TK27 Pick observes through.
add "$SESSION:manip" "ros2 run object_detection_generalist generalist_node"
retile "$SESSION:manip"

new_window "$SESSION" "audio"
run "$SESSION:audio" "ros2 run audio_pakage announce"

new_window "$SESSION" "nav"
run "$SESSION:nav" "ros2 launch navigation_bringup bringup_launch.py"

# The BT is NOT started here: this window is a sourced shell with a hint.
new_window "$SESSION" "bt"
run "$SESSION:bt" "echo 'Run the BT by hand once both manipulation GPU workers report ready:' && echo '  ros2 run behavior_tree hangzhougrasp --map <map> --enter-room <room> [--grasp] [--print-tree]'"

tmux select-window -t "$SESSION:manip"
echo "[hg] Session '$SESSION' created (BT not started). Attach: tmux a -t $SESSION"
echo "[hg] Then, in window 'bt': ros2 run behavior_tree hangzhougrasp --map <map> --enter-room <room> --grasp"
echo "[hg] NOTE: until the TK27 grasp change lands, --grasp still waits for TK25's joint_move_action/start_grasp and blocks at setup with this stack; --no-grasp is unaffected."
