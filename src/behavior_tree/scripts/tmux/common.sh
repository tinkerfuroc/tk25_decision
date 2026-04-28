#!/usr/bin/env bash
# Shared helpers for per-task tmux launchers.
# Sourced — do not run directly.

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../../.." && pwd)"
export WS_ROOT

# Each tmux pane is its own shell with no inherited env, so every pane
# re-sources the workspace. Cost is amortized at startup only.
source_ws_cmd() {
    printf '%s' "unset ALL_PROXY all_proxy http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ; export FASTRTPS_DEFAULT_PROFILES_FILE=\"$WS_ROOT/src/tk26_vision/config/fastdds_shm.xml\" ; source \"$WS_ROOT/install/setup.zsh\""
}

pan_tilt_dev() {
    if [ -e /dev/ttyUSB0 ]; then
        echo "/dev/ttyUSB0"
    elif [ -e /dev/ttyUSB1 ]; then
        echo "/dev/ttyUSB1"
    else
        echo ""
    fi
}

warn_no_pan_tilt() {
    local label="$1"
    echo "[$label] No pan-tilt device on /dev/ttyUSB[0-1]. Continuing — pan-tilt panes will fail."
}

new_window() {
    local session="$1"
    local window="$2"
    if ! tmux has-session -t "$session" 2>/dev/null; then
        tmux new-session -d -s "$session" -n "$window"
    else
        tmux kill-window -t "$session:$window" 2>/dev/null
        tmux new-window -t "$session" -n "$window"
    fi
}

pane_send() {
    local target="$1"
    local cmd="$2"
    tmux send-keys -t "$target" "$(source_ws_cmd) && $cmd ; exec zsh" C-m
}

pane_add() {
    local window="$1"
    local cmd="$2"
    tmux split-window -h -t "$window"
    pane_send "$window" "$cmd"
}

retile() {
    tmux select-layout -t "$1" tiled
}

# --- Shared bringup blocks ---

bringup_cameras() {
    local session="$1"
    new_window "$session" "cameras"
    pane_send "$session:cameras" "ros2 launch orbbec_camera femto_bolt.launch.py depth_registration:=true enable_ir:=false enable_frame_sync:=false"
    pane_add "$session:cameras" "ros2 launch realsense2_camera rs_launch.py camera_name:=xarm_camera align_depth.enable:=true config_file:=$WS_ROOT/src/tk26_vision/config/realsense_qos.yaml"
    retile "$session:cameras"
}

# bringup_drivers <session> <pan_tilt_dev_or_empty> <with_follow_head 0|1>
bringup_drivers() {
    local session="$1"
    local dev="$2"
    local with_follow_head="$3"
    new_window "$session" "drivers"
    pane_send "$session:drivers" "ros2 launch navigation_bringup driver.launch.py"
    if [ -n "$dev" ]; then
        pane_add "$session:drivers" "ros2 run pan_tilt controller --ros-args -p device:=$dev"
    fi
    pane_add "$session:drivers" "ros2 run pan_tilt state_publisher"
    if [ "$with_follow_head" = "1" ]; then
        pane_add "$session:drivers" "ros2 run pan_tilt follow_head"
    fi
    retile "$session:drivers"
}

bringup_nav() {
    local session="$1"
    new_window "$session" "nav"
    pane_send "$session:nav" "ros2 launch navigation_bringup bringup_launch.py"
}

# bringup_audio <session> <extra_node_or_empty>
# Always brings up announce + phrase_extraction. Pass an extra entry-point
# (e.g. "get_confirmation_action") as $2 to add a third pane.
bringup_audio() {
    local session="$1"
    local extra="$2"
    new_window "$session" "audio"
    pane_send "$session:audio" "ros2 run audio_pakage announce"
    pane_add "$session:audio" "ros2 run audio_pakage phrase_extraction"
    if [ -n "$extra" ]; then
        pane_add "$session:audio" "ros2 run audio_pakage $extra"
    fi
    retile "$session:audio"
}

bringup_bt() {
    local session="$1"
    local entry="$2"
    new_window "$session" "bt"
    pane_send "$session:bt" "ros2 run behavior_tree $entry"
}
