#!/usr/bin/env bash
set -eo pipefail
WS="${HANGZHOU_WORKSPACE:-/home/tinker/tk25_ws}"
DECISION="${HANGZHOU_DECISION_ROOT:-/home/tinker/tk25_ws/src/tk25_decision}"
MODE="${1:-services}"
if [ "$#" -gt 0 ]; then shift; fi
cd "$WS"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export PYTHONNOUSERSITE=1
export ROBOT_NAME="${ROBOT_NAME:-tinker2}"
export ALL_PROXY='' all_proxy=''
if [ -f "$WS/.env" ]; then set -a; source "$WS/.env"; set +a; fi
export PYTHONPATH="$DECISION/src/behavior_tree:$PYTHONPATH"
case "$MODE" in
services) exec /usr/bin/python3 -u -m behavior_tree.HomeServiceHangzhou.bringup --workspace "$WS" "$@";;
run) exec /usr/bin/python3 -u -m behavior_tree.HomeServiceHangzhou.dry_run "$@";;
check) exec /usr/bin/python3 -u -m behavior_tree.HomeServiceHangzhou.dry_run --preflight "$@";;
*) echo "Usage: $0 {services|check|run} [options]" >&2; exit 2;;
esac

