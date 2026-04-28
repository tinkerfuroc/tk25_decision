# Per-task tmux launchers

One launcher per RoboCup task. Each spawns a tmux session named `tinker-<task>` with a window per subsystem (cameras, drivers, vision, manip, audio, nav, bt).

## Quick start

```bash
# from the workspace root, after a successful colcon build + source:
bash src/tk25_decision/src/behavior_tree/scripts/tmux/hri.sh
tmux a -t tinker-hri
```

After ~30 s of stabilization (cameras + Nav2 + arm), verify the live ROS graph:

```bash
ros2 run behavior_tree verify-task-endpoints --task hri
```

The verifier prints an OK/MISS table and always exits 0; fix any MISS row by inspecting the corresponding pane.

## Scripts

| Script | Session | BT entry point |
|---|---|---|
| `hri.sh` | `tinker-hri` | `behavior_tree hri` |
| `restaurant.sh` | `tinker-restaurant` | `behavior_tree restaurant` (override with `BT_ENTRY=restaurant-simplified`) |
| `pick_and_place.sh` | `tinker-pick-and-place` | `behavior_tree pick-and-place` |

Each script sources `common.sh` for: workspace sourcing, FastDDS profile env, pan-tilt device probing, and tmux pane helpers.

## Prerequisites

- Workspace built: `./src/tk26_vision/scripts/build.sh` (vision shebangs) **plus** `colcon build --packages-select pick_and_place arm_api anygrasp_ros2 mobile_bringup behavior_tree`.
- Pan-tilt servo on `/dev/ttyUSB0` or `/dev/ttyUSB1`.
- `OPENROUTER_API_KEY` in workspace-root `.env` (kimi_api nodes need it).
- Per `src/tk26_vision/CLAUDE.md`, `FASTRTPS_DEFAULT_PROFILES_FILE` is set automatically by `common.sh`.

## Known gotchas

- **`drop_service`** (PickAndPlace tmux only) waits for an undefined `arm_cartesian_service` and will spin forever on init. Drop calls will not work until that interface is added to `tinker_arm_msgs`. Tracked in the plan as out-of-scope.
- **`messages.py:43` `TTSCnRequest` import** may fail system-wide if `tinker_audio_msgs` doesn't export the symbol. The `bt` pane will not start until that import is wrapped in a try/except or the audio msgs package is rebuilt.
- Vendored Orbbec + Realsense launches drop to ~3 Hz without the FastDDS profile; `common.sh` sets `FASTRTPS_DEFAULT_PROFILES_FILE` for you.

## Stopping a session

```bash
tmux kill-session -t tinker-hri
```
