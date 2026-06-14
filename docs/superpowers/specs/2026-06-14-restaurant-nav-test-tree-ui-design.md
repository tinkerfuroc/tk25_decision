# Restaurant pure-nav test tree + web UI — design

**Date:** 2026-06-14
**Status:** approved (design)
**Spans two repos:** `tk25_decision` (the behavior tree) + `tk26_vision` (the web app).

## Goal

A minimal, operator-runnable **pure-nav test** for the Restaurant task: scan for a
waving person, then navigate to the closest one — exercising only the
vision-scan→nav path (no order-taking, arm, or delivery). It is provided as a
browser dashboard (modeled on `vision_track/track_web`) that can bring up the
prerequisite stack, launch the test, and show live status + camera + approach
progress.

## Scope

- **In:** (1) a new BT test tree reusing existing restaurant/scan/approach nodes;
  (2) a new web dashboard package (process-spawn control + live status/camera);
  (3) operator-driven prerequisite bring-up via an allowlisted ProcessManager.
- **Out (non-goals):** queue/loop behaviors (one-shot closest waver only);
  readiness *gating/ordering guarantees* (bring-up is best-effort, status shown);
  per-frame waving re-detection on the live video (overlays come from the last
  scan); any change to the reused BT nodes, the waving service, or the nav stack.

## Architecture — two components

### Component 1 — the test behavior tree (repo: `tk25_decision`, package `behavior_tree`)

New file `behavior_tree/Restaurant/restaurant_nav_test.py`. Composes existing
nodes (verified signatures):

- **Scan (pan-tilt sweep + accumulate wavers):** reuse the `test_scan.py` pattern
  — `scan_once(pan, tilt, target_frame)` which does `BtNode_TurnPanTilt` (degrees)
  → settle `Timer` → `BtNode_DetectCallingCustomer` (calls `/detect_waving_persons`,
  appends) → `BtNode_PackWavingCustomers` (dedup-by-distance accumulate into
  `KEY_CUSTOMER_CENTROIDS`), run across a few sweep positions gated by
  `BtNode_GateBlackBoardList`. Sweep positions (for now): `[0, -60, +60]` deg
  (centre first, then left, then right) at tilt ~10° — a short forward-facing
  fan, fewer than the full restaurant 11-pos sweep since this is a nav test
  (a tunable module-level constant).
  `target_frame="map"` so accumulated centroids are map-frame and directly
  usable as a nav goal.
- **Select closest:** a small leaf picks, from the accumulated
  `KEY_CUSTOMER_CENTROIDS` (map-frame, dedup'd), the centroid with the **minimum
  distance to the robot** — computed by looking up the robot pose via TF
  (`map`→`base_link`); if TF is unavailable it falls back to list index 0. It
  writes the chosen centroid as a `PointStamped(map)` to the approach target key.
  (Nearest-to-robot is recomputed here because closest-first ordering from a
  single detection no longer holds globally after accumulating across the sweep.)
- **Go-to:** `BtNode_Approach(bb_target_key=<target>)` → `/go_to_approach`
  (`GoToApproach`), interpreting `result.status` (0 = success).
- **Status publishing:** a new local leaf `BtNode_PublishNavTestStatus` (defined
  in the new file, not in shared TemplateNodes) creates a `std_msgs/String`
  publisher on `/restaurant_nav_test/status` in `setup(**kwargs)` via
  `self.node = kwargs['node']` (the established pattern), and at each phase
  publishes a JSON document:
  `{phase, waver_count, wavers:[{x,y}], target:{x,y}, approach_status:int|null,
  errormsg:str, result:str}`. Inserted between phases of the root Sequence. No
  edits to the reused nodes.
- **Runtime + entry point:** `main()` calls
  `run_tree(build_tree, period_ms=500.0, title="Restaurant Nav Test",
  node_name="restaurant_nav_test")`. New `setup.py` console script
  `restaurant-nav-test = behavior_tree.Restaurant.restaurant_nav_test:main`.
- **Mock:** all reused nodes already honor `BT_MOCK_MODE`; the status leaf
  short-circuits its publisher creation in mock and still publishes synthetic
  status so the UI can be exercised without hardware.

Root tree (one-shot Sequence): announce/reset → scan sweep (accumulate) →
select-closest → publish "approaching" → `BtNode_Approach` → publish final
result.

### Component 2 — the web dashboard (repo: `tk26_vision`, new package `restaurant_nav_test_web`)

Placed in `tk26_vision` to reuse its proven web infra: the
`.venv-vision-main` py3.10 venv already carries `fastapi`/`uvicorn`/`opencv`, and
`scripts/build.sh` fixes the entry-point shebangs to that venv. The package does
**not** build-depend on `behavior_tree` or `approach_planner` (it spawns them as
processes and talks to them only over the ROS graph), so no dependency-direction
issue. Mirrors `track_web` 1:1:

- `restaurant_nav_test_web/restaurant_nav_test_web.py` — rclpy node + thread-safe
  bridge + uvicorn in a daemon thread + `MultiThreadedExecutor(4)`. Params:
  `bind` (default `0.0.0.0`), `port` (default `8768`), `camera_topic`
  (default `/camera/color/image_raw`), `status_topic`
  (default `/restaurant_nav_test/status`), `target_frame`/`robot_frame`
  (`map`/`base_link`, for distance). Subscribes the status topic (`String`) and
  the camera (`Image` → `cv2.imencode` JPEG under a lock, like track_web's
  `_on_image`). Bridge contract:
  `snapshot() / latest_state() / latest_jpeg() / start_test() / stop_test() /
  proc_status() / proc_start(name) / proc_stop(name) / proc_group_start(group)`.
- `restaurant_nav_test_web/restaurant_nav_test_web_app.py` — ROS-free FastAPI
  factory `create_app(bridge, webui_dir)`: `GET /`, `/style.css`, `/app.js`
  (no-cache `FileResponse`); `GET /api/status`; `POST /api/test/start`,
  `/api/test/stop`; `POST /api/proc/{name}/start|stop` and
  `/api/proc/group/{group}/start|stop` (group routes declared BEFORE `{name}`);
  `GET /api/proc/status`; `GET /stream.mjpg` (heartbeat MJPEG generator);
  `WS /ws/state` (poll-seq loop pushing `state` + `proc` + `readiness`).
- `webui/{index.html, app.js, style.css}` — track_web-style: header(conn) +
  video pane (MJPEG `<img>` + box/target overlay) + Prerequisites panel
  (per-process toggles + "Start all") + Readiness row + Test controls
  (Start/Stop + mock checkbox) + Status panel + Log. `app.js`: WS connect/
  reconnect, `post()` helper, `renderState`, MJPEG `src`, overlay draw from the
  status JSON's `wavers`/`target`/boxes.
- `launch/restaurant_nav_test_web.launch.py` (params bind/port/topics; sets
  `FASTRTPS_DEFAULT_PROFILES_FILE` so the node's camera subscription hits 30 Hz),
  `setup.py` (`data_files` install `webui/*`, `launch/*`, `config/*`;
  console script `restaurant_nav_test_web = …:main`), `requirements.txt`
  (`fastapi`, `uvicorn`), `package.xml` (rclpy, std_msgs, sensor_msgs,
  geometry_msgs, tf2_ros, cv_bridge).

### ProcessManager — allowlisted prerequisite bring-up

A `process_manager.py` mirroring track_web's (REGISTRY dict of name → argv list,
GROUPS dict, `subprocess.Popen(start_new_session=True, env=os.environ.copy())`,
SIGTERM→SIGKILL teardown, status `{name, running, pid, returncode}`, allowlist:
only registered names runnable). The registry is **seeded from a config file**
`config/processes.yaml` so machine-specific values are editable without code
edits. Seeded entries (commands taken from the real launch files, not guessed):

| name | command | env / args | provides | deps |
|---|---|---|---|---|
| `camera_femto` | `ros2 launch orbbec_camera femto_bolt.launch.py depth_registration:=true enable_ir:=false enable_frame_sync:=false` | `FASTRTPS_DEFAULT_PROFILES_FILE=<ws>/src/tk26_vision/config/fastdds_shm.xml` | `/camera/color/image_raw` | — |
| `camera_realsense` | `ros2 launch realsense2_camera rs_launch.py camera_name:=xarm_camera align_depth.enable:=true config_file:=<ws>/src/tk26_vision/config/realsense_qos.yaml` | same FASTRTPS env | `/camera/xarm_camera/color/image_raw` | — |
| `pan_tilt` | `ros2 launch pan_tilt pan_tilt.launch.py device:=/dev/ttyUSB0` | needs `ROBOT_NAME` | `/pan_tilt_controller/{cmd,state}` | — |
| `waving` | `ros2 launch tk_vision_specialized detect_waving.launch.py` | — | `/detect_waving_persons` | camera |
| `nav_driver` | `ros2 launch navigation_bringup driver.launch.py` | needs `ROBOT_NAME` | base + LiDAR + TF | — |
| `nav2` | `ros2 launch navigation_bringup bringup_launch.py slam:=false map:=<ws>/src/tk26_navigation/src/navigation_bringup/maps/xlab_2d_0430.map.yaml use_sim_time:=false autostart:=true` | needs `ROBOT_NAME` | `/navigate_to_pose`, `/global_costmap/costmap` | nav_driver |
| `approach` | `ros2 launch approach_planner approach_planner.launch.py use_sim_time:=false` | — | `/go_to_approach` | nav2 |
| `test_bt` | `ros2 run behavior_tree restaurant-nav-test` (± `BT_MOCK_MODE=true`) | — | runs the test | all above |

GROUPS: `prereqs = [camera_femto, pan_tilt, waving, nav_driver, nav2, approach]`
started with the existing stagger (dependency-ordered). The UI "Start all
prerequisites" calls `proc_group_start("prereqs")`. `test_bt` is started/stopped
by the separate Test Start/Stop buttons (with the mock flag toggling the env).
Config values referencing `<ws>` and the map path are resolved at load from a
`workspace_root` param / env; `ROBOT_NAME` is inherited from the launching shell
(operator runs `source robot-env.sh` first, same as every real-robot launch).

### Readiness (graph-based, no type deps)

The node computes a readiness dict from the ROS **graph** (names only, so no
message-type imports): `camera` (the `camera_topic` present and a frame received
recently), `pan_tilt` (`/pan_tilt_controller/state` present), `waving`
(`/detect_waving_persons` in `get_service_names_and_types()`), `goto`
(`/go_to_approach/_action/send_goal` present in services). Pushed over `/ws/state`
and rendered as green/red dots. The Test Start button warns (does not block) when
`goto` is red.

## Data flow

```
Browser ──WS /ws/state──▶ FastAPI ──▶ bridge.latest_state()/proc_status()/readiness()
   │   ◀─MJPEG /stream──  (uvicorn,    bridge.latest_jpeg()  (camera + overlay client-side)
   │                       daemon thread)
   ├─POST /api/proc/group/prereqs/start ─▶ ProcessManager.start_group  ─spawn▶ camera/pan_tilt/waving/nav…
   └─POST /api/test/start ──────────────▶ ProcessManager.start("test_bt") ─spawn▶ ros2 run behavior_tree restaurant-nav-test
ROS node subscribes: <camera_topic>(Image) + /restaurant_nav_test/status(String) ; reads TF(map→base_link) for distance
            the BT ───publishes status JSON each phase──────────────────────────┘
```

## Error handling

- No wavers found → BT publishes `{phase:"scan", result:"no_wavers"}` and ends;
  UI shows "no wavers", Test returns to idle.
- `go_to_approach` failure → BT publishes `approach_status` = the GoToApproach
  code (1..10) + `errormsg`; UI shows it (e.g. "status 5 TARGET_OOB").
- BT / prereq process crash or exit → ProcessManager status flips to
  not-running with `returncode`; UI shows exited + code; last status retained.
- Camera/TF absent → MJPEG heartbeat shows last/placeholder frame; distance `—`;
  readiness dot red.
- Bring-up of a prereq fails (bad device, missing map) → that process exits;
  status + returncode shown; other prereqs unaffected (best-effort, no gating).

## Testing

- `restaurant_nav_test_web_app.py` (ROS-free) unit-tested with a fake bridge:
  routes return the bridge payloads; `/stream.mjpg` emits boundary frames; WS
  pushes on seq change — mirror `track_web_app`'s tests.
- `process_manager.py` unit test: unknown name rejected (allowlist); start/stop/
  status transitions with a trivial command (e.g. `sleep`); group start staggers.
- BT smoke test in `BT_MOCK_MODE`: `build_tree()` ticks to SUCCESS with a
  synthetic waver; `BtNode_PublishNavTestStatus` runs without a real publisher.
- Manual: `source robot-env.sh`; launch the web app; "Start all prerequisites";
  watch readiness go green; Start → scan → approach → result.

## Open decision (for spec review)

- **Web-app repo placement:** recommended `tk26_vision` (infra reuse). Alternative:
  `tk26_navigation` or `tk25_decision` (would need a fastapi/uvicorn/cv2 venv set
  up there). Flag if you'd prefer a different home.

## Invariants

- No edits to reused BT nodes, the waving service, nav stack, or any deployed
  config. The ProcessManager only runs allowlisted, pre-declared commands (no
  user-supplied command strings ever reach `Popen`).
