# tkmr physical launch

## Speech context

Listen.action accepts an initial_prompt string per goal. Empty prompts preserve
the server default. Hangzhou supplies separate examples for owner enrollment,
electrical switches, post-fall assistance and general help; prompts do not leak
between requests. The audio server logs the chosen prompt and, when debug WAV
capture is enabled, writes a matching .wav.json with the prompt and transcript.

Enrollment normalizes homophones and conservative single-syllable near matches
against OWNER_NAMES. Context events preserve the original transcript and matched
name. Electrical requests normalize exact verb homophones before semantic
interpretation; both versions are sent to the interpreter, preserving negation
and corrections. General requests retain their original text.

After rebuilding tinker_audio_msgs, audio_pakage and behavior_tree, restart the
Listen server and decision process together so both load the new action type.
Existing processes retain their old imported interface until restarted.

Start basic services without arguments:

```bash
bash /home/tinker/tk25_ws/src/tk25_basic/src/scripts/master_hangzhou.sh
```

The script opens the decision window with the tree process already running.
After localization on the map, press Enter in that terminal to start the task.
No command needs to be typed. Ctrl+C or EOF cancels without starting the task.
After Enter, the tree initializes its interfaces and waits silently for five
seconds before navigating to register.

For manual startup outside tmux, the same interactive command is available:

```bash
source /home/tinker/tk25_ws/src/tk25_basic/src/scripts/hangzhou_env.sh
ros2 run behavior_tree hangzhou-home
```

The script automatically attaches to tmux, or switches clients when already
inside tmux. Existing sessions are reused. Inspect later with
`tmux attach -t robot`. Do not run another base/arm provider alongside
this session. Like master_hri.sh, a new session runs basic setup.sh for CAN and
USB permissions (sudo may request a password), and selects USB0 before USB1.
With no USB device, other services start and head control is skipped. The
decision tree is still started separately. master_hangzhou_live.sh is an alias.

The session is named robot. Windows follow HRI naming: master, navigation
(base/Nav2/approach, three panes), vision (Orbbec, head controller, head state,
generalist, features, behaviour, litter and head camera server: eight panes when USB is present),
audio (TTS, listen and phrase extraction: three panes), grasp (TK27 hardware
and planning: two panes), decision (tree waiting for Enter). Each service runs in
the foreground with its logs visible. On exit the pane returns to a shell
and retains the output. The decision process starts automatically, but the
mission requires Enter. No dependency-discovery wrapper is used by this script.

Each script-started service is owned by hangzhou_pane.py. Closing its pane or
killing the session sends SIGINT to its process group, waits up to 12 seconds,
then escalates to SIGTERM (4 seconds) and SIGKILL if necessary. Ordinary detach
keeps services running. Normal service exit also cleans surviving children
before returning to a shell. The manipulation pane removes only its tracked,
owned, inactive Unix sockets; an active listener is never unlinked. Stale
worker sockets are checked before the next launch. This does not alter global
IPC, CUDA, drivers, or system configuration. Existing pre-change processes
must be restarted through the script to acquire supervision; commands started
manually in the returned shell are not managed by the old supervisor.

Regression: an isolated real tmux session with a signal-resistant grandchild
was closed; both worker processes and the socket were gone after 16.2 seconds.

Hangzhou uses the live /go_to_approach Action. Its HRI-style tkinter dashboard
opens before ROS leaf setup; task setup allows 120 seconds, without changing
other tasks' 15-second default. Full real-interface setup and GUI creation
were verified without ticking the tree: setup completed in 16.34 seconds.
CAN reconnection can leave can0 DOWN; setup.sh must restore the interface
before restarting driver.launch.py. The SDK's I/O-error self-join path is not
changed by this launch/configuration fix.

The map is robocup_home_2026_measured_5cm.yaml (0.05 m/cell), selected by
map_yaml in tkmr_live.json. The basic launch script reads the same setting.
Room scan points are livingroom_search, bedroom_search, diningroom_search,
and kitchen_search. Living-room and bedroom switches use livingroom_switch
and bedroom_switch for both on/off commands. Coordinates and yaw are loaded
at task startup from the map's .poses.yaml using Map Studio's PoseStore.
Editing a point in Map Studio takes effect on the next task start.

The task waits silently for five seconds, then navigates to the Map Studio
point register before enrollment. There is no entry waypoint, exit navigation,
or overall mission/patrol time budget. Individual request timeouts remain.
Only the living room and bedroom have switches. Dining-room and kitchen patrols
retain person reporting, assistance/help interactions and litter inspection;
resting people are reported there without asking for a switch operation.

Switch requests use full Chinese speech from listen_action and semantic intent
classification: on, off, none (keep the current state), or unknown (ask again).
Negation and self-corrections are interpreted before any switch motion.

Arm actions send
saved pose names to TK27: navigation for stow, help for simulated assistance,
and click for switch operation. The server loads web_manip/export/<name>.yaml
for each request; the decision tree stores no joint-angle snapshots.

Every navigation and approach is preceded by a successful MoveToSavedPose
request for navigation. Failure blocks the base action. The endpoint is
/manipulation/move_to_saved_pose, using tk27_manipulation_msgs/action/MoveToSavedPose.
Collision avoidance is enabled. Both action success and application success
are required; TK27 checks measured joint convergence before reporting success.

The CLI uses the shared mock configuration and respects BT_MOCK_MODE and
BT_MOCK_CONFIG overrides. Task wrappers use their underlying leaf's configuration;
Tk27SavedPose follows BtNode_MoveArmSingle. Historical dry-run entrypoints remain
separate. Chinese audio and actual VLM/head services are
started or reused by the dependency bringup. Physical navigation requires a
valid map localization; startup does not invent an initial robot pose.

Shells opened by the older launcher may retain its forced overrides. Run
`unset BT_MOCK_MODE BT_MOCK_CONFIG` once in such a shell before restarting the
tree to use the shared configuration. Existing tree processes must be restarted.

tkbuild selects tk25_decision/.venv_decision/bin/python for decision entry
points. Install pip runtime dependencies in that environment, not under the
ROS install output. pypinyin 0.55.0 is installed there and recorded in the
decision repository's .venv_decision.uv-project dependency files. Rebuilding
behavior_tree does not recreate or remove that environment.

Current validation: 173 regression tests passed, including five PTY startup
tests with robot execution replaced at the runtime boundary. The real ROS CLI
was also started in an isolated tmux pane: it waited for Enter, and Ctrl+C
returned to the shell without starting the mission. The production CLI
configuration check passes with register, four search points and two switches.
Twelve text-only requests to the configured model passed on/off, negation,
self-correction, keep-state and ambiguous/out-of-scope checks. Measured model
latency was 0.52-0.76 seconds. This is a text interpretation check, not an ASR
or physical interaction acceptance test. Runtime imports resolve to the updated
source through the existing build links. No physical base or arm motion was
started during this change.
