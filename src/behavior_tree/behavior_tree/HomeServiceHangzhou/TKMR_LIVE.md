# tkmr physical launch

Start basic services without arguments:

```bash
bash /home/tinker/tk25_ws/src/tk25_basic/src/scripts/master_hangzhou.sh
```

After localization on the map, start the behavior tree in another terminal:

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
generalist, features, behaviour and litter: seven panes when USB is present),
audio (TTS, listen and phrase extraction: three panes), grasp (TK27 hardware
and planning: two panes), decision (prepared terminal). Each service runs in
the foreground with its logs visible. On exit the pane returns to a shell
and retains the output. The decision terminal does not run the mission
automatically. No dependency-discovery wrapper is used by this script.

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

The map is `1013_xlab.yaml`. Room order is living_room=sofa, bedroom=pillar,
dining_room=round_table, kitchen=corner. The user-approved temporary mapping
uses sofa for entry, corner for exit, and each room scan point for both switch
commands. Switch arm targets are copied from TK27 web_manip/export/click.yaml;
assistance targets are copied from default.yaml. Coordinates and joints are
snapshots in tkmr_live.json; editing an exported YAML does not change this snapshot.

The navigation arm pose is HRI/constants.json arm_pos_navigating:
[-86, -52, 18, 2, 34, -73, -9] degrees, converted to radians. Every navigation
and approach is preceded by a successful real arm retraction. Failure blocks
the base action. The task uses TK27 /manipulation/joint_move with joint1..joint7,
and requires action success plus a successful application result.

The live CLI disables mocks and refuses mocked interfaces. Historical dry-run
entrypoints remain separate. Chinese audio and actual VLM/head services are
started or reused by the dependency bringup. Physical navigation requires a
valid map localization; startup does not invent an initial robot pose.

Validation: 30 offline tests passed, live tree construction has zero mock
leaves, package build and shell syntax checks passed. No physical base or arm
motion was started during this configuration change.
