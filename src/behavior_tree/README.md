# Behavior Tree Package

ROS2 package for behavior tree-based robot control in the Tinker robot competition framework.

## ✨ What's New (v2.0)

**Major Update:** Complete mock mode restructuring with JSON-based configuration!

- 🎛️ **Subsystem-Level Control** - Independently mock vision, manipulation, navigation, audio, and announcements
- ⌨️ **Keyboard Step-Through** - Press 's' to advance through each mock action
- 📢 **Optional TTS Announcements** - Hear node names announced during execution (configurable per subsystem)
- 🔧 **Zero Code Changes** - All existing scripts work without modification
- 📝 **JSON Configuration** - Fine-grained control via `mock_config.json`
- 🤖 **Unified Behavior** - Navigation nodes now follow same mock pattern as other subsystems

All 30+ template nodes updated with native mock support. No more conditional logic needed in behavior tree scripts!

## 🎯 Features

- **Behavior Tree Architecture** - Modular, composable robot behaviors using py_trees
- **Competition Tasks** - Complete implementations for RoboCup@Home tasks
- **Advanced Mock Mode** - Subsystem-level mocking with JSON configuration
- **Keyboard Control** - Step-through execution for testing and debugging
- **TTS Announcements** - Optional audio feedback during mock execution
- **Zero-Dependency Testing** - Run without any Tinker packages installed
- **Auto-Detection** - Automatically enables mock mode when dependencies are missing

## 🚀 Quick Start

### Basic Usage

```bash
# Source your ROS2 workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run a behavior tree script (auto-detects available hardware)
ros2 run behavior_tree receptionist_secondcall
```

### Mock Mode (No Hardware Required)

The package automatically detects missing dependencies and enables mock mode. You can also force mock mode:

```bash
# Enable mock mode explicitly
export BT_MOCK_MODE=true

# Run any script - press 's' to step through each action
ros2 run behavior_tree receptionist_secondcall

# Disable mock mode (use real hardware)
export BT_MOCK_MODE=false
ros2 run behavior_tree receptionist_secondcall
```

### Fine-Grained Control

Mock individual subsystems by editing `mock_config.json`:

```json
{
  "mock_mode": {
    "enabled": true,
    "subsystems": {
      "vision": {"enabled": true},      // Mock cameras
      "manipulation": {"enabled": false}, // Use real arm
      "navigation": {"enabled": true},   // Mock navigation
      "audio_input": {"enabled": true},  // Mock speech recognition
      "announcement": {"enabled": false} // Use real TTS
    }
  }
}
```

## � Migration Guide (v1 → v2)

### For Existing Scripts

**Good news:** No changes required! All existing behavior tree scripts work without modification.

**What changed:**
- ❌ **Old:** Conditional logic with `MOCK_MODE` variable in each script
- ✅ **New:** Template nodes automatically check `mock_config.json`

**Example - Old Pattern (no longer needed):**
```python
MOCK_MODE = is_mock_mode()
if not MOCK_MODE:
    nav = BtNode_GotoAction("go to sofa", KEY_SOFA_POSE)
else:
    nav = BtNode_WaitKeyboardPress("MOCK: go to sofa", 's')
root.add_child(nav)
```

**Example - New Pattern (automatic):**
```python
# Just use the node - mock mode handled automatically
nav = BtNode_GotoAction("go to sofa", KEY_SOFA_POSE)
root.add_child(nav)
```

### Configuration Changes

**Old:** Single `BT_MOCK_MODE` environment variable  
**New:** Subsystem-level control via `mock_config.json`

```bash
# Still works for backwards compatibility
export BT_MOCK_MODE=true

# But now you can also control individual subsystems
# Edit mock_config.json to mock only what you need
```

### For Custom Nodes

If you created custom nodes, add them to `mock_config.json`:

```json
{
  "subsystems": {
    "your_subsystem": {
      "enabled": true,
      "announce_movement": false,
      "nodes": ["BtNode_YourCustomNode"]
    }
  }
}
```

## �📦 Installation

### With Tinker Packages (Full Installation)

```bash
cd ~/ros2_ws/src
git clone <this-repo>

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
cd ~/ros2_ws
colcon build --packages-select behavior_tree

# Run with real hardware
ros2 run behavior_tree receptionist_secondcall
```

### Without Tinker Packages (Mock Mode Only)

```bash
# Install only ROS2 and py_trees
pip3 install py-trees py-trees-ros

# Optional: Install pyttsx3 for TTS announcements (may cause segfaults on some systems)
# pip3 install pyttsx3

# Clone and build
cd ~/ros2_ws
colcon build --packages-select behavior_tree

# Run - mock mode auto-enabled
ros2 run behavior_tree receptionist_secondcall
```

## 📚 Documentation

### Mock Mode Configuration

The package uses **subsystem-level mocking** controlled by `mock_config.json`:

- **vision** - Object detection, face recognition, point clouds, door detection
- **manipulation** - Arm control, grasping, gripper, pointing
- **navigation** - Mobile base movement, path planning, following
- **audio_input** - Speech recognition, phrase extraction, listening
- **announcement** - Text-to-speech (TTS) in English and Chinese

Each subsystem can be independently mocked or use real hardware.

### Key Files

- `mock_config.json` - Subsystem-level mock configuration
- `config.py` - Configuration loader and helper functions
- `messages.py` - Conditional imports for ROS messages
- `mock_messages.py` - Mock implementations when packages unavailable
- `TemplateNodes/` - Base classes with built-in mock support

## 🎮 Available Scripts

### Competition Tasks

- `receptionist` / `receptionist_secondcall` - Receptionist task
- `GPSR` / `gpsr-demo` - General Purpose Service Robot
- `EGPSR` - Enhanced GPSR
- `store-groceries` - Storing Groceries task
- `help-me-carry` - Help Me Carry task
- `serve-breakfast` - Serve Breakfast task
- `yanglaozhucan` / `ZGC` - Custom tasks
- `restaurant` - Restaurant task
- `inspection` - Inspection task

### Test Scripts

- `test-mock-mode` - Test mock mode functionality
- `test_follow_head` - Test head tracking
- `test-prompt-reached` - Test navigation prompts

### Development Tools

- `draw` - Visualize behavior tree structure
- `hmc-mock-nav` - Mock navigation server for testing
- `hmc-mock-track` - Mock tracking server for testing

### Follow Person

- `follow-person` - Reacquisition-aware follow-person behaviour tree
- `dummy-nav` - **DEPRECATED (2026-06-10), NOT part of the follow pipeline** —
  standalone `/follow_target` subscriber stub (logs, no motion). Its topic has
  no publisher since the FollowPerson rewire; retained for standalone
  experiments only. Real navigation is driven by `follow_server` (see below).

## 🚶 Follow person

A behaviour tree that follows a person via the real `/track_person` action and
reacts to the tracker's **reacquisition state** with non-overlapping voice
announcements. Navigation is driven by the `Follow` action on `follow_server`
(tk26_navigation `following` package), which consumes the tracker's
`/target_points` topic directly. The legacy `dummy-nav` / `/follow_target` stub
is **deprecated** and no longer part of this pipeline (its topic has no
publisher since the 2026-06-10 rewire).

**What it does**

- `BtNode_TrackPersonAction` keeps the `/track_person` goal alive and writes the
  tracker state to the blackboard, including a
  `track/reacquisition_state` key (`0` TRACKING, `1` PASSIVE, `2` NEEDS_HELP).
- `BtNode_FollowAction` keeps the navigation `Follow` action
  (`tinker_nav_msgs/action/Follow`, server `follow_server`) alive and writes the
  follow-executive state to the blackboard (`follow/state` uint8,
  `follow/distance` float, `follow/reacq` uint8, `follow/goal_held` bool). The
  `follow/state` enum:
  - `1` TRACKING — person in view, robot trailing at the standoff distance.
  - `2` PURSUIT_LAST_SEEN — tracker lost/reacquiring; robot heads to the last-seen point.
  - `3` APPROACHING_FINAL — person stationary; robot parking behind them.
  - Terminal outcomes are not `follow/state` values — they surface via the action result instead.
  `follow/goal_held` is `True` on a tick where the executive reused its cached
  fallback goal or skipped dispatch (no fresh reachable standoff) — a hook for the
  tree to announce / pause while the robot is holding rather than actively closing.
  The follow executive consumes
  the tracker's `/target_points` topic directly, so there is **no** per-tick
  follow-goal publisher — `BtNode_FollowAction` drives navigation entirely
  through the long-running action.
- `BtNode_ReacqAnnounce` speaks reacquisition guidance through a
  `CoalescingTTS` (single active utterance, latest-wins pending — speech never
  overlaps and never blocks the tick):
  - PASSIVE → *"Please slow down so I can keep up."*
  - NEEDS_HELP → *"I've lost you. Please raise your hand."*
  - Announces on a state transition and re-announces every ~5 s while still in a
    help state; resets on a return to TRACKING.

**Tree shape**

```
Parallel(SuccessOnAll, synchronise=False)
├── BtNode_TrackPersonAction        # child A — refreshes track/*
├── BtNode_FollowAction             # child B — drives follow_server, refreshes follow/*
└── Sequence(memory=False)          # child C — reacts to the tracker each tick
    └── BtNode_ReacqAnnounce
```

`BtNode_FollowAction` is long-running, so it sits beside the tracker as a
top-level Parallel child — not inside the per-tick reactions Sequence. If either
long-running action terminates (permanent loss / abort) its child returns
FAILURE, the Parallel returns FAILURE, and the follow process ends.

**Flags** (`follow-person [--no-nav] [--breadcrumbs]`)

- `--no-nav` — vision+audio-only tree: omit the `BtNode_FollowAction` child so
  the tracker + reacq announcer run but the base never moves (no `Follow` goal).
- `--breadcrumbs` — route through the person's own dropped trail
  (`follow_server` NavigateThroughPoses). **Off by default**: open following uses
  single-goal standoff pursuit (NavigateToPose re-planned to the person's live
  position at 2 Hz), which tracks a moving person directly. On a long open route
  the accumulated breadcrumb corridor instead pins the robot to a stale trail it
  never advances on. Enable `--breadcrumbs` only for cluttered/doorway following,
  where threading the person's exact trail is what gets the robot through the gap.

**Run sequence** (real tracker + nav stack only — these must already be running):

```bash
# 1. Real tracker + audio TTS service (not started by the launch file)
ros2 run vision_track person_track_server
#    ... and the TextToSpeech ("announce") service from the audio stack

# 2. The follow executive (consumes /target_points, drives Nav2)
ros2 run following follow_server

# 3. The behaviour tree
ros2 run behavior_tree follow-person                # open following (no breadcrumbs)
ros2 run behavior_tree follow-person --breadcrumbs  # clutter/doorway trail following
ros2 run behavior_tree follow-person --no-nav       # vision+audio only (no base motion)
```

All nodes honour the package mock-mode config, so they remain importable and
unit-testable with no ROS graph (see `test/test_coalescing_tts.py`,
`test/test_reacq_announce.py`, `test/test_follow_action_node.py`,
`test/test_feedback_buffer_reacq.py`, `test/test_follow_tree_build.py`).

## 🏗️ Architecture

### Package Structure

```
behavior_tree/
├── config.py                 # Configuration system with JSON loader
├── mock_config.json          # Subsystem-level mock configuration
├── messages.py               # Conditional message imports
├── mock_messages.py          # Mock implementations
├── TemplateNodes/            # Reusable node templates with mock support
│   ├── BaseBehaviors.py      # ServiceHandler base class
│   ├── ActionBase.py         # ActionHandler base class
│   ├── Navigation.py         # Navigation nodes
│   ├── Manipulation.py       # Manipulation nodes
│   ├── Vision.py             # Vision nodes
│   ├── Audio.py              # Audio nodes
│   └── WaitKeyPress.py       # Keyboard press utility
├── GPSR/                     # GPSR task implementations
├── Receptionist/             # Receptionist task
├── StoringGroceries/         # Storing Groceries task
├── HelpMeCarry/              # Help Me Carry task
└── ...                       # Other tasks
```

### Mock Mode System

The package includes a sophisticated mock mode system:

1. **Auto-detects** available Tinker packages at runtime
2. **Subsystem-level control** via JSON configuration
3. **Node-specific mocking** - each node checks its subsystem config
4. **Keyboard control** - step through execution with key presses
5. **Optional TTS** - announce node names during mock execution
6. **Zero code changes** - existing scripts work without modification

## 🔧 Configuration

### Mock Configuration File (`mock_config.json`)

```json
{
  "mock_mode": {
    "enabled": true,
    "auto_detect": true,
    "subsystems": {
      "vision": {
        "enabled": true,
        "announce_movement": false,
        "nodes": ["BtNode_ScanFor", "BtNode_TrackPerson", ...]
      },
      "manipulation": {
        "enabled": true,
        "announce_movement": false,
        "nodes": ["BtNode_Grasp", "BtNode_MoveArmJoint", ...]
      },
      "navigation": {
        "enabled": true,
        "announce_movement": false,
        "nodes": ["BtNode_GotoAction", "BtNode_FollowAction", ...]
      },
      "audio_input": {
        "enabled": true,
        "announce_movement": false,
        "nodes": ["BtNode_PhraseExtraction", "BtNode_Listen", ...]
      },
      "announcement": {
        "enabled": false,
        "announce_movement": false,
        "nodes": ["BtNode_Announce", "BtNode_TTSCN"]
      }
    }
  },
  "keyboard_control": {
    "enabled": true,
    "description": "Wait for 's' key press to advance through mock actions"
  },
  "logging": {
    "print_mock_operations": true,
    "use_emoji": true
  }
}
```

### Environment Variables

```bash
# Override all settings - force mock mode on
export BT_MOCK_MODE=true

# Override all settings - force mock mode off
export BT_MOCK_MODE=false

# Use custom config file location
export BT_MOCK_CONFIG=/path/to/custom/mock_config.json

# Auto-detect (default behavior)
unset BT_MOCK_MODE
```

**Priority:** `BT_MOCK_MODE` env var > `mock_config.json` settings > auto-detection

### Programmatic Configuration

```python
from behavior_tree.config import (
    get_config, 
    is_mock_mode,
    is_subsystem_mocked,
    is_node_mocked
)

# Check global mock mode
if is_mock_mode():
    print("Running in mock mode")

# Check specific subsystem
if is_subsystem_mocked('navigation'):
    print("Navigation is mocked")

# Check specific node
if is_node_mocked('BtNode_GotoAction'):
    print("GotoAction node is mocked")

# Print configuration status
config = get_config()
config.print_status()
```

## 📝 Creating New Behavior Trees

### Using Template Nodes (Recommended)

All template nodes automatically support mock mode. No conditional logic needed:

```python
import py_trees
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Vision import BtNode_ScanFor

def create_tree():
    root = py_trees.composites.Sequence("My Task", memory=True)
    
    # These nodes automatically check mock_config.json
    root.add_child(BtNode_Announce("Greet", "Hello!"))
    root.add_child(BtNode_GotoAction("Go to target", "target_pose"))
    root.add_child(BtNode_ScanFor("Find object", "cup"))
    
    return root
```

### Example: Mixed Real/Mock Execution

Edit `mock_config.json` to use real TTS but mock everything else:

```json
{
  "mock_mode": {
    "subsystems": {
      "vision": {"enabled": true},       // Mocked
      "manipulation": {"enabled": true}, // Mocked
      "navigation": {"enabled": true},   // Mocked
      "audio_input": {"enabled": true},  // Mocked
      "announcement": {"enabled": false} // REAL - uses actual TTS
    }
  }
}
```

### Adding Mock Support to New Nodes

For ServiceHandler nodes (in `BaseBehaviors.py`):

```python
from behavior_tree.TemplateNodes.BaseBehaviors import ServiceHandler
from behavior_tree.config import is_node_mocked

class BtNode_MyCustomNode(ServiceHandler):
    def __init__(self, name: str):
        super().__init__(
            name=name,
            service_type=MyServiceType,
            service_name="/my_service",
            key="my_key",
            # Mock mode automatically handled
        )
        self.mock_mode = is_node_mocked(self.__class__.__name__)
```

Then add the node to `mock_config.json` under the appropriate subsystem.

## 🧪 Testing

### Quick Test

```bash
# Test with mock mode (no hardware needed)
export BT_MOCK_MODE=true
ros2 run behavior_tree receptionist_secondcall

# Press 's' to step through each action
# Press Ctrl+C to exit
```

### Test Specific Subsystems

```bash
# Edit mock_config.json to enable/disable subsystems
# Example: Test with real audio but mock vision/navigation
nano src/behavior_tree/behavior_tree/mock_config.json

# Set:
# "announcement": {"enabled": false}  // Use real TTS
# "vision": {"enabled": true}         // Mock cameras
# "navigation": {"enabled": true}     // Mock navigation

# Rebuild and test
colcon build --packages-select behavior_tree
ros2 run behavior_tree receptionist_secondcall
```

### Configuration Status

```bash
# Run any script to see configuration at startup
ros2 run behavior_tree receptionist_secondcall

# Output shows:
# - Mock mode status
# - Subsystem states (MOCKED or REAL)
# - Keyboard control status
# - Available dependencies
```

## 🐛 Troubleshooting

### Script waits for keyboard press

**Expected behavior** - In mock mode, each action waits for 's' key press:

```
🎤 MOCK AUDIO: Announcing 'Hello'
Press 's' to continue...
```

Press `s` and Enter to advance. To disable: set `keyboard_control.enabled` to `false` in `mock_config.json`.

### "Import could not be resolved" warnings in IDE

**Expected** - When Tinker packages aren't installed, IDEs show import warnings. The code still runs using mock implementations.

**To fix:**
- Install Tinker packages for full functionality
- Or configure IDE to ignore these warnings
- Or use `# type: ignore` comments

### Segmentation fault with TTS announcements

**Cause:** pyttsx3 library can segfault on some systems.

**Solution:** Disable TTS in `mock_config.json`:

```json
{
  "subsystems": {
    "vision": {"announce_movement": false},
    "manipulation": {"announce_movement": false},
    ...
  }
}
```

The system will automatically disable TTS after first failure and use print statements instead.

### Real hardware not responding

```bash
# 1. Verify mock mode is disabled
export BT_MOCK_MODE=false

# 2. Check Tinker packages are installed
ros2 pkg list | grep tinker

# 3. Verify services/actions are running
ros2 service list
ros2 action list

# 4. Check configuration
ros2 run behavior_tree receptionist_secondcall
# Look for "REAL" status for subsystems
```

### Subsystem stays mocked despite config changes

**Priority order:**
1. `BT_MOCK_MODE` environment variable (highest)
2. `mock_config.json` settings
3. Auto-detection (lowest)

**Fix:** Unset environment variable to use JSON config:

```bash
unset BT_MOCK_MODE
ros2 run behavior_tree receptionist_secondcall
```

## 🤝 Contributing

### Adding New Tasks

1. Create directory: `behavior_tree/YourTask/`
2. Implement behavior tree using template nodes
3. **No mock logic needed** - template nodes handle it automatically
4. Add console script entry in `setup.py`
5. Test with: `export BT_MOCK_MODE=true && ros2 run behavior_tree your_task`

### Adding New Template Nodes

1. Add class to appropriate file in `TemplateNodes/`
2. Inherit from `ServiceHandler` or `ActionHandler`
3. Set `self.mock_mode = is_node_mocked(self.__class__.__name__)`
4. Add node name to `mock_config.json` under correct subsystem
5. Test mock behavior

Example:

```python
# In TemplateNodes/Vision.py
from behavior_tree.config import is_node_mocked

class BtNode_NewVisionNode(ServiceHandler):
    def __init__(self, name: str):
        super().__init__(...)
        self.mock_mode = is_node_mocked(self.__class__.__name__)
```

```json
// In mock_config.json
{
  "subsystems": {
    "vision": {
      "nodes": [
        "BtNode_ScanFor",
        "BtNode_NewVisionNode"  // Add here
      ]
    }
  }
}
```

## 📜 Changelog

_Append-only. Newest entries on top._

- f4_mock_config.json (installed to share/): navigation+vision REAL, announcement MOCKED — the BT config the F4 launch points BT_MOCK_CONFIG at.
- follow-person BT: --no-nav flag builds the vision+audio-only tree (no follow-navigation child).
- **2026-06-11** — refactor: disambiguate the legacy HRI follow node. Renamed the
  `HRI/follow.py` class `BtNode_FollowAction` → `BtNode_FollowActionLegacy`
  (it targets the removed `tracking_server` action and parses v1 `Follow`
  feedback). It shared a short name with the live executive node
  `behavior_tree.TemplateNodes.FollowAction.BtNode_FollowAction` (drives
  `follow_server`) — a different class entirely; the canonical node and all of
  its import sites are untouched. The legacy clone is retained only for the
  `hri-follow` tuning harness and now carries a DEPRECATED docstring. Also moved
  the mock `Follow.Feedback` to the v2 schema (`state`, `distance_to_person`,
  `reacq_state=255`, `breadcrumbs_pending`, `goal_held`), dropping the v1
  `status`/`point_header`/`nav_goal_header` fields; the legacy HRI node reads
  `getattr(feedback, "status", "")`, so it degrades gracefully against the new
  mock. (tk26_navigation follow review-fixes, Phase P7.)
- **2026-06-11** — `BtNode_FollowAction` mirrors the new `Follow` feedback field
  `goal_held` to the blackboard key `follow/goal_held` (bool). Set `True` on any
  tick where the follow executive reused its cached fallback goal or skipped
  dispatch (no fresh reachable standoff), `False` otherwise. Read from feedback via
  `getattr(..., "goal_held", False)`, so it stays safe against an older
  `follow_server` whose `Follow.action` lacks the field. New configurable
  `bb_key_goal_held` ctor arg (default `follow/goal_held`); seeded `False` in
  `initialise()` + mock `send_goal()`; `regular_update()` writes it every tick and
  appends `HOLDING` to the feedback message while held. Mirrors the additive
  `tinker_nav_msgs` Follow feedback field (tk26_navigation Commit B).
- **2026-06-11** — docs: add the `follow/state` enum legend next to the
  blackboard-key description (1=TRACKING, 2=PURSUIT_LAST_SEEN,
  3=APPROACHING_FINAL; terminal outcomes surface via the action result, not
  `follow/state`). Docs-only.
- **2026-06-10** — Mark the `/follow_target` consumers deprecated after the
  FollowPerson rewire. Since `BtNode_PublishFollowGoal` (the only publisher of
  `/follow_target`) was removed, navigation is driven entirely by the `Follow`
  action on `follow_server` (tk26_navigation `following` package), which consumes
  the tracker's `/target_points` directly. Added deprecation/non-pipeline notes
  to `dummy_nav_node.py` (module docstring), `launch/follow_process.launch.py`
  (comment header), the `dummy-nav` entry-point and Follow-person sections of
  this README, and corrected the stale `follow_person.py` docstring. Docs-only;
  no code or topic behaviour changed and `dummy-nav` is retained for standalone
  experiments.
- **2026-06-10** — Wire `BtNode_FollowAction` into the follow-person tree
  (P5 of the person-following plan). New `TemplateNodes/FollowAction.py`: a
  continuous-action node for `tinker_nav_msgs/action/Follow` on `follow_server`,
  mirroring `BtNode_TrackPersonAction` (feedback buffer under a lock,
  RUNNING-while-following, SUCCESS on success/cancel, FAILURE on abort/reject,
  KEYPRESS/IMMEDIATE mock mode). It writes `follow/state` (uint8),
  `follow/distance` (float) and `follow/reacq` (uint8) to the blackboard. The
  follow-person tree now has three top-level Parallel children
  (tracker, follow executive, reactions); the long-running `BtNode_FollowAction`
  sits beside the tracker, not inside the reactions Sequence. **Removed**
  `BtNode_PublishFollowGoal` and the `/follow_target` publisher entirely —
  navigation consumes the tracker's `/target_points` directly via `follow_server`.
  `test_publish_follow_goal.py` deleted; `test_follow_action_node.py` added and
  `test_follow_tree_build.py` updated for the new wiring.
- **2026-06-10** — Add the follow-person behaviour tree. `BtNode_TrackPersonAction`
  now exposes `track/reacquisition_state` on the blackboard. New
  `FollowPerson/` package: `CoalescingTTS` (non-overlapping latest-wins speaker),
  `BtNode_ReacqAnnounce` (reacq-driven announcements), `BtNode_PublishFollowGoal`
  (publishes `/follow_target`), `create_follow_person_tree`, and a `cli`. Added
  the standalone `dummy_nav_node`, the `follow_process.launch.py` launch file,
  and `dummy-nav` / `follow-person` console-script entry points.

## 📄 License

Apache 2.0

## 👥 Maintainers

- Cindy (cindy.w0135@gmail.com)

## 🙏 Acknowledgments

- RoboCup@Home competition
- Tinker team
- py_trees library
- ROS2 community

## 📖 References

- [py_trees Documentation](https://py-trees.readthedocs.io/)
- [RoboCup@Home Rules](https://athome.robocup.org/)
- [ROS2 Documentation](https://docs.ros.org/)

## Changelog

- **2026-06-11** — feat(restaurant): both kitchen-bar returns (Phase-2 barman
  trip and the per-item Phase-3 pickup verification) now drive through goal
  projection. A new `BtNode_ProjectPose` ServiceHandler calls
  `find_approach_pose` with `projection_mode=ANCHOR_NEAREST_FREE` (3) — the raw
  operator-placed bar anchor becomes `target.point`, its yaw becomes
  `preferred_yaw_rad`, and the server returns the anchor unchanged when free or
  the nearest footprint-free cell (preserving yaw) when blocked. Each bar trip
  is now `Selector( Sequence(ProjectPose(bar→projected), Goto(projected)),
  Goto(bar) )` inside its existing `Retry`: if the projection service is
  unavailable or fails, the Selector degrades gracefully to today's raw-anchor
  `BtNode_GotoAction`. Mock mode is a pass-through copy (in-key→out-key, no
  service contacted), so existing mock restaurant trees behave unchanged.

- **2026-06-10** — fix(restaurant): the approach-customer `BtNode_Approach`
  now sets `action_timeout_ticks=220` (110 s at the 500 ms restaurant tick).
  Previously 0 (disabled) — combined with the feedback callback refreshing
  `feedback_timeout` every frame, a hung `go_to_approach` server blocked the
  whole tree indefinitely. Pairs with the approach_planner timeout reduction
  to 25 s/75 s (see that package's changelog, same date).
