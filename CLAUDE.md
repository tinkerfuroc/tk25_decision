# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2 behavior tree package for the Tinker robot competing in RoboCup@Home. It implements decision-making logic for various competition tasks using the `py_trees` library. The package features a sophisticated mock mode system that allows testing behaviors without any hardware or Tinker-specific ROS packages installed.

## Build Commands

```bash
# Build the package
colcon build --packages-select behavior_tree

# Source after building
source install/setup.bash
```

## Running Tasks

Tasks are run as ROS2 console scripts defined in `src/behavior_tree/setup.py`:

```bash
# Competition tasks
ros2 run behavior_tree receptionist
ros2 run behavior_tree store-groceries
ros2 run behavior_tree help-me-carry
ros2 run behavior_tree GPSR
ros2 run behavior_tree EGPSR
ros2 run behavior_tree restaurant
ros2 run behavior_tree serve-breakfast
ros2 run behavior_tree inspection

# Development/test scripts
ros2 run behavior_tree test-mock-mode
ros2 run behavior_tree draw  # Visualize behavior tree as DOT/PNG/SVG
```

## Mock Mode

The package automatically enables mock mode when Tinker packages are unavailable. Control it via:

- **Environment variable**: `export BT_MOCK_MODE=true` (force on) or `false` (force off)
- **JSON config**: Edit `src/behavior_tree/behavior_tree/mock_config.json`
- **Priority**: `BT_MOCK_MODE` env var > JSON config > auto-detect

Mock interaction modes per node:
- `KEYPRESS`: Wait for 's' key press before succeeding
- `TELEOP`: Use teleoperation-style keyboard control (for manipulation nodes)
- `IMMEDIATE`: Return success immediately

### Mock Configuration

Edit `mock_config.json` to control subsystems independently:

```json
{
  "mock_mode": {
    "subsystems": {
      "vision": {"enabled": true},
      "manipulation": {"enabled": true},
      "navigation": {"enabled": true},
      "audio_input": {"enabled": true},
      "announcement": {"enabled": false}  // Use real TTS
    }
  }
}
```

## Architecture

### Core Files

- `config.py` - Singleton configuration manager, dependency detection, TTS announcement system
- `messages.py` / `mock_messages.py` - Conditional imports for ROS message types
- `main.py` - Entry point functions that create trees and run the ROS2 spin loop
- `visualization.py` - Post-tick handler for tree visualization in terminal

### TemplateNodes/

Base classes with built-in mock support. All nodes check `is_node_mocked()` at construction:

- `BaseBehaviors.py` - `ServiceHandler` (service-based nodes), blackboard utilities
- `ActionBase.py` - `ActionHandler` (action-based nodes like navigation)
- `Navigation.py` - GoTo, FollowPerson, etc.
- `Manipulation.py` - Grasp, Place, MoveArm, Gripper, etc.
- `Vision.py` - ScanFor, TrackPerson, FindObj, FeatureExtraction, etc.
- `Audio.py` - Announce, Listen, PhraseExtraction, etc.
- `TeleopNodes.py` - Keyboard-based arm teleoperation for mock mode
- `MockInputController.py` - Shared keyboard input router for mock mode

### Task Directories

Each task has its own directory under `behavior_tree/`:
- `Receptionist/` - Receptionist task implementation
- `GPSR/` - General Purpose Service Robot
- `StoringGroceries/` - Store Groceries task
- `HelpMeCarry/` - Help Me Carry task
- `Restaurant/` - Restaurant task
- etc.

### Blackboard Keys

Nodes use py_trees blackboard for inter-node communication. Common patterns:
- Nodes write results to namespaced keys (e.g., `persons`, `guest_name`)
- Use `BtNode_WriteToBlackboard` to pass data between nodes
- Check `BtNode_CheckIfEmpty` for conditional execution

## Creating New Nodes

1. Inherit from `ServiceHandler` or `ActionHandler`
2. Set `self.mock_mode = is_node_mocked(self.__class__.__name__)`
3. Override `update()` for real behavior, call `wait_for_keypress_in_mock()` for mock fallback
4. Add node class name to `mock_config.json` under appropriate subsystem

## Creating New Tasks

1. Create directory under `behavior_tree/YourTask/`
2. Implement `createYourTask()` function returning a `py_trees.composites.Sequence`
3. Add entry point in `setup.py`: `'your-task = behavior_tree.YourTask.your_task:main'`
4. Rebuild with `colcon build --packages-select behavior_tree`

## Dependencies

Core (always required):
- `py_trees`, `py_trees_ros` - Behavior tree library
- `rclpy` - ROS2 Python client

Tinker packages (optional - auto-mocked if missing):
- `tinker_vision_msgs_26` - Vision services/actions (canonical; tk23's `tinker_vision_msgs` is retired)
- `tinker_arm_msgs` - Arm control services/actions
- `tinker_audio_msgs` - Speech recognition/announcement services
- `tinker_nav_msgs` - Navigation services
- `nav2_msgs`, `control_msgs`, `action_msgs` - Standard ROS2 messages

## Testing

Run tests with pytest:
```bash
pytest src/behavior_tree/test/
```

Or run the mock mode test script:
```bash
ros2 run behavior_tree test-mock-mode
```

## Constants Files

Tasks like Receptionist require `constants.json` with pose data and task-specific parameters. The file path is hardcoded in each task file (e.g., `receptionist.py` line 34). Ensure this file exists on the target system.

## Known issues & broken nodes

Surfaced during the 2026-04-22 tk26_vision follow-up work. These are *pre-existing* behavior gaps — they have existed since the tk23→tk26 detection migration — that the srv-type migration in that session made visible but did not cause.

### Nodes broken against current tk26 detection services

| Node | Defined | Broken because | Live use? | Fix |
|---|---|---|---|---|
| `BtNode_TrackPerson` | `TemplateNodes/Vision.py:194` | Depends on `flags="register_person"` + `result.person_id` for persistent-ID tracking. tk26 detection nodes silently ignore the flag and always return `person_id=0`. | Only via dev/test entry points: `follow`, `test-track`, `follow-action`, `draw_follow` (through `HelpMeCarry/Track.py:createFollowPerson`). **Not used by the production `help-me-carry` task**, which uses `BtNode_TrackPersonAction` against `/track_person` (`vision_track/person_track_server`, action-based, real ReID). | Swap `HelpMeCarry/Track.py:15` to `BtNode_TrackPersonAction` against `/track_person`. Match the action-based pattern the production HelpMeCarry tree already uses. |
| `BtNode_ScanForWavingPerson` | `GPSR/custom_nodes.py:544` | Filters response for `obj.being_pointed == 3`; tk26 detection nodes never populate `being_pointed` (always 0). | **Live in `GPSR` task** (`gpsr_new.py:146`, unconditional). EGPSR already gates behind `USE_NEW_SCAN_WAVING=True` and uses the sibling `BtNode_ScanForWavingPersonNew`. | In `gpsr_new.py:146`, replace with `BtNode_ScanForWavingPersonNew` (defined `custom_nodes.py:618`). It calls `/detect_waving_persons` (`DetectWaving` srv, served by `tk_vision_specialized/waving_person_server.py`) and propagates the waving-person centroid correctly. EGPSR is a working reference. |
| `BtNode_FindPointedLuggage` | `HelpMeCarry/customNodes.py:16` | Branches on `obj.being_pointed == 1 or == 2` to classify left/right. Same issue — tk26 doesn't populate the field. | Live in the HelpMeCarry task (via `help_me_carry.py`). | Needs a dedicated gesture service (e.g. extend `DetectWaving`-style pattern) or a VLM prompt that encodes the left/right semantics in the text response rather than a numeric field. Not trivial; flag for re-scope. |

### Legacy `ObjectDetection.Request.flags` strings that are no-ops in tk26

The tk26 detection nodes (`object_seg_yolo.py:1080`+) parse only `sort_closest` / `sort_highest` / `sort_none` substrings from `request.flags`. Every other legacy string is accepted without error and silently ignored:

- `"scan"` (was `BtNode_ScanFor`, pre-migration)
- `"register_person"` (was `BtNode_TrackPerson`)
- `"find_for_grasp"` (was `BtNode_FindObj`, `BtNode_FindObjTable`, `anygrasp_test.py`)
- `"find_waving_person"` (was `BtNode_ScanForWavingPerson`)
- `"detect_gesture"` (was `BtNode_DetectCallingCustomer`)
- `"find_pointed_object"` (was `BtNode_FindPointedLuggage`)
- `"request_image"` / `"request_segments"` / `"request_segmentation"` — these now always return populated (tk26 doesn't gate on them).

Dropping these strings on migration is behavior-preserving, not a semantic change.

### Other gotchas

- **`BtNode_TrackPerson` import in `GPSR/gpsr.py:15`** — present but the one instantiation at line 88 is commented out. And `gpsr.py` is **not** the `GPSR` entry point (`gpsr_new.py` is, per `setup.py:48`). Cleaning up this import is harmless.
- **`messages.py:43` `TTSCnRequest` import failure** — on this workstation (2026-04-22), `from tinker_audio_msgs.srv import TTSCnRequest, ...` raises `ImportError: cannot import name 'TTSCnRequest'`. `tinker_audio_msgs` is installed but doesn't export that symbol; the `has_dependency('tinker_audio_msgs')` check returns True, so the mock fallback doesn't trigger. Importing `behavior_tree.messages` fails system-wide on this host. Either rebuild `tinker_audio_msgs` from a newer source or downgrade the import to a symbol-by-symbol try/except. Not caused by the tk26_vision follow-up — surfaced when the follow-up tried to verify BT imports.
- **Two distinct `BtNode_TrackPersonAction` definitions.** `TemplateNodes/TrackPersonAction.py:123` and `HelpMeCarry/Follow.py:115` both define a class with this name. `HelpMeCarry/help_me_carry.py:17` imports from `.Follow`; `HelpMeCarry/test/test_follow.py:26` imports from elsewhere. Confusing but not broken today; worth consolidating eventually.

