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
- `tinker_vision_msgs` - Vision services/actions
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
