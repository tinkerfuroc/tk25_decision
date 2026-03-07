# Testing the Follow Behavior Tree

This document describes how to test the Follow behavior tree for the HelpMeCarry task.

## Overview

The Follow behavior tree uses:
- **TrackPerson action** - Tracks a person using the orbbec camera
- **NavigateToPose action** - Navigates the robot to follow the tracked person
- **TF transforms** - Transforms person position from camera frame to map frame

## Prerequisites

1. Build the workspace:
   ```bash
   cd ~/Documents/tk25_ws
   colcon build --packages-select behavior_tree tinker_vision_msgs_26
   source install/setup.bash
   ```

2. Ensure the following packages are available:
   - `py_trees`
   - `py_trees_ros`
   - `nav2_msgs`
   - `tinker_vision_msgs_26`
   - `tf2_ros`

## Test Setup

### Terminal 1: Start Mock Navigation Server

The mock navigation server simulates Nav2's NavigateToPose action server and publishes the necessary TF transforms.

```bash
cd ~/Documents/tk25_ws
source install/setup.bash

# Run the mock navigation server (using ros2 run after colcon build)
ros2 run behavior_tree hmc-mock-nav

# Or run directly with python3:
# python3 src/tk25_decision/src/behavior_tree/behavior_tree/HelpMeCarry/test/mock_nav_server.py
```

**Parameters:**
- `linear_speed` (default: 0.5 m/s) - Simulated robot movement speed
- `angular_speed` (default: 1.0 rad/s) - Simulated robot rotation speed
- `goal_tolerance` (default: 0.1 m) - Distance to consider goal reached

The server publishes these transforms:
- `map` → `odom` (static, identity)
- `odom` → `base_link` (dynamic, robot position)
- `base_link` → `camera_link` (static, camera mount position)
- `camera_link` → `camera_color_optical_frame` (static, optical frame convention)

### Terminal 2: Start Mock Tracking Server

The mock tracking server simulates the TrackPerson action server.

```bash
cd ~/Documents/tk25_ws
source install/setup.bash

# Run the mock tracking server with default settings (person walks forward)
ros2 run behavior_tree hmc-mock-track

# Or with different movement patterns:
ros2 run behavior_tree hmc-mock-track --ros-args -p movement_pattern:=stationary
ros2 run behavior_tree hmc-mock-track --ros-args -p movement_pattern:=stop_after_5s
ros2 run behavior_tree hmc-mock-track --ros-args -p movement_pattern:=circle

# Or run directly with python3:
# python3 src/tk25_decision/src/behavior_tree/behavior_tree/HelpMeCarry/test/mock_track_server.py --ros-args -p movement_pattern:=walk_forward
```

**Parameters:**
- `feedback_rate` (default: 10.0 Hz) - How often to publish tracking feedback
- `initial_person_x` (default: 2.0 m) - Initial person distance from camera
- `person_speed` (default: 0.3 m/s) - How fast the person moves
- `movement_pattern` - Movement pattern options:
  - `walk_forward` - Person walks away from robot
  - `walk_left` - Person walks to the left
  - `stationary` - Person stands still
  - `stop_after_5s` - Person walks for 5s then stops
  - `circle` - Person walks in a circle
  - `random` - Random walk

### Terminal 3: Run the Follow Test

```bash
cd ~/Documents/tk25_ws
source install/setup.bash

# Run full follow test (with confirmation)
ros2 run behavior_tree hmc-test-follow --ros-args -p mode:=full

# Or test just tracking (no navigation)
ros2 run behavior_tree hmc-test-follow --ros-args -p mode:=track_only

# Or test follow until stopped (no confirmation)
ros2 run behavior_tree hmc-test-follow --ros-args -p mode:=until_stopped

# Or run directly with python3:
# python3 src/tk25_decision/src/behavior_tree/behavior_tree/HelpMeCarry/test/test_follow.py full
```

### Terminal 4 (Optional): Monitor TF and Topics

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Echo person position in map frame
ros2 topic echo /tf

# Check action servers
ros2 action list
ros2 action info /track_person
ros2 action info /navigate_to_pose
```

## Test Scenarios

### Scenario 1: Basic Following
1. Start mock nav server (Terminal 1)
2. Start mock track server with `walk_forward` pattern (Terminal 2)
3. Run `test_follow.py until_stopped` (Terminal 3)
4. **Expected:** Robot follows the person until they stop or move out of range

### Scenario 2: Stop Detection
1. Start mock nav server (Terminal 1)
2. Start mock track server with `stop_after_5s` pattern (Terminal 2)
3. Run `test_follow.py until_stopped` (Terminal 3)
4. **Expected:** Robot follows for 5 seconds, then detects person has stopped (SUCCESS after stationary_duration)

### Scenario 3: Full HelpMeCarry Flow
1. Start mock nav server (Terminal 1)
2. Start mock track server with `stop_after_5s` pattern (Terminal 2)
3. Start mock audio services (if available)
4. Run `test_follow.py full` (Terminal 3)
5. **Expected:** Robot follows, asks confirmation when person stops, completes if confirmed

### Scenario 4: Transform Validation
1. Start mock nav server (Terminal 1)
2. Start mock track server (Terminal 2)
3. Check TF is working: `ros2 run tf2_ros tf2_echo map camera_link`
4. Run `test_follow.py track_only` (Terminal 3)
5. **Expected:** Feedback shows positions in map frame with `is_transformation_successful=True`

## Troubleshooting

### "Action server not available"
- Ensure mock servers are running before starting the test
- Check action is advertised: `ros2 action list`

### "Transform failed"
- Check TF tree: `ros2 run tf2_tools view_frames`
- Ensure mock nav server is publishing transforms
- Verify frames exist: `ros2 run tf2_ros tf2_echo map camera_link`

### "No position data available"
- Check mock track server is sending feedback
- Verify action goal was accepted

### Behavior tree stuck in RUNNING
- Check mock servers are responsive
- Look at feedback messages in test output
- Verify navigation goals are being sent and received

## Notes on 250ms Tick Rate

The behavior tree ticks at 250ms intervals (4 Hz). The TrackPerson action may send feedback faster than this (e.g., 10 Hz). The `FeedbackBuffer` class in `Follow.py` accumulates all feedback messages between ticks and provides the latest valid position when the tree ticks.

This means:
- Multiple feedback messages may arrive between ticks
- Only the latest valid position (with successful transform) is used
- Statistics on feedback count are available in the node's feedback message

## Integration with Real System

To use with the real robot:
1. Replace mock servers with real Nav2 and TrackPerson action servers
2. Ensure camera TF is published correctly
3. Verify transform from camera frame to map frame is available
4. Test with real person tracking before full HelpMeCarry task
