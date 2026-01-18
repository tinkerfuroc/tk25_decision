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
        "nodes": ["BtNode_GotoAction", "BtNode_FollowPerson", ...]
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
