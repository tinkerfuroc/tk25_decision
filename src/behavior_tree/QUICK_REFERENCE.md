# Mock Mode v2.0 - Quick Reference

## 🎯 What Changed

### Before (v1.x) - Manual Mock Logic
```python
# Every script needed conditional logic
from behavior_tree.config import is_mock_mode
from behavior_tree.TemplateNodes.WaitKeyPress import BtNode_WaitKeyboardPress

MOCK_MODE = is_mock_mode()

def create_tree():
    root = py_trees.composites.Sequence("Task", memory=True)
    
    # Conditionals everywhere
    if not MOCK_MODE:
        nav = BtNode_GotoAction("go to sofa", KEY_SOFA_POSE)
    else:
        nav = BtNode_WaitKeyboardPress("MOCK: go to sofa", 's')
    
    root.add_child(nav)
    return root
```

### After (v2.0) - Automatic Mock Support
```python
# Just use template nodes directly
def create_tree():
    root = py_trees.composites.Sequence("Task", memory=True)
    
    # Node automatically checks mock_config.json
    nav = BtNode_GotoAction("go to sofa", KEY_SOFA_POSE)
    
    root.add_child(nav)
    return root
```

## 📋 Configuration

### mock_config.json Structure
```json
{
  "mock_mode": {
    "enabled": true,           // Global mock mode on/off
    "auto_detect": true,       // Auto-enable if packages missing
    "subsystems": {
      "vision": {
        "enabled": true,       // Mock this subsystem
        "announce_movement": false,  // TTS announcement
        "nodes": ["BtNode_ScanFor", "BtNode_TrackPerson", ...]
      },
      "manipulation": { ... },
      "navigation": { ... },
      "audio_input": { ... },
      "announcement": { ... }
    }
  },
  "keyboard_control": {
    "enabled": true            // Step-through with 's' key
  },
  "logging": {
    "print_mock_operations": true,
    "use_emoji": true
  }
}
```

## 🚀 Quick Commands

```bash
# Auto-detect mode (default)
ros2 run behavior_tree receptionist_secondcall

# Force mock mode
export BT_MOCK_MODE=true
ros2 run behavior_tree receptionist_secondcall

# Force real hardware
export BT_MOCK_MODE=false
ros2 run behavior_tree receptionist_secondcall

# Use custom config
export BT_MOCK_CONFIG=/path/to/config.json
ros2 run behavior_tree receptionist_secondcall
```

## 🎛️ Subsystem Control

### Mixed Real/Mock Example
Want to test with real TTS but mock everything else?

Edit `mock_config.json`:
```json
{
  "subsystems": {
    "vision": {"enabled": true},       // MOCK
    "manipulation": {"enabled": true}, // MOCK
    "navigation": {"enabled": true},   // MOCK
    "audio_input": {"enabled": true},  // MOCK
    "announcement": {"enabled": false} // REAL - use actual TTS
  }
}
```

## 🔑 Key Features

| Feature | Description | Configure |
|---------|-------------|-----------|
| **Subsystem Mocking** | Mock entire subsystems | `mock_config.json` |
| **Keyboard Control** | Step through with 's' | `keyboard_control.enabled` |
| **TTS Announcements** | Hear node names | `announce_movement: true` |
| **Auto-Detection** | Auto-enable if deps missing | `auto_detect: true` |
| **Environment Override** | Force mode via env var | `export BT_MOCK_MODE=true` |

## 📊 Node Coverage

### Updated Nodes (30+)
- ✅ **Audio** (9): TTSCN, Announce, WaitForStart, Listen, PhraseExtraction, etc.
- ✅ **Vision** (9): ScanFor, TrackPerson, FindObj, FeatureExtraction, etc.
- ✅ **Manipulation** (7): Grasp, Place, MoveArmJoint, GripperAction, etc.
- ✅ **Navigation** (4): GotoAction, FollowPerson, ConvertGraspPose, etc.

All nodes automatically check configuration at runtime!

## 🐛 Troubleshooting

### Script waits indefinitely
➡️ **Solution:** Press 's' and Enter in the terminal (keyboard control is enabled)

### TTS segmentation fault
➡️ **Solution:** Set `announce_movement: false` for all subsystems

### "Import could not be resolved" in IDE
➡️ **Expected:** Code runs fine using mock implementations

### Subsystem won't use real hardware
➡️ **Check:** `unset BT_MOCK_MODE` (env var overrides config)

## 📝 Adding New Nodes

1. Inherit from ServiceHandler or ActionHandler
2. Add mock mode detection:
   ```python
   from behavior_tree.config import is_node_mocked
   
   class BtNode_MyNode(ServiceHandler):
       def __init__(self, name: str):
           super().__init__(...)
           self.mock_mode = is_node_mocked(self.__class__.__name__)
   ```
3. Add to `mock_config.json`:
   ```json
   {
     "subsystems": {
       "your_subsystem": {
         "nodes": ["BtNode_MyNode"]
       }
     }
   }
   ```

## 🎉 Benefits

- 🎯 **Zero Code Changes** - Existing scripts work unchanged
- 🔧 **Fine-Grained Control** - Mock only what you need
- ⌨️ **Better Debugging** - Step through execution
- 📢 **Audio Feedback** - Hear what's happening (optional)
- 🤖 **Consistent Behavior** - All nodes follow same pattern
- 🚀 **Easy Testing** - No hardware needed for development

---

**Full documentation:** See [README.md](README.md)  
**Change history:** See [CHANGELOG.md](CHANGELOG.md)
