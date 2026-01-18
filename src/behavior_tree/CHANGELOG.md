# Changelog

## [2.0.0] - 2026-01-18

### 🎉 Major Release - Complete Mock Mode Restructuring

This release completely overhauls the mock mode system to provide subsystem-level control, keyboard-based step-through execution, and zero-code-change compatibility.

### ✨ New Features

#### JSON-Based Configuration System
- **`mock_config.json`** - Central configuration file for subsystem-level mock control
- **5 Subsystems** - Independently control vision, manipulation, navigation, audio_input, and announcement
- **Per-Node Configuration** - Each node listed under its subsystem with granular control
- **Announce Movement** - Optional TTS announcements for each subsystem (using pyttsx3)
- **Auto-Detection** - Automatically enables mock mode when Tinker packages are unavailable

#### Keyboard Control
- **Step-Through Execution** - Press 's' to advance through each mock action
- **Non-Blocking Input** - Uses termios/tty for proper terminal handling
- **Configurable** - Can be disabled via `keyboard_control.enabled` in config

#### TTS Announcements (Optional)
- **pyttsx3 Integration** - Native Python text-to-speech for mock actions
- **Per-Subsystem Control** - Enable/disable via `announce_movement` flag
- **Error Handling** - Automatically falls back to print if TTS fails
- **Reusable Engine** - Global TTS engine instance to avoid segfaults

#### Configuration System
- **`config.py`** - Enhanced configuration loader with helper functions
- **Priority System** - ENV variable > JSON config > auto-detection
- **Helper Functions**:
  - `is_mock_mode()` - Check global mock status
  - `is_subsystem_mocked(name)` - Check specific subsystem
  - `is_node_mocked(class_name)` - Check specific node
  - `should_announce_movement(class_name)` - Check TTS setting
  - `announce_node_action(name, class_name)` - Announce via TTS
- **Status Printing** - `print_status()` shows complete configuration

### 🔧 Updated Components

#### Template Nodes (30+ nodes updated)
All template nodes now have native mock support with automatic configuration checking:

**Audio.py** (9 nodes)
- BtNode_TTSCN, BtNode_Announce
- BtNode_WaitForStart, BtNode_GraspRequest
- BtNode_PhraseExtraction, BtNode_TargetExtraction
- BtNode_GetConfirmation, BtNode_Listen, BtNode_CompareInterest

**Vision.py** (9 nodes)
- BtNode_ScanFor, BtNode_TrackPerson, BtNode_FindObj
- BtNode_FeatureExtraction, BtNode_FeatureMatching
- BtNode_SeatRecommend, BtNode_GetPointCloud
- BtNode_DoorDetection, BtNode_TurnPanTilt

**Manipulation.py** (7 nodes)
- BtNode_Grasp, BtNode_Place, BtNode_Drop
- BtNode_MoveArmJoint, BtNode_MoveArmSingle
- BtNode_GripperAction, BtNode_PointTo

**Navigation.py** (4 nodes)
- BtNode_GotoAction, BtNode_FollowPerson
- BtNode_ConvertGraspPose, BtNode_GoToLuggage

#### Base Classes
- **BaseBehaviors.py** - Updated ServiceHandler with:
  - Node-specific mock detection via `is_node_mocked()`
  - `wait_for_keypress_in_mock()` helper
  - TTS announcement integration
  - Mock service call handling

- **ActionBase.py** - Updated ActionHandler with:
  - Same mock detection pattern as ServiceHandler
  - Mock action client and futures
  - Mock result messages
  - Keyboard control integration

### 📝 Scripts Updated

#### receptionist_2ndcall.py
- Removed `MOCK_MODE` constant
- Navigation nodes use unified configuration
- No more conditional node creation
- All mock behavior controlled via `mock_config.json`

### 🗑️ Deprecated/Removed

#### Removed Files
- `MOCK_CONFIG_GUIDE.md` - Merged into README.md
- `MOCK_MODE_README.md` - Merged into README.md
- `RESTRUCTURING_SUMMARY.md` - Merged into README.md
- `NODES_FIXED_SUMMARY.md` - Merged into README.md
- `QUICKSTART.md` - Merged into README.md
- `CHANGES_SUMMARY.md` - Replaced by this CHANGELOG.md

#### Deprecated Patterns
- ❌ Manual `MOCK_MODE` checks in behavior tree scripts
- ❌ Conditional node creation (real vs mock)
- ❌ `BtNode_WaitKeyboardPress` as mock replacement
- ✅ Now: Use template nodes directly, they handle mocking automatically

### 🔄 Migration Path

No breaking changes! All existing scripts work without modification. However, you can clean up old patterns:

**Before (v1.x):**
```python
MOCK_MODE = is_mock_mode()
if not MOCK_MODE:
    node = BtNode_GotoAction("go somewhere", "pose_key")
else:
    node = BtNode_WaitKeyboardPress("MOCK: go somewhere", 's')
root.add_child(node)
```

**After (v2.0):**
```python
# Just use the node - mocking automatic
node = BtNode_GotoAction("go somewhere", "pose_key")
root.add_child(node)
```

### 📚 Documentation

- **README.md** - Completely rewritten with:
  - What's New section
  - Migration guide
  - Comprehensive configuration documentation
  - Updated examples and troubleshooting
- **CHANGELOG.md** - This file!

### 🐛 Bug Fixes

- Fixed navigation nodes not following unified mock pattern
- Fixed pyttsx3 segmentation faults with reusable engine instance
- Fixed AttributeError when action_client is None in mock mode
- Fixed TTS announcement calls with proper error handling

### 🎯 Configuration Priority

The system now uses a clear priority order:
1. `BT_MOCK_MODE` environment variable (highest priority)
2. `mock_config.json` settings
3. Auto-detection based on available packages (lowest priority)

### 📊 Statistics

- **Files Modified**: 15+ core files
- **Nodes Updated**: 30+ template nodes
- **Lines Added**: ~2000+ lines of new functionality
- **Documentation**: 1 comprehensive README, 1 CHANGELOG
- **Backwards Compatible**: 100% - no breaking changes

### 🙏 Acknowledgments

Special thanks to the Tinker team and RoboCup@Home community for testing and feedback.

---

## [1.x] - Previous Versions

Earlier versions with basic mock mode support. See git history for details.
