# Changelog

## [2.2.2] - 2026-04-19

### 🧪 Standalone intro harness + mock-config catch-up

- New `HRI/intro.py` + `hri-intro` console script. Seeds two mock guest
  profiles on the blackboard and runs just `createTwoWayIntroduction`, so
  the feature-match → pre-orient → announce path can be KEYPRESS-stepped
  end-to-end without the full HRI task. Mirrors `hri-follow` / `hri-intake`.
- `mock_config.json` — added missing class-name registrations so the new
  harness runs under `BT_MOCK_MODE=true`:
    - `announcement` subsystem: `BtNode_Introduce`, `BtNode_Confirm`.
    - `vision` subsystem: `BtNode_TurnTo` (was inheriting from
      `BtNode_TurnPanTilt` but class-name lookup doesn't walk the MRO).
- `HRI/hri.py` — `_with_gaze_supervisor` migrated from the deprecated
  `BtNode_HeadTrackingAction` shim to `BtNode_MaintainEyeContact` directly,
  per the migration path documented in `.claude/rules/behavior-tree.md`.
  Removes a DeprecationWarning on every HRI boot.

---

## [2.2.1] - 2026-04-19

### ✨ Target-guided gaze during HRI introductions

Partially fixes the rulebook 5.1 "look to the correct guest while talking
about the other guest" 2×50 pts item. Previously the parallel-sibling
`BtNode_MaintainEyeContact` locked onto whichever face was geometrically
closest to the base — a coin flip for which of two seated guests got the
right intro gaze.

- `BtNode_FeatureMatching` — added `trim_last_person: bool = True` kwarg.
  Default preserves Receptionist's "new guest just walked up, not seated
  yet" behavior; `False` sends every persons's features so centroids come
  back for all seated guests. Mock path now emits one fake centroid per
  (non-trimmed) person instead of a single element.
- `HRI/config.py` — new `KEY_PERSON_CENTROIDS` blackboard key.
- `HRI/hri.py:createTwoWayIntroduction` — runs feature matching once with
  `trim_last_person=False` to populate `KEY_PERSON_CENTROIDS`, then each
  intro now does `BtNode_TurnTo(target_id)` → `BtNode_Introduce`
  inside the existing gaze supervisor. Physically turning the head first
  makes the target guest the closest face, so `MaintainEyeContact`
  locks on the correct one.
- Best-effort wrapping (`FailureIsSuccess` around feature matching and
  turn-to) ensures the intro still fires if the scan fails — falls back
  to previous closest-face behavior without crashing the tree.

---

## [2.2.0] - 2026-04-19

### ✨ New — Action-based phrase extraction integrated into HRI intake

Migrates the HRI name/drink capture flow from a confirmation-heavy
service pipeline to a high-confidence-first action pipeline, to earn the
RoboCup@Home 2026 rulebook's **4×15 "no non-essential questions"** bonus
when the ASR cross-check agrees.

#### New template node
- `BtNode_PhraseExtractionAction` (`TemplateNodes/Audio.py`) wraps
  `tk_24_audio`'s new `phrase_extraction_action`
  (`tinker_audio_msgs/action/PhraseExtraction`). The server runs Whisper
  + Qwen ASR sequentially and calls `goal_handle.succeed()` only on
  server-status=0 (both engines agreed on the same wordlist entry);
  every other status calls `goal_handle.abort()`. Terminal
  `STATUS_SUCCEEDED` is therefore a high-confidence signal.

#### HRI integration
- `HRI/hri.py:_create_get_info` rewritten as
  `Selector(Retry(2, primary), last-resort-fallback)`:
  - **Primary** (up to 2 attempts): prompt → `BtNode_PhraseExtractionAction`.
    On success, no confirmation question is asked — banks the no-nonessential-questions bonus.
  - **Fallback** (one attempt, only when both primary attempts abort):
    prompt → legacy `BtNode_PhraseExtraction` → `BtNode_Confirm` →
    `BtNode_GetConfirmationAction`. Preserves partial-score coverage in
    noisy environments, at the cost of the bonus for that field.
- New `HRI/intake.py` + `hri-intake` console script — isolated harness
  for the name/drink intake subtree. Same pattern as `hri-follow`.

#### Messages / mock plumbing
- `messages.py` imports `PhraseExtraction as PhraseExtractionAction` with
  mock fallback.
- `mock_messages.py` adds `PhraseExtractionAction(MockAction)` stub.
- `mock_config.json` adds `"BtNode_PhraseExtractionAction": "KEYPRESS"`
  under `audio_input.nodes`.

#### Deprecations
- `BtNode_PhraseExtraction` (service-based) now emits
  `DeprecationWarning`, matching the `BtNode_Listen` /
  `BtNode_GetConfirmation` retirement policy. HRI is the first migrated
  consumer; Receptionist, Restaurant demo, GPSR, grasp-intel still call
  the legacy node and will migrate when next touched.

#### Fixes
- `HRI/config.py` — `arm_pos_point_to` re-added to `HRI/constants.json`
  so the eagerly-evaluated `.get(key, constants[other])` default in the
  arm-pose loader succeeds. (Unrelated pose-tuning edits to
  `HRI/constants.json` were made separately.)

### Files modified
- `behavior_tree/TemplateNodes/Audio.py`
- `behavior_tree/messages.py`
- `behavior_tree/mock_messages.py`
- `behavior_tree/mock_config.json`
- `behavior_tree/HRI/hri.py`
- `behavior_tree/HRI/intake.py` *(new)*
- `behavior_tree/HRI/constants.json` *(added `arm_pos_point_to` key)*
- `setup.py` *(new `hri-intake` console script)*

---

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

**Navigation.py** (3 nodes)
- BtNode_GotoAction, BtNode_ConvertGraspPose, BtNode_GoToLuggage

**HRI/follow.py** (8 nodes, action-based follow-person subtree)
- BtNode_TrackPersonAction, BtNode_FollowAction, BtNode_IsTargetVisible
- BtNode_UpdateLossElapsed, BtNode_LossElapsedAtLeast, BtNode_WriteBBIfVisible
- BtNode_FlagIsFalse, BtNode_SetFlag
- `createFollowPerson(cfg)` factory; `hri-follow` console entry point

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
