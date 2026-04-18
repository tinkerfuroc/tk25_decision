# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
TemplateNodes Package
=====================

This package provides reusable behavior tree node templates with built-in
mock mode support for the Tinker robot behavior tree system.

All template nodes inherit from either ServiceHandler or ActionHandler base
classes, which automatically handle mock mode detection and keyboard-based
interaction when running without real hardware.

Key Modules
-----------
- BaseBehaviors: Service-based node base class and blackboard utilities
- ActionBase: Action-based node base class for long-running operations
- Navigation: Navigation nodes (GoTo, FollowPerson, etc.)
- Manipulation: Arm control nodes (Grasp, Place, MoveArm, etc.)
- Vision: Vision nodes (ScanFor, TrackPerson, FeatureExtraction, etc.)
- Audio: Audio nodes (Announce, Listen, PhraseExtraction, etc.)
- TeleopNodes: Keyboard-based arm teleoperation for mock mode
- MockInputController: Shared keyboard input router for mock mode
- WaitKeyPress: Simple keyboard wait utility
- MockableNodes: Legacy wrapper for mock mode support
- structs: Data structures (Person class)
"""
