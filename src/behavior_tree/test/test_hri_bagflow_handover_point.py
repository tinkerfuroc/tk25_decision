# Copyright 2026 Tinker Team
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

"""Bag-flow handover pose: aimed at guest 2 like the intro pointing.

The hri-2026 bag flow must move the arm into the bag-handover pose before
opening the gripper, with joint0 computed from guest 2's centroid via the
same bearing math the two-way introduction uses (``BtNode_PointTo``,
``pan_bias=0.0``) and joints 1-6 from ``KEY_ARM_HANDOVER``. When the aim is
impossible (centroid missing, arm refuses) the canonical fixed handover pose
is the fallback, and the whole move is best-effort so an arm refusal cannot
forfeit the gripper handover + follow-to-drop scoring.
"""

import os

os.environ.setdefault("BT_MOCK_MODE", "true")

from behavior_tree.HRI.config import (  # noqa: E402
    KEY_ARM_HANDOVER,
    KEY_PERSONS,
    KEY_PERSON_CENTROIDS,
)
from behavior_tree.HRI.hri_2026 import createBagFlowReal2026  # noqa: E402


def _by_class(root, class_name):
    return [b for b in root.iterate() if b.__class__.__name__ == class_name]


def test_handover_point_aims_at_guest2_with_handover_joints():
    root = createBagFlowReal2026()
    points = _by_class(root, "BtNode_PointTo")
    assert len(points) == 1, "the bag flow has exactly one aimed arm move"
    node = points[0]
    # KEY_PERSONS layout after host scan + both intakes: [host, guest1, guest2]
    assert node.target_id == 2
    assert node.pan_bias == 0.0
    assert node.bb_key_persons == KEY_PERSONS
    assert node.bb_key_points == KEY_PERSON_CENTROIDS
    assert node.blackboard.remappings["/arm_joint_pose"] == "/" + KEY_ARM_HANDOVER


def test_handover_pose_precedes_gripper_open():
    root = createBagFlowReal2026()
    names = [child.name for child in root.children]
    pose_idx = names.index("Arm to handover pose + announce ready")
    open_idx = names.index("Open gripper for bag")
    assert pose_idx < open_idx


def test_fixed_handover_pose_is_the_fallback():
    root = createBagFlowReal2026()
    selector = next(b for b in root.iterate() if b.name == "Arm to bag-handover pose")
    assert selector.__class__.__name__ == "Selector"
    primary, fallback = selector.children
    assert _by_class(primary, "BtNode_PointTo"), "aimed pose is the primary"
    fixed = _by_class(fallback, "BtNode_MoveArmSingle")
    assert len(fixed) == 1, "fallback is the canonical fixed handover move"
    assert fixed[0].arm_pose_bb_key == KEY_ARM_HANDOVER


def test_handover_move_is_best_effort():
    root = createBagFlowReal2026()
    guard = next(
        b for b in root.iterate() if b.name == "Arm to bag-handover pose (best effort)"
    )
    assert guard.__class__.__name__ == "FailureIsSuccess"
