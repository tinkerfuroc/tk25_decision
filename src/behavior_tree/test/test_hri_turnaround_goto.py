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

"""Bag-flow turn-around: after handover, before the follow announcements.

The hri-2026 bag flow must drive to the reversed sofa pose (same point,
heading flipped 180 deg) after the bag is grasped and the arm is back at
the nav pose, and before the ``Look at host`` + follow announcements block.
``BtNode_GotoAction`` does not retain its key as an attribute, so the key is
pinned via the ``goal`` blackboard remap (see test_bar_return_projection.py).
"""

import os

os.environ.setdefault("BT_MOCK_MODE", "true")

import behavior_tree.HRI.hri as hri  # noqa: E402
from behavior_tree.HRI.config import (  # noqa: E402
    KEY_SOFA_POSE_REVERSED,
    POSE_SOFA_REVERSED,
)
from behavior_tree.HRI.hri_2026 import createBagFlowReal2026  # noqa: E402


def test_constant_writer_seeds_reversed_sofa_pose():
    writer = hri.createConstantWriter()
    writes = [
        b
        for b in writer.iterate()
        if getattr(b, "bb_key", None) == KEY_SOFA_POSE_REVERSED
    ]
    assert len(writes) == 1, "exactly one writer for the reversed sofa pose"
    assert writes[0].object is POSE_SOFA_REVERSED


def test_turnaround_sits_between_handover_and_follow_announcements():
    root = createBagFlowReal2026()
    names = [child.name for child in root.children]
    turn_idx = names.index("Turn around at sofa (best effort)")
    close_idx = names.index("Close gripper with bag")
    look_idx = names.index("Look at host")
    assert close_idx < turn_idx < look_idx
    assert turn_idx == look_idx - 1, (
        "turn-around must be the last step before the follow "
        "announcements block (Look at host + host announcements)"
    )


def test_turnaround_goto_targets_reversed_sofa_key():
    root = createBagFlowReal2026()
    gotos = [
        b for b in root.iterate() if b.__class__.__name__ == "BtNode_GotoAction"
    ]
    assert len(gotos) == 1, "the turn-around is the bag flow's only base nav"
    assert gotos[0].name == "Turn around at sofa"
    assert gotos[0].blackboard.remappings == {
        "/goal": "/" + KEY_SOFA_POSE_REVERSED
    }
