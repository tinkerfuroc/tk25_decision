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

"""The comfort prompt to both seated guests asks for a head-camera look.

The prompt opens createTwoWayIntroduction() and runs immediately before the
seated-guest BtNode_FeatureMatching scan; guests looking at the head camera
is what makes that scan reliable.
"""

import os

os.environ.setdefault("BT_MOCK_MODE", "true")

import behavior_tree.HRI.hri as hri  # noqa: E402


def test_comfort_prompt_includes_head_camera_request():
    root = hri.createTwoWayIntroduction()
    announces = [
        b for b in root.iterate() if b.name == "Complete escort announcement"
    ]
    assert len(announces) == 1
    assert announces[0].given_msg == (
        "Please sit down and make yourself comfortable. "
        "Please look at my head camera and remain seated."
    )
