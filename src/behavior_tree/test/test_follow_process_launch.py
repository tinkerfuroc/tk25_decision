# Copyright 2026 Tinker
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
"""Static contract for the installed follow-person launch description."""

import ast
from pathlib import Path


LAUNCH_FILE = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "follow_process.launch.py"
)


def _node_targets():
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    targets = []
    for call in (node for node in ast.walk(tree) if isinstance(node, ast.Call)):
        function = call.func
        if not isinstance(function, ast.Attribute) or function.attr != "Node":
            continue
        keywords = {keyword.arg: keyword.value for keyword in call.keywords}
        targets.append(
            (
                ast.literal_eval(keywords["package"]),
                ast.literal_eval(keywords["executable"]),
            )
        )
    return targets


def test_follow_launch_contains_only_the_canonical_runner():
    assert _node_targets() == [("behavior_tree", "follow-person")]
