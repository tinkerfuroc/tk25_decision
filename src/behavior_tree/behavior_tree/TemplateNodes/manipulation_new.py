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

"""Compatibility re-export shim for ``behavior_tree.TemplateNodes.manipulation_new``.

Several task modules (``DoingLaundry/laundry.py``, ``DoingLaundry/sampling.py``
and the ``PickAndPlace`` dev subtrees) import ``BtNode_JointMoveAction`` from
``behavior_tree.TemplateNodes.manipulation_new``. The canonical definitions live
in :mod:`behavior_tree.TemplateNodes.Manipulation`; this thin module re-exports
them so the historical import path keeps resolving (it was otherwise a missing
module that raised ``ModuleNotFoundError`` at import time, breaking the
``doing-laundry`` and ``pp-*`` entry points).

New code should import directly from ``behavior_tree.TemplateNodes.Manipulation``.
"""

from behavior_tree.TemplateNodes.Manipulation import *  # noqa: F401,F403
from behavior_tree.TemplateNodes.Manipulation import (  # noqa: F401
    BtNode_JointMoveAction,
)
