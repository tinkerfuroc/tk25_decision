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
"""Static contracts for the live task endpoint verifier."""

from behavior_tree.tools.verify_task_endpoints import TASK_SPECS


CONVERTED = {
    "/feature_extraction_service":
        "tinker_vision_msgs_26/action/FeatureExtraction",
    "/feature_matching_service":
        "tinker_vision_msgs_26/action/FeatureMatching",
    "/seat_recommend_service":
        "tinker_vision_msgs_26/action/SeatRecommendation",
    "/seat_recommend_bbox_service":
        "tinker_vision_msgs_26/action/SeatRecommendBbox",
    "/object_scan": "tinker_vision_msgs_26/action/ObjectScan",
    "/detect_waving_persons": "tinker_vision_msgs_26/action/DetectWaving",
}


def test_converted_endpoints_are_actions_not_services():
    services = {
        name
        for spec in TASK_SPECS.values()
        for name, _type in spec["services"]
    }
    actions = {
        name: interface
        for spec in TASK_SPECS.values()
        for name, interface in spec["actions"]
    }

    assert services.isdisjoint(CONVERTED)
    assert actions.items() >= CONVERTED.items()
