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

from behavior_tree.interfaces import messages
from behavior_tree.interfaces import mock_messages


ACTION_FIELDS = {
    "DetectWavingAction": {
        "Goal": {"threshold_meters", "target_frame", "min_waving_persons"},
        "Result": {
            "status",
            "error_msg",
            "waving_persons",
            "waving_boxes",
            "rgb_image",
            "depth_image",
            "segments",
        },
        "Feedback": {"status", "delay_limit", "stage", "message"},
    },
    "FeatureExtractionAction": {
        "Goal": {"camera"},
        "Result": {"status", "error_msg", "feature", "comparison_image"},
        "Feedback": {"status", "delay_limit", "stage", "message"},
    },
    "SeatRecommendationAction": {
        "Goal": {"camera", "names", "features"},
        "Result": {"status", "error_msg", "recommendation"},
        "Feedback": {"status", "delay_limit", "stage", "message"},
    },
    "FeatureMatchingAction": {
        "Goal": {
            "camera",
            "features",
            "comparison_images",
            "max_distance",
            "target_frame",
        },
        "Result": {"status", "error_msg", "centroids"},
        "Feedback": {"status", "delay_limit", "stage", "message"},
    },
    "SeatRecommendBboxAction": {
        "Goal": {"camera", "names", "features", "target_frame", "known_seats"},
        "Result": {"status", "error_msg", "recommendation", "bbox", "centroid"},
        "Feedback": {"status", "delay_limit", "stage", "message"},
    },
    "ObjectScanAction": {
        "Goal": {"camera", "vocabulary"},
        "Result": {"header", "status", "error_msg", "found_labels"},
        "Feedback": {"status", "delay_limit", "stage", "message"},
    },
}


def test_messages_retains_services_and_exposes_action_aliases():
    for service_name, action_name in (
        ("DetectWaving", "DetectWavingAction"),
        ("FeatureExtraction", "FeatureExtractionAction"),
        ("SeatRecommendation", "SeatRecommendationAction"),
        ("FeatureMatching", "FeatureMatchingAction"),
        ("SeatRecommendBbox", "SeatRecommendBboxAction"),
        ("ObjectScan", "ObjectScanAction"),
    ):
        service_type = getattr(messages, service_name)
        action_type = getattr(messages, action_name)
        assert service_type is not action_type
        assert hasattr(service_type, "Request")
        assert hasattr(service_type, "Response")
        assert hasattr(action_type, "Goal")
        assert hasattr(action_type, "Result")
        assert hasattr(action_type, "Feedback")


def test_mock_vision_action_fields_match_definitions_exactly():
    for action_name, parts in ACTION_FIELDS.items():
        action_type = getattr(mock_messages, action_name)
        assert issubclass(action_type, mock_messages.MockAction)
        for part_name, expected_fields in parts.items():
            message = getattr(action_type, part_name)()
            assert set(vars(message)) == expected_fields, (
                action_name,
                part_name,
                set(vars(message)),
            )
