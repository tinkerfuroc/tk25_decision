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

#
# Vision Nodes Module
# ===================
#
# This module provides behavior tree nodes for robot vision operations.
# All nodes inherit from ServiceHandler and include built-in mock mode support.
#
# Classes
# -------
# BtNode_ScanFor
#     Scans for objects using YOLO-based detection.
# BtNode_TrackPerson
#     Tracks a specific person across frames using person ID.
# BtNode_FindObj
#     Finds objects for grasping with distance validation.
# BtNode_FeatureExtraction
#     Extracts visual features for person recognition.
# BtNode_LoadPersonReference
#     Loads a pre-recorded person photo + .txt description from disk and writes
#     them to the same blackboard keys BtNode_FeatureExtraction would write,
#     so downstream matching consumes pre-registered data with no changes.
# BtNode_SeatRecommend
#     Recommends a seat based on current scene.
# BtNode_SeatRecommendBbox
#     Seat recommendation that also returns bbox + 3D centroid of the seat.
# BtNode_FeatureMatching
#     Matches extracted features to known persons.
# BtNode_GetPointCloud
#     Retrieves a point cloud from the depth camera.
# BtNode_DoorDetection
#     Detects if a door is open or closed.
# BtNode_TurnPanTilt
#     Controls the pan-tilt unit for camera orientation.
# BtNode_TurnTo
#     Turns the camera to face a specific point/person.
#
# Mock Mode
# ---------
# All vision nodes support mock mode via the mock_config.json settings.
# In mock mode, they return simulated detection results or wait for
# keyboard input.
#
# Exception: BtNode_LoadPersonReference is pure local file I/O with no
# hardware dependency, so it performs the same real load whether or not
# the vision subsystem is mocked. It is registered in mock_config.json
# only for naming-convention completeness.
#

import asyncio
import os
import py_trees as pytree
import time
import math
from typing import Any, Optional

# from tinker_decision_msgs.srv import ObjectDetection
# from tinker_vision_msgs.srv import ObjectDetection

from behavior_tree.messages import ObjectDetection, ObjectDetectionGeneralist, Object, FeatureExtraction, SeatRecommendation, FeatureMatching, GetPointCloud, DoorDetection, PanTiltCtrl, BoundingBox, PanTiltCommand, PanTiltState, FollowHeadAction, SeatRecommendBbox, DetectWaving, PlacingLocation
from behavior_tree.config import is_node_mocked
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from py_trees.common import Status
import action_msgs.msg as action_msgs

from .BaseBehaviors import ServiceHandler
from .ActionBase import ActionHandler
from .structs import Person


class BtNode_ScanFor(ServiceHandler):
    """
    Scans for objects using YOLO-based detection.

    This node performs object detection using the specified camera and stores
    the detection results on the blackboard for use by subsequent nodes.
    """

    def __init__(self, 
                 name: str,
                 bb_source: str,
                 bb_key:str,
                 service_name : str = "object_detection_yolo",
                 object: str = None,
                 use_orbbec = True,
                 transform_to_map = False,
                 category = "detected objects"
                 ):
        """
        executed when creating tree diagram, therefor very minimal
        """
        super(BtNode_ScanFor, self).__init__(name, service_name, ObjectDetection)
        self.bb_key = bb_key
        self.bb_source = bb_source
        self.object = object
        self.use_orbbec = use_orbbec
        self.transform_to_map = transform_to_map
        self.read = True
        self.category = category
        if self.object is not None:
            self.read = False


    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)
        # attaches a blackboard (more like a shared memory section with key-value pair references) under the namespace Locations
        self.bb_write_client = self.attach_blackboard_client(name=f"ScanFor")
         # register a key with the name of the object, with this client having write access
        self.bb_write_client.register_key(self.bb_key, access=pytree.common.Access.WRITE)

        if self.read:
            self.bb_read_client = self.attach_blackboard_client(name="ScanFor Read")
            self.bb_read_client.register_key(self.bb_source, access=pytree.common.Access.READ)

            # debugger info (shown with DebugVisitor)
            self.logger.debug(f"Setup ScanFor, reading from {self.bb_source}")
        else:
            self.logger.debug(f"Setup ScanFor, for {self.object}")

    def initialise(self) -> None:
        """
        Called when the node is visited
        """
        super().initialise()

        # Handle mock mode
        if self.mock_mode:
            if self.read:
                try:
                    self.object = self.bb_read_client.get(self.bb_source)
                    assert isinstance(self.object, str)
                except Exception as e:
                    self.feedback_message = f"ScanFor reading object name failed"
                    raise e
            print(f"🔍 MOCK: Scanning for {self.object}")
            # Create mock detection result
            from behavior_tree.mock_messages import MockMessage
            mock_result = MockMessage()
            mock_result.status = 0
            mock_result.objects = []  # Empty list means object not found but service succeeded
            self.bb_write_client.set(self.bb_key, mock_result, overwrite=True)
            self.feedback_message = f"MOCK: Scanned for {self.object}"
            return
            
        if self.read:
            try:
                self.object = self.bb_read_client.get(self.bb_source)
                assert isinstance(self.object, str)
            except Exception as e:
                self.feedback_message = f"ScanFor reading object name failed"
                raise e

        request = ObjectDetection.Request()
        request.prompt = self.object
        request.flags = "scan"
        request.category = self.category
        if self.use_orbbec:
            request.camera = "orbbec"
        else:
            request.camera = "realsense"
        if self.transform_to_map:
            request.target_frame = "map"
        # setup things that needs to be cleared
        self.response = self.call_service_async(request)

        self.feedback_message = f"Initialized ScanFor for {self.object}"

    def update(self):
        # Handle mock mode
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
            
        self.logger.debug(f"Update ScanFor {self.object} with self.orbbec = {self.use_orbbec}")
        
        # Check if response exists
        if self.response is None:
            self.feedback_message = "No response object"
            return pytree.common.Status.FAILURE
            
        if self.response.done():
            if self.response.result().status == 0:
                self.bb_write_client.set(self.bb_key, self.response.result(), overwrite=True)
                self.feedback_message = f"Found object, stored to blackboard / {self.bb_key}"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Scanning for {self.object} failed with error code {self.response.result().status}: {self.response.result().error_msg}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still scanning..."
            return pytree.common.Status.RUNNING


class BtNode_ScanForGeneralist(ServiceHandler):
    """
    Scans for objects via the generalist detection service.

    Uses tk26 `tinker_vision_msgs_26/srv/ObjectDetectionGeneralist` (boolean
    flag schema) on `/object_detection_generalist`. Open-vocabulary prompts
    are supported through the YOLO-World / Gemini + FastSAM fallback when
    `use_vlm_sam_fallback=True`.
    """

    def __init__(self,
                 name: str,
                 bb_source: Optional[str],
                 bb_key: str,
                 service_name: str = "object_detection_generalist",
                 object: Optional[str] = None,
                 use_orbbec: bool = True,
                 transform_to_map: bool = False,
                 use_vlm_sam_fallback: bool = True,
                 force_vlm_sam: bool = False,
                 sort_closest: bool = True,
                 sort_highest: bool = False,
                 return_rgb_image: bool = False,
                 return_depth_image: bool = False,
                 return_segments: bool = True,
                 ):
        super(BtNode_ScanForGeneralist, self).__init__(name, service_name, ObjectDetectionGeneralist)
        self.bb_key = bb_key
        self.bb_source = bb_source
        self.object = object
        self.use_orbbec = use_orbbec
        self.transform_to_map = transform_to_map
        self.use_vlm_sam_fallback = use_vlm_sam_fallback
        self.force_vlm_sam = force_vlm_sam
        self.sort_closest = sort_closest
        self.sort_highest = sort_highest
        self.return_rgb_image = return_rgb_image
        self.return_depth_image = return_depth_image
        self.return_segments = return_segments
        self.read = self.object is None

    def setup(self, **kwargs):
        ServiceHandler.setup(self, **kwargs)
        self.bb_write_client = self.attach_blackboard_client(name="ScanForGeneralist")
        self.bb_write_client.register_key(self.bb_key, access=pytree.common.Access.WRITE)

        if self.read:
            if self.bb_source is None:
                self.read = False
                self.object = "object"
                self.logger.warning("no object or valid bb_souce given, using 'object'")
            self.bb_read_client = self.attach_blackboard_client(name="ScanForGeneralist Read")
            self.bb_read_client.register_key(self.bb_source, access=pytree.common.Access.READ)
            self.logger.debug(f"Setup ScanForGeneralist, reading from {self.bb_source}")
        else:
            self.logger.debug(f"Setup ScanForGeneralist, for {self.object}")

    def initialise(self) -> None:
        super().initialise()

        if self.mock_mode:
            if self.read:
                try:
                    self.object = self.bb_read_client.get(self.bb_source)
                    assert isinstance(self.object, str)
                except Exception as e:
                    self.feedback_message = "ScanForGeneralist reading object name failed"
                    raise e
            print(f"🔍 MOCK: Scanning (generalist) for {self.object}")
            from behavior_tree.mock_messages import MockMessage
            mock_result = MockMessage()
            mock_result.status = 0
            mock_result.objects = []
            self.bb_write_client.set(self.bb_key, mock_result, overwrite=True)
            self.feedback_message = f"MOCK: ScanForGeneralist for {self.object}"
            return

        if self.read:
            try:
                self.object = self.bb_read_client.get(self.bb_source)
                assert isinstance(self.object, str)
            except Exception as e:
                self.feedback_message = "ScanForGeneralist reading object name failed"
                raise e

        request = ObjectDetectionGeneralist.Request()
        request.prompt = self.object
        request.camera = "orbbec" if self.use_orbbec else "realsense"
        request.target_frame = "map" if self.transform_to_map else ""
        request.sort_closest = self.sort_closest
        request.sort_highest = self.sort_highest
        request.return_rgb_image = self.return_rgb_image
        request.return_depth_image = self.return_depth_image
        request.return_segments = self.return_segments
        request.force_vlm_sam = self.force_vlm_sam
        request.use_vlm_sam_fallback = self.use_vlm_sam_fallback


        self.response = self.call_service_async(request)
        self.feedback_message = f"Initialized ScanForGeneralist for {self.object}"

    def update(self):
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()

        if self.response is None:
            self.feedback_message = "No response object"
            return pytree.common.Status.FAILURE

        if self.response.done():
            result = self.response.result()
            if result.status == 0:
                self.bb_write_client.set(self.bb_key, result, overwrite=True)
                self.feedback_message = f"Generalist found objects, stored to {self.bb_key} (source={result.detection_source})"
                return pytree.common.Status.SUCCESS
            self.feedback_message = (
                f"ScanForGeneralist for {self.object} failed status={result.status}: {result.error_msg}"
            )
            return pytree.common.Status.FAILURE
        self.feedback_message = "Still scanning (generalist)..."
        return pytree.common.Status.RUNNING


class BtNode_TrackPerson(ServiceHandler):
    """
    Tracks a specific person across frames using person ID.

    This node registers and tracks a person, storing their position on the
    blackboard. On first execution, it registers a new person and obtains an ID.
    On subsequent executions, it tracks that person by ID.

    This is useful for maintaining persistent tracking of a specific individual
    throughout a behavior tree execution.
    """

    def __init__(self,
                 name: str,
                 bb_namespace: str,
                 bb_key:str,
                 service_name : str = "object_detection_generalist",
                 use_orbbec = True,
                 transform_to_map = True,
                 ):
        """
        executed when creating tree diagram, therefor very minimal
        """
        # NOTE: tk26 generalist does not implement person-ID registration
        # (returns person_id=0). Downstream by-id matching below is a legacy
        # tk23 codepath — see Wave 2.1 follow-up note.
        super(BtNode_TrackPerson, self).__init__(name, service_name, ObjectDetectionGeneralist)
        self.bb_namespace = bb_namespace
        self.bb_key = bb_key
        self.use_orbbec = use_orbbec
        self.transform_to_map = transform_to_map
        self.person_id = None
        self.do_send_tracking = True

    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)
        # attaches a blackboard (more like a shared memory section with key-value pair references) under the namespace Locations
        self.bb_write_client = self.attach_blackboard_client(name=f"TrackPerson", namespace=self.bb_namespace)
         # register a key with the name of the object, with this client having write access
        self.bb_write_client.register_key(self.bb_key, access=pytree.common.Access.WRITE)

        self.logger.debug(f"Setup Track Person, writing to namespace {self.bb_namespace} and key {self.bb_key}")
    
    def send_request(self):
        # Handle mock mode - don't send request
        if self.mock_mode:
            return
            
        request = ObjectDetectionGeneralist.Request()
        request.prompt = "person"
        if self.use_orbbec:
            request.camera = "orbbec"
        else:
            request.camera = "realsense"
        if self.transform_to_map:
            request.target_frame = "map"

        # tk23's flags="register_person" had no effect on tk26 detection nodes
        # (no-op flag string); dropped on migration to the generalist srv.

        self.response = self.call_service_async(request)

    def initialise(self) -> None:
        """
        Called when the node is visited
        """
        super().initialise()

        # Handle mock mode
        if self.mock_mode:
            if self.person_id is None:
                self.person_id = 1  # Mock person ID
                print(f"👤 MOCK: Registered person with ID {self.person_id}")
            else:
                print(f"👤 MOCK: Tracking person with ID {self.person_id}")
            # Create mock point stamped
            from geometry_msgs.msg import PointStamped
            from std_msgs.msg import Header
            point_stamped = PointStamped()
            point_stamped.point.x = 2.0
            point_stamped.point.y = 0.0
            point_stamped.point.z = 1.0
            point_stamped.header = Header()
            point_stamped.header.frame_id = "map" if self.transform_to_map else "camera_link"
            self.bb_write_client.set(self.bb_key, point_stamped, overwrite=True)
            self.feedback_message = f"MOCK: Tracking person {self.person_id}"
            return
        
        if self.do_send_tracking:
            self.send_request()
        self.do_send_tracking = True

        self.logger.debug(f"Initialized Track Person with person id {self.person_id}")
        self.feedback_message = f"Initialized Track Person"

    def update(self):
        # Handle mock mode
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
            
        self.logger.debug(f"Update Track Person")

        # Check if response exists
        if self.response is None:
            self.feedback_message = "No response object"
            return pytree.common.Status.FAILURE

        if self.response.done():
            # success
            if self.response.result().status == 0:
                # if the person has not been registered yet
                if self.person_id is None:
                    self.person_id = self.response.result().person_id
                    if not (self.person_id > 0):
                        self.feedback_message = f'Track Person returned invalid id: {self.person_id}'
                        self.person_id = None
                        return pytree.common.Status.FAILURE
                
                # proceed with picking out the person with the registered id
                persons : list[Object] = self.response.result().objects
                for person in persons:
                    print(f"person id: {person.id}")
                    if person.id == self.person_id:
                    # if person.id == 1:
                        point_stamped = PointStamped()
                        point_stamped.point = person.centroid
                        point_stamped.header = self.response.result().header
                        self.bb_write_client.set(self.bb_key, point_stamped, overwrite=True)
                        self.feedback_message = f"Detected person with id {self.person_id}, centroid as PointStamped stored to {self.bb_namespace} / {self.bb_key}"
                        
                        # send the next request without waiting for pytree to restart the node
                        self.send_request()
                        self.do_send_tracking = False
                        return pytree.common.Status.SUCCESS
                
                # if none of the people inside persons has matching id
                self.send_request()
                self.do_send_tracking = False
                self.feedback_message = f"Unable to find person with id {self.person_id}"
                return pytree.common.Status.FAILURE
            else:
                self.feedback_message = f"Track Person failed with error code {self.response.result().status}: {self.response.result().error_msg}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still tracking..."
            return pytree.common.Status.RUNNING


class BtNode_FindObj(ServiceHandler):
    """
    Finds objects for grasping with distance validation.

    This node locates objects suitable for grasping using the RealSense camera.
    It validates that detected objects are within reachable distance for the
    robot arm before returning success.
    """

    def __init__(self,
                 name: str,
                 bb_source,
                 bb_namespace: str,
                 bb_key:str,
                 service_name:str = "object_detection_yolo",
                 object:str = None,
                 target_object_cls:str = None
                 ):
        """
        executed when creating tree diagram, therefor very minimal
        """
        super(BtNode_FindObj, self).__init__(name, service_name, ObjectDetection)
        self.bb_namespace = bb_namespace
        self.bb_key = bb_key
        self.bb_source = bb_source
        self.object = object
        self.target_object_cls = target_object_cls
        self.read = True
        if self.object is not None:
            self.read = False

    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)

        if self.read:
            self.bb_read_client = self.attach_blackboard_client(name="FindObj Read")
            self.bb_read_client.register_key(self.bb_source, access=pytree.common.Access.READ)

        # attaches a blackboard (more like a shared memory section with key-value pair references) under the namespace Locations
        self.bb_write_client = self.attach_blackboard_client(name=f"FindObj", namespace=self.bb_namespace)
         # register a key with the name of the object, with this client having write access
        self.bb_write_client.register_key(self.bb_key, access=pytree.common.Access.WRITE)

        # debugger info (shown with DebugVisitor)
        self.logger.debug(f"Setup FindObj, reading from {self.bb_source}")

    def initialise(self) -> None:
        """
        Called when the node is visited
        """
        super().initialise()

        # Handle mock mode
        if self.mock_mode:
            from behavior_tree.messages import ObjectDetection
            # Create a mock response
            mock_result = ObjectDetection.Response()
            self.feedback_message = f"MOCK: Found object '{self.object}'"
            print(f"🔍 MOCK FIND OBJ: Found '{self.object}'")
            self.bb_write_client.set(self.bb_key, mock_result, overwrite=True)
            return
            
        if self.read:
            try:
                self.object = self.bb_read_client.get(self.bb_source)
                assert isinstance(self.object, str)
            except Exception as e:
                self.feedback_message = f"FindObj reading object name failed"
                raise e

        request = ObjectDetection.Request()
        request.prompt = self.object
        request.flags = "find_for_grasp"
        request.camera = "realsense"
        # setup things that needs to be cleared
        self.response = self.call_service_async(request)

        self.feedback_message = f"Initialized FindObj for {self.object}"

    def update(self):
        # Handle mock mode
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
            
        if self.response is None:
            return pytree.common.Status.FAILURE
            
        self.logger.debug(f"Update FindObj {self.object}")
        if self.response.done():
            if self.response.result().status == 0:
                # removing objetcs too far away for arm to reach
                point_stamped = PointStamped()
                point_stamped.point = self.response.result().objects[0].centroid
                point_stamped.header = self.response.result().header
                if point_stamped.point.z > 0.375: # realsense depth is further away than returned, 0.375 ~= 0.6m
                    self.feedback_message = f"Detected Object is too far away"
                    return pytree.common.Status.FAILURE

                self.bb_write_client.set(self.bb_key, self.response.result(), overwrite=True)
                self.feedback_message = f"Found object, stored to blackboard {self.bb_namespace} / {self.bb_key} with distance {point_stamped.point.z}"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Find Obj for {self.object} failed with error code {self.response.result().status}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still finding obj..."
            return pytree.common.Status.RUNNING


class BtNode_FeatureExtraction(ServiceHandler):
    """
    Extracts visual features for person recognition.

    This node captures an image from the specified camera and extracts feature
    vectors that can be used for person identification or matching. The features
    are stored on the blackboard for use by matching nodes.
    """
    def __init__(self,
                 name: str,
                 bb_dest_key: str,
                 bb_image_key: str,
                 service_name : str = "feature_extraction_service",
                 use_orbbec = True,
                 ):
        super(BtNode_FeatureExtraction, self).__init__(name, service_name, FeatureExtraction)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.key = bb_dest_key
        self.blackboard.register_key(
            key="features",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_dest_key)
        )
        self.blackboard.register_key(
            key="comparison_image",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_image_key)
        )
        if use_orbbec:
            self.camera = "orbbec"
        else:
            self.camera = "realsense"

        self.node = None

    def initialise(self):
        super().initialise()

        # Handle mock mode
        if self.mock_mode:
            from sensor_msgs.msg import Image
            print(f"👁️  MOCK: Feature extraction from {self.camera}")
            self.feedback_message = f"MOCK: Feature extraction completed"
            self.blackboard.features = "[mock features]"
            self.blackboard.comparison_image = Image()
            return

        request = FeatureExtraction.Request()
        request.camera = self.camera
        self.response = self.call_service_async(request)

        self.feedback_message = f"Initialized Feature Extraction"

    def update(self) -> Status:
        # Handle mock mode
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()

        # Check if response exists
        if self.response is None:
            self.feedback_message = "No response object"
            return pytree.common.Status.FAILURE

        self.logger.debug(f"Updated FeatureExtraction")
        if self.response.done():
            result : FeatureExtraction.Response = self.response.result()
            if result.status == 0:
                self.blackboard.features = result.feature
                self.blackboard.comparison_image = result.comparison_image
                img = result.comparison_image
                self.feedback_message = (
                    f"Features: {result.feature} | image: {img.width}x{img.height}"
                )
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Feature extration failed with error code {result.status}: {result.error_msg}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still extracting feature..."
            return pytree.common.Status.RUNNING


class BtNode_LoadPersonReference(pytree.behaviour.Behaviour):
    """
    Load a pre-recorded person photo + text description from disk and write
    them to the blackboard in the same format ``BtNode_FeatureExtraction``
    emits, so ``BtNode_CombinePerson`` and ``BtNode_FeatureMatching`` (and
    therefore ``/feature_matching_service``) consume them transparently.

    Use case: pre-register the host before the Receptionist task starts,
    avoiding a live camera + LLM scan at task entry.

    Mock mode: this node has no hardware/service dependency, so it performs
    the same real local I/O regardless of ``mock_config.json``. The mock
    flag is consulted only for log-prefix consistency.
    """

    def __init__(
        self,
        name: str,
        image_path: str,
        description_path: str,
        bb_features_key: str,
        bb_image_key: str,
    ):
        super().__init__(name)
        self.image_path = os.path.expanduser(image_path)
        self.description_path = os.path.expanduser(description_path)
        self.mock_mode = is_node_mocked(self.__class__.__name__)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="features",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_features_key),
        )
        self.blackboard.register_key(
            key="comparison_image",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_image_key),
        )

        self._cached_image = None
        self._cached_text = None

    def _load_once(self) -> Optional[str]:
        """Return None on success, error string on failure."""
        if self._cached_image is not None and self._cached_text is not None:
            return None

        import cv2
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image  # noqa: F401  (returned via cv_bridge)

        if not os.path.exists(self.image_path):
            return f"image file not found: {self.image_path}"
        if not os.path.exists(self.description_path):
            return f"description file not found: {self.description_path}"

        bgr = cv2.imread(self.image_path, cv2.IMREAD_COLOR)
        if bgr is None:
            return f"cv2.imread returned None for {self.image_path} (corrupt or unsupported format)"

        self._cached_image = CvBridge().cv2_to_imgmsg(bgr, encoding="bgr8")

        with open(self.description_path, "r", encoding="utf-8") as f:
            self._cached_text = f.read().strip()

        return None

    def update(self) -> Status:
        err = self._load_once()
        if err is not None:
            self.feedback_message = f"LoadPersonReference: {err}"
            return pytree.common.Status.FAILURE

        self.blackboard.features = self._cached_text
        self.blackboard.comparison_image = self._cached_image

        prefix = "MOCK" if self.mock_mode else "LOAD"
        snippet = self._cached_text[:60] + ("…" if len(self._cached_text) > 60 else "")
        self.feedback_message = (
            f"{prefix}: {self._cached_image.width}x{self._cached_image.height} | "
            f"desc: '{snippet}'"
        )
        return pytree.common.Status.SUCCESS


class BtNode_SeatRecommend(ServiceHandler):
    """
    Recommends a seat based on the current scene.

    This node analyzes the scene and existing seated persons to generate a
    seating recommendation for a new guest. It uses the features of known
    persons to provide context for the recommendation.
    """
    def __init__(self, 
                 name: str,
                 bb_dest_key: str,
                 bb_source_key: str,
                 service_name : str = "seat_recommend_service",
                 use_orbbec = True,
                 ):
        super(BtNode_SeatRecommend, self).__init__(name, service_name, SeatRecommendation)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.key = bb_dest_key
        self.blackboard.register_key(
            key="recommendation",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_dest_key)
        )
        self.blackboard.register_key(
            key="persons",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_source_key)
        )

        if use_orbbec:
            self.camera = "orbbec"
        else:
            self.camera = "realsense"

        self.node = None

    def initialise(self):
        super().initialise()

        # Handle mock mode
        if self.mock_mode:
            print(f"💺 MOCK: Seat recommendation from {self.camera}")
            mock_recommendation = "I recommend the seat on the left side of the table."
            self.blackboard.recommendation = "Dear guest, " + mock_recommendation
            self.feedback_message = f"MOCK: Recommendation completed"
            return
            
        request = SeatRecommendation.Request()
        request.camera = self.camera
        request.names = []
        request.features = []
        if self.blackboard.persons is not None:
            # minus one because the newest registered person is not yet seated and thus will not be in the picture
            for i in range(len(self.blackboard.persons) - 1):
                request.names.append(self.blackboard.persons[i].name)
                request.features.append(self.blackboard.persons[i].features)
        self.response = self.call_service_async(request)
        self.feedback_message = f"Initialized seat recommendation"
    
    def update(self) -> Status:
        # Handle mock mode
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
            
        self.logger.debug(f"Updated SeatRecommendation")
        
        # Check if response exists
        if self.response is None:
            self.feedback_message = "No response object"
            return pytree.common.Status.FAILURE
            
        if self.response.done():
            result : SeatRecommendation.Response = self.response.result()
            if result.status == 0:
                self.blackboard.recommendation = "Dear guest, " + result.recommendation
                self.feedback_message = f"Recommendation: {result.recommendation}"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Seat recommendation failed with error code {result.status}: {result.error_msg}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still getting seat recommendation..."
            return pytree.common.Status.RUNNING


class BtNode_SeatRecommendBbox(ServiceHandler):
    """
    Seat recommendation that also returns the bbox and 3D centroid of the seat.

    Wraps `seat_recommend_bbox_service` (kimi_api/seat_recommend_bbox.py).
    Writes three blackboard keys on success:
      - recommendation : str   (prefixed with "Dear guest, ")
      - bbox           : tinker_vision_msgs_26/BoundingBox
      - point          : geometry_msgs/PointStamped (in `target_frame`)
    """
    def __init__(self,
                 name: str,
                 bb_recommendation_key: str,
                 bb_bbox_key: str,
                 bb_point_key: str,
                 bb_source_key: str,
                 service_name: str = "seat_recommend_bbox_service",
                 use_orbbec: bool = True,
                 target_frame: str = "base_link",
                 ):
        super(BtNode_SeatRecommendBbox, self).__init__(name, service_name, SeatRecommendBbox)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="recommendation",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_recommendation_key)
        )
        self.blackboard.register_key(
            key="bbox",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_bbox_key)
        )
        self.blackboard.register_key(
            key="point",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_point_key)
        )
        self.blackboard.register_key(
            key="persons",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_source_key)
        )

        self.camera = "orbbec" if use_orbbec else "realsense"
        self.target_frame = target_frame

    def initialise(self):
        super().initialise()

        if self.mock_mode:
            print(f"💺 MOCK: Seat recommendation (bbox) from {self.camera}")
            self.blackboard.recommendation = "Dear guest, I recommend the seat on the left side of the table."
            self.blackboard.bbox = BoundingBox(xmin=300, ymin=200, xmax=380, ymax=280)
            mock_point = PointStamped()
            mock_point.header.frame_id = self.target_frame
            mock_point.point.x = 1.0
            mock_point.point.y = 0.5
            mock_point.point.z = 0.6
            self.blackboard.point = mock_point
            self.feedback_message = "MOCK: Seat-bbox recommendation completed"
            return

        request = SeatRecommendBbox.Request()
        request.camera = self.camera
        request.names = []
        request.features = []
        request.target_frame = self.target_frame
        if self.blackboard.persons is not None:
            # minus one because the newest registered person is not yet seated
            for i in range(len(self.blackboard.persons) - 1):
                request.names.append(self.blackboard.persons[i].name)
                request.features.append(self.blackboard.persons[i].features)
        self.response = self.call_service_async(request)
        self.feedback_message = "Initialized seat-bbox recommendation"

    def update(self) -> Status:
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()

        self.logger.debug("Updated SeatRecommendBbox")

        if self.response is None:
            self.feedback_message = "No response object"
            return pytree.common.Status.FAILURE

        if self.response.done():
            result: SeatRecommendBbox.Response = self.response.result()
            if result.status == 0:
                self.blackboard.recommendation = "Dear guest, " + result.recommendation
                self.blackboard.bbox = result.bbox
                self.blackboard.point = result.centroid
                self.feedback_message = f"Recommendation: {result.recommendation}"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Seat-bbox recommendation failed with error code {result.status}: {result.error_msg}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still getting seat-bbox recommendation..."
            return pytree.common.Status.RUNNING


class BtNode_FeatureMatching(ServiceHandler):
    """
    Matches extracted features to known persons.

    This node takes feature vectors of known persons and attempts to locate
    them in the current camera view. It returns the centroid positions of
    matched persons for use in subsequent operations like turning to face them.
    """
    def __init__(self,
                 name: str,
                 bb_dest_key: str,
                 bb_persons_key: str,
                 service_name: str = "feature_matching_service",
                 use_orbbec: bool = True,
                 max_distance: float = 6.0,
                 target_frame: str = "base_link",
                 trim_last_person: bool = True,
                 ):
        """
        Args:
            ...
            trim_last_person: when True (default, matches Receptionist's
                flow where the newest registered guest has not sat down yet),
                drop the last feature vector before sending. Set to False
                in HRI-style intros where **all** registered persons are
                already seated and their centroids are needed.
        """
        super().__init__(name, service_name, FeatureMatching)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.key = bb_dest_key
        self.blackboard.register_key(
            key="centroids",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_dest_key)
        )
        self.blackboard.register_key(
            key="persons",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_persons_key)
        )

        if use_orbbec:
            self.camera = "orbbec"
        else:
            self.camera = "realsense"
        self.max_distance = max_distance
        self.target_frame = target_frame
        self.trim_last_person = trim_last_person

        self.node = None

    def initialise(self):
        super().initialise()

        # Handle mock mode
        if self.mock_mode:
            print(f"🔍 MOCK: Feature matching from {self.camera}")
            from geometry_msgs.msg import PointStamped
            persons = self.blackboard.persons or []
            n = len(persons)
            if self.trim_last_person and n > 0:
                n -= 1
            n = max(n, 1)  # keep a single centroid even if persons is empty
            centroids = []
            for i in range(n):
                mc = PointStamped()
                mc.point.x = 1.5
                mc.point.y = -0.5 + i * 1.0   # spread mock guests laterally
                mc.point.z = 1.3
                mc.header.frame_id = self.target_frame
                centroids.append(mc)
            self.blackboard.centroids = centroids
            self.feedback_message = f"MOCK: Feature matching → {n} centroid(s)"
            return

        from sensor_msgs.msg import Image
        request = FeatureMatching.Request()
        request.camera = self.camera
        persons = self.blackboard.persons
        request.features = [p.features for p in persons]
        request.comparison_images = [
            p.comparison_image if p.comparison_image is not None else Image()
            for p in persons
        ]
        if self.trim_last_person:
            # Receptionist flow: the newest registered guest has not sat down yet
            # and won't match anything at the sofa scan, so drop them.
            request.features = request.features[:-1]
            request.comparison_images = request.comparison_images[:-1]
        request.max_distance = self.max_distance
        request.target_frame = self.target_frame
        self.response = self.call_service_async(request)
        self.feedback_message = (
            f"Initialized feature matching (|features|={len(request.features)}, "
            f"|images|={len(request.comparison_images)}, "
            f"trim_last={self.trim_last_person})"
        )

    def update(self) -> Status:
        # Handle mock mode
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
            
        self.logger.debug(f"Updated Feature Matching")
        
        # Check if response exists
        if self.response is None:
            self.feedback_message = "No response object"
            self.blackboard.centroids = []  # Clear centroids on failure
            return pytree.common.Status.FAILURE
            
        if self.response.done():
            result : FeatureMatching.Response = self.response.result()
            if result.status == 0:
                self.blackboard.centroids = result.centroids
                # for p in self.blackboard.centroids:
                #     # p.point.z = 1.30
                #     fac = p.point.x / 0.6
                #     p.point.x /= (fac + 1e-6)
                #     p.point.y /= (fac + 1e-6)
                #     p.point.z = 1.3
                self.feedback_message = f"Centroids: {result.centroids}"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Feature Matching failed with error code {result.status}: {result.error_msg}"
                self.blackboard.centroids = []  # Clear centroids on failure
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still matching features..."
            return pytree.common.Status.RUNNING


class BtNode_GetPointCloud(ServiceHandler):
    """
    Retrieves a point cloud from the depth camera.

    This node captures a point cloud from the specified depth camera and stores
    it on the blackboard for use by other nodes requiring 3D spatial data.
    """
    def __init__(self,
                 name: str,
                 bb_point_cloud_key: str,
                 service_name: str = "get_point_cloud_service",
                 camera_name: str = "orbbec",
                 ):
        super().__init__(name, service_name, GetPointCloud)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.key = bb_point_cloud_key
        self.blackboard.register_key(
            key="point_cloud",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_point_cloud_key)
        )

        if "orbbec" in camera_name:
            self.camera = "orbbec"
        else:
            self.camera = "realsense"

        self.node = None
    
    def initialise(self):
        super().initialise()

        # Handle mock mode
        if self.mock_mode:
            from sensor_msgs.msg import PointCloud2
            mock_pc = PointCloud2()
            mock_pc.height = 480
            mock_pc.width = 640
            self.blackboard.point_cloud = mock_pc
            self.feedback_message = f"MOCK: Got point cloud with {mock_pc.height * mock_pc.width} points"
            print(f"☁️ MOCK GET POINT CLOUD: {mock_pc.height}x{mock_pc.width} points")
            return
            
        request = GetPointCloud.Request()
        request.camera = self.camera
        self.response = self.call_service_async(request)
        self.feedback_message = f"Initialized Getting Point Cloud Service"

    def update(self) -> Status:
        # Handle mock mode
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
            
        if self.response is None:
            return pytree.common.Status.FAILURE
            
        self.logger.debug(f"Updated Point Cloud Service")
        if self.response.done():
            result : GetPointCloud.Response = self.response.result()
            if result.status == 0:
                self.blackboard.point_cloud = result.points
                self.feedback_message = f"Successfully got point cloud with {result.points.height * result.points.width} points"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Point cloud service failed with error code {result.status}: {result.error_msg}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still getting point cloud..."
            return pytree.common.Status.RUNNING


class BtNode_FindPlacingLocation(ServiceHandler):
    """
    Calls /placing_location (VLM-only) to find an empty spot on a desktop.

    Reads either a literal item description or a string from a blackboard key
    and writes the best-ranked PointStamped to ``bb_key_point``. If the VLM
    finds nothing usable, the node fails so a parent Retry/Selector can react.
    """

    def __init__(self,
                 name: str,
                 bb_key_point: str,
                 bb_source_item: Optional[str] = None,
                 item_description: Optional[str] = None,
                 service_name: str = "placing_location",
                 use_orbbec: bool = True,
                 target_frame: str = "map",
                 max_candidates: int = 3,
                 ):
        super().__init__(name, service_name, PlacingLocation)
        self.bb_key_point = bb_key_point
        self.bb_source_item = bb_source_item
        self.item_description = item_description
        self.target_frame = target_frame
        self.max_candidates = int(max_candidates)
        self.camera = "orbbec" if use_orbbec else "realsense"
        if bb_source_item is None and not item_description:
            raise ValueError(
                "BtNode_FindPlacingLocation requires bb_source_item or item_description"
            )

    def setup(self, **kwargs):
        ServiceHandler.setup(self, **kwargs)
        self.bb_write = self.attach_blackboard_client(name="FindPlacingLocation Write")
        self.bb_write.register_key(self.bb_key_point, access=pytree.common.Access.WRITE)
        if self.bb_source_item is not None:
            self.bb_read = self.attach_blackboard_client(name="FindPlacingLocation Read")
            self.bb_read.register_key(self.bb_source_item, access=pytree.common.Access.READ)

    def initialise(self) -> None:
        super().initialise()

        item = self.item_description
        if self.bb_source_item is not None:
            try:
                item = self.bb_read.get(self.bb_source_item)
            except Exception:
                item = self.item_description
        if not isinstance(item, str) or not item.strip():
            self.feedback_message = "FindPlacingLocation: empty item_description"
            self.response = None
            self._invalid = True
            return
        self._invalid = False

        if self.mock_mode:
            mock_point = PointStamped()
            mock_point.header = Header(frame_id=self.target_frame)
            mock_point.point.x = 0.5
            mock_point.point.y = 0.0
            mock_point.point.z = 0.7
            self.bb_write.set(self.bb_key_point, mock_point, overwrite=True)
            self.feedback_message = f"MOCK: placing point for '{item}'"
            self.response = None
            return

        request = PlacingLocation.Request()
        request.camera = self.camera
        request.item_description = item
        request.target_frame = self.target_frame
        request.max_candidates = self.max_candidates
        request.return_rgb_image = False
        request.return_debug_overlay = False
        self.response = self.call_service_async(request)
        self.feedback_message = f"Initialized FindPlacingLocation for '{item}'"

    def update(self):
        if getattr(self, "_invalid", False):
            return pytree.common.Status.FAILURE
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
        if self.response is None:
            return pytree.common.Status.FAILURE
        if not self.response.done():
            self.feedback_message = "Waiting for placing-location service..."
            return pytree.common.Status.RUNNING
        result = self.response.result()
        if result.status != 0 or not result.candidate_points:
            self.feedback_message = (
                f"PlacingLocation failed (status={result.status}): "
                f"{getattr(result, 'error_msg', '')}"
            )
            return pytree.common.Status.FAILURE
        self.bb_write.set(self.bb_key_point, result.candidate_points[0], overwrite=True)
        self.feedback_message = (
            f"Placing point set ({len(result.candidate_points)} candidate(s))"
        )
        return pytree.common.Status.SUCCESS


class BtNode_DoorDetection(ServiceHandler):
    """
    Detects if a door is open or closed.

    This node uses the Orbbec camera to analyze the scene and determine
    whether a door is currently open or closed. The result is stored on
    the blackboard as a boolean (1 for open, 0 for closed).
    """
    def __init__(self,
                 name: str,
                 bb_door_state_key: str,
                 service_name: str = "door_detection_srv"
                 ):
        super().__init__(name, service_name, DoorDetection)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.key = bb_door_state_key
        self.blackboard.register_key(
            key="is_open",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_door_state_key)
        )

        self.camera = "orbbec"

        self.node = None
    
    def initialise(self):
        super().initialise()

        # Handle mock mode
        if self.mock_mode:
            print(f"🚪 MOCK: Door detection from {self.camera}")
            self.blackboard.is_open = 1  # Mock: door is open
            self.feedback_message = f"MOCK: Door is open"
            return
            
        request = DoorDetection.Request()
        request.camera = self.camera
        self.response = self.call_service_async(request)
        self.feedback_message = f"Initialized Door Detection Service"

    def update(self) -> Status:
        # Handle mock mode
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
            
        self.logger.debug(f"Updated Door Detection Service")
        
        # Check if response exists
        if self.response is None:
            self.feedback_message = "No response object"
            return pytree.common.Status.FAILURE
            
        if self.response.done():
            result : DoorDetection.Response = self.response.result()
            if result.is_open == 1:
                # 0 for close, 1 for open
                self.blackboard.is_open = result.is_open
                self.feedback_message = f"Successfully return with is_open = {result.is_open}"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Door detection service failed with error code {result.status}: {result.error_msg}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still waiting door service..."
            return pytree.common.Status.RUNNING


class BtNode_TurnPanTilt(pytree.behaviour.Behaviour):
    """
    Publish an absolute pan-tilt target to the tk26 servo controller.

    Publishes `tinker_vision_msgs_26/PanTiltCommand` (mode=ABSOLUTE) to the
    runtime command topic (`/pan_tilt_controller/cmd`). The legacy
    `/pan_tilt_ctrl` topic + `PanTiltCtrl` message are no-ops against the
    current `pan_tilt_controller` and have been retired.

    `x`, `y` are pan/tilt angles in degrees. `speed` is the servo
    speed_raw value (0 lets the controller pick its default).

    `update()` holds RUNNING until `/pan_tilt_controller/state` reports
    `feedback_ok` and `|state - cmd| < SETTLE_EPS_DEG` for
    `SETTLE_SAMPLES_REQUIRED` consecutive samples, OR until
    `SETTLE_TIMEOUT_SEC` elapses (in which case it warns and returns
    SUCCESS so a stuck servo doesn't block the whole tree).
    """

    PAN_TILT_COMMAND_TOPIC  = "/pan_tilt_controller/cmd"
    PAN_TILT_STATE_TOPIC    = "/pan_tilt_controller/state"
    SETTLE_EPS_DEG          = 1.5
    SETTLE_SAMPLES_REQUIRED = 1
    SETTLE_TIMEOUT_SEC      = 2.0

    def __init__(self, 
                 name: str, 
                 x: float = 0.0, 
                 y: float = 0.0, 
                 speed: float = 0.0,
                 x_key: Optional[str] = None,
                 y_key: Optional[str] = None,
                 speed_key: Optional[str] = None,
                 ):
        """
        x_key and y_key takes in RADIANS. x and y are in DEGREES.
        """
        super().__init__(name)
        self.x = x
        self.y = y
        self.speed = speed
        self.publisher = None
        self.state_sub = None
        self._latest_state = None
        self._cmd_pan_deg = x
        self._cmd_tilt_deg = y
        self._settle_count = 0
        self._settle_deadline = None
        self._skip_settle = False
        self.x_key = x_key
        self.y_key = y_key
        self.speed_key = speed_key
        self.mock_mode = is_node_mocked(self.__class__.__name__)

    def setup(self, **kwargs) -> None:
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.name, self.__class__.__name__)
            raise KeyError(error_message) from e

        if self.mock_mode:
            print("MOCK MODE: Skipping pan/tilt publisher creation")
            return
    
        self.bb_read_client = self.attach_blackboard_client(name=self.name + " Blackboard Read")
        if self.x_key:
            self.bb_read_client.register_key(
                key="x",
                access=pytree.common.Access.READ,
                remap_to=pytree.blackboard.Blackboard.absolute_name("/", self.x_key) if self.x_key else None
            )
        if self.y_key:
            self.bb_read_client.register_key(
                key="y",
                access=pytree.common.Access.READ,
                remap_to=pytree.blackboard.Blackboard.absolute_name("/", self.y_key) if self.y_key else None
            )
        if self.speed_key:
            self.bb_read_client.register_key(
                key="speed",
                access=pytree.common.Access.READ,
                remap_to=pytree.blackboard.Blackboard.absolute_name("/", self.speed_key) if self.speed_key else None
            )

        self.publisher = self.node.create_publisher(
            PanTiltCommand, self.PAN_TILT_COMMAND_TOPIC, 1
        )
        self.state_sub = self.node.create_subscription(
            PanTiltState, self.PAN_TILT_STATE_TOPIC, self._state_cb, 10
        )

    def _state_cb(self, msg: "PanTiltState") -> None:
        self._latest_state = msg

    def _build_command_msg(self, pan_deg: float, tilt_deg: float) -> "PanTiltCommand":
        msg = PanTiltCommand()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.mode = PanTiltCommand.ABSOLUTE
        msg.pan_rad = float(pan_deg) * math.pi / 180.0
        msg.tilt_rad = float(tilt_deg) * math.pi / 180.0
        msg.speed_raw = int(self.speed)
        msg.accel_raw = 0
        return msg

    def _arm_settle(self, pan_deg: float, tilt_deg: float) -> None:
        """Record the commanded target and reset the settle predicate."""
        self._cmd_pan_deg = pan_deg
        self._cmd_tilt_deg = tilt_deg
        self._settle_count = 0
        self._settle_deadline = time.time() + self.SETTLE_TIMEOUT_SEC
        self._skip_settle = False

    def initialise(self) -> None:
        if self.bb_read_client.exists("x"):
            self.x = self.bb_read_client.get("x") / math.pi * 180.0
        if self.bb_read_client.exists("y"):
            self.y = self.bb_read_client.get("y") / math.pi * 180.0
        if self.bb_read_client.exists("speed"):
            self.speed = self.bb_read_client.get("speed")

        if self.mock_mode:
            self._skip_settle = True
            return
        
        msg = self._build_command_msg(self.x, self.y)
        self.publisher.publish(msg)
        self.logger.info(
            f"Publishing PanTiltCommand ABSOLUTE pan={self.x:+.2f}° "
            f"tilt={self.y:+.2f}° speed_raw={int(self.speed)}"
        )
        self._arm_settle(self.x, self.y)

    def update(self) -> Status:
        if self.mock_mode or self._skip_settle:
            return pytree.common.Status.SUCCESS

        state = self._latest_state
        now = time.time()
        if state is None or not getattr(state, 'feedback_ok', False):
            if now > self._settle_deadline:
                print("PanTilt settle timeout (no feedback); proceeding")
                return pytree.common.Status.SUCCESS
            return pytree.common.Status.RUNNING

        pan_err  = abs(math.degrees(state.pan_rad)  - self._cmd_pan_deg)
        tilt_err = abs(math.degrees(state.tilt_rad) - self._cmd_tilt_deg)
        if pan_err < self.SETTLE_EPS_DEG and tilt_err < self.SETTLE_EPS_DEG:
            self._settle_count += 1
            if self._settle_count >= self.SETTLE_SAMPLES_REQUIRED:
                return pytree.common.Status.SUCCESS
        else:
            self._settle_count = 0

        if now > self._settle_deadline:
            print(
                f"PanTilt settle timeout (pan_err={pan_err:.2f}°, "
                f"tilt_err={tilt_err:.2f}°); proceeding"
            )
            return pytree.common.Status.SUCCESS
        return pytree.common.Status.RUNNING

    async def wait_seconds(self, seconds):
        await asyncio.sleep(seconds)
        return True


class BtNode_TurnTo(BtNode_TurnPanTilt):
    """
    Turn to a specific point relevant to 'base_link'.
    """
    def __init__(self, name: str,
                 bb_key_persons: str,
                 bb_key_points: str,
                 target_id: int = 0,
                 ):
        super().__init__(name, x=0, y=0, speed=0.0)
        self.bb_key_persons = bb_key_persons
        self.bb_key_points = bb_key_points
        self.target_id = target_id
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="persons",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_key_persons)
        )
        self.blackboard.register_key(
            key='points',
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_key_points)
        )

    def initialise(self) -> None:
        try:
            persons = self.blackboard.persons
        except KeyError:
            persons = None
        try:
            points = self.blackboard.points
        except KeyError:
            points = None

        if (persons is None or len(persons) <= self.target_id
                or points is None or len(points) <= self.target_id):
            # Best-effort gaze: feature matching produced no PointStamped for
            # this target. Skip publish; fast-success so the wrapping
            # FailureIsSuccess sees a clean SUCCESS instead of a Python
            # exception aborting the parent Sequence.
            self.feedback_message = (
                f"TurnTo skipped: persons={'None' if persons is None else len(persons)}, "
                f"points={'None' if points is None else len(points)}, target_id={self.target_id}"
            )
            self.logger.info(self.feedback_message)
            self.response = None
            self._skip_settle = True
            return

        if self.mock_mode:
            self._skip_settle = True
            self.feedback_message = (
                f"MOCK TurnTo for point {points[self.target_id].point} target_id={self.target_id}"
            )
            return

        point = points[self.target_id]
        self.logger.info(
            f"Turning to point {point.point} id={self.target_id} from blackboard {self.bb_key_points}"
        )
        pan_rad = math.atan2(-point.point.y, max(point.point.x - 0.25, 0.01))
        pan_deg = pan_rad / math.pi * 180.0
        tilt_deg = 20.0
        msg = self._build_command_msg(pan_deg, tilt_deg)
        self.publisher.publish(msg)
        self.logger.info(
            f"Publishing PanTiltCommand ABSOLUTE pan={pan_deg:+.2f}° "
            f"tilt={tilt_deg:+.2f}° speed_raw={int(self.speed)}"
        )
        self.feedback_message = (
            f"Initialized TurnTo for point {point.point} id={self.target_id} "
            f"pan={pan_deg:+.2f}° tilt={tilt_deg:+.2f}°"
        )
        self._arm_settle(pan_deg, tilt_deg)


class BtNode_ScanForWavingPerson(ServiceHandler):
    """
    Call tk26's `detect_waving_persons` service (tinker_vision_msgs_26/DetectWaving),
    write the full list and the closest person to the blackboard.

    Blackboard:
      - `bb_key_all_persons`    ← list[PointStamped] sorted closest-first
      - `bb_key_closest_person` ← PointStamped (index 0)
      - `bb_key_pictures` (opt) ← list[str] filesystem paths of per-person RGB crops

    Succeeds when at least one waving person is detected within `threshold_meters`.
    """

    def __init__(self,
                 name: str,
                 bb_key_all_persons: str,
                 bb_key_closest_person: str,
                 threshold_meters: float,
                 service_name: str = "detect_waving_persons",
                 target_frame: str = "map",
                 bb_key_pictures: Optional[str] = None,
                 picture_output_dir: str = "/tmp",
                 ):
        super(BtNode_ScanForWavingPerson, self).__init__(name, service_name, DetectWaving)
        self.bb_key_all_persons = bb_key_all_persons
        self.bb_key_closest_person = bb_key_closest_person
        self.bb_key_pictures = bb_key_pictures
        self.threshold_meters = threshold_meters
        self.target_frame = target_frame
        self.picture_output_dir = picture_output_dir
        self.bb_write_client = None

    def setup(self, **kwargs):
        ServiceHandler.setup(self, **kwargs)
        self.bb_write_client = self.attach_blackboard_client(name="ScanForWavingPerson")
        self.bb_write_client.register_key(self.bb_key_all_persons, access=pytree.common.Access.WRITE)
        self.bb_write_client.register_key(self.bb_key_closest_person, access=pytree.common.Access.WRITE)
        if self.bb_key_pictures is not None:
            self.bb_write_client.register_key(self.bb_key_pictures, access=pytree.common.Access.WRITE)
        self.logger.debug("Setup ScanForWavingPerson")

    def _crop_and_save(self, rgb_msg, segments):
        """
        Convert rgb_image + per-person segment masks into per-person cropped PNGs on disk.
        Returns list[str] of paths (same order as `segments`). Empty list on failure.
        """
        try:
            import cv2
            import numpy as np
            from cv_bridge import CvBridge
        except Exception as e:
            self.logger.warning(f"ScanForWavingPerson: cv2/cv_bridge unavailable ({e}); skipping picture export")
            return []

        bridge = CvBridge()
        try:
            rgb = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        except Exception as e:
            self.logger.warning(f"ScanForWavingPerson: failed to decode rgb_image ({e})")
            return []

        os.makedirs(self.picture_output_dir, exist_ok=True)
        ts = int(time.time() * 1000)
        paths = []
        for idx, seg_msg in enumerate(segments):
            try:
                seg = bridge.imgmsg_to_cv2(seg_msg, desired_encoding="passthrough")
                mask = seg if seg.ndim == 2 else seg[..., 0]
                ys, xs = np.where(mask > 0)
                if ys.size == 0 or xs.size == 0:
                    # fall back to whole frame if mask is empty
                    crop = rgb
                else:
                    y0, y1 = int(ys.min()), int(ys.max()) + 1
                    x0, x1 = int(xs.min()), int(xs.max()) + 1
                    crop = rgb[y0:y1, x0:x1]
                path = os.path.join(
                    self.picture_output_dir,
                    f"restaurant_customer_{idx}_{ts}.png",
                )
                cv2.imwrite(path, crop)
                paths.append(path)
            except Exception as e:
                self.logger.warning(f"ScanForWavingPerson: failed to crop person {idx}: {e}")
                paths.append("")
        return paths

    def initialise(self) -> None:
        super().initialise()

        if self.mock_mode:
            mock_person = PointStamped()
            mock_person.header = Header(frame_id=self.target_frame or "map")
            mock_person.point.x = 1.0
            mock_person.point.y = 0.0
            mock_person.point.z = 0.0
            self.bb_write_client.set(self.bb_key_all_persons, [mock_person], overwrite=True)
            self.bb_write_client.set(self.bb_key_closest_person, mock_person, overwrite=True)
            if self.bb_key_pictures is not None:
                self.bb_write_client.set(self.bb_key_pictures, ["/tmp/mock_customer.png"], overwrite=True)
            print(f"👋 MOCK: ScanForWavingPerson → wrote synthetic person at (1.0, 0.0, 0.0) in {self.target_frame or 'map'}")
            self.feedback_message = "MOCK: Scanned for waving person"
            return

        request = DetectWaving.Request()
        request.threshold_meters = self.threshold_meters
        request.target_frame = self.target_frame
        self.response = self.client.call_async(request)
        self.feedback_message = "Initialized ScanForWavingPerson"

    def update(self):
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()

        self.logger.debug("Update ScanForWavingPerson")
        if self.response is None:
            self.feedback_message = "No response object"
            return Status.FAILURE

        if self.response.done():
            result = self.response.result()
            if result.status == 0 and result.waving_persons:
                self.bb_write_client.set(self.bb_key_all_persons, result.waving_persons, overwrite=True)
                self.bb_write_client.set(self.bb_key_closest_person, result.waving_persons[0], overwrite=True)
                if self.bb_key_pictures is not None:
                    paths = self._crop_and_save(result.rgb_image, result.segments)
                    # pad/truncate to match waving_persons length so index alignment is safe
                    n = len(result.waving_persons)
                    if len(paths) < n:
                        paths = paths + [""] * (n - len(paths))
                    else:
                        paths = paths[:n]
                    self.bb_write_client.set(self.bb_key_pictures, paths, overwrite=True)
                closest = result.waving_persons[0]
                self.feedback_message = (
                    f"Found {len(result.waving_persons)} waving person(s). "
                    f"Closest at ({closest.point.x:.3f}, {closest.point.y:.3f}, {closest.point.z:.3f}) "
                    f"in {closest.header.frame_id}"
                )
                return Status.SUCCESS
            elif result.status == 0:
                self.feedback_message = "Service succeeded, but no waving person found."
                return Status.FAILURE
            else:
                self.feedback_message = f"Service failed with status {result.status}: {result.error_msg}"
                return Status.FAILURE
        else:
            self.feedback_message = "Still scanning for waving persons..."
            return Status.RUNNING


class BtNode_MaintainEyeContact(ActionHandler):
    """
    Wrap the HRI `follow_head_action` (tinker_vision_msgs_26.action.FollowHeadAction) to
    maintain eye-contact with a designated guest. Server returns success after a single
    gaze lock.

    Three fresh-lock modes, in priority order:

    * **Seeded** — pass `target_id` + `bb_key_persons` + `bb_key_points`. The
      centroid at `KEY_PERSON_CENTROIDS[target_id]` (in base_link, populated by
      `BtNode_FeatureMatching`) is shipped as `target_seed_xyz`. The server
      locks onto the candidate within `seed_radius_m` of the seed.
    * **Centermost** — pass `track_centermost=True`. The server locks onto
      whoever is most image-centered. Use after a `BtNode_TurnPanTilt` that
      centered the designated person but no centroid is on the blackboard.
    * **Closest** (default, no args) — server locks onto the closest person
      (smallest XY distance to the robot in pan-tilt-root). Original upstream
      behavior.

    Sticky logic on the server is purely spatial reassoc within
    `reassoc_dist_m` of the previous lock — no track IDs, no appearance.

    Feedback schema is `{pan, tilt, person_visible, current_pan/tilt, target_pan/tilt,
    error_deg}` — not the BT canonical `{stage, stage_name, status, delay_limit}`.
    We override `feedback_callback` to stamp `last_feedback_time`, force `action_status=0`,
    and keep a generous `feedback_timeout`.
    """
    def __init__(self,
                 name: str,
                 *,
                 target_id: Optional[int] = None,
                 bb_key_persons: Optional[str] = None,
                 bb_key_points: Optional[str] = None,
                 seed_radius_m: float = 0.5,
                 track_centermost: bool = False,
                 action_name: str = "follow_head_action",
                 feedback_timeout_secs: float = 30.0,
                 wait_for_server_timeout_sec: float = -3.0,
                 follow_timeout: float = 30.0,
                 ):
        super().__init__(name, FollowHeadAction, action_name, None, wait_for_server_timeout_sec)
        self._feedback_timeout_secs = feedback_timeout_secs
        self._follow_timeout = follow_timeout
        self.start_time = None
        self._target_id = target_id
        self._bb_key_persons = bb_key_persons
        self._bb_key_points = bb_key_points
        self._seed_radius_m = float(seed_radius_m)
        self._track_centermost = bool(track_centermost)

        # Targeted-lock requires both keys + a target_id; partial config is
        # almost always a wiring mistake, so refuse rather than silently
        # dropping into seedless mode.
        if target_id is not None and (
            bb_key_persons is None or bb_key_points is None
        ):
            raise ValueError(
                "BtNode_MaintainEyeContact: target_id requires both "
                "bb_key_persons and bb_key_points."
            )
        # Seed and centermost are mutually exclusive on the server (seed
        # wins). Surfacing this at construction prevents ambiguous tree
        # code reading like the centermost mode is active when it isn't.
        if target_id is not None and track_centermost:
            raise ValueError(
                "BtNode_MaintainEyeContact: target_id and track_centermost "
                "are mutually exclusive (seed wins on the server)."
            )

        self._blackboard_targeted = None
        if target_id is not None:
            self._blackboard_targeted = self.attach_blackboard_client(
                name=f"{self.name}_targeted",
            )
            self._blackboard_targeted.register_key(
                key="persons",
                access=pytree.common.Access.READ,
                remap_to=pytree.blackboard.Blackboard.absolute_name(
                    "/", bb_key_persons,
                ),
            )
            self._blackboard_targeted.register_key(
                key="points",
                access=pytree.common.Access.READ,
                remap_to=pytree.blackboard.Blackboard.absolute_name(
                    "/", bb_key_points,
                ),
            )

    def _resolve_seed_point(self) -> Point:
        """Read the targeted guest's centroid from the blackboard.

        Returns a `Point()` with all-zero fields on any miss (key absent,
        index out of range, sentinel zero centroid). The action server
        treats (0,0,0) as "no seed" and uses the head-direction fallback.
        """
        if self._target_id is None or self._blackboard_targeted is None:
            return Point()
        try:
            persons = self._blackboard_targeted.persons
        except KeyError:
            persons = None
        try:
            points = self._blackboard_targeted.points
        except KeyError:
            points = None
        if (
            points is None
            or len(points) <= self._target_id
            or persons is None
            or len(persons) <= self._target_id
        ):
            self.feedback_message = (
                f"MaintainEyeContact: targeted seed unavailable "
                f"(persons={'None' if persons is None else len(persons)}, "
                f"points={'None' if points is None else len(points)}, "
                f"target_id={self._target_id}); falling back to "
                f"head-direction lock."
            )
            self.logger.warning(self.feedback_message)
            return Point()
        ps = points[self._target_id]
        seed = Point()
        seed.x = float(ps.point.x)
        seed.y = float(ps.point.y)
        seed.z = float(ps.point.z)
        # Sentinel guard: a literal (0,0,0) centroid is indistinguishable
        # from "no seed" on the wire; treat as missing.
        if seed.x == 0.0 and seed.y == 0.0 and seed.z == 0.0:
            self.logger.warning(
                f"MaintainEyeContact: target_id={self._target_id} centroid "
                "is exactly (0,0,0); treating as no seed.",
            )
        return seed

    def send_goal(self):
        self._cancel_pending = False
        if self.mock_mode:
            self.feedback_message = "MOCK: eye-contact goal sent"
            class MockFuture:
                def done(self):
                    return True
            self.send_goal_future = MockFuture()
            return
        goal = FollowHeadAction.Goal()
        goal.start_following = True
        seed = self._resolve_seed_point()
        goal.target_seed_xyz = seed
        seed_set = (seed.x != 0.0 or seed.y != 0.0 or seed.z != 0.0)
        if seed_set:
            goal.seed_radius_m = self._seed_radius_m
            goal.track_centermost = False  # seed wins on server, but be explicit
            self.feedback_message = (
                f"Eye-contact goal sent (mode=seed, "
                f"xyz=({seed.x:.2f},{seed.y:.2f},{seed.z:.2f}), "
                f"radius={self._seed_radius_m:.2f} m, "
                f"target_id={self._target_id})"
            )
        elif self._track_centermost:
            goal.seed_radius_m = 0.0
            goal.track_centermost = True
            self.feedback_message = "Eye-contact goal sent (mode=centermost)"
        else:
            goal.seed_radius_m = 0.0
            goal.track_centermost = False
            self.feedback_message = "Eye-contact goal sent (mode=closest)"
        self.send_goal_request(goal)
        self.start_time = time.time()

    def update(self) -> Status:
        if time.time() - self.start_time > self._follow_timeout:
            self.feedback_message = "Eye-contact follow timeout reached"
            return pytree.common.Status.SUCCESS
        return super().update()

    def feedback_callback(self, msg: Any):
        feedback = msg.feedback
        self.last_feedback_time = time.time()
        self.feedback_timeout = self._feedback_timeout_secs
        self.action_status = 0
        pan = getattr(feedback, "pan", None)
        tilt = getattr(feedback, "tilt", None)
        self.feedback_message = f"eye-contact: pan={pan}, tilt={tilt}"

    def goal_response_callback(self, future):
        super().goal_response_callback(future)
        # Pre-acceptance terminate window: if the parallel cancelled this
        # node before the server accepted the goal, the base-class cancel
        # in terminate() was a no-op (goal_handle was still None). Now that
        # we finally have a handle, cancel so the server doesn't keep
        # gazing after this node is INVALID.
        if self._cancel_pending and self.goal_handle is not None:
            self.send_cancel_request()
            self._cancel_pending = False

    def process_result(self):
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = f"Eye-contact action failed with status {self.result_status}"
            return pytree.common.Status.FAILURE
        if getattr(self.result_message.result, "success", False):
            self.feedback_message = "Eye-contact succeeded"
            return pytree.common.Status.SUCCESS
        err = getattr(self.result_message.result, "message", "")
        self.feedback_message = f"Eye-contact failed: {err}"
        return pytree.common.Status.FAILURE

    def terminate(self, new_status: pytree.common.Status):
        # On RUNNING→INVALID (parallel cut us off), the base class already
        # sends a cancel when `goal_handle` is set. If the goal is still
        # in flight (`send_goal_future` not yet resolved), defer the cancel
        # to `goal_response_callback` so the server is reliably stopped.
        if (
            not self.mock_mode
            and self.status == pytree.common.Status.RUNNING
            and new_status == pytree.common.Status.INVALID
            and self.goal_handle is None
            and self.send_goal_future is not None
        ):
            self._cancel_pending = True
        super().terminate(new_status)


class BtNode_ShowImage(pytree.behaviour.Behaviour):
    """
    Display an image to the referee/audience. Currently a **stub**:
    reads the file path from the blackboard, logs it, and returns SUCCESS.

    TODO: Replace body with a publisher to the on-robot display topic once identified.
    """
    def __init__(self,
                 name: str,
                 bb_image_path_key: str,
                 ):
        super().__init__(name)
        self.bb_image_path_key = bb_image_path_key
        self.mock_mode = is_node_mocked(self.__class__.__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="image_path",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_image_path_key)
        )

    def update(self) -> Status:
        try:
            path = self.blackboard.image_path
        except Exception as e:
            self.feedback_message = f"ShowImage: no image path on blackboard ({e})"
            return pytree.common.Status.FAILURE
        if not path:
            self.feedback_message = "ShowImage: empty image path"
            return pytree.common.Status.FAILURE
        exists = os.path.exists(path)
        prefix = "MOCK" if self.mock_mode else "STUB"
        if exists:
            self.feedback_message = f"{prefix}: displaying {path}"
            print(f"🖼️  {prefix} SHOW IMAGE: {path}")
        else:
            self.feedback_message = f"{prefix}: image path {path} not on disk (displaying anyway)"
            print(f"🖼️  {prefix} SHOW IMAGE (missing file): {path}")
        return pytree.common.Status.SUCCESS

