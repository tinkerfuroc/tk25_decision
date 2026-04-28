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

import asyncio
import os
import py_trees as pytree
import time
import math
from typing import Any, Optional

from behavior_tree.messages import ObjectDetection, ObjectDetectionGeneralist, Object, FeatureExtraction, SeatRecommendation, SeatRecommendBbox, FeatureMatching, GetPointCloud, DoorDetection, PanTiltCtrl, BoundingBox
from behavior_tree.config import is_node_mocked
from geometry_msgs.msg import PointStamped
from py_trees.common import Status

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
        if use_orbbec:
            self.camera = "orbbec"
        else:
            self.camera = "realsense"

        self.node = None

    def initialise(self):
        super().initialise()

        # Handle mock mode
        if self.mock_mode:
            print(f"👁️  MOCK: Feature extraction from {self.camera}")
            self.feedback_message = f"MOCK: Feature extraction completed"
            # Store mock features to blackboard
            self.blackboard.features = [0.1, 0.2, 0.3, 0.4, 0.5]  # Mock feature vector
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
                self.feedback_message = f"Features: {result.feature}"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Feature extration failed with error code {result.status}: {result.error_msg}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still extracting feature..."
            return pytree.common.Status.RUNNING

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
                 max_distance: float = 2.0,
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

        request = FeatureMatching.Request()
        request.camera = self.camera
        request.features = [person.features for person in self.blackboard.persons]
        if self.trim_last_person:
            # Receptionist flow: the newest registered guest has not sat down yet
            # and won't match anything at the sofa scan, so drop them.
            request.features = request.features[:-1]
        request.max_distance = self.max_distance
        request.target_frame = self.target_frame
        self.response = self.call_service_async(request)
        self.feedback_message = (
            f"Initialized feature matching (|features|={len(request.features)}, "
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
    Controls the pan-tilt unit for camera orientation.

    This node publishes pan-tilt control commands to orient the camera.
    Unlike other vision nodes, this inherits directly from Behaviour since
    it uses topic publishing rather than service calls.

    Attributes:
        x: Pan angle in degrees.
        y: Tilt angle in degrees.
        speed: Movement speed.
    """
    def __init__(self, name: str, x: float = 0.0, y: float = 0.0, speed: float = 0.0):
        super().__init__(name)
        self.x = x
        self.y = y
        self.speed = speed
        self.client = None
        self.mock_mode = is_node_mocked(self.__class__.__name__)
    
    def setup(self, **kwargs) -> None:
        # node should be passed down the tree from the root node
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.name, self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        # Skip creating publisher in mock mode
        if self.mock_mode:
            print(f"MOCK MODE: Skipping pan/tilt publisher creation")
            return
        
        # create publisher to a topic
        self.publisher = self.node.create_publisher(PanTiltCtrl, "/pan_tilt_ctrl", 10)


    def initialise(self) -> None:
        # Initialize counter regardless of mode
        self.cnt = 0
        
        # Skip ROS operations in mock mode
        if self.mock_mode:
            return
            
        # publish message
        msg = PanTiltCtrl()
        msg.x = self.x
        msg.y = self.y
        msg.speed = self.speed * 1.0  # convert to float, as the message expects a float

        self.publisher.publish(msg)
        self.logger.info(f"Publishing PanTiltCtrl with x: {self.x}, y: {self.y}, speed: {self.speed}")

        # call wait_seconds to start the timer in a separate thread
        # self.timer_future = asyncio.ensure_future(self.wait_seconds(2.0))

    def update(self) -> Status:
        # In mock mode, return success immediately
        if self.mock_mode:
            self.logger.info("MOCK: PanTiltCtrl completed immediately")
            return pytree.common.Status.SUCCESS
            
        # TODO: count 8 loops then return
        # if self.timer_future.done():
        if self.cnt > 3:
            # if the timer is done, cancel the future and return success
            # self.timer_future.cancel()
            self.cnt = 0
            self.logger.info("2 seconds passed, PanTiltCtrl finished")
            return pytree.common.Status.SUCCESS
        else:
            self.cnt += 1
            # if the timer is not done, return running
            self.logger.info("PanTiltCtrl still running")
            return pytree.common.Status.RUNNING
    
    async def wait_seconds(self, seconds):
        await asyncio.sleep(seconds)
        return True
    

class BtNode_TurnTo(BtNode_TurnPanTilt):
    """
    Turn to a specific point relevant to 'base_link'
    """
    def __init__(self, name: str,
                 bb_key_persons: str,
                 bb_key_points: str,
                 target_id: int = 0,
                 ):
        super().__init__(name, x=0, y=0, speed = 0.0)
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
        if len(self.blackboard.persons) <= self.target_id:
            self.feedback_message = f"Failed to initialize point_to"
            self.response = None
        else:
            point = self.blackboard.points[self.target_id]
            self.logger.info(f"Turning to point {point.point} with id {self.target_id} from blackboard {self.bb_key_points}")
            x = math.atan2(-point.point.y, max(point.point.x-0.25, 0.01)) 
            # x = math.atan2(self.blackboard.point.point.y, self.blackboard.point.point.x)
            y = 20.0
            msg = PanTiltCtrl()
            msg.x = x / math.pi * 180.0  # convert to degrees
            msg.y = y
            msg.speed = self.speed
            self.publisher.publish(msg)
            self.logger.info(f"Publishing PanTiltCtrl with x: {x}, y: {y}, speed: {self.speed}")
            self.feedback_message = f"Initialized TurnTo for point {point.point} with id {self.target_id} and pan tilt angle: x: {msg.x}, y: {msg.y}, speed: {self.speed}"
        self.cnt = 0


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
    Wrap the HRI `follow_head_action` (tinker_vision_msgs_26.action.FollowHeadAction) to maintain
    eye-contact with the closest face. Server returns success after a single gaze lock.

    Feedback schema is `{pan, tilt}` — not the BT canonical `{stage, stage_name, status, delay_limit}`.
    We override `feedback_callback` to stamp `last_feedback_time`, force `action_status=0`, and
    keep a generous `feedback_timeout`.
    """
    def __init__(self,
                 name: str,
                 action_name: str = "follow_head_action",
                 feedback_timeout_secs: float = 30.0,
                 wait_for_server_timeout_sec: float = -3.0,
                 ):
        super().__init__(name, FollowHeadAction, action_name, None, wait_for_server_timeout_sec)
        self._feedback_timeout_secs = feedback_timeout_secs

    def send_goal(self):
        if self.mock_mode:
            self.feedback_message = "MOCK: eye-contact goal sent"
            class MockFuture:
                def done(self):
                    return True
            self.send_goal_future = MockFuture()
            return
        goal = FollowHeadAction.Goal()
        goal.start_following = True
        self.send_goal_request(goal)
        self.feedback_message = "Eye-contact goal sent"

    def feedback_callback(self, msg: Any):
        feedback = msg.feedback
        self.last_feedback_time = time.time()
        self.feedback_timeout = self._feedback_timeout_secs
        self.action_status = 0
        pan = getattr(feedback, "pan", None)
        tilt = getattr(feedback, "tilt", None)
        self.feedback_message = f"eye-contact: pan={pan}, tilt={tilt}"

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
        if not self.mock_mode and self.goal_handle is not None:
            self.send_cancel_request()
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

