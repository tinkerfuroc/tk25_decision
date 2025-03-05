import py_trees as pytree
import time

# from tinker_decision_msgs.srv import ObjectDetection
# from tinker_vision_msgs.srv import ObjectDetection

from behavior_tree.messages import ObjectDetection, Object, FeatureExtraction, SeatRecommendation, FeatureMatching
from geometry_msgs.msg import PointStamped
from py_trees.common import Status

from .BaseBehaviors import ServiceHandler
from .structs import Person


class BtNode_ScanFor(ServiceHandler):

    # # deprecated
    # def __init__(self, 
    #              name: str,
    #              bb_source: str,
    #              bb_namespace: str,
    #              bb_key:str,
    #              service_name : str = "object_detection",
    #              object: str = None,
    #              use_orbbec = True,
    #              transform_to_map = False
    #              ):
    #     """
    #     executed when creating tree diagram, therefor very minimal
    #     """
    #     super(BtNode_ScanFor, self).__init__(name, service_name, ObjectDetection)
    #     self.bb_namespace = bb_namespace
    #     self.bb_key = bb_key
    #     self.bb_source = bb_source
    #     self.object = object
    #     self.use_orbbec = use_orbbec
    #     self.transform_to_map = transform_to_map
    #     self.read = True
    #     if self.object is not None:
    #         self.read = False
    
    # def __init__(self, behavior_name: str, key_obj_type: str, key_object_position: str, service_name: str = "object_detection", use_orbbec: bool = True, transform_to_map: bool = False):
    #     super(BtNode_ScanFor, self).__init__(behavior_name, service_name, ObjectDetection)
    #     self.bb_source = key_obj_type
    #     self.bb_namespace = None
    #     self.bb_key = key_object_position
    #     self.object = None
    #     self.use_orbbec = use_orbbec
    #     self.transform_to_map = transform_to_map
    #     self.read = True
    
    def __init__(self, name: str, obj_type: str, key_object_position: str, service_name: str = "object_detection", use_orbbec: bool = True, transform_to_map: bool = False):
        super(BtNode_ScanFor, self).__init__(name, service_name, ObjectDetection)
        self.bb_source = None
        self.bb_namespace = None
        self.bb_key = key_object_position
        self.object = obj_type
        self.use_orbbec = use_orbbec
        self.transform_to_map = transform_to_map
        self.read = False


    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)
        # attaches a blackboard (more like a shared memory section with key-value pair references) under the namespace Locations
        self.bb_write_client = self.attach_blackboard_client(name=f"ScanFor", namespace=self.bb_namespace)
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
        if self.use_orbbec:
            request.camera = "orbbec"
        else:
            request.camera = "realsense"
        if self.transform_to_map:
            request.target_frame = "map"
        # setup things that needs to be cleared
        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized ScanFor for {self.object}"

    def update(self):
        self.logger.debug(f"Update ScanFor {self.object} with self.orbbec = {self.use_orbbec}")
        if self.response.done():
            if self.response.result().status == 0:
                self.bb_write_client.set(self.bb_key, self.response.result(), overwrite=True)
                self.feedback_message = f"Found object, stored to blackboard {self.bb_namespace} / {self.bb_key}"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Scanning for {self.object} failed with error code {self.response.result().status}: {self.response.result().error_msg}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still scanning..."
            return pytree.common.Status.RUNNING


class BtNode_TrackPerson(ServiceHandler):

    def __init__(self, 
                 name: str,
                 bb_namespace: str,
                 bb_key:str,
                 service_name : str = "object_detection",
                 use_orbbec = True,
                 transform_to_map = True,
                 ):
        """
        executed when creating tree diagram, therefor very minimal
        """
        super(BtNode_TrackPerson, self).__init__(name, service_name, ObjectDetection)
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
        request = ObjectDetection.Request()
        request.prompt = "person"
        if self.use_orbbec:
            request.camera = "orbbec"
        else:
            request.camera = "realsense"
        if self.transform_to_map:
            request.target_frame = "map"

        if self.person_id is None:
            request.flags = "register_person"
        
        self.response = self.client.call_async(request)

    def initialise(self) -> None:
        """
        Called when the node is visited
        """
        
        if self.do_send_tracking:
            self.send_request()
        self.do_send_tracking = True

        self.logger.debug(f"Initialized Track Person with person id {self.person_id}")
        self.feedback_message = f"Initialized Track Person"

    def update(self):
        self.logger.debug(f"Update Track Person")

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

    # def __init__(self, 
    #              name: str,
    #              bb_source,
    #              bb_namespace: str,
    #              bb_key:str,
    #              service_name:str = "object_detection",
    #              object:str = None
    #              ):
    #     """
    #     executed when creating tree diagram, therefor very minimal
    #     """
    #     super(BtNode_FindObj, self).__init__(name, service_name, ObjectDetection)
    #     self.bb_namespace = bb_namespace
    #     self.bb_key = bb_key
    #     self.bb_source = bb_source
    #     self.object = object
    #     self.read = True
    #     if self.object is not None:
    #         self.read = False
    
    def __init__(self, name: str, obj_type: str, key_extracted_obj_segment: str, service_name: str = "object_detection"):
        super(BtNode_FindObj, self).__init__(name, service_name, ObjectDetection)
        self.bb_source = None
        self.bb_namespace = None
        self.bb_key = key_extracted_obj_segment
        self.object = obj_type
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
        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized FindObj for {self.object}"

    def update(self):
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
        request = FeatureExtraction.Request()
        request.camera = self.camera
        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized Feature Extraction"
    
    def update(self) -> Status:
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
        request = SeatRecommendation.Request()
        request.camera = self.camera
        request.names = []
        request.features = []
        if self.blackboard.persons is not None:
            # minus one because the newest registered person is not yet seated and thus will not be in the picture
            for i in range(len(self.blackboard.persons) - 1):
                request.names.append(self.blackboard.persons[i].name)
                request.features.append(self.blackboard.persons[i].features)
        self.response = self.client.call_async(request)
        self.feedback_message = f"Initialized seat recommendation"
    
    def update(self) -> Status:
        self.logger.debug(f"Updated SeatRecommendation")
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


class BtNode_FeatureMatching(ServiceHandler):
    def __init__(self,
                 name: str,
                 bb_dest_key: str,
                 bb_persons_key: str,
                 service_name: str = "feature_matching_service",
                 use_orbbec: bool = True,
                 max_distance: float = 2.0,
                 target_frame: str = "base_link"
                 ):
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

        self.node = None
    
    def initialise(self):
        request = FeatureMatching.Request()
        request.camera = self.camera
        request.features = [person.features for person in self.blackboard.persons]
        request.features = request.features[:-1] # remove last person as that is the new guest who is not seated yet
        request.max_distance = self.max_distance
        request.target_frame = self.target_frame
        self.response = self.client.call_async(request)
        self.feedback_message = f"Initializec feature matching"

    def update(self) -> Status:
        self.logger.debug(f"Updated Feature Matching")
        if self.response.done():
            result : FeatureMatching.Response = self.response.result()
            if result.status == 0:
                self.blackboard.centroids = result.centroids
                self.feedback_message = f"Centroids: {result.centroids}"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Feature Matching failed with error code {result.status}: {result.error_msg}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still matching features..."
            return pytree.common.Status.RUNNING
