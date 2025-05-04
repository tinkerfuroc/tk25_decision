import asyncio
from behavior_tree.TemplateNodes.BaseBehaviors import ServiceHandler
from behavior_tree.messages import ObjectDetection
import py_trees
from py_trees.common import Status, Access
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access
from py_trees.blackboard import Blackboard
import json
import time

from behavior_tree.messages import QuestionAnswer, Listen

from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy


class BtNode_GetCommand(ServiceHandler):
    def __init__(self,
                 name: str,
                 bb_dest_key: str,
                 service_name = "listen_service",
                 timeout : float = 5.0
                 ):
        super().__init__(name, service_name, Listen)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="message",
            access=Access.WRITE,
            remap_to=Blackboard.absolute_name("/", bb_dest_key)
        )
        self.timeout = timeout
    
    def initialise(self):
        request = Listen.Request()
        request.timeout = self.timeout
        self.response = self.client.call_async(request)
        self.feedback_message = f"Initialized GetCommand"
    
    def update(self):
        self.logger.debug(f"Update get command")
        if self.response.done():
            if self.response.result().status == 0:
                self.feedback_message = "got command"
                self.blackboard.message = self.response.result().message
                return Status.SUCCESS
            else:
                self.feedback_message = f"Get Command failed with error code {self.response.result().status}: {self.response.result().error_message}"
                return Status.FAILURE
        else:
            self.feedback_message = "Still getting command..."
            return Status.RUNNING


class BtNode_ScanForWavingPerson(ServiceHandler):

    def __init__(self, 
                 name: str,
                 bb_target: str,
                 service_name : str = "object_detection",
                 use_orbbec = True,
                 target_frame = "map"
                 ):
        """
        executed when creating tree diagram, therefor very minimal
        """
        super(BtNode_ScanForWavingPerson, self).__init__(name, service_name, ObjectDetection)
        self.bb_target = bb_target
        self.use_orbbec = use_orbbec
        self.target_frame = target_frame


    def setup(self, **kwargs):
        """
        setup for the node, recursively called with tree.setup()
        """
        ServiceHandler.setup(self, **kwargs)
        # attaches a blackboard (more like a shared memory section with key-value pair references) under the namespace Locations
        self.bb_write_client = self.attach_blackboard_client(name=f"ScanForWavingPerson")
         # register a key with the name of the object, with this client having write access
        self.bb_write_client.register_key(self.bb_target, access=py_trees.common.Access.WRITE)

        self.logger.debug(f"Setup ScanForWavingPerson")

    def initialise(self) -> None:
        """
        Called when the node is visited
        """
        request = ObjectDetection.Request()
        request.prompt = "person"
        request.flags = "find_waving_person"
        if self.use_orbbec:
            request.camera = "orbbec"
        else:
            request.camera = "realsense"
        request.target_frame = self.target_frame
        # setup things that needs to be cleared
        self.response = self.client.call_async(request)

        self.feedback_message = f"Initialized ScanForWavingPerson"

    def update(self):
        self.logger.debug(f"Update ScanForWavingPerson with self.orbbec = {self.use_orbbec}")
        if self.response.done():
            if self.response.result().status == 0:
                for obj in self.response.result().objects:
                    if obj.being_pointed == 3:
                        self.bb_write_client.set(self.bb_target, PointStamped(header=self.response.result().header, point=obj.centroid), overwrite=True)
                        self.feedback_message = f"Found waving person at ({obj.centroid.x: .4f}, {obj.centroid.y: .4f}, {obj.centroid.z: .4f})" + \
                                                f"in {self.response.result().header.frame_id}, stored in {self.bb_target}"
                        break
                else:
                    self.feedback_message = f"Service succeed, but no waving person found."
                    return py_trees.common.Status.FAILURE
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = f"Service failed with status {self.response.result().status}: {self.response.result().error_msg}"
                return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = "Still scanning..."
            return py_trees.common.Status.RUNNING
