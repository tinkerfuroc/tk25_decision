import py_trees
import time
from typing import List, Dict, Any
from behavior_tree.TemplateNodes.BaseBehaviors import ServiceHandler
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.ActionBase import ActionHandler
from behavior_tree.messages import ObjectDetection, TextToSpeech, Listen, PhraseExtraction
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import String

class BtNode_DetectCallingCustomer(ServiceHandler):
    def __init__(self, 
                 name: str,
                 bb_dest_key: str,
                 service_name: str = "object_detection",
                 timeout: float = 10.0):
        super().__init__(name, service_name, ObjectDetection)
        self.bb_dest_key = bb_dest_key
        self.timeout = timeout
        self.start_time = None
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="customer_location",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_dest_key)
        )
    
    def initialise(self):
        self.start_time = time.time()
        request = ObjectDetection.Request()
        request.prompt = "person waving or calling"
        request.flags = "detect_gesture"
        request.camera = "orbbec"
        request.target_frame = "map"
        self.response = self.client.call_async(request)
        self.feedback_message = "Looking for calling or waving customer"
    
    def update(self):
        if time.time() - self.start_time > self.timeout:
            self.feedback_message = "Timeout: No calling customer detected"
            return py_trees.common.Status.FAILURE
            
        if self.response.done():
            result = self.response.result()
            if result.status == 0 and len(result.objects) > 0:
                customer_obj = result.objects[0]
                customer_pose = PoseStamped()
                customer_pose.header = result.header
                customer_pose.pose.position = customer_obj.centroid
                customer_pose.pose.orientation.w = 1.0
                
                self.blackboard.customer_location = customer_pose
                self.feedback_message = f"Detected calling customer at position"
                return py_trees.common.Status.SUCCESS
            else:
                self.initialise()
                return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.RUNNING

class BtNode_TakeOrder(ServiceHandler):
    def __init__(self, 
                 name: str,
                 bb_dest_key: str,
                 service_name: str = "phrase_extraction_service",
                 timeout: float = 15.0):
        super().__init__(name, service_name, PhraseExtraction)
        self.bb_dest_key = bb_dest_key
        self.timeout = timeout
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="customer_order",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_dest_key)
        )
    
    def initialise(self):
        request = PhraseExtraction.Request()
        request.timeout = self.timeout
        request.wordlist = [
            "apple", "banana", "orange", "milk", "coffee", "tea", 
            "water", "juice", "bread", "sandwich", "cake", "cookie"
        ]
        self.response = self.client.call_async(request)
        self.feedback_message = "Listening for customer order"
    
    def update(self):
        if self.response.done():
            result = self.response.result()
            if result.status == 0:
                self.blackboard.customer_order = result.phrase
                self.feedback_message = f"Received order: {result.phrase}"
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = f"Failed to understand order: {result.error_message}"
                return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.RUNNING

class BtNode_ConfirmOrder(BtNode_Announce):
    def __init__(self, 
                 name: str,
                 bb_order_key: str,
                 service_name: str = "announce"):
        super(BtNode_Announce, self).__init__(name, service_name, TextToSpeech)
        self.bb_order_key = bb_order_key
        self.bb_source = None
        self.given_msg = None
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="order",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_order_key)
        )
    
    def initialise(self):
        order = self.blackboard.order
        self.given_msg = f"I understand your order is {order}. Is this correct?"
        return super().initialise()

class BtNode_CommunicateWithBarman(BtNode_Announce):
    def __init__(self, 
                 name: str,
                 bb_order_key: str,
                 service_name: str = "announce"):
        super(BtNode_Announce, self).__init__(name, service_name, TextToSpeech)
        self.bb_order_key = bb_order_key
        self.bb_source = None
        self.given_msg = None
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="order",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_order_key)
        )
    
    def initialise(self):
        order = self.blackboard.order
        self.given_msg = f"Hello, I need to place an order: {order}. Please prepare these items."
        return super().initialise()

class BtNode_DetectTray(ServiceHandler):
    def __init__(self, 
                 name: str,
                 bb_dest_key: str,
                 service_name: str = "object_detection"):
        super().__init__(name, service_name, ObjectDetection)
        self.bb_dest_key = bb_dest_key
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="tray_location",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_dest_key)
        )
    
    def initialise(self):
        request = ObjectDetection.Request()
        request.prompt = "tray"
        request.flags = "scan"
        request.camera = "orbbec"
        request.target_frame = "map"
        self.response = self.client.call_async(request)
        self.feedback_message = "Looking for available tray"
    
    def update(self):
        if self.response.done():
            result = self.response.result()
            if result.status == 0 and len(result.objects) > 0:
                tray_obj = result.objects[0]
                tray_point = PointStamped()
                tray_point.header = result.header
                tray_point.point = tray_obj.centroid
                
                self.blackboard.tray_location = tray_point
                self.feedback_message = "Found available tray"
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = "No tray found"
                return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.RUNNING

class BtNode_ServeOrder(BtNode_Announce):
    def __init__(self, 
                 name: str,
                 bb_order_key: str,
                 service_name: str = "announce"):
        super(BtNode_Announce, self).__init__(name, service_name, TextToSpeech)
        self.bb_order_key = bb_order_key
        self.bb_source = None
        self.given_msg = None
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="order",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_order_key)
        )
    
    def initialise(self):
        order = self.blackboard.order
        self.given_msg = f"Here is your order: {order}. Enjoy your meal!"
        return super().initialise()