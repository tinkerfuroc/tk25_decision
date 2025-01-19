import py_trees
import threading
from py_trees.common import Status
from rclpy.node import Node
from typing import Any

class ServiceHandler(py_trees.behaviour.Behaviour):
    """
    Base class for all nodes which handle a service
    """
    def __init__(self, 
                 name: str,
                 service_name:str,
                 service_type: Any,
                 ):
        super(ServiceHandler, self).__init__(name=name)
        self.service_name = service_name
        self.service_type = service_type

        # guard the data
        self.data_guard = threading.Lock()

        self.node : Node = None

        self.response = None    # Future object for receiving response
        self.client = None      # service client
    
    def setup(self, **kwargs):
        # node should be passed down the tree from the root node
        try:
            self.node : Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.name, self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # create the service client and wait until it connects
        self.client = self.node.create_client(self.service_type, self.service_name)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.debug('service not available, waiting again...')
    
    def initialise(self):
        # some debugging info
        self.logger.debug("%s.initialise()" % self.__class__.__name__)
        # clear data
        with self.data_guard:
            self.msg = None
            # if self.clearing_policy == py_trees.common.ClearingPolicy.ON_INITIALISE:
            #     self.msg = None


class BtNode_WriteToBlackboard(py_trees.behaviour.Behaviour):
    def __init__(self,
                 name: str,
                 bb_namespace: str,
                 bb_key: str,
                 bb_source: str,
                 object: Any = None
                 ):
        super(BtNode_WriteToBlackboard, self).__init__(name=name)
        self.bb_namespace = bb_namespace
        self.bb_key = bb_key
        self.bb_source = bb_source
        self.object = object
    
    def setup(self, **kwargs: Any) -> None:
        try:
            self.node : Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.name, self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # attaches a blackboard (more like a shared memory section with key-value pair references) under the namespace Locations
        self.bb_write_client = self.attach_blackboard_client(name=f"Write {self.name}", namespace=self.bb_namespace)
         # register a key with the name of the object, with this client having write access
        self.bb_write_client.register_key(self.bb_key, access=py_trees.common.Access.WRITE)

        if self.object is None:
            self.bb_read_client = self.attach_blackboard_client(name="WriteToBlackboard")
            self.bb_read_client.register_key(self.bb_source, access=py_trees.common.Access.READ)

            # debugger info (shown with DebugVisitor)
            self.logger.debug(f"Setup Write to Blackboard, reading from {self.bb_source}")
        else:
            self.logger.debug(f"Setup Write to Blackboard, object already provided")
        
    def initialise(self) -> None:
        if self.object is None:
            try:
                self.object = self.bb_read_client.get(self.bb_source)
            except Exception as e:
                self.feedback_message = f"Write to blackboard reading object failed"
                raise e

        self.logger.debug(f"Setup write for namespace {self.bb_namespace}, key {self.bb_key}")
    
    def update(self) -> Status:
        self.logger.debug(f"Update writing to blackboard")

        try:
            self.bb_write_client.set(self.bb_key, self.object, overwrite=True)
            self.feedback_message = "Success writing to namespace {self.bb_namespace}, key {self.bb_key}"
            return py_trees.common.Status.SUCCESS
        except Exception as e:
            self.feedback_message = f"Writing to blackboard failed: {e}"
            return py_trees.common.Status.FAILURE


class BtNode_ClearBlackboard(py_trees.behaviour.Behaviour):
    def __init__(self,
                 name: str,
                 bb_namespace: str,
                 bb_key: str,
                 ):
        super(BtNode_ClearBlackboard, self).__init__(name=name)
        self.bb_namespace = bb_namespace
        self.bb_key = bb_key
    
    def setup(self, **kwargs: Any) -> None:
        try:
            self.node : Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.name, self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # attaches a blackboard (more like a shared memory section with key-value pair references) under the namespace Locations
        self.bb_write_client = self.attach_blackboard_client(name=f"Write {self.name}", namespace=self.bb_namespace)
         # register a key with the name of the object, with this client having write access
        self.bb_write_client.register_key(self.bb_key, access=py_trees.common.Access.WRITE)

        self.logger.debug(f"Clear blackboard setup for namespace {self.bb_namespace}: {self.bb_key}")
        
    def initialise(self) -> None:
        self.logger.debug(f"Setup write for namespace {self.bb_namespace}, key {self.bb_key}")
    
    def update(self) -> Status:
        self.logger.debug(f"Update writing to blackboard")

        try:
            self.bb_write_client.set(self.bb_key, None, overwrite=True)
            self.feedback_message = "Success clearing namespace {self.bb_namespace}, key {self.bb_key}"
            return py_trees.common.Status.SUCCESS
        except Exception as e:
            self.feedback_message = f"Clearing blackboard failed: {e}"
            return py_trees.common.Status.FAILURE    


class BtNode_CheckIfEmpty(py_trees.behaviour.Behaviour):
    def __init__(self,
                 name: str,
                 bb_source: str,
                 ):
        super(BtNode_CheckIfEmpty, self).__init__(name=name)
        self.bb_source = bb_source
    
    def setup(self, **kwargs: Any) -> None:
        try:
            self.node : Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.name, self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.bb_read_client = self.attach_blackboard_client(name="WriteToBlackboard")
        self.bb_read_client.register_key(self.bb_source, access=py_trees.common.Access.READ)

        # debugger info (shown with DebugVisitor)
        self.logger.debug(f"Setup Check if empty for {self.bb_source}")
        
    def initialise(self) -> None:
        try:
            self.object = self.bb_read_client.get(self.bb_source)
        except Exception as e:
            self.feedback_message = f"Check if empty reading object failed"
            raise e

        self.logger.debug(f"Checking {self.bb_namespace}, key {self.bb_key}")
    
    def update(self) -> Status:
        self.logger.debug(f"Updating check blackboard")

        if self.object:
            self.feedback_message = "{self.bb_source} not empty"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "{self.bb_source} is empty"
            return py_trees.common.Status.FAILURE