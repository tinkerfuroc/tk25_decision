import py_trees
from py_trees.common import Status
import threading
from rclpy.node import Node
from typing import Any
from behavior_tree.config import is_mock_mode
import sys
import tty
import termios


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
        self.mock_mode = is_mock_mode()

        # guard the data
        self.data_guard = threading.Lock()

        self.node : Node = None

        self.response = None    # Future object for receiving response
        self.client = None      # service client
        
        # For mock mode keyboard press
        self._mock_pressed = False
        self._old_settings = None
    
    def setup(self, **kwargs):
        print("Setting up service handler %s for node name %s" % (self.service_name, self.name))
        # node should be passed down the tree from the root node
        try:
            self.node : Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.name, self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Skip creating service client in mock mode
        if self.mock_mode:
            print(f"MOCK MODE: Skipping service client creation for {self.service_name}")
            return

        # create the service client and wait until it connects
        print("Connecting to service %s" % self.service_name)
        self.client = self.node.create_client(self.service_type, self.service_name)
        print("Created client for service %s" % self.service_name)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.debug('service not available, waiting again...')
        print("Finished setting up service handler")

    def initialise(self):
        # some debugging info
        self.logger.debug("%s.initialise()" % self.__class__.__name__)
        # clear data
        with self.data_guard:
            self.msg = None
            self.response = None  # Clear response for mock mode
            # if self.clearing_policy == py_trees.common.ClearingPolicy.ON_INITIALISE:
            #     self.msg = None
    
    def call_service_async(self, request):
        """
        Helper method to call service asynchronously.
        Returns None in mock mode.
        """
        if self.mock_mode:
            return None
        return self.client.call_async(request)
    
    def wait_for_keypress_in_mock(self):
        """
        Helper method for mock mode - wait for keyboard press and return status.
        Returns RUNNING until key is pressed, then returns SUCCESS.
        """
        if not self.mock_mode:
            return None
            
        if self._mock_pressed:
            return py_trees.common.Status.SUCCESS
            
        # Setup terminal for non-blocking input on first call
        if self._old_settings is None:
            self._old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        
        # Check if key is pressed (non-blocking)
        import select
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.read(1)
            self._mock_pressed = True
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
            self._old_settings = None
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """
        Clean up terminal settings if in mock mode.
        """
        if self.mock_mode and self._old_settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
                self._old_settings = None
            except:
                pass
        super().terminate(new_status)
    
    def update(self):
        """
        Default update for mock mode - return SUCCESS immediately.
        Override this in subclasses for real behavior.
        """
        if self.mock_mode:
            self.feedback_message = f"MOCK: Service {self.service_name} completed"
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


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

        # attaches a blackboard (more like a shared memory section with key-value pair references) under the namespace Locations
        self.bb_write_client = self.attach_blackboard_client(name=f"Write {self.name}", namespace=self.bb_namespace)
         # register a key with the name of the object, with this client having write access
        self.bb_write_client.register_key(self.bb_key, access=py_trees.common.Access.WRITE)
    
    def setup(self, **kwargs: Any) -> None:
        try:
            self.node : Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.name, self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability

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
            self.feedback_message = f"Success writing to namespace {self.bb_namespace}, key {self.bb_key}"
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


class BtNode_WaitTicks(py_trees.behaviour.Behaviour):
    def __init__(
            self,
            name: str,
            ticks: int
    ):
        super().__init__(name)
        self.n_ticks = ticks
        self.counter = 0
    
    def initialise(self) -> None:
        self.counter = 0
    
    def update(self) -> Status:
        self.counter += 1
        
        if self.counter > self.n_ticks:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
        
class BtNode_WaitKeyboardPress(py_trees.behaviour.Behaviour):
    def __init__(
            self,
            name: str,
            key: str = None
    ):
        super().__init__(name)
        self.key = key
        self.pressed = False
    
    def initialise(self) -> None:
        self.pressed = False
        if self.key:
            print(f"Press '{self.key}' to continue...")
        else:
            print('Press any key to continue...')

    def update(self) -> Status:
        import sys
        import select
        import termios
        import tty

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if self.key and ch == self.key:
                    self.pressed = True
                elif ch and not self.key:
                    self.pressed = True
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        if self.pressed:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING