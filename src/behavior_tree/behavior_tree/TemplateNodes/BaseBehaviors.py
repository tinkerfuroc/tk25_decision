import py_trees
from py_trees.common import Status
import threading
from rclpy.node import Node
from typing import Any
from behavior_tree.config import (
    is_node_mocked,
    announce_node_action,
    should_announce_movement,
    is_mock_tts_active,
    get_node_mock_interaction_mode,
    get_mock_teleop_params,
    get_node_subsystem_name,
    get_mock_keyboard_config,
)
from .MockInputController import get_mock_input_controller
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
        
        # Check if this specific node should be mocked
        # Use the class name to determine mock status
        self.mock_mode = is_node_mocked(self.__class__.__name__)
        self.mock_interaction_mode = get_node_mock_interaction_mode(self.__class__.__name__)
        self.mock_subsystem = get_node_subsystem_name(self.__class__.__name__)

        # guard the data
        self.data_guard = threading.Lock()

        self.node : Node = None

        self.response = None    # Future object for receiving response
        self.client = None      # service client
        
        # For mock mode keyboard press
        self._mock_pressed = False
        self._mock_announced = False
        self._old_settings = None
        self._mock_teleop_node = None
        self._mock_input_controller = get_mock_input_controller()
        self._mock_teleop_detailed_feedback = bool(
            get_mock_teleop_params().get("detailed_feedback", True)
        )
        self._mock_auto_ticks_required = 2
        self._mock_tick_counter = 0
        self._mock_consumer_id = f"{self.__class__.__name__}:{id(self)}"
        self._mock_start_tick = -1
        self._mock_start_event = -1
    
    def setup(self, **kwargs):
        print("Setting up service handler %s for node name %s" % (self.service_name, self.name))
        # node should be passed down the tree from the root node
        try:
            self.node : Node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.name, self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # If the service type originates from our mock_messages module but the node
        # wasn't explicitly marked as mocked in the config, force mock mode here so
        # we don't attempt to create a real ROS client for a non-ROS type.
        if not self.mock_mode:
            service_mod = getattr(self.service_type, '__module__', '')
            if isinstance(service_mod, str) and service_mod.startswith('behavior_tree.mock_messages'):
                print(f"WARNING: Service type for {self.service_name} comes from mock_messages; forcing mock mode to avoid creating client.")
                self.mock_mode = True

        # Skip creating service client in mock mode
        if self.mock_mode:
            self._mock_input_controller.configure(get_mock_keyboard_config())
            self._mock_input_controller.start()
            if self.mock_interaction_mode == "teleop":
                self._setup_mock_teleop_node()
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
        self._mock_pressed = False
        self._mock_announced = False
        self._mock_tick_counter = 0
        self._mock_start_tick = self._mock_input_controller.get_tick_index()
        self._mock_start_event = self._mock_input_controller.get_event_index()
        # clear data
        with self.data_guard:
            self.msg = None
            self.response = None  # Clear response for mock mode
            # if self.clearing_policy == py_trees.common.ClearingPolicy.ON_INITIALISE:
            #     self.msg = None
        if self.mock_mode and self.mock_interaction_mode == "teleop" and self._mock_teleop_node is not None:
            self._mock_teleop_node.initialise()
    
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

        if self.mock_interaction_mode == "immediate":
            self._mock_tick_counter += 1
            if self._mock_tick_counter <= self._mock_auto_ticks_required:
                self.feedback_message = f"MOCK: auto-completing ({self._mock_tick_counter}/{self._mock_auto_ticks_required})"
                return py_trees.common.Status.RUNNING
            if should_announce_movement(self.__class__.__name__) and is_mock_tts_active():
                self.feedback_message = "MOCK: waiting for TTS broadcast"
                return py_trees.common.Status.RUNNING
            self.feedback_message = "MOCK: auto-complete finished"
            return py_trees.common.Status.SUCCESS

        if self.mock_interaction_mode == "teleop" and self._mock_teleop_node is not None:
            if not self._mock_announced:
                announce_node_action(self.name, self.__class__.__name__)
                self._mock_announced = True
            status = self._mock_teleop_node.update()
            teleop_feedback = getattr(self._mock_teleop_node, "feedback_message", "")
            if self._mock_teleop_detailed_feedback and teleop_feedback:
                self.feedback_message = f"MOCK: {teleop_feedback}"
            else:
                if status == py_trees.common.Status.SUCCESS:
                    self.feedback_message = "MOCK: Teleop finished (Enter pressed)"
                else:
                    self.feedback_message = "MOCK: Teleop active"
            return status
        
        if not self._mock_announced:
            announce_node_action(self.name, self.__class__.__name__)
            self._mock_announced = True
            
        key = self._mock_input_controller.pop_key(
            self.mock_subsystem,
            consumer_id=self._mock_consumer_id,
            consumer_start_tick=self._mock_start_tick,
            consumer_start_event=self._mock_start_event,
        )
        if key in ("\n", "\r"):
            self._mock_pressed = True
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def _setup_mock_teleop_node(self):
        try:
            from behavior_tree.TemplateNodes.TeleopNodes import BtNode_MoveArmTeleop

            teleop_params = get_mock_teleop_params()
            self._mock_teleop_node = BtNode_MoveArmTeleop(
                name=f"{self.name}_mock_teleop",
                **teleop_params,
            )
            self._mock_teleop_node.setup(node=self.node)
            self._mock_teleop_node.set_key_provider(
                lambda: self._mock_input_controller.pop_key(
                    self.mock_subsystem,
                    consumer_id=self._mock_consumer_id,
                    consumer_start_tick=self._mock_start_tick,
                    consumer_start_event=self._mock_start_event,
                )
            )
            print(f"MOCK MODE: Using teleop interaction for {self.__class__.__name__}")
        except Exception as exc:
            self._mock_teleop_node = None
            self.mock_interaction_mode = "wait_keypress"
            print(
                f"WARNING: Failed to initialize teleop mock for {self.__class__.__name__}: {exc}. "
                "Falling back to wait_keypress."
            )
    
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
        if self.mock_mode and self._mock_teleop_node is not None:
            try:
                self._mock_teleop_node.terminate(new_status)
            except Exception:
                pass
        super().terminate(new_status)
    
    def update(self):
        """
        Default update for mock mode - return SUCCESS immediately.
        Override this in subclasses for real behavior.
        """
        if self.mock_mode:
            if self.mock_interaction_mode in ("wait_keypress", "teleop", "immediate"):
                return self.wait_for_keypress_in_mock()
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
