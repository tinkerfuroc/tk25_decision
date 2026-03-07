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
# Base Behaviors Module
# =================
#
# This module provides the base class for service-based behavior tree nodes
# and utility nodes for blackboard operations. All service-based nodes
# inherit from ServiceHandler, which provides:
#
# - Automatic mock mode detection via config.py
# - ROS2 service client management
# - Keyboard-based interaction for mock mode
# - Optional TTS announcements during mock execution
#
# The mock mode system supports multiple interaction modes per node:
#
# - ``wait_keypress``: Wait for keyboard press before succeeding (default)
# - ``teleop``: Use teleoperation-style keyboard control for arm manipulation
# - ``immediate``: Return success immediately without user interaction
#
# Classes
# -------
ServiceHandler
    Base class for all nodes that call ROS2 services.
# BtNode_WriteToBlackboard
    Writes a value to the blackboard for inter-node communication.
# BtNode_ClearBlackboard
    Clears a value from the blackboard.
# BtNode_CheckIfEmpty
    Checks if a blackboard key has a value.
# BtNode_WaitTicks
    Waits for a specified number of behavior tree ticks.
# BtNode_WaitKeyboardPress
    Simple keyboard wait utility (also in WaitKeyPress.py).
#
# Usage
# -----
# Inherit from ServiceHandler and implement the update() method for real behavior,
# or rely on the built-in mock mode support:
#
# >>> class MyCustomNode(ServiceHandler):
# ...     def __init__(self, name, str):
# ...         super().__init__(name, service_name="my_service", service_type=MyServiceType)
# ...         # Mock mode is automatically handled
# ...
# ...     def update(self):
# ...         if self.mock_mode:
# ...             return self.wait_for_keypress_in_mock()
# ...         # Real implementation here
# ...         return py_trees.common.Status.RUNNING
#

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
    """Base class for all nodes that call ROS2 services.

    This class provides the foundation for service-based behavior tree nodes.
    It automatically handles mock mode detection and keyboard-based
    interaction when running without real hardware.

    The mock mode system supports three interaction modes:
    - ``wait_keypress``: Wait for keyboard press before succeeding
    - ``teleop``: Use teleoperation-style keyboard control for arm manipulation
    - ``immediate``: Return success immediately

    Attributes
    ----------
    service_name : str
        Name of the ROS2 service to connect to.
    service_type : Any
        The ROS2 service type class.
    mock_mode : bool
        Whether this node is running in mock mode.
    mock_interaction_mode : str
        The mock interaction mode ('wait_keypress', 'teleop', or 'immediate').
    mock_subsystem : str or None
        The subsystem this node belongs to (for keyboard routing).
    node : Node
        The ROS2 node instance.
    client : Client
        The ROS2 service client.
    response : Future
        The future object for the asynchronous service response.

    """

    def __init__(self,
                 name: str,
                 service_name: str,
                 service_type: Any,
                 ):
        """Initialize the ServiceHandler.

        Parameters
        ----------
        name : str
            The name of this behavior tree node.
        service_name : str
            The name of the ROS2 service to connect to.
        service_type : Any
            The ROS2 service type class.
        """
        super(ServiceHandler, self).__init__(name=name)
        self.service_name = service_name
        self.service_type = service_type

        # Check if this specific node should be mocked
        # Use the class name to determine mock status
        self.mock_mode = is_node_mocked(self.__class__.__name__)
        self.mock_interaction_mode = get_node_mock_interaction_mode(self.__class__.__name__)
        self.mock_subsystem = get_node_subsystem_name(self.__class__.__name__)

        # Guard the data for thread safety
        self.data_guard = threading.Lock()

        self.node: Node = None

        self.response = None    # Future object for receiving response
        self.client = None      # Service client

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
        self._mock_teleop_setup_error = None

    def setup(self, **kwargs):
        """Set up the service client and mock mode infrastructure.

        Called by the behavior tree during the setup phase. Creates the
        ROS2 service client or initializes mock mode infrastructure.

        Parameters
        ----------
        **kwargs : dict
            Must contain 'node' key with the ROS2 node instance.

        Raises
        ------
        KeyError
            If 'node' is not found in kwargs.
        """
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
                if self._mock_teleop_node is None:
                    warn = (
                        f"MOCK TELEOP SETUP FAILED [{self.__class__.__name__}/{self.name}]: "
                        f"{self._mock_teleop_setup_error or 'unknown error'}. "
                        "Falling back to wait_keypress."
                    )
                    self.feedback_message = warn
                    print(f"⚠ {warn}")
                    if self.node is not None:
                        self.node.get_logger().warning(warn)
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
        """Reset internal state when behavior starts.

        Called by the behavior tree when this node is visited.
        Resets mock mode state and clears any pending response.
        """
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
        elif self.mock_mode and self.mock_interaction_mode == "teleop":
            warn = (
                f"MOCK TELEOP INIT WARNING [{self.__class__.__name__}/{self.name}]: "
                "teleop backend is unavailable; node is running in wait_keypress fallback."
            )
            self.feedback_message = warn
            print(f"⚠ {warn}")
            if self.node is not None:
                self.node.get_logger().warning(warn)

    def call_service_async(self, request):
        """Call the ROS2 service asynchronously.

        Helper method to call service asynchronously. Returns None in
        mock mode instead of making a real service call.

        Parameters
        ----------
        request : Any
            The service request message.

        Returns
        -------
        Future or None
            The future object for the service response, or None in mock mode.
        """
        if self.mock_mode:
            return None
        return self.client.call_async(request)

    def wait_for_keypress_in_mock(self):
        """Handle mock mode interaction and return appropriate status.

        This method implements the three mock interaction modes:
        - ``immediate``: Auto-complete after a few ticks
        - ``teleop``: Use keyboard teleoperation for arm control
        - ``wait_keypress``: Wait for success key (default: ENTER)

        Returns
        -------
        Status or None
            RUNNING while waiting, SUCCESS when complete.
            Returns None if not in mock mode.
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
        if self._mock_input_controller.is_success_event(key):
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
                lambda: self._mock_input_controller.pop_keys(
                    self.mock_subsystem,
                    consumer_id=self._mock_consumer_id,
                    consumer_start_tick=self._mock_start_tick,
                    consumer_start_event=self._mock_start_event,
                    max_keys=128,
                )
            )
            print(f"MOCK MODE: Using teleop interaction for {self.__class__.__name__}")
        except Exception as exc:
            self._mock_teleop_node = None
            self._mock_teleop_setup_error = str(exc)
            self.mock_interaction_mode = "wait_keypress"
            warn = (
                f"Failed to initialize teleop mock for {self.__class__.__name__}/{self.name}: {exc}. "
                "Falling back to wait_keypress."
            )
            print(f"⚠ WARNING: {warn}")
            if self.node is not None:
                self.node.get_logger().warning(warn)
    
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
