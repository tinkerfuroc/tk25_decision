import py_trees
from typing import Any
# from asyncio.tasks import wait_for
from behavior_tree.messages import action_msgs  # Import from our conditional import system
from behavior_tree.config import (
    is_node_mocked,
    announce_node_action,
    get_node_mock_interaction_mode,
    get_mock_teleop_params,
)
import rclpy.action
import time
import sys
import tty
import termios
from py_trees_ros import exceptions
import sys
import tty
import termios

class ActionHandler(py_trees.behaviour.Behaviour):
    """
    Blackboard variable should already exist when this node is initialized, use py_trees.behaviours.WaitForBlackboardVariable as a guard if unsure
    adapted from py_trees_ros.action_clients.FromBlackboard
    """
    def __init__(self,
                 name: str,
                 action_type: Any,
                 action_name: str,
                 key: str,
                 wait_for_server_timeout_sec: float=-3.0,
                 action_timeout_ticks:int = 0
                 ):
        super(ActionHandler, self).__init__(name)
        self.action_type = action_type
        self.action_name = action_name
        self.wait_for_server_timeout_sec = wait_for_server_timeout_sec
        
        # Check if this specific node should be mocked
        self.mock_mode = is_node_mocked(self.__class__.__name__)
        self.mock_interaction_mode = get_node_mock_interaction_mode(self.__class__.__name__)
        
        # For mock mode keyboard press
        self._mock_pressed = False
        self._old_settings = None
        self._mock_teleop_node = None
        
        if key is not None:
            self.blackboard = self.attach_blackboard_client(name=self.name)
            self.blackboard.register_key(
                key="goal",
                access=py_trees.common.Access.READ,
                # make sure to namespace it if not already
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key)
            )

        self.node = None
        self.action_client = None

        self.status_strings = {
                action_msgs.GoalStatus.STATUS_UNKNOWN : "STATUS_UNKNOWN",  # noqa
                action_msgs.GoalStatus.STATUS_ACCEPTED : "STATUS_ACCEPTED",  # noqa
                action_msgs.GoalStatus.STATUS_EXECUTING: "STATUS_EXECUTING",  # noqa
                action_msgs.GoalStatus.STATUS_CANCELING: "STATUS_CANCELING",  # noqa
                action_msgs.GoalStatus.STATUS_SUCCEEDED: "STATUS_SUCCEEDED",  # noqa
                action_msgs.GoalStatus.STATUS_CANCELED : "STATUS_CANCELED",  # noqa
                action_msgs.GoalStatus.STATUS_ABORTED  : "STATUS_ABORTED"  # noqa
            }

        self.action_timeout_ticks = action_timeout_ticks

    def setup(self, **kwargs):
        """
        Setup the action client services and subscribers.

        Args:
            **kwargs (:obj:`dict`): distribute arguments to this
               behaviour and in turn, all of it's children

        Raises:
            :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
            :class:`~py_trees_ros.exceptions.TimedOutError`: if the action server could not be found
        """
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Skip creating action client in mock mode
        if self.mock_mode:
            if self.mock_interaction_mode == "teleop":
                self._setup_mock_teleop_node()
            print(f"MOCK MODE: Skipping action client creation for {self.action_name}")
            return
        
        self.action_client = rclpy.action.ActionClient(
            node=self.node,
            action_type=self.action_type,
            action_name=self.action_name
        )
        result = None
        if self.wait_for_server_timeout_sec > 0.0:
            result = self.action_client.wait_for_server(timeout_sec=self.wait_for_server_timeout_sec)
        else:
            iterations = 0
            period_sec = -1.0*self.wait_for_server_timeout_sec
            while not result:
                iterations += 1
                result = self.action_client.wait_for_server(timeout_sec=period_sec)
                if not result:
                    self.node.get_logger().warning(
                        "waiting for action server ... [{}s][{}][{}]".format(
                            iterations * period_sec,
                            self.action_name,
                            self.qualified_name
                        )
                    )
        if not result:
            self.feedback_message = "timed out waiting for the server [{}]".format(self.action_name)
            self.node.get_logger().error("{}[{}]".format(self.feedback_message, self.qualified_name))
            raise exceptions.TimedOutError(self.feedback_message)
        else:
            self.feedback_message = "... connected to action server [{}]".format(self.action_name)
            self.node.get_logger().info("{}[{}]".format(self.feedback_message, self.qualified_name))
    
    def send_goal(self):
        """
        child classes should override this funciton to how they wish to process the blackboard variable and send the goal
        """
        # Handle mock mode
        if self.mock_mode:
            self.feedback_message = "MOCK: goal sent (mock mode)"
            # Create a mock send_goal_future that appears done
            class MockFuture:
                def done(self):
                    return True
            self.send_goal_future = MockFuture()
            return
            
        try:
            self.send_goal_request(self.blackboard.goal)
            self.feedback_message = "sent goal request"
        except KeyError:
            pass  # self.send_goal_future will be None, check on that

    def goal_timeout(self):
        """
        called when the action has not provided feedback for longer than the time specified in its last message
        return: status of the node after this happens
        """
        self.feedback_message = "goal timeout"
        return py_trees.common.Status.FAILURE
     
    def process_feedback(self, feedback):
        """
        process action feedback after the stage, status and delay_limit have all been recorded
        """
        pass
    
    def process_abnormal_feedback(self):
        """
        called during update when the previous feedback returned status code other than 0
        return: status of the node after this happens
        """
        self.feedback_message = f"abnormal feedback: status {self.action_status}, stage {self.action_stage}"
        return py_trees.common.Status.FAILURE

    def process_result(self):
        """
        process the returned results after the action goal has been completed
        return: status of node after this happens
        """
        if self.result_status == action_msgs.GoalStatus.STATUS_SUCCEEDED:  # noqa
            self.feedback_message = "successfully completed"
            return py_trees.common.Status.SUCCESS
        else:
            result = self.result_message.result
            self.feedback_message = f"failed with code {result.status}, msg: {result.error_msg}"
            return py_trees.common.Status.FAILURE
    
    def regular_update(self):
        """
        called during update when action is not compelted and feedback shows no abnomalties
        """
        return py_trees.common.Status.RUNNING

    def initialise(self):
        """
        Reset the internal variables and kick off a new goal request.
        """
        self.logger.debug("{}.initialise()".format(self.qualified_name))
        self._mock_pressed = False

        # initialise some temporary variables
        self.goal_handle = None
        self.send_goal_future = None
        self.get_result_future = None

        self.result_message = None
        self.result_status = None
        self.result_status_string = None

        self.action_status = 0
        self.action_stage = None

        self.last_feedback_time = time.time()
        self.feedback_timeout = 10000.0
        if self.mock_mode and self.mock_interaction_mode == "teleop" and self._mock_teleop_node is not None:
            self._mock_teleop_node.initialise()
        
        # In mock mode, set result_status to SUCCESS immediately
        if self.mock_mode:
            self.result_status = action_msgs.GoalStatus.STATUS_SUCCEEDED
            self.result_status_string = "MOCK_SUCCESS"
            # Create a mock result message
            class MockResultMessage:
                class Result:
                    status = 0
                    error_msg = ""
                result = Result()
            self.result_message = MockResultMessage()
            # Create a mock future
            class MockFuture:
                def done(self):
                    return True
            self.get_result_future = MockFuture()
        
        self.send_goal()

        self.counter = 0
    
    def wait_for_keypress_in_mock(self):
        """
        Helper method for mock mode - wait for keyboard press and return status.
        Returns RUNNING until key is pressed, then returns SUCCESS.
        """
        if not self.mock_mode:
            return None

        if self.mock_interaction_mode == "teleop" and self._mock_teleop_node is not None:
            if not self._mock_pressed:
                announce_node_action(self.name, self.__class__.__name__)
                self._mock_pressed = True
            return self._mock_teleop_node.update()
        if self.mock_interaction_mode == "immediate":
            return py_trees.common.Status.SUCCESS
        
        # Announce on first call (when _mock_pressed is False and _old_settings is None)
        if not self._mock_pressed and self._old_settings is None:
            announce_node_action(self.name, self.__class__.__name__)
            
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

    def _setup_mock_teleop_node(self):
        try:
            from behavior_tree.TemplateNodes.TeleopNodes import BtNode_MoveArmTeleop

            teleop_params = get_mock_teleop_params()
            self._mock_teleop_node = BtNode_MoveArmTeleop(
                name=f"{self.name}_mock_teleop",
                **teleop_params,
            )
            self._mock_teleop_node.setup(node=self.node)
            print(f"MOCK MODE: Using teleop interaction for {self.__class__.__name__}")
        except Exception as exc:
            self._mock_teleop_node = None
            self.mock_interaction_mode = "wait_keypress"
            print(
                f"WARNING: Failed to initialize teleop mock for {self.__class__.__name__}: {exc}. "
                "Falling back to wait_keypress."
            )

    def update(self):
        """
        Check only to see whether the underlying action server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.

        Returns:
            :class:`py_trees.common.Status`
        """
        self.logger.debug("{}.update()".format(self.qualified_name))
        
        # In mock mode, wait for keyboard press
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()

        if self.action_timeout_ticks != 0:
            self.counter += 1
            if self.counter > self.action_timeout_ticks:
                # TODO: abort the action here
                self.feedback_message = "action timeout"
                return py_trees.common.Status.FAILURE

        # processing errors
        if self.send_goal_future is None:
            self.feedback_message = "no goal to send"
            return py_trees.common.Status.FAILURE
        if self.goal_handle is not None and not self.goal_handle.accepted:
            # goal was rejected
            self.feedback_message = "goal rejected"
            return py_trees.common.Status.FAILURE
        
        # checking on how the action is going
        if self.result_status is None:
            # the action is still running, so we need to check wether feedback message has been delivered on time
            now = time.time()
            if now - self.last_feedback_time > self.feedback_timeout:
                return self.goal_timeout()
            # then check if the last feedback returned with the correct staus code
            if self.action_status != 0:
                return self.process_abnormal_feedback()
            return self.regular_update()
        elif (self.get_result_future is None) or (not self.get_result_future.done()):
            # should never get here
            # if there is a result status but the future is not done or there is no goal_response_callback yet
            self.node.get_logger().warn("got result, but future not yet done [{}]".format(self.qualified_name))
            return py_trees.common.Status.RUNNING
        else:
            # action has finished with a certain result
            self.node.get_logger().debug("goal result [{}]".format(self.qualified_name))
            self.node.get_logger().debug("  status: {}".format(self.result_status_string))
            self.node.get_logger().debug("  message: {}".format(self.result_message))
            return self.process_result()            

    def terminate(self, new_status: py_trees.common.Status):
        """
        If running and the current goal has not already succeeded, cancel it.

        Args:
            new_status: the behaviour is transitioning to this new status
        """
        # Clean up terminal settings if in mock mode
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
        
        self.logger.debug(
            "{}.terminate({})".format(
                self.qualified_name,
                "{}->{}".format(self.status, new_status) if self.status != new_status else "{}".format(new_status)
            )
        )
        if (
            self.status == py_trees.common.Status.RUNNING and
            new_status == py_trees.common.Status.INVALID
        ):
            self.send_cancel_request()

    def shutdown(self):
        """
        Clean up the action client when shutting down.
        """
        # Only destroy if action client exists (not in mock mode)
        if self.action_client is not None:
            self.action_client.destroy()

    ########################################
    # Action Client Methods
    ########################################
    def feedback_callback(self, msg: Any):
        """
        Default generator for feedback messages from the action server. This will
        update the behaviour's feedback message with a stringified version of the
        incoming feedback message.

        Args:
            msg: incoming feedback message (e.g. move_base_msgs.action.MoveBaseFeedback)
        """
        feedback = msg.feedback
        self.last_feedback_time = time.time()
        self.feedback_timeout = feedback.delay_limit
        self.action_status = feedback.status
        self.action_stage = feedback.stage
        self.process_feedback(feedback)   

    def send_goal_request(self, goal: Any):
        """
        Send the goal, get a future back and start lining up the
        chain of callbacks that will lead to a result.
        """
        self.feedback_message = "sending goal ..."
        self.node.get_logger().debug("{} [{}]".format(
            self.feedback_message,
            self.qualified_name
        ))
        self.send_goal_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
            # A random uuid is always generated, since we're not sending more than one
            # at a time, we don't need to generate and track them here
            # goal_uuid=unique_identifier_msgs.UUID(uuid=list(uuid.uuid4().bytes))
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: rclpy.task.Future):
        """
        Handle goal response, proceed to listen for the result if accepted.

        Args:
            future: incoming goal request result delivered from the action server
        """
        if future.result() is None:
            self.feedback_message = "goal request failed :[ [{}]\n{!r}".format(self.qualified_name, future.exception())
            self.node.get_logger().debug('... {}'.format(self.feedback_message))
            return
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.feedback_message = "goal rejected :( [{}]".format(self.qualified_name)
            self.node.get_logger().debug('... {}'.format(self.feedback_message))
            return
        else:
            self.feedback_message = "goal accepted :) [{}]".format(self.qualified_name)
            self.node.get_logger().debug("... {}".format(self.feedback_message))
            self.node.get_logger().debug("  {!s}".format(future.result()))

        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def send_cancel_request(self):
        """
        Send a cancel request to the server. This is triggered when the
        behaviour's status switches from :attr:`~py_trees.common.Status.RUNNING` to
        :attr:`~py_trees.common.Status.INVALID` (typically a result of a priority
        interrupt).
        """
        self.feedback_message = "cancelling goal ... [{}]".format(self.qualified_name)
        self.node.get_logger().debug(self.feedback_message)

        if self.goal_handle is not None:
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future: rclpy.task.Future):
        """
        Immediate callback for the result of a cancel request. This will
        set the behaviour's feedback message accordingly.

        Args:
            future: incoming cancellation result delivered from the action server
        """
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.feedback_message = "goal successfully cancelled [{}]".format(self.qualified_name)
        else:
            self.feedback_message = "goal failed to cancel [{}]".format(self.qualified_name)
        self.node.get_logger().debug('... {}'.format(self.feedback_message))

    def get_result_callback(self, future: rclpy.task.Future):
        """
        Immediate callback for the result, saves data into local variables so that
        the update method can react accordingly.

        Args:
            future: incoming goal result delivered from the action server
        """
        self.result_message = future.result()
        self.result_status = future.result().status
        self.result_status_string = self.status_strings[self.result_status]
