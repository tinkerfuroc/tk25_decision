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
# This module provides a simple keyboard wait behavior for mock mode
# testing and interactive debugging. It waits for a specified key press
# before returning SUCCESS, enabling step-by-step execution of behavior
# trees without any real hardware dependencies.
# Dependencies.

import py_trees
import py_trees_ros
import rclpy
from py_trees.common import Status
import sys
import termios
import tty
import select


class BtNode_WaitKeyboardPress(py_trees.behaviour.Behaviour):
    """Behavior tree node that waits for a specific keyboard key press.

    This node blocks execution until the specified key is pressed,
    making it useful for mock mode testing and manual debugging.
    It sets the terminal to cbreak mode for non-blocking input.

    Attributes
    ----------
    key : str
        The key to wait for (default: 's').
    _pressed : bool
        Internal flag tracking if the key has been pressed.
    _old_settings : termios settings or None
        Saved terminal settings for restoration on terminate.

    Example
    -------
    >>> node = BtNode_WaitKeyboardPress(name="WaitForUser", key="s")
    >>> # In behavior tree: tree will pause until 's' is pressed
    """

    def __init__(
            self,
            name: str,
            key: str = None
    ):
        """Initialize the wait keyboard press behavior.

        Parameters
        ----------
        name : str
            The name of this behavior tree node.
        key : str, optional
            The key to wait for. Defaults to 's' if not specified.
        """
        super().__init__(name)
        self.key = key if key else 's'  # Default to 's' if no key specified
        self._pressed = False
        self._old_settings = None

    def initialise(self) -> None:
        """Reset internal state when behavior starts.

        Called by the behavior tree when this node is visited.
        Resets the pressed flag and terminal settings.
        """
        self._pressed = False
        self._old_settings = None

    def update(self) -> Status:
        """Check for keyboard input without blocking.

        This method is called on every tick of the behavior tree.
        It checks if the expected key has been pressed and returns
        the appropriate status.

        Returns
        -------
        Status
            SUCCESS if the key was pressed, RUNNING otherwise.
            Returns SUCCESS immediately if terminal setup fails.
        """
        if self._pressed:
            return py_trees.common.Status.SUCCESS

        # Setup terminal for non-blocking input on first update call
        if self._old_settings is None:
            try:
                self._old_settings = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())  # Use cbreak like mock nodes
                print(f"\n⌨️  Press '{self.key}' to continue...")
            except Exception as e:
                print(f"Warning: Could not set terminal mode: {e}")
                # If terminal setup fails, just succeed immediately
                return py_trees.common.Status.SUCCESS

        # Use select to check if input is available (non-blocking)
        if select.select([sys.stdin], [], [], 0)[0]:
            try:
                ch = sys.stdin.read(1)
                if ch == self.key:
                    self._pressed = True
                    print(f"✓ Key '{self.key}' pressed, continuing...\n")
                    # Restore terminal settings
                    if self._old_settings is not None:
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
                        self._old_settings = None
                    return py_trees.common.Status.SUCCESS
                elif ch == '\x03':  # Ctrl+C
                    print("\nCtrl+C detected, exiting...")
                    if self._old_settings is not None:
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
                        self._old_settings = None
                    raise KeyboardInterrupt
                else:
                    print(f"Wrong key '{repr(ch)}', press '{self.key}' to continue...")
            except KeyboardInterrupt:
                raise
            except Exception as e:
                print(f"Error reading input: {e}")

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        """Clean up terminal settings when behavior terminates.

        Restores the terminal to its original settings to prevent
        leaving the terminal in an inconsistent state.

        Parameters
        ----------
        new_status : Status
            The status the behavior is transitioning to.
        """
        if self._old_settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
            except Exception:
                pass
            self._old_settings = None
        

if __name__ == '__main__':
    # Standalone test: Run the node in a ROS2 behavior tree.
    # Press 'a' to complete the behavior, Ctrl+C to exit.
    rclpy.init()
    # test node in ros2
    node = BtNode_WaitKeyboardPress(name="TestWaitKey", key="a")
    tree = py_trees_ros.trees.BehaviourTree(node)
    tree.setup(node_name="root_node", timeout=15)

    # function for display the tree to standard output
    def print_tree(tree):
        print(py_trees.display.unicode_tree(root=tree.root, show_status=True))

    tree.tick_tock(period_ms=500.0, post_tick_handler=print_tree)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()