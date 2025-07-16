import copy
from typing import Any
import py_trees
from rclpy.node import Node
import select
import sys

import time
from behavior_tree.messages import PointStamped

def is_enter_pressed():
    """Check if Enter key is pressed on Unix-like systems."""
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

class BtNode_PressEnterToSucceed(py_trees.behaviour.Behaviour):
    """
    A py_trees behavior that waits for the user to press the Enter key.

    This node will return RUNNING until Enter is pressed in the console where
    the script is executing. Once Enter is detected, it returns SUCCESS on that
    tick. It only prints the prompt once upon initialization.
    """
    def __init__(self, name: str = "Press Enter to Succeed"):
        """
        Initialises the behavior with a given name.
        """
        super().__init__(name=name)
        self.prompt_printed = False

    def initialise(self) -> None:
        """
        This method is called once when the behavior becomes active.
        It prints the prompt for the user.
        """
        self.logger.info(f"'{self.name}': Press ENTER to return SUCCESS...")
        self.prompt_printed = True

    def update(self) -> py_trees.common.Status:
        """
        Called on every tick. Checks for keyboard input without blocking.

        Returns:
            - py_trees.common.Status.RUNNING if Enter has not been pressed.
            - py_trees.common.Status.SUCCESS if Enter has been pressed.
        """
        self.logger.debug(f"'{self.name}': Updating and checking for input.")

        if is_enter_pressed():
            self.feedback_message = "Enter key detected!"
            self.logger.info(f"'{self.name}': {self.feedback_message}")
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "Waiting for user to press Enter..."
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """
        Called once when the behavior transitions to a non-RUNNING state.
        """
        self.logger.info(
            f"'{self.name}': Terminating with status {new_status}."
        )
        self.prompt_printed = False
