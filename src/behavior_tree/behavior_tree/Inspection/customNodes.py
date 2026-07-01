import select
import sys

import py_trees


class BtNode_PressEnterToSucceed(py_trees.behaviour.Behaviour):
    """
    Wait for the user to press Enter, then return SUCCESS.

    Ticks RUNNING until a deliberate Enter (a newline-terminated line) is read
    on stdin, then returns SUCCESS on that tick and consumes the line so no
    stray newline leaks to later readers. On ``initialise`` it drains any stale
    buffered input, so a keystroke pressed earlier (e.g. during the preceding
    door-wait or navigation) cannot instantly satisfy the wait.

    Assumes an interactive, line-buffered (cooked) terminal, as provided by
    ``ros2 run``: stdin only selects readable once a full Enter-terminated line
    is available.
    """

    def __init__(self, name: str = "Press Enter to Succeed"):
        super().__init__(name=name)

    def initialise(self) -> None:
        # Drain stale/buffered stdin lines so a stray earlier keystroke can't
        # instantly satisfy the Enter-wait. Guard EOF (piped/closed stdin) so
        # the loop can't spin — a closed fd reads select-ready but readline()->"".
        while select.select([sys.stdin], [], [], 0)[0]:
            if sys.stdin.readline() == "":  # EOF
                break
        self.logger.info(f"'{self.name}': Press ENTER to continue...")

    def update(self) -> py_trees.common.Status:
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()  # consume the line incl. the trailing '\n'
            self.feedback_message = "Enter detected"
            return py_trees.common.Status.SUCCESS
        self.feedback_message = "Waiting for user to press Enter..."
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self.logger.info(f"'{self.name}': Terminating with status {new_status}.")
