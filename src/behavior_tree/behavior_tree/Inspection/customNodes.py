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

    EOF handling is symmetric: a closed/piped stdin (no TTY) selects readable
    but ``readline()`` returns ``""``. ``initialise`` breaks its drain loop on
    that; ``update`` treats it as "not Enter" and keeps waiting (RUNNING),
    rather than falsely succeeding. Assumes an interactive, line-buffered
    (cooked) terminal, as provided by ``ros2 run``: stdin only selects readable
    once a full Enter-terminated line is available.
    """

    def __init__(self, name: str = "Press Enter to Succeed"):
        super().__init__(name=name)

    def _stdin_ready(self) -> bool:
        """True when stdin has data pending (non-blocking)."""
        return bool(select.select([sys.stdin], [], [], 0)[0])

    def initialise(self) -> None:
        # Drain stale/buffered stdin lines so a stray earlier keystroke can't
        # instantly satisfy the Enter-wait. Guard EOF (piped/closed stdin) so
        # the loop can't spin — a closed fd reads select-ready but readline()->"".
        while self._stdin_ready():
            if sys.stdin.readline() == "":  # EOF
                break
        self.logger.info(f"'{self.name}': Press ENTER to continue...")

    def update(self) -> py_trees.common.Status:
        if self._stdin_ready():
            if sys.stdin.readline() == "":  # EOF (closed/piped stdin), not Enter
                self.feedback_message = "stdin at EOF; still waiting for Enter"
                return py_trees.common.Status.RUNNING
            self.feedback_message = "Enter detected"
            return py_trees.common.Status.SUCCESS
        self.feedback_message = "Waiting for user to press Enter..."
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self.logger.info(f"'{self.name}': Terminating with status {new_status}.")
