"""Task-local retry diagnostics and barriers for asynchronous motion actions."""
import time
import py_trees

S = py_trees.common.Status


class PacedRetry(py_trees.decorators.Retry):
    """HRI-style bounded Retry, with a pause after each unsuccessful attempt."""
    def __init__(self, name, child, attempts, delay):
        super().__init__(name=name, child=child, num_failures=attempts)
        self.delay = delay
        self.resume_at = 0.0

    def initialise(self):
        super().initialise()
        self.resume_at = 0.0

    def tick(self):
        if self.status == S.RUNNING and time.monotonic() < self.resume_at:
            yield self
            return
        yield from super().tick()

    def update(self):
        result = super().update()
        if self.decorated.status == S.FAILURE and result == S.RUNNING:
            self.resume_at = time.monotonic() + self.delay
        return result


class ReportFailure(py_trees.decorators.Decorator):
    def __init__(self, name, child, context):
        super().__init__(name=name, child=child)
        self.context = context

    def update(self):
        if self.decorated.status == S.FAILURE:
            failed = [node for node in self.decorated.iterate()
                      if node.status == S.FAILURE and node.feedback_message]
            detail = failed[0].feedback_message if failed else self.decorated.feedback_message
            self.feedback_message = detail
            self.context.event("step_failed", step=self.decorated.name, detail=detail)
        return self.decorated.status


def action_idle(action):
    """A cancel response alone does not prove the action has stopped."""
    if getattr(action, "mock_mode", False):
        return action.status != S.RUNNING
    future = getattr(action, "send_goal_future", None)
    if future is None:
        return True
    if not future.done():
        return False
    handle = getattr(action, "goal_handle", None)
    if handle is None:
        # The response callback may not have run yet, even if the future is done.
        try:
            handle = future.result()
        except Exception:
            return False  # transport failure does not prove remote rejection
        if handle is None:
            return False
    if not handle.accepted:
        return True
    return getattr(action, "result_status", None) in (4, 5, 6)


def operation_idle(operation):
    if hasattr(operation, "action_type") or hasattr(operation, "send_goal_future"):
        return action_idle(operation)
    response = getattr(operation, "response", None)
    return response is None or (response.done() and not response.cancelled())


class MotionBarrier(py_trees.behaviour.Behaviour):
    """Do not start another base/arm goal until our previous goals terminate."""
    def __init__(self, context, channel="motion"):
        super().__init__("Wait for previous " + channel + " operation to stop")
        self.context = context
        self.channel = channel

    def initialise(self):
        self.started = time.monotonic()

    def update(self):
        if getattr(self.context, self.channel + "_blocked"):
            self.feedback_message = self.channel + " stop unconfirmed; operator intervention required"
            return S.FAILURE
        pending = [a for a in getattr(self.context, self.channel + "_actions") if not operation_idle(a)]
        if not pending:
            return S.SUCCESS
        self.feedback_message = "Waiting for terminal result: " + ", ".join(a.name for a in pending)
        if time.monotonic() - self.started >= self.context.config.get("cancel_timeout_seconds", 10.0):
            setattr(self.context, self.channel + "_blocked", True)
            self.context.event(self.channel + "_stop_unconfirmed", detail=self.feedback_message)
            return S.FAILURE
        return S.RUNNING
