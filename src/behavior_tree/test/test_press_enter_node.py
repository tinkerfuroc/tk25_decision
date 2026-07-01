import py_trees
from behavior_tree.Inspection import customNodes as C


class FakeStdin:
    """Minimal stdin stand-in: readline() pops a queued line, '' at EOF."""

    def __init__(self, lines=()):
        self._lines = list(lines)

    def feed(self, *lines):
        self._lines.extend(lines)

    def has_data(self):
        return len(self._lines) > 0

    def readline(self):
        return self._lines.pop(0) if self._lines else ""


def _install(monkeypatch, fake, ready):
    """Route the node's module-level select/sys at the fake. `ready` -> bool."""
    monkeypatch.setattr(C.sys, "stdin", fake)
    monkeypatch.setattr(
        C.select, "select",
        lambda r, w, x, t: (([fake] if ready() else []), [], []),
    )


def test_running_when_no_input(monkeypatch):
    fake = FakeStdin()
    _install(monkeypatch, fake, fake.has_data)
    node = C.BtNode_PressEnterToSucceed()
    node.initialise()
    assert node.update() == py_trees.common.Status.RUNNING


def test_success_and_consumes_on_enter(monkeypatch):
    fake = FakeStdin()
    _install(monkeypatch, fake, fake.has_data)
    node = C.BtNode_PressEnterToSucceed()
    node.initialise()                                    # nothing buffered
    assert node.update() == py_trees.common.Status.RUNNING
    fake.feed("\n")                                      # user presses Enter
    assert node.update() == py_trees.common.Status.SUCCESS
    assert fake.has_data() is False                      # the newline was consumed


def test_initialise_drains_stale_input(monkeypatch):
    fake = FakeStdin(["garbage\n"])                      # buffered before checkpoint
    _install(monkeypatch, fake, fake.has_data)
    node = C.BtNode_PressEnterToSucceed()
    node.initialise()                                    # must drain the stale line
    assert fake.has_data() is False
    assert node.update() == py_trees.common.Status.RUNNING  # waits for a fresh Enter


def test_initialise_drain_stops_on_eof(monkeypatch):
    # A closed/EOF stdin selects readable but readline() -> "". The drain loop
    # must break on EOF, not spin. `ready` is capped so a missing guard fails
    # the assertion instead of hanging the suite.
    fake = FakeStdin()                                   # readline always "" (EOF)
    calls = {"n": 0}

    def ready():
        calls["n"] += 1
        return calls["n"] <= 5

    _install(monkeypatch, fake, ready)
    node = C.BtNode_PressEnterToSucceed()
    node.initialise()
    assert calls["n"] <= 2                               # broke on first EOF read


def test_update_returns_running_on_eof(monkeypatch):
    # A closed/piped stdin selects readable but readline() -> "" (EOF).
    # update() must treat EOF as "not Enter" and keep waiting, never SUCCESS.
    fake = FakeStdin()  # readline() always "" (EOF)
    _install(monkeypatch, fake, lambda: True)  # always "ready"
    node = C.BtNode_PressEnterToSucceed()
    node.initialise()  # drain loop breaks immediately on the first EOF read
    assert node.update() == py_trees.common.Status.RUNNING
