from behavior_tree.core.constants import (
    PRINT_BLACKBOARD,
    PRINT_DEBUG,
)


def run_tree(
    root_factory,
    *,
    period_ms: float,
    title: str,
    node_name: str = "root_node",
):
    """Run a behaviour tree created by ``root_factory`` until interrupted."""
    import signal
    import time
    import py_trees
    import py_trees_ros
    import rclpy
    from behavior_tree.core.visualization import create_post_tick_visualizer

    rclpy.init(args=None)

    # ProcessManager (track_web "Stop All") and most supervisors stop us with
    # SIGTERM, which Python does NOT turn into an exception — so without this the
    # graceful-shutdown `finally` never runs and any in-flight action goal (e.g.
    # TrackPerson) is never cancelled, leaving the tracker tracking after the BT
    # is stopped. Translate SIGTERM into KeyboardInterrupt (mirroring SIGINT) so
    # spin unwinds cleanly AND rclpy stays initialised, letting the finally below
    # flush the cancel. (We deliberately do NOT call rclpy.shutdown() here: that
    # would invalidate the context and make the cancel un-sendable.)
    def _sigterm_to_keyboardinterrupt(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sigterm_to_keyboardinterrupt)

    root = root_factory()
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node_name=node_name, timeout=15)
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title=title,
        print_blackboard=PRINT_BLACKBOARD,
    )

    if PRINT_DEBUG:
        py_trees.logging.level = py_trees.logging.Level.DEBUG

    tree.tick_tock(period_ms=period_ms, post_tick_handler=print_tree)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        shutdown_visualizer()
        # Cancel in-flight action goals (e.g. TrackPerson) BEFORE the node is
        # destroyed. tree.shutdown() also issues the cancel via terminate(INVALID),
        # but it then destroys the node in the same call, so the async
        # cancel_goal_async() request never reaches the server. Do it explicitly
        # here and spin briefly to flush it, while the node is alive and rclpy is
        # still up. Bounded well under ProcessManager's SIGTERM->SIGKILL grace.
        if rclpy.ok():
            # Stop the tick timer first so the flush-spin can't re-tick the tree.
            if getattr(tree, "timer", None) is not None:
                tree.timer.cancel()
            # RUNNING -> INVALID triggers each action node's send_cancel_request().
            tree.root.stop(py_trees.common.Status.INVALID)
            deadline = time.time() + 2.0
            while rclpy.ok() and time.time() < deadline:
                rclpy.spin_once(tree.node, timeout_sec=0.05)
        tree.shutdown()
        rclpy.try_shutdown()


def draw_tree(
    root_factory,
    *,
    with_blackboard_variables: bool = True,
):
    """Render a dot tree from a root factory."""
    import py_trees

    root = root_factory()
    py_trees.display.render_dot_tree(
        root,
        with_blackboard_variables=with_blackboard_variables,
    )
