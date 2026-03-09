from .Constants import (
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
    import py_trees
    import py_trees_ros
    import rclpy
    from .visualization import create_post_tick_visualizer

    rclpy.init(args=None)
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
