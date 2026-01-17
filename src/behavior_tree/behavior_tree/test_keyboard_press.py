import py_trees
import py_trees_ros
import rclpy
from py_trees.common import Status

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
        

if __name__ == '__main__':
    # test node in ros2
    node = BtNode_WaitKeyboardPress(name="TestWaitKey", key="a")
    tree = py_trees_ros.trees.BehaviourTree(node)
    tree.setup(node_name="root_node", timeout=15)

    # function for display the tree to standard output
    def print_tree(tree):
        print(py_trees.display.unicode_tree(root=tree.root, show_status=True))
    
    tree.tick_tock(period_ms=500.0,post_tick_handler=print_tree)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()