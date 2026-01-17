import py_trees
import py_trees_ros
import rclpy
from py_trees.common import Status
import sys
import termios
import tty
import threading


class BtNode_WaitKeyboardPress(py_trees.behaviour.Behaviour):
    def __init__(
            self,
            name: str,
            key: str = None
    ):
        super().__init__(name)
        self.key = key
        self.pressed = False
        self._input_thread = None
        self._stop_thread = False
        self._old_settings = None
    
    def _read_key_thread(self):
        """Background thread to read keyboard input."""
        while not self._stop_thread and not self.pressed:
            try:
                ch = sys.stdin.read(1)
                if ch:
                    if self.key and ch == self.key:
                        self.pressed = True
                    elif not self.key:
                        self.pressed = True
            except Exception:
                break
    
    def initialise(self) -> None:
        self.pressed = False
        self._stop_thread = False
        
        # Save old terminal settings and set to cbreak mode
        self._old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        
        if self.key:
            print(f"Press '{self.key}' to continue...")
        else:
            print('Press any key to continue...')
        
        # Start background thread to read keyboard input
        self._input_thread = threading.Thread(target=self._read_key_thread, daemon=True)
        self._input_thread.start()

    def update(self) -> Status:
        if self.pressed:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status: Status) -> None:
        """Restore terminal settings when behavior terminates."""
        self._stop_thread = True
        if self._old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
            self._old_settings = None
        if self._input_thread is not None:
            self._input_thread.join(timeout=0.1)
            self._input_thread = None
        

if __name__ == '__main__':
    rclpy.init()
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