import py_trees
import py_trees_ros
import rclpy
from py_trees.common import Status
import sys
import termios
import tty
import select


class BtNode_WaitKeyboardPress(py_trees.behaviour.Behaviour):
    def __init__(
            self,
            name: str,
            key: str = None
    ):
        super().__init__(name)
        self.key = key if key else 's'  # Default to 's' if no key specified
        self._pressed = False
        self._old_settings = None
    
    def initialise(self) -> None:
        """Initialize the behavior - prepare for keyboard input."""
        self._pressed = False
        self._old_settings = None

    def update(self) -> Status:
        """Check for keyboard input without blocking."""
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
        """Restore terminal settings when behavior terminates."""
        if self._old_settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
            except Exception:
                pass
            self._old_settings = None
        

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