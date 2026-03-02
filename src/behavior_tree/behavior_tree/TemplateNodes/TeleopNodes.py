import select
import sys
import termios
import threading
import tty

import py_trees as pytree
from py_trees.common import Status

from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from behavior_tree.config import get_mock_teleop_params


class BtNode_MoveArmTeleop(pytree.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        dof: int = None,
        cartesian_command_in_topic: str = None,
        joint_command_in_topic: str = None,
        gripper_command_in_topic: str = None,
        command_frame: str = None,
        linear_speed: float = None,
        angular_speed: float = None,
        joint_speed: float = None,
        linear_speed_step: float = None,
        angular_speed_step: float = None,
        joint_speed_step: float = None,
        min_speed: float = None,
        max_speed: float = None,
        twist_keymap: dict = None,
        joint_keymap: dict = None,
    ):
        super().__init__(name=name)
        cfg = get_mock_teleop_params()

        def _resolve(value, key, fallback):
            if value is not None:
                return value
            if key in cfg:
                return cfg[key]
            return fallback

        self.dof = _resolve(dof, "dof", 7)
        self.cartesian_command_in_topic = _resolve(
            cartesian_command_in_topic, "cartesian_command_in_topic", "/servo_server/delta_twist_cmds"
        )
        self.joint_command_in_topic = _resolve(
            joint_command_in_topic, "joint_command_in_topic", "/servo_server/delta_joint_cmds"
        )
        self.gripper_command_in_topic = _resolve(
            gripper_command_in_topic, "gripper_command_in_topic", "/gripper_servo_cmd"
        )
        self.command_frame = _resolve(command_frame, "command_frame", "link_base")
        self.linear_speed = _resolve(linear_speed, "linear_speed", 0.6)
        self.angular_speed = _resolve(angular_speed, "angular_speed", 0.6)
        self.joint_speed = _resolve(joint_speed, "joint_speed", 1.0)
        self.linear_speed_step = _resolve(linear_speed_step, "linear_speed_step", 0.05)
        self.angular_speed_step = _resolve(angular_speed_step, "angular_speed_step", 0.05)
        self.joint_speed_step = _resolve(joint_speed_step, "joint_speed_step", 0.1)
        self.min_speed = _resolve(min_speed, "min_speed", 0.05)
        self.max_speed = _resolve(max_speed, "max_speed", 3.0)
        self.twist_keymap = self._build_twist_keymap(
            twist_keymap if twist_keymap is not None else cfg.get("twist_keymap")
        )
        self.joint_keymap = self._build_joint_keymap(
            joint_keymap if joint_keymap is not None else cfg.get("joint_keymap")
        )

        self.node = None
        self.twist_pub = None
        self.joint_pub = None
        self.gripper_pub = None
        self.servo_start_client = None

        self._old_settings = None
        self._printed_help = False
        self._input_thread = None
        self._stop_event = threading.Event()
        self._finished = False
        self._interrupted = False
        self._key_provider = None

    def set_key_provider(self, provider):
        """
        Inject external non-blocking key provider.
        Provider must return one character or None.
        """
        self._key_provider = provider

    def setup(self, **kwargs):
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__
            )
            raise KeyError(error_message) from e

        self.twist_pub = self.node.create_publisher(
            TwistStamped, self.cartesian_command_in_topic, 10
        )
        self.joint_pub = self.node.create_publisher(
            JointJog, self.joint_command_in_topic, 10
        )
        self.gripper_pub = self.node.create_publisher(
            Float32, self.gripper_command_in_topic, 10
        )
        self.servo_start_client = self.node.create_client(Trigger, "/servo_server/start_servo")

    def initialise(self) -> None:
        self._finished = False
        self._interrupted = False
        self._stop_event.clear()

        if self._key_provider is None and self._old_settings is None:
            self._old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

        self._start_servo()
        self._print_help()
        self.feedback_message = "Teleop active"

        if self._key_provider is None:
            if self._input_thread is not None and self._input_thread.is_alive():
                self._stop_event.set()
                self._input_thread.join(timeout=0.5)
                self._stop_event.clear()

            self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
            self._input_thread.start()

    def update(self) -> Status:
        if self._key_provider is not None:
            key = self._key_provider()
            if key is not None:
                if key in ("\n", "\r"):
                    self._publish_stop()
                    self._finished = True
                elif key == "\x03":
                    self._interrupted = True
                else:
                    self._process_key(key)
        if self._interrupted:
            raise KeyboardInterrupt
        if self._finished:
            self.feedback_message = "Teleop finished (Enter pressed)"
            return pytree.common.Status.SUCCESS
        return pytree.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self._stop_event.set()
        if self._input_thread is not None:
            self._input_thread.join(timeout=0.5)
            self._input_thread = None

        self._publish_stop()
        if self._old_settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
            except Exception:
                pass
            self._old_settings = None

    def _input_loop(self) -> None:
        while not self._stop_event.is_set() and not self._finished and not self._interrupted:
            if not select.select([sys.stdin], [], [], 0.1)[0]:
                continue
            key = sys.stdin.read(1)
            if key in ("\n", "\r"):
                self._publish_stop()
                self._finished = True
                return
            if key == "\x03":
                self._interrupted = True
                return
            self._process_key(key)

    def _process_key(self, key: str) -> None:
        if self._handle_speed_key(key):
            return
        if key == " ":
            self._publish_stop()
            self.feedback_message = "Stop command published"
            print("[MoveArmTeleop] key='space' -> stop (zero twist)")
            return
        if key == "g":
            self._publish_gripper(0.7)
            self.feedback_message = "Gripper open command published"
            print("[MoveArmTeleop] key='g' -> gripper=0.7 (open)")
            return
        if key == "h":
            self._publish_gripper(0.0)
            self.feedback_message = "Gripper close command published"
            print("[MoveArmTeleop] key='h' -> gripper=0.0 (close)")
            return

        joint_cmd = self._joint_command_from_key(key)
        if joint_cmd is not None:
            joint_name, velocity = joint_cmd
            self._publish_joint(joint_name, velocity)
            self.feedback_message = (
                f"Joint command published: key '{key}' -> {joint_name}, velocity={velocity:.3f}"
            )
            print(f"[MoveArmTeleop] key='{key}' -> {joint_name}, velocity={velocity:.3f}")
            return

        twist_cmd = self._twist_command_from_key(key)
        if twist_cmd is not None:
            lx, ly, lz, ax, ay, az = twist_cmd
            self._publish_twist(lx, ly, lz, ax, ay, az)
            self.feedback_message = (
                "Cartesian command published: "
                f"key '{key}' -> linear=({lx:.3f}, {ly:.3f}, {lz:.3f}), "
                f"angular=({ax:.3f}, {ay:.3f}, {az:.3f})"
            )
            print(
                "[MoveArmTeleop] "
                f"key='{key}' -> linear=({lx:.3f}, {ly:.3f}, {lz:.3f}), "
                f"angular=({ax:.3f}, {ay:.3f}, {az:.3f})"
            )
            return

        self.feedback_message = f"Ignored key '{repr(key)}'"

    def _handle_speed_key(self, key: str) -> bool:
        if key == "z":
            self.linear_speed = max(self.min_speed, self.linear_speed - self.linear_speed_step)
            self.feedback_message = f"Linear speed set to {self.linear_speed:.3f}"
            print(f"[MoveArmTeleop] key='z' -> linear_speed={self.linear_speed:.3f}")
            return True
        if key == "x":
            self.linear_speed = min(self.max_speed, self.linear_speed + self.linear_speed_step)
            self.feedback_message = f"Linear speed set to {self.linear_speed:.3f}"
            print(f"[MoveArmTeleop] key='x' -> linear_speed={self.linear_speed:.3f}")
            return True

        if key == "c":
            self.angular_speed = max(self.min_speed, self.angular_speed - self.angular_speed_step)
            self.feedback_message = f"Angular speed set to {self.angular_speed:.3f}"
            print(f"[MoveArmTeleop] key='c' -> angular_speed={self.angular_speed:.3f}")
            return True
        if key == "v":
            self.angular_speed = min(self.max_speed, self.angular_speed + self.angular_speed_step)
            self.feedback_message = f"Angular speed set to {self.angular_speed:.3f}"
            print(f"[MoveArmTeleop] key='v' -> angular_speed={self.angular_speed:.3f}")
            return True

        if key == "b":
            self.joint_speed = max(self.min_speed, self.joint_speed - self.joint_speed_step)
            self.feedback_message = f"Joint speed set to {self.joint_speed:.3f}"
            print(f"[MoveArmTeleop] key='b' -> joint_speed={self.joint_speed:.3f}")
            return True
        if key == "n":
            self.joint_speed = min(self.max_speed, self.joint_speed + self.joint_speed_step)
            self.feedback_message = f"Joint speed set to {self.joint_speed:.3f}"
            print(f"[MoveArmTeleop] key='n' -> joint_speed={self.joint_speed:.3f}")
            return True

        return False

    def _start_servo(self) -> None:
        if self.servo_start_client is None:
            return
        try:
            if self.servo_start_client.wait_for_service(timeout_sec=1.0):
                self.servo_start_client.call_async(Trigger.Request())
            else:
                self.node.get_logger().warning(
                    "MoveArmTeleop: /servo_server/start_servo service not available"
                )
        except Exception as exc:
            self.node.get_logger().warning(f"MoveArmTeleop: failed to start servo: {exc}")

    def _publish_stop(self) -> None:
        if self.twist_pub is None:
            return
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frame
        self.twist_pub.publish(msg)

    def _publish_gripper(self, value: float) -> None:
        if self.gripper_pub is None:
            return
        msg = Float32()
        msg.data = float(value)
        self.gripper_pub.publish(msg)

    def _publish_joint(self, joint_name: str, velocity: float) -> None:
        if self.joint_pub is None:
            return
        msg = JointJog()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "joint"
        msg.joint_names = [joint_name]
        msg.velocities = [float(velocity)]
        self.joint_pub.publish(msg)

    def _publish_twist(self, lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0) -> None:
        if self.twist_pub is None:
            return
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frame
        msg.twist.linear.x = float(lx)
        msg.twist.linear.y = float(ly)
        msg.twist.linear.z = float(lz)
        msg.twist.angular.x = float(ax)
        msg.twist.angular.y = float(ay)
        msg.twist.angular.z = float(az)
        self.twist_pub.publish(msg)

    def _joint_command_from_key(self, key: str):
        for idx, (joint_name, plus_key, minus_key) in enumerate(self.joint_keymap):
            if idx >= self.dof:
                break
            if key == plus_key:
                return joint_name, self.joint_speed
            if key == minus_key:
                return joint_name, -self.joint_speed
        return None

    def _twist_command_from_key(self, key: str):
        if key not in self.twist_keymap:
            return None
        cmd = self.twist_keymap[key]
        return (
            cmd[0] * self.linear_speed,
            cmd[1] * self.linear_speed,
            cmd[2] * self.linear_speed,
            cmd[3] * self.angular_speed,
            cmd[4] * self.angular_speed,
            cmd[5] * self.angular_speed,
        )

    def _print_help(self) -> None:
        if self._printed_help:
            return
        print("")
        print("MoveArmTeleop controls:")
        print(f"  Twist keys: {', '.join(sorted(self.twist_keymap.keys()))}")
        joint_pairs = [f"{p}/{n}({j} +/-)" for j, p, n in self.joint_keymap[: self.dof]]
        print(f"  Joint keys: {', '.join(joint_pairs)}")
        print("  Speeds:    z/x(linear -/+), c/v(angular -/+), b/n(joint -/+)")
        print(
            "             current "
            f"linear={self.linear_speed:.3f}, angular={self.angular_speed:.3f}, joint={self.joint_speed:.3f}"
        )
        print("  Gripper:   g(open), h(close)")
        print("  Stop:      space")
        print("  Finish:    Enter")
        print("")
        self._printed_help = True

    def _build_twist_keymap(self, configured):
        label_map = {
            "linear_x_pos": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            "linear_x_neg": (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            "linear_y_pos": (0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
            "linear_y_neg": (0.0, -1.0, 0.0, 0.0, 0.0, 0.0),
            "linear_z_pos": (0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
            "linear_z_neg": (0.0, 0.0, -1.0, 0.0, 0.0, 0.0),
            "angular_x_pos": (0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
            "angular_x_neg": (0.0, 0.0, 0.0, -1.0, 0.0, 0.0),
            "angular_y_pos": (0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
            "angular_y_neg": (0.0, 0.0, 0.0, 0.0, -1.0, 0.0),
            "angular_z_pos": (0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            "angular_z_neg": (0.0, 0.0, 0.0, 0.0, 0.0, -1.0),
        }
        default_map = {
            "w": "linear_x_pos",
            "s": "linear_x_neg",
            "a": "linear_y_pos",
            "d": "linear_y_neg",
            "r": "linear_z_pos",
            "f": "linear_z_neg",
            "j": "angular_x_neg",
            "l": "angular_x_pos",
            "i": "angular_y_pos",
            "k": "angular_y_neg",
            "u": "angular_z_pos",
            "o": "angular_z_neg",
        }
        source = configured if isinstance(configured, dict) else default_map
        output = {}
        for key, value in source.items():
            if not isinstance(key, str) or len(key) != 1:
                continue
            if isinstance(value, str) and value in label_map:
                output[key] = label_map[value]
            elif isinstance(value, (list, tuple)) and len(value) == 6:
                try:
                    output[key] = tuple(float(v) for v in value)
                except Exception:
                    continue
        return output if output else {k: label_map[v] for k, v in default_map.items()}

    def _build_joint_keymap(self, configured):
        default_pairs = [
            ("joint1", "1", "2"),
            ("joint2", "3", "4"),
            ("joint3", "5", "6"),
            ("joint4", "7", "8"),
            ("joint5", "9", "0"),
            ("joint6", "-", "="),
            ("joint7", "[", "]"),
        ]
        if not isinstance(configured, dict):
            return default_pairs
        output = []
        for joint_name in [f"joint{i}" for i in range(1, 8)]:
            entry = configured.get(joint_name)
            if not isinstance(entry, dict):
                continue
            pos = entry.get("pos")
            neg = entry.get("neg")
            if isinstance(pos, str) and len(pos) == 1 and isinstance(neg, str) and len(neg) == 1:
                output.append((joint_name, pos, neg))
        return output if output else default_pairs
