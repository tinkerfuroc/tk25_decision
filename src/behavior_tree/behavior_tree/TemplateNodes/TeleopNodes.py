import select
import sys
import termios
import threading
import time
import tty

import py_trees as pytree
from py_trees.common import Status

from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from behavior_tree.config import (
    get_mock_teleop_params,
    is_node_mocked,
    get_node_subsystem_name,
    get_mock_keyboard_config,
)
from .MockInputController import get_mock_input_controller


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
        speed_control_keymap: dict = None,
        gripper_keymap: dict = None,
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
        self.speed_control_keymap = self._build_speed_control_keymap(
            speed_control_keymap if speed_control_keymap is not None else cfg.get("speed_control_keymap")
        )
        self.gripper_keymap = self._build_gripper_keymap(
            gripper_keymap if gripper_keymap is not None else cfg.get("gripper_keymap")
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
        self._verbose_key_log = bool(cfg.get("verbose_key_log", True))
        self._mock_input_controller = get_mock_input_controller()
        self._forced_mock_mode = is_node_mocked(self.__class__.__name__)
        self._mock_subsystem = get_node_subsystem_name(self.__class__.__name__) or "mock_controls"
        self._mock_consumer_id = f"{self.__class__.__name__}:{id(self)}"
        self._mock_start_tick = -1
        self._mock_start_event = -1

    def set_key_provider(self, provider):
        """
        Inject external non-blocking key provider.
        Provider may return one character, a list of characters, or None.
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
        self._mock_start_tick = self._mock_input_controller.get_tick_index()
        self._mock_start_event = self._mock_input_controller.get_event_index()

        if self._key_provider is None and self._forced_mock_mode:
            self._mock_input_controller.configure(get_mock_keyboard_config())
            self._mock_input_controller.start()
            self._key_provider = lambda: self._mock_input_controller.pop_keys(
                self._mock_subsystem,
                consumer_id=self._mock_consumer_id,
                consumer_start_tick=self._mock_start_tick,
                consumer_start_event=self._mock_start_event,
                max_keys=128,
            )

        if self._key_provider is None and self._old_settings is None:
            self._old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

        self._start_servo()
        self._print_help()
        self.feedback_message = "Teleop active"

        if self._input_thread is not None and self._input_thread.is_alive():
            self._stop_event.set()
            self._input_thread.join(timeout=0.5)
            self._stop_event.clear()

        # Always process teleop input on a dedicated thread so key handling
        # remains responsive even when BT tick periods are large.
        self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self._input_thread.start()

    def update(self) -> Status:
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
            keys = None
            if self._key_provider is not None:
                provided = self._key_provider()
                if provided is None:
                    time.sleep(0.001)
                    continue
                if isinstance(provided, (list, tuple)):
                    keys = list(provided)
                else:
                    keys = [provided]
                if len(keys) == 0:
                    # In mock teleop, providers commonly return [] when idle.
                    # Sleep briefly to avoid busy-spinning and starving other
                    # threads (mock input router / BT tick thread).
                    time.sleep(0.001)
                    continue
            else:
                if not select.select([sys.stdin], [], [], 0.01)[0]:
                    continue
                keys = [sys.stdin.read(1)]

            for key in keys:
                tokens = self._event_tokens(key)
                if self._event_has_tokens(tokens, ("enter",)):
                    self._publish_stop()
                    self._finished = True
                    return
                if self._event_has_tokens(tokens, ("ctrl", "c")):
                    self._interrupted = True
                    return
                self._process_tokens(tokens)

    def _process_tokens(self, tokens: set[str]) -> None:
        if self._handle_speed_tokens(tokens):
            return
        if self._event_has_tokens(tokens, ("space",)):
            self._publish_stop()
            self.feedback_message = "Stop command published"
            if self._verbose_key_log:
                print("[MoveArmTeleop] key='space' -> stop (zero twist)")
            return
        if self._combo_active(self.gripper_keymap["open"], tokens):
            self._publish_gripper(0.75)
            self.feedback_message = "Gripper open command published"
            if self._verbose_key_log:
                print("[MoveArmTeleop] gripper=0.75 (open)")
            return
        if self._combo_active(self.gripper_keymap["close"], tokens):
            self._publish_gripper(0.0)
            self.feedback_message = "Gripper close command published"
            if self._verbose_key_log:
                print("[MoveArmTeleop] gripper=0.0 (close)")
            return

        joint_cmds = self._joint_commands_from_tokens(tokens)
        if joint_cmds:
            self._publish_joint_multi(joint_cmds)
            joints_txt = ", ".join([f"{name}:{vel:.3f}" for name, vel in joint_cmds])
            self.feedback_message = f"Joint command published: {joints_txt}"
            if self._verbose_key_log:
                print(f"[MoveArmTeleop] joints -> {joints_txt}")
            return

        twist_cmd = self._twist_command_from_tokens(tokens)
        if twist_cmd is not None:
            lx, ly, lz, ax, ay, az = twist_cmd
            self._publish_twist(lx, ly, lz, ax, ay, az)
            self.feedback_message = (
                "Cartesian command published: "
                f"linear=({lx:.3f}, {ly:.3f}, {lz:.3f}), "
                f"angular=({ax:.3f}, {ay:.3f}, {az:.3f})"
            )
            if self._verbose_key_log:
                print(
                    "[MoveArmTeleop] "
                    f"linear=({lx:.3f}, {ly:.3f}, {lz:.3f}), "
                    f"angular=({ax:.3f}, {ay:.3f}, {az:.3f})"
                )
            return

        self.feedback_message = f"Ignored input '{'+'.join(sorted(tokens))}'"

    def _handle_speed_tokens(self, tokens: set[str]) -> bool:
        if self._combo_active(self.speed_control_keymap["linear_dec"], tokens):
            self.linear_speed = max(self.min_speed, self.linear_speed - self.linear_speed_step)
            self.feedback_message = f"Linear speed set to {self.linear_speed:.3f}"
            if self._verbose_key_log:
                print(f"[MoveArmTeleop] linear_speed={self.linear_speed:.3f}")
            return True
        if self._combo_active(self.speed_control_keymap["linear_inc"], tokens):
            self.linear_speed = min(self.max_speed, self.linear_speed + self.linear_speed_step)
            self.feedback_message = f"Linear speed set to {self.linear_speed:.3f}"
            if self._verbose_key_log:
                print(f"[MoveArmTeleop] linear_speed={self.linear_speed:.3f}")
            return True

        if self._combo_active(self.speed_control_keymap["angular_dec"], tokens):
            self.angular_speed = max(self.min_speed, self.angular_speed - self.angular_speed_step)
            self.feedback_message = f"Angular speed set to {self.angular_speed:.3f}"
            if self._verbose_key_log:
                print(f"[MoveArmTeleop] angular_speed={self.angular_speed:.3f}")
            return True
        if self._combo_active(self.speed_control_keymap["angular_inc"], tokens):
            self.angular_speed = min(self.max_speed, self.angular_speed + self.angular_speed_step)
            self.feedback_message = f"Angular speed set to {self.angular_speed:.3f}"
            if self._verbose_key_log:
                print(f"[MoveArmTeleop] angular_speed={self.angular_speed:.3f}")
            return True

        if self._combo_active(self.speed_control_keymap["joint_dec"], tokens):
            self.joint_speed = max(self.min_speed, self.joint_speed - self.joint_speed_step)
            self.feedback_message = f"Joint speed set to {self.joint_speed:.3f}"
            if self._verbose_key_log:
                print(f"[MoveArmTeleop] joint_speed={self.joint_speed:.3f}")
            return True
        if self._combo_active(self.speed_control_keymap["joint_inc"], tokens):
            self.joint_speed = min(self.max_speed, self.joint_speed + self.joint_speed_step)
            self.feedback_message = f"Joint speed set to {self.joint_speed:.3f}"
            if self._verbose_key_log:
                print(f"[MoveArmTeleop] joint_speed={self.joint_speed:.3f}")
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

    def _publish_joint_multi(self, commands) -> None:
        if self.joint_pub is None:
            return
        msg = JointJog()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "joint"
        msg.joint_names = [name for name, _ in commands]
        msg.velocities = [float(vel) for _, vel in commands]
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

    def _joint_commands_from_tokens(self, tokens: set[str]):
        commands = []
        for idx, (joint_name, plus_combo, minus_combo) in enumerate(self.joint_keymap):
            if idx >= self.dof:
                break
            pos_active = self._combo_active(plus_combo, tokens)
            neg_active = self._combo_active(minus_combo, tokens)
            if pos_active and not neg_active:
                commands.append((joint_name, self.joint_speed))
            elif neg_active and not pos_active:
                commands.append((joint_name, -self.joint_speed))
        return commands

    def _twist_command_from_tokens(self, tokens: set[str]):
        if not self.twist_keymap:
            return None
        accum = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        active = False
        for combo, cmd in self.twist_keymap.items():
            if self._combo_active(combo, tokens):
                active = True
                for i in range(6):
                    accum[i] += cmd[i]
        if not active:
            return None
        return (
            accum[0] * self.linear_speed,
            accum[1] * self.linear_speed,
            accum[2] * self.linear_speed,
            accum[3] * self.angular_speed,
            accum[4] * self.angular_speed,
            accum[5] * self.angular_speed,
        )

    def _print_help(self) -> None:
        if self._printed_help:
            return
        print("")
        print("MoveArmTeleop controls:")
        print(f"  Twist keys: {', '.join(sorted(self._combo_to_str(c) for c in self.twist_keymap.keys()))}")
        joint_pairs = [f"{self._combo_to_str(p)}/{self._combo_to_str(n)}({j} +/-)" for j, p, n in self.joint_keymap[: self.dof]]
        print(f"  Joint keys: {', '.join(joint_pairs)}")
        sk = self.speed_control_keymap
        print(
            f"  Speeds:    {self._combo_to_str(sk['linear_dec'])}/{self._combo_to_str(sk['linear_inc'])}(linear -/+), "
            f"{self._combo_to_str(sk['angular_dec'])}/{self._combo_to_str(sk['angular_inc'])}(angular -/+), "
            f"{self._combo_to_str(sk['joint_dec'])}/{self._combo_to_str(sk['joint_inc'])}(joint -/+)"
        )
        print(
            "             current "
            f"linear={self.linear_speed:.3f}, angular={self.angular_speed:.3f}, joint={self.joint_speed:.3f}"
        )
        print(
            f"  Gripper:   {self._combo_to_str(self.gripper_keymap['open'])}(open), "
            f"{self._combo_to_str(self.gripper_keymap['close'])}(close)"
        )
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
            combo = self._parse_key_combo(key)
            if not combo:
                continue
            if isinstance(value, str) and value in label_map:
                output[combo] = label_map[value]
            elif isinstance(value, (list, tuple)) and len(value) == 6:
                try:
                    output[combo] = tuple(float(v) for v in value)
                except Exception:
                    continue
        if output:
            return output
        return {self._parse_key_combo(k): label_map[v] for k, v in default_map.items()}

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
            pos_combo = self._parse_key_combo(pos)
            neg_combo = self._parse_key_combo(neg)
            if pos_combo and neg_combo:
                output.append((joint_name, pos_combo, neg_combo))
        if output:
            return output
        return [(name, self._parse_key_combo(pos), self._parse_key_combo(neg)) for name, pos, neg in default_pairs]

    def _build_speed_control_keymap(self, configured):
        default_map = {
            "linear_dec": "z",
            "linear_inc": "x",
            "angular_dec": "c",
            "angular_inc": "v",
            "joint_dec": "b",
            "joint_inc": "n",
        }
        if not isinstance(configured, dict):
            return {k: self._parse_key_combo(v) for k, v in default_map.items()}
        out = dict(default_map)
        for k in default_map.keys():
            val = configured.get(k)
            combo = self._parse_key_combo(val)
            if combo:
                out[k] = combo
            else:
                out[k] = self._parse_key_combo(default_map[k])
        if not configured:
            out = {k: self._parse_key_combo(v) for k, v in default_map.items()}
        return out

    def _build_gripper_keymap(self, configured):
        default_map = {"open": "g", "close": "h"}
        if not isinstance(configured, dict):
            return {k: self._parse_key_combo(v) for k, v in default_map.items()}
        out = dict(default_map)
        for k in ("open", "close"):
            val = configured.get(k)
            combo = self._parse_key_combo(val)
            if combo:
                out[k] = combo
            else:
                out[k] = self._parse_key_combo(default_map[k])
        if not configured:
            out = {k: self._parse_key_combo(v) for k, v in default_map.items()}
        return out

    def _parse_key_combo(self, spec):
        if isinstance(spec, str):
            parts = [p.strip().lower() for p in spec.split("+") if p.strip()]
            normalized = [self._normalize_token(p) for p in parts]
            normalized = [p for p in normalized if p]
            if normalized:
                return frozenset(normalized)
            return None
        if isinstance(spec, (tuple, list, set, frozenset)):
            normalized = [self._normalize_token(str(p)) for p in spec]
            normalized = [p for p in normalized if p]
            return frozenset(normalized) if normalized else None
        return None

    def _normalize_token(self, token: str) -> str:
        alias = {
            "return": "enter",
            "kp_enter": "enter",
            " ": "space",
            "spacebar": "space",
            "control": "ctrl",
            "control_l": "ctrl",
            "control_r": "ctrl",
            "shift_l": "shift",
            "shift_r": "shift",
            "alt_l": "alt",
            "alt_r": "alt",
        }
        return alias.get(token.lower(), token.lower())

    def _event_tokens(self, event) -> set[str]:
        if event is None:
            return set()
        if isinstance(event, str):
            combo = self._parse_key_combo(event)
            return set(combo) if combo else set()
        if isinstance(event, (tuple, list, set, frozenset)):
            combo = self._parse_key_combo(event)
            return set(combo) if combo else set()
        return set()

    def _event_has_tokens(self, tokens: set[str], required: tuple[str, ...]) -> bool:
        return all(tok in tokens for tok in required)

    def _combo_active(self, combo: frozenset, tokens: set[str]) -> bool:
        return bool(combo) and combo.issubset(tokens)

    def _combo_to_str(self, combo: frozenset) -> str:
        if not combo:
            return "-"
        return "+".join(sorted(combo))
