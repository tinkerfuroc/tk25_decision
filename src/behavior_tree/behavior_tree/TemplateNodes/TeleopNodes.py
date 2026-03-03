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
        stop_key: str = None,
        verbose_key_log: bool = None,
        gripper_width_min_cm: float = None,
        gripper_width_max_cm: float = None,
        gripper_width_step_cm: float = None,
        gripper_initial_width_cm: float = None,
        gripper_hw_inverted: bool = None,
        visual_feedback_min_period_s: float = None,
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
        self.gripper_width_min_cm = float(_resolve(gripper_width_min_cm, "gripper_width_min_cm", 0.0))
        self.gripper_width_max_cm = float(_resolve(gripper_width_max_cm, "gripper_width_max_cm", 8.0))
        if self.gripper_width_max_cm < self.gripper_width_min_cm:
            self.gripper_width_min_cm, self.gripper_width_max_cm = self.gripper_width_max_cm, self.gripper_width_min_cm
        self.gripper_width_step_cm = float(_resolve(gripper_width_step_cm, "gripper_width_step_cm", 0.5))
        self.gripper_hw_inverted = bool(_resolve(gripper_hw_inverted, "gripper_hw_inverted", True))
        initial_width = float(_resolve(gripper_initial_width_cm, "gripper_initial_width_cm", self.gripper_width_max_cm))
        self.gripper_width_cm = self._clamp(initial_width, self.gripper_width_min_cm, self.gripper_width_max_cm)
        self._visual_feedback_min_period_s = float(
            _resolve(visual_feedback_min_period_s, "visual_feedback_min_period_s", 0.05)
        )
        self.stop_key_combo = self._parse_key_combo(
            _resolve(stop_key, "stop_key", "space")
        )
        if not self.stop_key_combo:
            self.stop_key_combo = self._parse_key_combo("space")

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
        self._verbose_key_log = bool(_resolve(verbose_key_log, "verbose_key_log", False))
        self._mock_input_controller = get_mock_input_controller()
        self._forced_mock_mode = is_node_mocked(self.__class__.__name__)
        self._mock_subsystem = get_node_subsystem_name(self.__class__.__name__) or "mock_controls"
        self._mock_consumer_id = f"{self.__class__.__name__}:{id(self)}"
        self._mock_start_tick = -1
        self._mock_start_event = -1
        self._last_twist_cmd = None
        self._last_joint_cmds = []
        self._last_visual_feedback_ts = 0.0
        self._controls_snapshot_cache = self._build_controls_snapshot()

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
        self._last_twist_cmd = None
        self._last_joint_cmds = []
        self._last_visual_feedback_ts = 0.0
        self._controls_snapshot_cache = self._build_controls_snapshot()
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
        self._publish_visual_feedback(note="teleop started", force=True)

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
        self._last_twist_cmd = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._last_joint_cmds = []
        self._publish_visual_feedback(note="teleop stopped", active=False, force=True)
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
                    time.sleep(0.003)
                    continue
                # Provider contract:
                # - list[...]  : batch of events
                # - tuple/set  : one combo event
                # - scalar str : one single-key event
                if isinstance(provided, list):
                    keys = list(provided)
                elif isinstance(provided, (tuple, set, frozenset)):
                    keys = [provided]
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
            # print(f"[MoveArmTeleop] key provider returned: {provided}")
            for key in keys:
                try:
                    tokens = self._event_tokens(key)
                    if self._event_has_tokens(tokens, ("enter",)):
                        self._publish_stop()
                        self._last_twist_cmd = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                        self._last_joint_cmds = []
                        self.feedback_message = "Teleop finished (Enter pressed)"
                        self._publish_visual_feedback(tokens=tokens, note="finish key", active=False, force=True)
                        self._finished = True
                        return
                    if self._event_has_tokens(tokens, ("ctrl", "c")):
                        self.feedback_message = "Teleop interrupted (Ctrl+C)"
                        self._publish_visual_feedback(tokens=tokens, note="ctrl+c", active=False, force=True)
                        self._interrupted = True
                        return
                    self._process_tokens(tokens)
                except Exception as exc:
                    # Keep teleop loop alive on malformed single events.
                    self.feedback_message = f"Teleop input error: {exc}"
                    if self._verbose_key_log:
                        print(f"[MoveArmTeleop] dropped malformed event {key!r}: {exc}")

    def _process_tokens(self, tokens: set[str]) -> None:
        # print(f"Processing tokens: {tokens}")
        if self._handle_speed_tokens(tokens):
            self._publish_visual_feedback(tokens=tokens, note="speed updated", force=True)
            return
        if self._combo_active(self.stop_key_combo, tokens):
            self._publish_stop()
            self.feedback_message = "Stop command published"
            self._last_twist_cmd = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self._last_joint_cmds = []
            self._publish_visual_feedback(tokens=tokens, note="stop published", force=True)
            if self._verbose_key_log:
                print(f"[MoveArmTeleop] key='{self._combo_to_str(self.stop_key_combo)}' -> stop (zero twist)")
            return
        if self._combo_active(self.gripper_keymap["open"], tokens):
            self._set_gripper_width(self.gripper_width_max_cm)
            self.feedback_message = f"Gripper opened (width={self.gripper_width_cm:.2f} cm)"
            self._publish_visual_feedback(tokens=tokens, note="gripper instant open", force=True)
            if self._verbose_key_log:
                print(f"[MoveArmTeleop] gripper instant open -> width={self.gripper_width_cm:.2f}cm")
            return
        if self._combo_active(self.gripper_keymap["close"], tokens):
            self._set_gripper_width(self.gripper_width_min_cm)
            self.feedback_message = f"Gripper closed (width={self.gripper_width_cm:.2f} cm)"
            self._publish_visual_feedback(tokens=tokens, note="gripper instant close", force=True)
            if self._verbose_key_log:
                print(f"[MoveArmTeleop] gripper instant close -> width={self.gripper_width_cm:.2f}cm")
            return
        if self._combo_active(self.gripper_keymap["width_inc"], tokens):
            self._set_gripper_width(self.gripper_width_cm + self.gripper_width_step_cm)
            self.feedback_message = f"Gripper width increased to {self.gripper_width_cm:.2f} cm"
            self._publish_visual_feedback(tokens=tokens, note="gripper width increased", force=True)
            if self._verbose_key_log:
                print(f"[MoveArmTeleop] gripper_width={self.gripper_width_cm:.2f}cm (inc)")
            return
        if self._combo_active(self.gripper_keymap["width_dec"], tokens):
            self._set_gripper_width(self.gripper_width_cm - self.gripper_width_step_cm)
            self.feedback_message = f"Gripper width decreased to {self.gripper_width_cm:.2f} cm"
            self._publish_visual_feedback(tokens=tokens, note="gripper width decreased", force=True)
            if self._verbose_key_log:
                print(f"[MoveArmTeleop] gripper_width={self.gripper_width_cm:.2f}cm (dec)")
            return

        joint_cmds = self._joint_commands_from_tokens(tokens)
        if joint_cmds:
            self._publish_joint_multi(joint_cmds)
            self._last_joint_cmds = list(joint_cmds)
            self._last_twist_cmd = None
            joints_txt = ", ".join([f"{name}:{vel:.3f}" for name, vel in joint_cmds])
            self.feedback_message = f"Joint command published: {joints_txt}"
            self._publish_visual_feedback(tokens=tokens, note="joint command")
            if self._verbose_key_log:
                print(f"[MoveArmTeleop] joints -> {joints_txt}")
            return

        twist_cmd = self._twist_command_from_tokens(tokens)
        if twist_cmd is not None:
            lx, ly, lz, ax, ay, az = twist_cmd
            self._publish_twist(lx, ly, lz, ax, ay, az)
            self._last_twist_cmd = (lx, ly, lz, ax, ay, az)
            self._last_joint_cmds = []
            self.feedback_message = (
                "Cartesian command published: "
                f"linear=({lx:.3f}, {ly:.3f}, {lz:.3f}), "
                f"angular=({ax:.3f}, {ay:.3f}, {az:.3f})"
            )
            self._publish_visual_feedback(tokens=tokens, note="twist command")
            if self._verbose_key_log:
                print(
                    "[MoveArmTeleop] "
                    f"linear=({lx:.3f}, {ly:.3f}, {lz:.3f}), "
                    f"angular=({ax:.3f}, {ay:.3f}, {az:.3f})"
                )
            return

        self.feedback_message = f"Ignored input '{'+'.join(sorted(tokens))}'"
        self._publish_visual_feedback(tokens=tokens, note="ignored input")

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
        # print(f"Publishing gripper width command: {value:.2f} cm")
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
            f"  Gripper:   {self._combo_to_str(self.gripper_keymap['width_inc'])}(width +), "
            f"{self._combo_to_str(self.gripper_keymap['width_dec'])}(width -), "
            f"{self._combo_to_str(self.gripper_keymap['open'])}(open instant), "
            f"{self._combo_to_str(self.gripper_keymap['close'])}(close instant)"
        )
        print(
            "             current "
            f"width={self.gripper_width_cm:.2f}cm step={self.gripper_width_step_cm:.2f}cm "
            f"range=[{self.gripper_width_min_cm:.2f}, {self.gripper_width_max_cm:.2f}] "
            f"inverted_hw={self.gripper_hw_inverted}"
        )
        print(f"  Stop:      {self._combo_to_str(self.stop_key_combo)}")
        print("  Finish:    Enter")
        print("")
        self._printed_help = True

    def _publish_visual_feedback(
        self,
        tokens: set[str] = None,
        note: str = "",
        active: bool = True,
        force: bool = False,
    ) -> None:
        now = time.time()
        if not force and (now - self._last_visual_feedback_ts) < self._visual_feedback_min_period_s:
            return
        self._last_visual_feedback_ts = now
        payload = {
            "active": bool(active) and not self._finished and not self._interrupted,
            "node_name": self.name,
            "updated_at": now,
            "message": self.feedback_message,
            "note": note,
            "tokens": sorted(tokens) if isinstance(tokens, set) else [],
            "speeds": {
                "linear": float(self.linear_speed),
                "angular": float(self.angular_speed),
                "joint": float(self.joint_speed),
                "gripper_width_cm": float(self.gripper_width_cm),
            },
            "twist": [float(v) for v in self._last_twist_cmd] if self._last_twist_cmd is not None else None,
            "joints": [{"name": n, "velocity": float(v)} for n, v in self._last_joint_cmds],
            "controls": self._controls_snapshot_cache,
        }
        self._mock_input_controller.publish_teleop_feedback(payload)

    def _build_controls_snapshot(self) -> dict:
        twist_controls = []
        for combo, cmd in self.twist_keymap.items():
            axis = self._twist_label_from_cmd(cmd)
            twist_controls.append(f"{self._combo_to_str(combo)} => {axis}")
        twist_controls.sort()
        joint_controls = []
        for joint_name, pos_combo, neg_combo in self.joint_keymap[: self.dof]:
            joint_controls.append(
                f"{joint_name}: +{self._combo_to_str(pos_combo)} / -{self._combo_to_str(neg_combo)}"
            )
        speed_controls = {
            "linear_dec": self._combo_to_str(self.speed_control_keymap["linear_dec"]),
            "linear_inc": self._combo_to_str(self.speed_control_keymap["linear_inc"]),
            "angular_dec": self._combo_to_str(self.speed_control_keymap["angular_dec"]),
            "angular_inc": self._combo_to_str(self.speed_control_keymap["angular_inc"]),
            "joint_dec": self._combo_to_str(self.speed_control_keymap["joint_dec"]),
            "joint_inc": self._combo_to_str(self.speed_control_keymap["joint_inc"]),
        }
        return {
            "twist": twist_controls,
            "joint": joint_controls,
            "speed": speed_controls,
            "gripper_width_inc": self._combo_to_str(self.gripper_keymap["width_inc"]),
            "gripper_width_dec": self._combo_to_str(self.gripper_keymap["width_dec"]),
            "gripper_open": self._combo_to_str(self.gripper_keymap["open"]),
            "gripper_close": self._combo_to_str(self.gripper_keymap["close"]),
            "gripper_width_step_cm": float(self.gripper_width_step_cm),
            "gripper_width_range_cm": [float(self.gripper_width_min_cm), float(self.gripper_width_max_cm)],
            "gripper_hw_inverted": bool(self.gripper_hw_inverted),
            "stop": self._combo_to_str(self.stop_key_combo),
            "finish": "enter",
        }

    def _twist_label_from_cmd(self, cmd) -> str:
        labels = ("lx", "ly", "lz", "ax", "ay", "az")
        parts = []
        for i, value in enumerate(cmd):
            try:
                v = float(value)
            except Exception:
                continue
            if abs(v) > 1e-9:
                sign = "+" if v > 0 else "-"
                parts.append(f"{labels[i]}{sign}")
        return ",".join(parts) if parts else "zero"

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
            return [(name, self._parse_key_combo(pos), self._parse_key_combo(neg)) for name, pos, neg in default_pairs]
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
        default_map = {
            "width_inc": "g",
            "width_dec": "h",
            "open": "c",
            "close": "b",
        }
        if not isinstance(configured, dict):
            return {k: self._parse_key_combo(v) for k, v in default_map.items()}

        out = {k: self._parse_key_combo(v) for k, v in default_map.items()}
        # Backward compatibility: old configs used open/close as width +/-.
        legacy_open_combo = self._parse_key_combo(configured.get("open"))
        legacy_close_combo = self._parse_key_combo(configured.get("close"))
        width_inc_combo = self._parse_key_combo(configured.get("width_inc"))
        width_dec_combo = self._parse_key_combo(configured.get("width_dec"))

        out["width_inc"] = width_inc_combo if width_inc_combo else (legacy_open_combo or out["width_inc"])
        out["width_dec"] = width_dec_combo if width_dec_combo else (legacy_close_combo or out["width_dec"])

        # Instant open/close keys are independent controls.
        out["open"] = legacy_open_combo if legacy_open_combo else out["open"]
        out["close"] = legacy_close_combo if legacy_close_combo else out["close"]
        return out

    def _set_gripper_width(self, logical_width_cm: float) -> None:
        self.gripper_width_cm = self._clamp(logical_width_cm, self.gripper_width_min_cm, self.gripper_width_max_cm)
        cmd_width = self.gripper_width_cm
        if self.gripper_hw_inverted:
            cmd_width = self.gripper_width_max_cm + self.gripper_width_min_cm - self.gripper_width_cm
        self._publish_gripper(cmd_width)

    def _clamp(self, value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, float(value)))

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
        if combo is None:
            return False
        if not isinstance(combo, frozenset):
            combo = self._parse_key_combo(combo)
        return bool(combo) and combo.issubset(tokens)

    def _combo_to_str(self, combo: frozenset) -> str:
        if not combo:
            return "-"
        return "+".join(sorted(combo))
