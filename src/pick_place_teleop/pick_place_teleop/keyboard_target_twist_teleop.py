import select
import sys
import termios
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from teleop_bridge_msgs.msg import TargetTwistStates


HELP_TEXT = """
Keyboard teleop for the UR5e + Hand-E tabletop scene

Move end effector:
  w/s : +/- X
  a/d : +/- Y
  r/f : +/- Z

Rotate end effector:
  i/k : +/- pitch
  j/l : +/- yaw
  u/o : +/- roll

Gripper and reset:
  c   : close gripper until stopped/opened
  v   : open gripper until stopped/closed
  b   : reset robot home + cube pose

Other:
  space : stop arm and gripper motion
  + / - : linear speed up/down
  ] / [ : angular speed up/down
  h     : print this help
  q     : quit
"""


class KeyboardTargetTwistTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_target_twist_teleop")

        if not sys.stdin.isatty():
            raise RuntimeError("keyboard_target_twist_teleop requires an interactive TTY.")

        self.declare_parameter("output_topic", "/target_twist_states")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_rate_hz", 60.0)
        self.declare_parameter("linear_speed", 0.16)
        self.declare_parameter("angular_speed", 0.70)
        self.declare_parameter("key_timeout_sec", 0.18)
        self.declare_parameter("gripper_timeout_sec", 0.12)
        self.declare_parameter("latch_gripper_commands", True)
        self.declare_parameter("reset_pulse_sec", 0.25)
        self.declare_parameter("speed_increment_ratio", 0.10)
        self.declare_parameter("print_help_on_start", True)

        self.output_topic = str(self.get_parameter("output_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_rate_hz = max(5.0, float(self.get_parameter("publish_rate_hz").value))
        self.linear_speed = max(0.01, float(self.get_parameter("linear_speed").value))
        self.angular_speed = max(0.05, float(self.get_parameter("angular_speed").value))
        self.key_timeout_sec = max(0.05, float(self.get_parameter("key_timeout_sec").value))
        self.gripper_timeout_sec = max(0.05, float(self.get_parameter("gripper_timeout_sec").value))
        self.latch_gripper_commands = bool(self.get_parameter("latch_gripper_commands").value)
        self.reset_pulse_sec = max(0.05, float(self.get_parameter("reset_pulse_sec").value))
        self.speed_increment_ratio = min(
            0.50, max(0.01, float(self.get_parameter("speed_increment_ratio").value))
        )
        self.print_help_on_start = bool(self.get_parameter("print_help_on_start").value)

        self.publisher = self.create_publisher(TargetTwistStates, self.output_topic, 20)
        self.create_timer(1.0 / self.publish_rate_hz, self._tick)

        self._stdin_fd = sys.stdin.fileno()
        self._old_term_settings = termios.tcgetattr(self._stdin_fd)
        tty.setraw(self._stdin_fd)

        self._active_linear = [0.0, 0.0, 0.0]
        self._active_angular = [0.0, 0.0, 0.0]
        self._gripper_cmd = 0
        self._reset_enable = False
        self._quit_requested = False

        now = self.get_clock().now().nanoseconds / 1e9
        self._last_motion_time = now - 10.0
        self._last_gripper_time = now - 10.0
        self._last_reset_time = now - 10.0
        self._last_status_log_time = now

        if self.print_help_on_start:
            self._print_help()

        self.get_logger().info(
            f"Publishing keyboard teleop on {self.output_topic} with "
            f"linear_speed={self.linear_speed:.3f}, angular_speed={self.angular_speed:.3f}"
        )

    def destroy_node(self):
        self._restore_terminal()
        return super().destroy_node()

    def _restore_terminal(self):
        if getattr(self, "_old_term_settings", None) is not None:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._old_term_settings)
            self._old_term_settings = None

    def _print_help(self):
        print(HELP_TEXT, flush=True)

    def _tick(self):
        self._drain_keyboard()
        now = self.get_clock().now().nanoseconds / 1e9

        if now - self._last_motion_time > self.key_timeout_sec:
            self._active_linear = [0.0, 0.0, 0.0]
            self._active_angular = [0.0, 0.0, 0.0]

        if (not self.latch_gripper_commands) and (now - self._last_gripper_time > self.gripper_timeout_sec):
            self._gripper_cmd = 0

        if now - self._last_reset_time > self.reset_pulse_sec:
            self._reset_enable = False

        msg = TargetTwistStates()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = float(self._active_linear[0])
        msg.twist.linear.y = float(self._active_linear[1])
        msg.twist.linear.z = float(self._active_linear[2])
        msg.twist.angular.x = float(self._active_angular[0])
        msg.twist.angular.y = float(self._active_angular[1])
        msg.twist.angular.z = float(self._active_angular[2])
        msg.gripper_cmd = int(self._gripper_cmd)
        msg.rotate_enable = any(abs(v) > 1e-9 for v in self._active_angular)
        msg.tracked = True
        msg.reset_enable = bool(self._reset_enable)
        self.publisher.publish(msg)

        if now - self._last_status_log_time > 5.0:
            self.get_logger().info(
                "Keyboard teleop active: "
                f"lin=({msg.twist.linear.x:.2f},{msg.twist.linear.y:.2f},{msg.twist.linear.z:.2f}) "
                f"ang=({msg.twist.angular.x:.2f},{msg.twist.angular.y:.2f},{msg.twist.angular.z:.2f}) "
                f"gripper={msg.gripper_cmd} reset={msg.reset_enable} "
                f"speeds=({self.linear_speed:.2f} m/s, {self.angular_speed:.2f} rad/s)"
            )
            self._last_status_log_time = now

        if self._quit_requested:
            raise KeyboardInterrupt

    def _drain_keyboard(self):
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not ready:
                return
            key = sys.stdin.read(1)
            if not key:
                return
            self._handle_key(key)

    def _handle_key(self, key: str):
        now = self.get_clock().now().nanoseconds / 1e9

        linear_bindings = {
            "w": (self.linear_speed, 0.0, 0.0),
            "s": (-self.linear_speed, 0.0, 0.0),
            "a": (0.0, self.linear_speed, 0.0),
            "d": (0.0, -self.linear_speed, 0.0),
            "r": (0.0, 0.0, self.linear_speed),
            "f": (0.0, 0.0, -self.linear_speed),
        }
        angular_bindings = {
            "u": (self.angular_speed, 0.0, 0.0),
            "o": (-self.angular_speed, 0.0, 0.0),
            "i": (0.0, self.angular_speed, 0.0),
            "k": (0.0, -self.angular_speed, 0.0),
            "j": (0.0, 0.0, self.angular_speed),
            "l": (0.0, 0.0, -self.angular_speed),
        }

        if key in linear_bindings:
            self._active_linear = list(linear_bindings[key])
            self._last_motion_time = now
            return

        if key in angular_bindings:
            self._active_angular = list(angular_bindings[key])
            self._last_motion_time = now
            return

        if key == " ":
            self._active_linear = [0.0, 0.0, 0.0]
            self._active_angular = [0.0, 0.0, 0.0]
            self._gripper_cmd = 0
            self.get_logger().info("Motion stopped.")
            return

        if key == "c":
            self._gripper_cmd = 1
            self._last_gripper_time = now
            if self.latch_gripper_commands:
                self.get_logger().info("Gripper closing.")
            return

        if key == "v":
            self._gripper_cmd = -1
            self._last_gripper_time = now
            if self.latch_gripper_commands:
                self.get_logger().info("Gripper opening.")
            return

        if key == "b":
            self._reset_enable = True
            self._last_reset_time = now
            self.get_logger().warn("Reset requested.")
            return

        if key in ("+", "="):
            self.linear_speed *= 1.0 + self.speed_increment_ratio
            self.get_logger().info(f"Linear speed: {self.linear_speed:.3f} m/s")
            return

        if key == "-":
            self.linear_speed = max(0.01, self.linear_speed * (1.0 - self.speed_increment_ratio))
            self.get_logger().info(f"Linear speed: {self.linear_speed:.3f} m/s")
            return

        if key == "]":
            self.angular_speed *= 1.0 + self.speed_increment_ratio
            self.get_logger().info(f"Angular speed: {self.angular_speed:.3f} rad/s")
            return

        if key == "[":
            self.angular_speed = max(0.05, self.angular_speed * (1.0 - self.speed_increment_ratio))
            self.get_logger().info(f"Angular speed: {self.angular_speed:.3f} rad/s")
            return

        if key == "h":
            self._print_help()
            return

        if key in ("q", "\x03"):
            self._quit_requested = True
            self.get_logger().info("Quit requested.")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardTargetTwistTeleop()
        rclpy.spin(node)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            finally:
                node._restore_terminal()
        if rclpy.ok():
            rclpy.shutdown()
