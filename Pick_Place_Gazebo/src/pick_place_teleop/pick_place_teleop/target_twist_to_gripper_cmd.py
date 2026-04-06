import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from teleop_bridge_msgs.msg import TargetTwistStates


class TargetTwistToGripperCmd(Node):
    def __init__(self):
        super().__init__("target_twist_to_gripper_cmd")

        self.declare_parameter("input_topic", "/target_twist_states")
        self.declare_parameter("output_topic", "/hande_position_controller/commands")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("speed_m_per_s", 0.03)
        self.declare_parameter("min_pos", 0.0)
        self.declare_parameter("max_pos", 0.025)
        self.declare_parameter("initial_pos", 0.025)
        self.declare_parameter("joint_count", 1)
        self.declare_parameter("second_joint_negate", False)
        self.declare_parameter("stale_timeout_sec", 0.25)
        self.declare_parameter("require_tracked", True)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self._speed = max(0.0, float(self.get_parameter("speed_m_per_s").value))
        self._min_pos = float(self.get_parameter("min_pos").value)
        self._max_pos = float(self.get_parameter("max_pos").value)
        self._target_pos = float(self.get_parameter("initial_pos").value)
        self._joint_count = max(1, int(self.get_parameter("joint_count").value))
        self._second_joint_negate = bool(self.get_parameter("second_joint_negate").value)
        self._stale_timeout_sec = max(0.05, float(self.get_parameter("stale_timeout_sec").value))
        self._require_tracked = bool(self.get_parameter("require_tracked").value)

        if self._min_pos > self._max_pos:
            self._min_pos, self._max_pos = self._max_pos, self._min_pos
        self._target_pos = max(self._min_pos, min(self._max_pos, self._target_pos))

        self._pub = self.create_publisher(Float64MultiArray, self._output_topic, 20)
        self._sub = self.create_subscription(TargetTwistStates, self._input_topic, self._on_input, 20)
        self.create_timer(1.0 / self._publish_rate_hz, self._tick)

        self._hold_cmd = 0
        self._tracked = False
        self._reset_enable = False
        self._last_rx_time = 0.0
        self._last_tick_time = time.monotonic()

        self._rx_count = 0
        self._rx_window_start = time.monotonic()
        self._last_log_time = time.monotonic()

        self.get_logger().info(
            f"TargetTwist->Gripper bridge started: {self._input_topic} -> {self._output_topic}, "
            f"range=[{self._min_pos:.4f},{self._max_pos:.4f}], speed={self._speed:.4f}, "
            f"joint_count={self._joint_count}, second_joint_negate={self._second_joint_negate}, "
            f"publish_rate_hz={self._publish_rate_hz:.1f}, "
            f"stale_timeout_sec={self._stale_timeout_sec:.3f}, "
            f"require_tracked={self._require_tracked}"
        )

    def _on_input(self, msg: TargetTwistStates):
        if msg.gripper_cmd > 0:
            self._hold_cmd = 1
        elif msg.gripper_cmd < 0:
            self._hold_cmd = -1
        else:
            self._hold_cmd = 0

        self._tracked = bool(msg.tracked)
        self._reset_enable = bool(msg.reset_enable)
        self._last_rx_time = time.monotonic()
        self._rx_count += 1

    def _tick(self):
        now = time.monotonic()
        dt = max(0.0, now - self._last_tick_time)
        self._last_tick_time = now

        stale = (now - self._last_rx_time) > self._stale_timeout_sec
        tracked_ok = (not self._require_tracked) or self._tracked
        effective_hold_cmd = self._hold_cmd if (not stale and tracked_ok and (not self._reset_enable)) else 0

        if effective_hold_cmd != 0 and self._speed > 0.0:
            # +1 closes (towards min), -1 opens (towards max).
            direction = -1.0 if effective_hold_cmd > 0 else 1.0
            self._target_pos += direction * self._speed * dt
            self._target_pos = max(self._min_pos, min(self._max_pos, self._target_pos))

        out = Float64MultiArray()
        base = float(self._target_pos)
        out.data = [base] * self._joint_count
        if self._joint_count >= 2 and self._second_joint_negate:
            out.data[1] = -base
        self._pub.publish(out)

        if now - self._last_log_time > 2.0:
            dt_rx = max(now - self._rx_window_start, 1e-6)
            rx_hz = self._rx_count / dt_rx
            self.get_logger().info(
                f"RX {rx_hz:.1f} Hz, stale={stale}, tracked={self._tracked}, reset={self._reset_enable}, hold_cmd={effective_hold_cmd}, "
                f"target_pos={self._target_pos:.4f}"
            )
            self._rx_count = 0
            self._rx_window_start = now
            self._last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = TargetTwistToGripperCmd()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
