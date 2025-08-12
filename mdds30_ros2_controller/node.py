import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
import serial


class MDDS30Controller(Node):
    """
    ROS 2 (rclpy) port of the MDDS30 controller node (Serial Simplified mode).

    Subscriptions:
      - /cmd_vel (geometry_msgs/Twist)
          Uses linear.x and angular.z with configurable scales to compute left/right setpoints.
      - /mdds30_controller/motor_left (std_msgs/Int16)
      - /mdds30_controller/motor_right (std_msgs/Int16)
          Direct left/right setpoints in range [-100..100].

    Parameters:
      - port (string): Serial device path. Default: "/dev/ttyUSB0"
      - baud (int): Serial baudrate. Default: 9600
      - linear_scale (float): scales Twist.linear.x to duty percent. Default: 1.0
      - angular_scale (float): scales Twist.angular.z to duty percent. Default: 1.0
      - deadman_timeout_sec (float): stop motors if no cmd received within this time. Default: 0.5

    Protocol (Cytron MDDS30 Serial Simplified):
      This implementation follows the common mapping used in ROS1 versions:
        * Motor 1 (left): byte range [0..127], where ~64 is stop, 0 reverse full, 127 forward full.
        * Motor 2 (right): byte range [128..255], where ~192 is stop, 128 reverse full, 255 forward full.
      Input setpoints are -100..100 and mapped linearly onto those byte ranges.
    """

    def __init__(self):
        super().__init__('mdds30_controller')

        # --- Parameters
        self.declare_parameter('port', '/dev/serial0')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('deadman_timeout_sec', 0.5)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        # --- Serial init
        self.ser = None
        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.02)
            self.get_logger().info(f'Opened MDDS30 serial at {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial {port}: {e}')

        # --- QoS: best-effort for velocity streams
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- Subscriptions
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, qos)
        self.sub_left = self.create_subscription(Int16, 'mdds30_controller/motor_left', self._on_left, 10)
        self.sub_right = self.create_subscription(Int16, 'mdds30_controller/motor_right', self._on_right, 10)

        # --- Deadman timer
        self._last_cmd_time = time.monotonic()
        timeout = float(self.get_parameter('deadman_timeout_sec').value)
        self._deadman_timer = self.create_timer(max(0.05, timeout / 2.0), self._deadman_check)

        # Current cached outputs
        self._left_pct = 0
        self._right_pct = 0

    # -------------------- Callbacks --------------------
    def _on_cmd_vel(self, msg: Twist):
        lin_scale = float(self.get_parameter('linear_scale').value)
        ang_scale = float(self.get_parameter('angular_scale').value)

        # Convert to percentage commands [-100..100]
        left = self._saturate_pct((msg.linear.x * lin_scale) - (msg.angular.z * ang_scale) * 100.0)
        right = self._saturate_pct((msg.linear.x * lin_scale) + (msg.angular.z * ang_scale) * 100.0)

        # Cache & send
        self._left_pct = int(round(left))
        self._right_pct = int(round(right))
        self._apply_outputs()

    def _on_left(self, msg: Int16):
        self._left_pct = int(self._saturate_pct(msg.data))
        self._apply_outputs()

    def _on_right(self, msg: Int16):
        self._right_pct = int(self._saturate_pct(msg.data))
        self._apply_outputs()

    # -------------------- Helpers --------------------
    @staticmethod
    def _saturate_pct(x):
        if x > 100:
            return 100
        if x < -100:
            return -100
        return x

    def _deadman_check(self):
        timeout = float(self.get_parameter('deadman_timeout_sec').value)
        if time.monotonic() - self._last_cmd_time > timeout:
            # stop outputs if timed out
            if self._left_pct != 0 or self._right_pct != 0:
                self._left_pct = 0
                self._right_pct = 0
                self._apply_outputs()

    def _apply_outputs(self):
        self._last_cmd_time = time.monotonic()
        if not self.ser or not self.ser.is_open:
            return
        try:
            left_byte = self._pct_to_mdds30_left(self._left_pct)
            right_byte = self._pct_to_mdds30_right(self._right_pct)
            self.ser.write(bytes([left_byte, right_byte]))
        except Exception as e:
            self.get_logger().error(f'Failed to write to MDDS30: {e}')

    # --- Mapping: [-100..100] -> MDDS30 bytes
    @staticmethod
    def _pct_to_mdds30_left(pct: int) -> int:
        # Map -100..100 to 0..127, ~64 is stop
        # scale = 127/200 = 0.635
        value = int(round((pct + 100) * 0.635))
        return max(0, min(127, value))

    @staticmethod
    def _pct_to_mdds30_right(pct: int) -> int:
        # Map -100..100 to 128..255, ~192 is stop
        value = int(round((pct + 100) * 0.635)) + 128
        return max(128, min(255, value))


def main():
    rclpy.init()
    node = MDDS30Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.ser and node.ser.is_open:
                node.ser.write(bytes([64, 192]))  # stop both motors
                node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()