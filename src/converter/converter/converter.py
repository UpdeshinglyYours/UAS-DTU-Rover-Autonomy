import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn

RC_PASSTHROUGH = 65535   # tells the FCU to ignore this channel


def map_value(value: float,
              in_min: float, in_max: float,
              out_min: int,  out_max: int) -> int:
    
    value = max(in_min, min(in_max, value))
    ratio = (value - in_min) / (in_max - in_min)
    return int(round(out_min + ratio * (out_max - out_min)))


class Converter(Node):

    def __init__(self):
        super().__init__('conversion_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('pwm_min',              1051)
        self.declare_parameter('pwm_max',              1951)
        self.declare_parameter('pwm_neutral',          1501)
        self.declare_parameter('slope_linear',         314)
        self.declare_parameter('intercept_linear',     140)
        self.declare_parameter('slope_angular',        80)
        self.declare_parameter('intercept_angular',    250)
        self.declare_parameter('linear_vel_max',       1.0)   # m/s
        self.declare_parameter('angular_vel_max',      2.0)   # rad/s
        self.declare_parameter('ch_throttle',          1)     # CH2 (0-based)
        self.declare_parameter('ch_steering',          0)     # CH1 (0-based)

        self._read_params()

        # ── Pub / Sub ─────────────────────────────────────────────────────
        self.pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)

        self.get_logger().info(
            f'Rover cmd_vel_to_pwm ready | '
            f'PWM [{self.pwm_min}-{self.pwm_max}] neutral={self.pwm_neutral} | '
            f'CH{self.ch_throttle + 1}=throttle  CH{self.ch_steering + 1}=steering'
        )


    def _read_params(self):
        self.pwm_min            =   self.get_parameter('pwm_min').value
        self.pwm_max            =   self.get_parameter('pwm_max').value
        self.pwm_neutral        =   self.get_parameter('pwm_neutral').value
        self.slope_linear       =   self.get_parameter('slope_linear').value
        self.intercept_linear   =   self.get_parameter('intercept_linear').value
        self.slope_angular      =   self.get_parameter('slope_angular').value
        self.intercept_angular  =   self.get_parameter('intercept_angular').value
        self.linear_vel_max     =   self.get_parameter('linear_vel_max').value
        self.angular_vel_max    =   self.get_parameter('angular_vel_max').value
        self.ch_throttle        =   self.get_parameter('ch_throttle').value
        self.ch_steering        =   self.get_parameter('ch_steering').value


    def vel_to_pwm(self, vel: float, slope, intercept, pwm_max, pwm_min) -> int:

        pwm = (slope * vel) + intercept

        if pwm > pwm_max - 1500:
            pwm = pwm_max - 1500

        if pwm < 0:
            pwm = 0

        return int(round(pwm))


    def callback(self, msg: Twist):

        channels = [RC_PASSTHROUGH] * 18  # OverrideRCIn has 18 channels (MAVLink spec)

        # Throttle: forward (+) / reverse (-)
        pwm_gain_x = self.vel_to_pwm(abs(msg.linear.x), self.slope_linear, self.intercept_linear, self.pwm_max, self.pwm_min)

        if msg.linear.x > 0:
            pwm_x = 1500 + pwm_gain_x

        elif msg.linear.x < 0:
            pwm_x = 1500 - pwm_gain_x

        else:
            pwm_x = 1500

        channels[self.ch_throttle] = pwm_x

        # Steering: left (+angular.z) / right (-angular.z)
        pwm_gain_z = self.vel_to_pwm(abs(msg.angular.z), self.slope_angular, self.intercept_angular, self.pwm_max, self.pwm_min)

        if msg.angular.z > 0:
            pwm_z = 1500 + pwm_gain_z

        elif msg.angular.z < 0:
            pwm_z = 1500 - pwm_gain_z

        else:
            pwm_z = 1500

        channels[self.ch_steering] = pwm_z

        rc_msg = OverrideRCIn()
        rc_msg.channels = channels
        self.pub.publish(rc_msg)

        self.get_logger().debug(
            f'Throttle(CH{self.ch_throttle + 1})={channels[self.ch_throttle]}  '
            f'Steering(CH{self.ch_steering + 1})={channels[self.ch_steering]}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = Converter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
