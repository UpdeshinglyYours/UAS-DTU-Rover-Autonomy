"""Gate NavSatFix messages using MAVROS GPSRAW RTK quality."""

import json
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import GPSRAW
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from robot_localization.srv import FromLL
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


class GateLogic:
    """ROS-independent consecutive-fix and quality gate state."""

    def __init__(self, required_samples, max_variance):
        self.required_samples = required_samples
        self.max_variance = max_variance
        self.fix_type = GPSRAW.GPS_FIX_TYPE_NO_GPS
        self.consecutive_fixed = 0
        self.gate_open = False
        self.status_variance = None

    def update_status(self, fix_type, horizontal_accuracy_m=None):
        """Process one GPSRAW status and return whether it is trustworthy."""
        self.fix_type = fix_type
        self.status_variance = (
            None if horizontal_accuracy_m is None
            else horizontal_accuracy_m * horizontal_accuracy_m
        )
        fixed = fix_type == GPSRAW.GPS_FIX_TYPE_RTK_FIXED
        quality_ok = (
            self.status_variance is None
            or self.status_variance <= self.max_variance
        )
        if fixed and quality_ok:
            self.consecutive_fixed += 1
        else:
            self.consecutive_fixed = 0
            self.gate_open = False
        return fixed and quality_ok

    def fix_is_eligible(self, fix_variance):
        """Return true once status count and this fix covariance both pass."""
        return (
            self.fix_type == GPSRAW.GPS_FIX_TYPE_RTK_FIXED
            and self.consecutive_fixed >= self.required_samples
            and fix_variance is not None
            and fix_variance <= self.max_variance
        )

    def close(self):
        """Close without resetting the global filter."""
        self.gate_open = False


def horizontal_fix_variance(message):
    """Return conservative horizontal variance from NavSatFix, or None."""
    if message.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
        return None
    variance = max(
        message.position_covariance[0], message.position_covariance[4]
    )
    if not math.isfinite(variance) or variance < 0.0:
        return None
    return variance


def timestamp_is_fresh(now_seconds, stamp_seconds, timeout_seconds):
    """Check a ROS header stamp against the configured freshness window."""
    if stamp_seconds == 0.0:
        return False
    age = now_seconds - stamp_seconds
    return -0.5 <= age <= timeout_seconds


def reentry_distance(candidate_x, candidate_y, current_x, current_y):
    """Return planar distance between an RTK candidate and filter pose."""
    return math.hypot(candidate_x - current_x, candidate_y - current_y)


def antenna_to_base(candidate_x, candidate_y, base_yaw,
                    base_to_gps_x, base_to_gps_y):
    """Remove a planar GPS lever arm from a candidate antenna position."""
    cosine = math.cos(base_yaw)
    sine = math.sin(base_yaw)
    lever_x = cosine * base_to_gps_x - sine * base_to_gps_y
    lever_y = sine * base_to_gps_x + cosine * base_to_gps_y
    return candidate_x - lever_x, candidate_y - lever_y


class RtkFixGate(Node):
    """Republish only stable, accurate, and sane RTK-fixed NavSatFix data."""

    def __init__(self):
        super().__init__('rtk_fix_gate')
        self.declare_parameter('required_consecutive_fixed_samples', 10)
        self.declare_parameter('max_horizontal_variance_m2', 0.25)
        self.declare_parameter('max_reentry_position_error_m', 2.0)
        self.declare_parameter('enable_reentry_sanity_check', True)
        self.declare_parameter('stale_status_timeout_sec', 1.5)
        self.declare_parameter('stale_fix_timeout_sec', 1.5)
        self.declare_parameter(
            'raw_fix_topic', '/mavros/global_position/raw/fix'
        )
        self.declare_parameter(
            'gps_status_topic', '/mavros/gpsstatus/gps1/raw'
        )
        self.declare_parameter(
            'output_fix_topic',
            '/mavros/global_position/rtk_fixed_only',
        )
        self.declare_parameter('global_odometry_topic', '/odometry/global')
        self.declare_parameter('from_ll_service', '/navsat_transform/fromLL')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('gps_frame', 'gps_antenna')
        self.declare_parameter('tf_timeout_sec', 0.5)

        required = int(
            self.get_parameter('required_consecutive_fixed_samples').value
        )
        max_variance = float(
            self.get_parameter('max_horizontal_variance_m2').value
        )
        if required < 1 or max_variance < 0.0:
            raise ValueError(
                'RTK gate sample count and variance must be valid'
            )
        self.logic = GateLogic(required, max_variance)
        self.max_reentry_error = float(
            self.get_parameter('max_reentry_position_error_m').value
        )
        self.reentry_enabled = bool(
            self.get_parameter('enable_reentry_sanity_check').value
        )
        self.status_timeout = float(
            self.get_parameter('stale_status_timeout_sec').value
        )
        self.fix_timeout = float(
            self.get_parameter('stale_fix_timeout_sec').value
        )

        raw_fix_topic = self.get_parameter('raw_fix_topic').value
        gps_status_topic = self.get_parameter('gps_status_topic').value
        output_topic = self.get_parameter('output_fix_topic').value
        odometry_topic = self.get_parameter('global_odometry_topic').value
        from_ll_service = self.get_parameter('from_ll_service').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.gps_frame = self.get_parameter('gps_frame').value
        self.tf_timeout = float(self.get_parameter('tf_timeout_sec').value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.fix_pub = self.create_publisher(
            NavSatFix, output_topic, qos_profile_sensor_data
        )
        status_qos = QoSProfile(depth=1)
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(
            String, '/dtu_global_localization/status', status_qos
        )
        self.raw_fix_sub = self.create_subscription(
            NavSatFix, raw_fix_topic, self._fix_callback,
            qos_profile_sensor_data
        )
        self.gps_status_sub = self.create_subscription(
            GPSRAW, gps_status_topic, self._status_callback,
            qos_profile_sensor_data
        )
        self.global_odom_sub = self.create_subscription(
            Odometry, odometry_topic, self._global_odom_callback, 10
        )
        self.manual_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/dtu_global_localization/manual_pose',
            self._manual_pose_callback,
            10,
        )
        self.from_ll_client = self.create_client(FromLL, from_ll_service)
        self.timer = self.create_timer(0.25, self._timer_callback)

        self.last_status_time = None
        self.last_fix_time = None
        self.last_fix_variance = None
        self.latest_global_pose = None
        self.manual_initialized = False
        self.anchor_initialized = False
        self.reentry_distance = None
        self.pending_reentry = False
        self._last_status_json = None
        self._last_rejection_reason = 'waiting for GPS status'

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds / 1.0e9

    def _header_is_fresh(self, header, timeout):
        stamp = header.stamp.sec + header.stamp.nanosec / 1.0e9
        return timestamp_is_fresh(self._now_seconds(), stamp, timeout)

    def _status_callback(self, message):
        if not self._header_is_fresh(message.header, self.status_timeout):
            self.logic.consecutive_fixed = 0
            self.logic.close()
            self._last_rejection_reason = 'stale GPSRAW header'
            self._publish_status()
            return
        self.last_status_time = self._now_seconds()
        horizontal_accuracy = None
        if message.h_acc not in (0, 2**32 - 1):
            horizontal_accuracy = message.h_acc / 1000.0
        was_open = self.logic.gate_open
        quality_ok = self.logic.update_status(
            message.fix_type, horizontal_accuracy
        )
        if was_open and not self.logic.gate_open:
            self.get_logger().warning(
                'RTK gate CLOSED: fix type/quality is no longer trusted; '
                'global EKF continues without reset'
            )
        if not quality_ok:
            self._last_rejection_reason = (
                'GPSRAW is not RTK Fixed or reported accuracy is poor'
            )
        else:
            self._last_rejection_reason = 'RTK Fixed samples accumulating'
        self._publish_status()

    def _fix_callback(self, message):
        if not self._header_is_fresh(message.header, self.fix_timeout):
            self.logic.close()
            self._last_rejection_reason = 'stale NavSatFix header'
            self._publish_status()
            return
        self.last_fix_time = self._now_seconds()
        self.last_fix_variance = horizontal_fix_variance(message)
        if not self._status_is_fresh():
            self.logic.close()
            self._last_rejection_reason = 'GPSRAW status is stale'
            self._publish_status()
            return
        if not self.logic.fix_is_eligible(self.last_fix_variance):
            self.logic.close()
            self._last_rejection_reason = (
                'fix rejected by RTK count or covariance threshold'
            )
            self._publish_status()
            return
        if self.logic.gate_open:
            self.fix_pub.publish(message)
            self._last_rejection_reason = ''
            self._publish_status()
            return
        if self.pending_reentry:
            return
        if not self.reentry_enabled or not self.anchor_initialized:
            self._open_gate(message, None)
            return
        if self.latest_global_pose is None:
            self._last_rejection_reason = 'no current global odometry'
            self._publish_status()
            return
        if not self.from_ll_client.service_is_ready():
            self._last_rejection_reason = 'navsat fromLL service unavailable'
            self._publish_status()
            return

        request = FromLL.Request()
        request.ll_point.latitude = message.latitude
        request.ll_point.longitude = message.longitude
        request.ll_point.altitude = message.altitude
        self.pending_reentry = True
        future = self.from_ll_client.call_async(request)
        future.add_done_callback(
            lambda result: self._reentry_done(result, message)
        )

    def _reentry_done(self, future, message):
        self.pending_reentry = False
        try:
            response = future.result()
        except Exception as error:
            self._last_rejection_reason = 'fromLL failed: %s' % error
            self.get_logger().warning(self._last_rejection_reason)
            self._publish_status()
            return
        try:
            map_to_base = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.gps_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except Exception as error:  # tf2 exception classes vary by ROS distro.
            self._last_rejection_reason = 'GPS lever-arm TF unavailable'
            self.get_logger().warning(
                '%s: %s' % (self._last_rejection_reason, error)
            )
            self._publish_status()
            return
        map_to_base_yaw = math.atan2(
            2.0 * (
                map_to_base.transform.rotation.w
                * map_to_base.transform.rotation.z
                + map_to_base.transform.rotation.x
                * map_to_base.transform.rotation.y
            ),
            1.0 - 2.0 * (
                map_to_base.transform.rotation.y ** 2
                + map_to_base.transform.rotation.z ** 2
            ),
        )
        candidate_x, candidate_y = antenna_to_base(
            response.map_point.x,
            response.map_point.y,
            map_to_base_yaw,
            transform.transform.translation.x,
            transform.transform.translation.y,
        )
        self.reentry_distance = reentry_distance(
            candidate_x,
            candidate_y,
            map_to_base.transform.translation.x,
            map_to_base.transform.translation.y,
        )
        if self.reentry_distance > self.max_reentry_error:
            self.logic.close()
            self._last_rejection_reason = 'RTK reentry distance too large'
            self.get_logger().warning(
                'RTK reentry rejected: candidate is %.3f m from current '
                'global estimate (limit %.3f m); manually realign to accept '
                'this anchor' % (
                    self.reentry_distance, self.max_reentry_error
                )
            )
            self._publish_status()
            return
        self._open_gate(message, self.reentry_distance)

    def _open_gate(self, message, distance):
        self.logic.gate_open = True
        self.anchor_initialized = True
        self.reentry_distance = distance
        self.fix_pub.publish(message)
        self._last_rejection_reason = ''
        self.get_logger().info(
            'RTK gate OPEN after %d consecutive fixed samples%s' % (
                self.logic.consecutive_fixed,
                '' if distance is None
                else ' (reentry distance %.3f m)' % distance,
            )
        )
        self._publish_status()

    def _global_odom_callback(self, message):
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        self.latest_global_pose = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            yaw,
        )

    def _manual_pose_callback(self, _message):
        self.manual_initialized = True
        self.anchor_initialized = True
        if self.logic.gate_open:
            self.get_logger().warning(
                'Manual alignment accepted; closing RTK gate until reentry '
                'criteria pass against the new pose'
            )
        self.logic.close()
        self.pending_reentry = False
        self.reentry_distance = None
        self._last_rejection_reason = 'manual pose active; awaiting sane RTK'
        self._publish_status()

    def _status_is_fresh(self):
        return (
            self.last_status_time is not None
            and self._now_seconds() - self.last_status_time
            <= self.status_timeout
        )

    def _timer_callback(self):
        now = self._now_seconds()
        if not self._status_is_fresh():
            self.logic.consecutive_fixed = 0
            self.logic.close()
            self._last_rejection_reason = 'GPSRAW status timeout'
        if (
            self.last_fix_time is not None
            and now - self.last_fix_time > self.fix_timeout
        ):
            self.logic.close()
            self._last_rejection_reason = 'NavSatFix timeout'
        self._publish_status()

    def _state(self):
        if self.logic.gate_open:
            return 'RTK_ACTIVE'
        if self.logic.consecutive_fixed > 0:
            return 'RTK_PENDING'
        if self.anchor_initialized:
            return 'MANUAL_INITIALIZED'
        return 'UNINITIALIZED'

    def _publish_status(self):
        payload = {
            'state': self._state(),
            'gps_fix_type': self.logic.fix_type,
            'consecutive_fixed_count': self.logic.consecutive_fixed,
            'gps_gate_open': self.logic.gate_open,
            'last_gps_variance_m2': self.last_fix_variance,
            'last_status_variance_m2': self.logic.status_variance,
            'manual_pose_initialized': self.manual_initialized,
            'reentry_sanity_distance_m': self.reentry_distance,
            'reason': self._last_rejection_reason,
        }
        encoded = json.dumps(payload, sort_keys=True)
        if encoded != self._last_status_json:
            message = String()
            message.data = encoded
            self.status_pub.publish(message)
            self._last_status_json = encoded


def main(args=None):
    """Run the RTK gate node."""
    rclpy.init(args=args)
    node = RtkFixGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
