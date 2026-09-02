"""Own map->odom and hold it fixed between trusted global corrections."""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from robot_localization.srv import SetPose
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def quaternion_to_yaw(quaternion):
    """Return planar yaw for a geometry_msgs Quaternion-like object."""
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def compose_map_to_odom(clicked_x, clicked_y, clicked_yaw,
                        odom_x, odom_y, odom_yaw):
    """Compute planar T_map_odom from T_map_base and T_odom_base."""
    yaw = clicked_yaw - odom_yaw
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    x = clicked_x - (cosine * odom_x - sine * odom_y)
    y = clicked_y - (sine * odom_x + cosine * odom_y)
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    return x, y, yaw


def align_translation(map_base_x, map_base_y, map_odom_yaw,
                      odom_base_x, odom_base_y):
    """Return map->odom X/Y while preserving an established map yaw."""
    cosine = math.cos(map_odom_yaw)
    sine = math.sin(map_odom_yaw)
    x = map_base_x - (
        cosine * odom_base_x - sine * odom_base_y
    )
    y = map_base_y - (
        sine * odom_base_x + cosine * odom_base_y
    )
    return x, y


class MapOdomAuthority:
    """Store the authoritative transform without integrating at any rate."""

    def __init__(self):
        self._alignment = None

    @property
    def alignment(self):
        return self._alignment

    def set_manual(self, x, y, yaw):
        """Replace the transform from a deliberate manual alignment."""
        self._alignment = (x, y, yaw)
        return self._alignment

    def apply_rtk_position(self, map_base_x, map_base_y,
                           odom_base_x, odom_base_y,
                           initial_yaw=None):
        """Correct translation from RTK while never deriving motion by rate."""
        if self._alignment is None:
            if initial_yaw is None:
                return None
            yaw = initial_yaw
        else:
            yaw = self._alignment[2]
        x, y = align_translation(
            map_base_x,
            map_base_y,
            yaw,
            odom_base_x,
            odom_base_y,
        )
        self._alignment = (x, y, yaw)
        return self._alignment


class ManualGlobalAlignment(Node):
    """Bridge manual/RTK anchors to the sole map->odom TF publisher."""

    def __init__(self):
        super().__init__('manual_global_alignment')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter(
            'global_set_pose_service', '/ekf_filter_node_map/set_pose'
        )
        self.declare_parameter('global_odometry_topic', '/odometry/global')
        self.declare_parameter('gps_odometry_topic', '/odometry/gps')
        self.declare_parameter('max_initialpose_age_sec', 2.0)
        self.declare_parameter('max_global_odometry_age_sec', 1.0)
        self.declare_parameter('tf_timeout_sec', 0.5)
        self.declare_parameter('transform_publish_frequency', 30.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.max_age = float(
            self.get_parameter('max_initialpose_age_sec').value
        )
        self.max_global_odom_age = float(
            self.get_parameter('max_global_odometry_age_sec').value
        )
        self.tf_timeout = float(self.get_parameter('tf_timeout_sec').value)
        service_name = self.get_parameter('global_set_pose_service').value
        global_odometry_topic = self.get_parameter(
            'global_odometry_topic'
        ).value
        gps_odometry_topic = self.get_parameter(
            'gps_odometry_topic'
        ).value
        publish_frequency = float(
            self.get_parameter('transform_publish_frequency').value
        )
        if publish_frequency <= 0.0:
            raise ValueError('transform_publish_frequency must be positive')

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.authority = MapOdomAuthority()
        self.set_pose_client = self.create_client(SetPose, service_name)
        self.accepted_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/dtu_global_localization/manual_pose',
            1,
        )
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self._initialpose_callback,
            10,
        )
        self.global_odom_sub = self.create_subscription(
            Odometry,
            global_odometry_topic,
            self._global_odom_callback,
            10,
        )
        self.gps_odom_sub = self.create_subscription(
            Odometry,
            gps_odometry_topic,
            self._gps_odom_callback,
            10,
        )
        self.tf_timer = self.create_timer(
            1.0 / publish_frequency,
            self._broadcast_transform,
        )
        self._pending = False
        self._latest_global_yaw = None
        self._latest_global_stamp = None

    def _message_age(self, message):
        stamp = Time.from_msg(message.header.stamp)
        if stamp.nanoseconds == 0:
            return math.inf
        return (self.get_clock().now() - stamp).nanoseconds / 1.0e9

    def _initialpose_callback(self, message):
        age = self._message_age(message)
        if age > self.max_age or age < -0.5:
            self.get_logger().warning(
                'Rejected stale/future /initialpose (age %.3f s)' % age
            )
            return
        if message.header.frame_id != self.map_frame:
            self.get_logger().error(
                'Rejected /initialpose in frame %r; expected %r' % (
                    message.header.frame_id, self.map_frame
                )
            )
            return
        if self._pending:
            self.get_logger().warning(
                'A global set-pose request is already pending'
            )
            return
        if not self.set_pose_client.service_is_ready():
            self.get_logger().error(
                'Global EKF set-pose service is not available'
            )
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except Exception as error:  # tf2 exception classes vary by ROS distro.
            self.get_logger().error(
                'Rejected /initialpose: no current %s -> %s TF: %s' % (
                    self.odom_frame, self.base_frame, error
                )
            )
            return

        clicked = message.pose.pose
        odom = transform.transform
        clicked_yaw = quaternion_to_yaw(clicked.orientation)
        odom_yaw = quaternion_to_yaw(odom.rotation)
        alignment = compose_map_to_odom(
            clicked.position.x,
            clicked.position.y,
            clicked_yaw,
            odom.translation.x,
            odom.translation.y,
            odom_yaw,
        )

        request = SetPose.Request()
        request.pose = message
        self._pending = True
        future = self.set_pose_client.call_async(request)
        future.add_done_callback(
            lambda result: self._set_pose_done(
                result, message, clicked_yaw, transform, odom_yaw, alignment
            )
        )

    def _set_pose_done(self, future, message, clicked_yaw, transform,
                       odom_yaw, alignment):
        self._pending = False
        try:
            future.result()
        except Exception as error:
            self.get_logger().error(
                'Global EKF set-pose service failed: %s' % error
            )
            return

        clicked = message.pose.pose.position
        odom = transform.transform.translation
        self.authority.set_manual(*alignment)
        self.accepted_pose_pub.publish(message)
        self.get_logger().info(
            'MANUAL GLOBAL ALIGNMENT ACCEPTED\n'
            '  clicked map->%s: x=%.3f y=%.3f yaw=%.6f rad\n'
            '  current %s->%s: x=%.3f y=%.3f yaw=%.6f rad\n'
            '  resulting map->%s: x=%.3f y=%.3f yaw=%.6f rad\n'
            '  global EKF pose set to clicked map->%s pose' % (
                self.base_frame,
                clicked.x,
                clicked.y,
                clicked_yaw,
                self.odom_frame,
                self.base_frame,
                odom.x,
                odom.y,
                odom_yaw,
                self.odom_frame,
                alignment[0],
                alignment[1],
                alignment[2],
                self.base_frame,
            )
        )

    def _global_odom_callback(self, message):
        """Cache yaw only for RTK initialization before a manual alignment."""
        if message.header.frame_id != self.map_frame:
            return
        self._latest_global_yaw = quaternion_to_yaw(
            message.pose.pose.orientation
        )
        self._latest_global_stamp = Time.from_msg(message.header.stamp)

    def _gps_odom_callback(self, message):
        """Apply one translation correction produced by a gated RTK fix."""
        if message.header.frame_id != self.map_frame:
            self.get_logger().error(
                'Rejected RTK odometry in frame %r; expected %r' % (
                    message.header.frame_id, self.map_frame
                )
            )
            return
        stamp = Time.from_msg(message.header.stamp)
        if stamp.nanoseconds == 0:
            self.get_logger().warning(
                'Rejected RTK odometry with a zero timestamp'
            )
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                stamp,
                timeout=Duration(seconds=self.tf_timeout),
            )
        except Exception as error:  # tf2 exception classes vary by distro.
            self.get_logger().warning(
                'Rejected RTK correction: no timestamp-matched %s -> %s '
                'TF: %s' % (
                    self.odom_frame, self.base_frame, error
                )
            )
            return

        odom_yaw = quaternion_to_yaw(transform.transform.rotation)
        initial_yaw = None
        if self.authority.alignment is None:
            if self._latest_global_yaw is None:
                self.get_logger().warning(
                    'Rejected first RTK correction: no global yaw available'
                )
                return
            global_age = (
                self.get_clock().now() - self._latest_global_stamp
            ).nanoseconds / 1.0e9
            if global_age < -0.5 or global_age > self.max_global_odom_age:
                self.get_logger().warning(
                    'Rejected first RTK correction: global yaw is stale '
                    '(age %.3f s)' % global_age
                )
                return
            initial_yaw = math.atan2(
                math.sin(self._latest_global_yaw - odom_yaw),
                math.cos(self._latest_global_yaw - odom_yaw),
            )

        previous = self.authority.alignment
        position = message.pose.pose.position
        odom_position = transform.transform.translation
        alignment = self.authority.apply_rtk_position(
            position.x,
            position.y,
            odom_position.x,
            odom_position.y,
            initial_yaw=initial_yaw,
        )
        if alignment is None:
            return
        shift = 0.0 if previous is None else math.hypot(
            alignment[0] - previous[0],
            alignment[1] - previous[1],
        )
        self.get_logger().info(
            'Accepted gated RTK map alignment: map->odom '
            'x=%.3f y=%.3f yaw=%.6f rad (translation shift %.3f m)' % (
                alignment[0], alignment[1], alignment[2], shift
            )
        )

    def _broadcast_transform(self):
        """Republish the stored transform; never recalculate it on the timer."""
        alignment = self.authority.alignment
        if alignment is None:
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = alignment[0]
        transform.transform.translation.y = alignment[1]
        transform.transform.rotation.z = math.sin(alignment[2] / 2.0)
        transform.transform.rotation.w = math.cos(alignment[2] / 2.0)
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    """Run the manual alignment node."""
    rclpy.init(args=args)
    node = ManualGlobalAlignment()
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
