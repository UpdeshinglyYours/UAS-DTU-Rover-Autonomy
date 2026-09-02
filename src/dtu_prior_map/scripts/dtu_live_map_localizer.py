#!/usr/bin/env python3

"""Continuously localize the rover in the georeferenced DTU Nav2 map.

Inputs:
  /mavros/global_position/raw/fix  sensor_msgs/NavSatFix
  /mavros/imu/data                sensor_msgs/Imu
  /odometry/filtered              nav_msgs/Odometry (odom -> base_link pose)

Outputs:
  TF map -> odom
  /dtu_gps_map_pose               raw GPS/IMU pose in the map frame
  /dtu_localized_pose             odometry-propagated live rover pose in map

The map coordinates are obtained by projecting WGS84 GPS into UTM zone 43N
and subtracting the known DTU raster origin. GPS corrections update map->odom;
local odometry supplies smooth motion between GPS updates.
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from tf2_ros import TransformBroadcaster


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def stamp_seconds(message) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


def latlon_to_utm(latitude_deg: float, longitude_deg: float, zone: int):
    """Convert WGS84 latitude/longitude to northern-hemisphere UTM."""
    semimajor_axis = 6378137.0
    flattening = 1.0 / 298.257223563
    eccentricity_squared = flattening * (2.0 - flattening)
    second_eccentricity_squared = eccentricity_squared / (1.0 - eccentricity_squared)
    scale = 0.9996

    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    central_longitude = math.radians((zone - 1) * 6 - 180 + 3)

    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    tan_latitude = math.tan(latitude)

    prime_vertical_radius = semimajor_axis / math.sqrt(
        1.0 - eccentricity_squared * sin_latitude * sin_latitude
    )
    tangent_squared = tan_latitude * tan_latitude
    longitude_term = second_eccentricity_squared * cos_latitude * cos_latitude
    normalized_longitude = cos_latitude * (longitude - central_longitude)

    eccentricity_fourth = eccentricity_squared * eccentricity_squared
    eccentricity_sixth = eccentricity_fourth * eccentricity_squared
    meridional_arc = semimajor_axis * (
        (
            1.0
            - eccentricity_squared / 4.0
            - 3.0 * eccentricity_fourth / 64.0
            - 5.0 * eccentricity_sixth / 256.0
        )
        * latitude
        - (
            3.0 * eccentricity_squared / 8.0
            + 3.0 * eccentricity_fourth / 32.0
            + 45.0 * eccentricity_sixth / 1024.0
        )
        * math.sin(2.0 * latitude)
        + (
            15.0 * eccentricity_fourth / 256.0
            + 45.0 * eccentricity_sixth / 1024.0
        )
        * math.sin(4.0 * latitude)
        - 35.0 * eccentricity_sixth / 3072.0 * math.sin(6.0 * latitude)
    )

    easting = 500000.0 + scale * prime_vertical_radius * (
        normalized_longitude
        + (1.0 - tangent_squared + longitude_term)
        * normalized_longitude**3
        / 6.0
        + (
            5.0
            - 18.0 * tangent_squared
            + tangent_squared * tangent_squared
            + 72.0 * longitude_term
            - 58.0 * second_eccentricity_squared
        )
        * normalized_longitude**5
        / 120.0
    )

    northing = scale * (
        meridional_arc
        + prime_vertical_radius
        * tan_latitude
        * (
            normalized_longitude**2 / 2.0
            + (
                5.0
                - tangent_squared
                + 9.0 * longitude_term
                + 4.0 * longitude_term * longitude_term
            )
            * normalized_longitude**4
            / 24.0
            + (
                61.0
                - 58.0 * tangent_squared
                + tangent_squared * tangent_squared
                + 600.0 * longitude_term
                - 330.0 * second_eccentricity_squared
            )
            * normalized_longitude**6
            / 720.0
        )
    )

    if latitude_deg < 0.0:
        northing += 10000000.0

    return easting, northing


class DtuLiveMapLocalizer(Node):
    def __init__(self):
        super().__init__('dtu_live_map_localizer')

        self.declare_parameter('gps_topic', '/mavros/global_position/raw/fix')
        self.declare_parameter('imu_topic', '/mavros/imu/data')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('utm_zone', 43)
        self.declare_parameter('utm_origin_easting', 706231.750)
        self.declare_parameter('utm_origin_northing', 3181578.500)

        # Measured from the synchronized DTU bag. It converts the raw MAVROS
        # IMU yaw into the DTU map/base_link yaw convention.
        self.declare_parameter('heading_offset_deg', 159.2683)

        self.declare_parameter('initialization_samples', 5)
        self.declare_parameter('position_correction_gain', 0.20)
        self.declare_parameter('yaw_correction_gain', 0.10)
        self.declare_parameter('max_horizontal_variance', 100.0)
        # The synchronized DTU capture shows about 0.205 s of GPS delivery
        # latency relative to IMU/odometry, so 0.25 s avoids rejecting valid
        # live fixes while retaining a bounded synchronization check.
        self.declare_parameter('max_sensor_time_difference', 0.25)
        self.declare_parameter('max_position_innovation', 20.0)
        self.declare_parameter('max_yaw_innovation_deg', 60.0)
        self.declare_parameter('broadcast_rate', 30.0)

        self.gps_topic = str(self.get_parameter('gps_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.utm_zone = int(self.get_parameter('utm_zone').value)
        self.origin_easting = float(self.get_parameter('utm_origin_easting').value)
        self.origin_northing = float(self.get_parameter('utm_origin_northing').value)
        self.heading_offset = math.radians(
            float(self.get_parameter('heading_offset_deg').value)
        )
        self.initialization_samples = max(
            1, int(self.get_parameter('initialization_samples').value)
        )
        self.position_gain = float(
            self.get_parameter('position_correction_gain').value
        )
        self.yaw_gain = float(self.get_parameter('yaw_correction_gain').value)
        self.max_horizontal_variance = float(
            self.get_parameter('max_horizontal_variance').value
        )
        self.max_sensor_dt = float(
            self.get_parameter('max_sensor_time_difference').value
        )
        self.max_position_innovation = float(
            self.get_parameter('max_position_innovation').value
        )
        self.max_yaw_innovation = math.radians(
            float(self.get_parameter('max_yaw_innovation_deg').value)
        )

        if not 0.0 < self.position_gain <= 1.0:
            raise ValueError('position_correction_gain must be in (0, 1].')
        if not 0.0 < self.yaw_gain <= 1.0:
            raise ValueError('yaw_correction_gain must be in (0, 1].')

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.latest_imu = None
        self.latest_odom = None
        self.initial_candidates = []
        self.map_odom_x = None
        self.map_odom_y = None
        self.map_odom_yaw = None
        self.rejected_fixes = 0

        self.tf_broadcaster = TransformBroadcaster(self)
        self.gps_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/dtu_gps_map_pose', 10
        )
        self.localized_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/dtu_localized_pose', 10
        )

        self.create_subscription(Imu, self.imu_topic, self.imu_callback, sensor_qos)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, sensor_qos)
        self.create_subscription(NavSatFix, self.gps_topic, self.gps_callback, sensor_qos)

        broadcast_rate = float(self.get_parameter('broadcast_rate').value)
        self.create_timer(1.0 / broadcast_rate, self.publish_live_state)

        self.get_logger().info(
            'DTU live map localizer started. Wait for a good GPS fix and keep '
            f'the rover stationary for {self.initialization_samples} GPS samples.'
        )

    def imu_callback(self, message: Imu):
        self.latest_imu = message

    def odom_callback(self, message: Odometry):
        self.latest_odom = message

    def gps_callback(self, gps: NavSatFix):
        if gps.status.status < NavSatStatus.STATUS_FIX:
            self.get_logger().warning('GPS has no valid fix; correction skipped.')
            return

        if not math.isfinite(gps.latitude) or not math.isfinite(gps.longitude):
            return

        if self.latest_imu is None or self.latest_odom is None:
            return

        gps_time = stamp_seconds(gps)
        imu_time = stamp_seconds(self.latest_imu)
        odom_time = stamp_seconds(self.latest_odom)
        if (
            abs(imu_time - gps_time) > self.max_sensor_dt
            or abs(odom_time - gps_time) > self.max_sensor_dt
        ):
            self.get_logger().warning(
                'GPS/IMU/odometry timestamps are not synchronized; correction skipped.'
            )
            return

        if gps.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            horizontal_variance = max(
                gps.position_covariance[0], gps.position_covariance[4]
            )
            if horizontal_variance > self.max_horizontal_variance:
                self.get_logger().warning(
                    f'GPS variance {horizontal_variance:.2f} m^2 is too large; '
                    'correction skipped.'
                )
                return

        easting, northing = latlon_to_utm(
            gps.latitude, gps.longitude, self.utm_zone
        )
        map_base_x = easting - self.origin_easting
        map_base_y = northing - self.origin_northing

        map_base_yaw = wrap_angle(
            quaternion_yaw(self.latest_imu.orientation) + self.heading_offset
        )
        odom_pose = self.latest_odom.pose.pose
        odom_base_yaw = quaternion_yaw(odom_pose.orientation)

        candidate_yaw = wrap_angle(map_base_yaw - odom_base_yaw)
        cosine = math.cos(candidate_yaw)
        sine = math.sin(candidate_yaw)
        candidate_x = map_base_x - (
            cosine * odom_pose.position.x - sine * odom_pose.position.y
        )
        candidate_y = map_base_y - (
            sine * odom_pose.position.x + cosine * odom_pose.position.y
        )

        self.publish_gps_pose(gps, map_base_x, map_base_y, map_base_yaw)

        if self.map_odom_x is None:
            self.initial_candidates.append((candidate_x, candidate_y, candidate_yaw))
            count = len(self.initial_candidates)
            self.get_logger().info(
                f'Localization initialization sample {count}/{self.initialization_samples}'
            )
            if count >= self.initialization_samples:
                self.initialize_transform()
            return

        position_innovation = math.hypot(
            candidate_x - self.map_odom_x, candidate_y - self.map_odom_y
        )
        yaw_innovation = wrap_angle(candidate_yaw - self.map_odom_yaw)

        if (
            position_innovation > self.max_position_innovation
            or abs(yaw_innovation) > self.max_yaw_innovation
        ):
            self.rejected_fixes += 1
            self.get_logger().warning(
                'Rejected GPS correction: '
                f'position innovation={position_innovation:.2f} m, '
                f'yaw innovation={math.degrees(yaw_innovation):.1f} deg.'
            )
            return

        self.map_odom_x += self.position_gain * (candidate_x - self.map_odom_x)
        self.map_odom_y += self.position_gain * (candidate_y - self.map_odom_y)
        self.map_odom_yaw = wrap_angle(
            self.map_odom_yaw + self.yaw_gain * yaw_innovation
        )

    def initialize_transform(self):
        count = len(self.initial_candidates)
        self.map_odom_x = sum(value[0] for value in self.initial_candidates) / count
        self.map_odom_y = sum(value[1] for value in self.initial_candidates) / count
        mean_sine = sum(math.sin(value[2]) for value in self.initial_candidates)
        mean_cosine = sum(math.cos(value[2]) for value in self.initial_candidates)
        self.map_odom_yaw = math.atan2(mean_sine, mean_cosine)

        self.get_logger().info(
            'LIVE LOCALIZATION READY: '
            f'map->odom x={self.map_odom_x:.3f} m, '
            f'y={self.map_odom_y:.3f} m, '
            f'yaw={self.map_odom_yaw:.6f} rad '
            f'({math.degrees(self.map_odom_yaw):.2f} deg).'
        )

    def publish_gps_pose(
        self, gps: NavSatFix, map_x: float, map_y: float, map_yaw: float
    ):
        pose = PoseWithCovarianceStamped()
        pose.header = gps.header
        pose.header.frame_id = self.map_frame
        pose.pose.pose.position.x = map_x
        pose.pose.pose.position.y = map_y
        pose.pose.pose.orientation.z = math.sin(map_yaw / 2.0)
        pose.pose.pose.orientation.w = math.cos(map_yaw / 2.0)
        pose.pose.covariance[0] = gps.position_covariance[0]
        pose.pose.covariance[7] = gps.position_covariance[4]
        pose.pose.covariance[14] = gps.position_covariance[8]
        pose.pose.covariance[35] = math.radians(10.0) ** 2
        self.gps_pose_publisher.publish(pose)

    def publish_live_state(self):
        if self.map_odom_x is None or self.latest_odom is None:
            return

        now = self.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = self.map_odom_x
        transform.transform.translation.y = self.map_odom_y
        transform.transform.rotation.z = math.sin(self.map_odom_yaw / 2.0)
        transform.transform.rotation.w = math.cos(self.map_odom_yaw / 2.0)
        self.tf_broadcaster.sendTransform(transform)

        odom_pose = self.latest_odom.pose.pose
        cosine = math.cos(self.map_odom_yaw)
        sine = math.sin(self.map_odom_yaw)
        map_x = self.map_odom_x + (
            cosine * odom_pose.position.x - sine * odom_pose.position.y
        )
        map_y = self.map_odom_y + (
            sine * odom_pose.position.x + cosine * odom_pose.position.y
        )
        map_yaw = wrap_angle(
            self.map_odom_yaw + quaternion_yaw(odom_pose.orientation)
        )

        localized_pose = PoseWithCovarianceStamped()
        localized_pose.header.stamp = now
        localized_pose.header.frame_id = self.map_frame
        localized_pose.pose.pose.position.x = map_x
        localized_pose.pose.pose.position.y = map_y
        localized_pose.pose.pose.orientation.z = math.sin(map_yaw / 2.0)
        localized_pose.pose.pose.orientation.w = math.cos(map_yaw / 2.0)
        localized_pose.pose.covariance = list(self.latest_odom.pose.covariance)
        self.localized_pose_publisher.publish(localized_pose)


def main(args=None):
    rclpy.init(args=args)
    node = DtuLiveMapLocalizer()
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
