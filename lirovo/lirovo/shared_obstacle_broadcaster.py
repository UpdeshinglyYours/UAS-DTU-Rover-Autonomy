#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


class SharedObstacleMapScanBroadcaster(Node):
    def __init__(self):
        super().__init__('shared_obstacle_map_scan_broadcaster')

        # Topic & Frame Parameters
        self.declare_parameter('costmap_topic', '/rover2/global_costmap/costmap')
        self.declare_parameter('output_topic', '/obstacle_keepout_1') #for rover1s layer
        self.declare_parameter('target_frame_id', 'rover1/map')

        # Static Map-to-Map Transform Parameters (Rover 2's map origin wrt Rover 1's map)
        self.declare_parameter('tf_x', 0.0)        # X offset in meters
        self.declare_parameter('tf_y', 0.0)        # Y offset in meters
        self.declare_parameter('tf_yaw', 0.0)      # Rotation in radians

        # Scan Synthesis Parameters
        self.declare_parameter('cost_threshold', 90)     # Cost threshold for lethal obstacles (0-100)
        self.declare_parameter('num_beams', 1080)        # High angular resolution for large maps
        self.declare_parameter('max_range', 600.0)       # Covers large outdoor arenas
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('publish_rate', 2.0)      # Rate in Hz
        self.declare_parameter('stride', 2)              # Costmap cell skipping to optimize CPU

        costmap_topic = self.get_parameter('costmap_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.target_frame_id = self.get_parameter('target_frame_id').get_parameter_value().string_value
        
        self.tf_x = self.get_parameter('tf_x').get_parameter_value().double_value
        self.tf_y = self.get_parameter('tf_y').get_parameter_value().double_value
        self.tf_yaw = self.get_parameter('tf_yaw').get_parameter_value().double_value

        self.cost_threshold = self.get_parameter('cost_threshold').get_parameter_value().integer_value
        self.num_beams = self.get_parameter('num_beams').get_parameter_value().integer_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        self.stride = max(1, self.get_parameter('stride').get_parameter_value().integer_value)
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Precompute trigonometric constants
        self.cos_yaw = math.cos(self.tf_yaw)
        self.sin_yaw = math.sin(self.tf_yaw)

        # Match Costmap QoS (Transient Local)
        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.latest_costmap = None

        self.sub_costmap = self.create_subscription(
            OccupancyGrid,
            costmap_topic,
            self.costmap_callback,
            costmap_qos
        )

        self.pub_scan = self.create_publisher(
            LaserScan,
            self.output_topic,
            10
        )

        self.timer = self.create_timer(1.0 / publish_rate, self.publish_synthetic_scan)
        self.get_logger().info(
            f"Transforming '{costmap_topic}' -> '{self.output_topic}' in frame '{self.target_frame_id}' "
            f"with offset [x: {self.tf_x:.2f}, y: {self.tf_y:.2f}, yaw: {self.tf_yaw:.2f}]"
        )

    def costmap_callback(self, msg: OccupancyGrid):
        self.latest_costmap = msg

    def publish_synthetic_scan(self):
        if self.latest_costmap is None:
            return

        grid = self.latest_costmap
        res = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        width = grid.info.width
        height = grid.info.height
        data = grid.data

        ranges = [float('inf')] * self.num_beams
        angle_min = -math.pi
        angle_max = math.pi
        angle_inc = (angle_max - angle_min) / self.num_beams

        threshold = self.cost_threshold
        max_r = self.max_range
        min_r = self.min_range
        stride = self.stride

        # Extract obstacle cells and transform to Target Map frame
        for r in range(0, height, stride):
            row_offset = r * width
            for c in range(0, width, stride):
                cost = data[row_offset + c]
                if cost >= threshold:
                    # 1. World coordinates in Source Map frame
                    wx2 = origin_x + (c + 0.5) * res
                    wy2 = origin_y + (r + 0.5) * res

                    # 2. Transform to Target Map frame (Rover 1's frame)
                    wx1 = self.cos_yaw * wx2 - self.sin_yaw * wy2 + self.tf_x
                    wy1 = self.sin_yaw * wx2 + self.cos_yaw * wy2 + self.tf_y

                    # 3. Project to polar coordinates from (0,0) of target map
                    dist = math.hypot(wx1, wy1)

                    if min_r <= dist <= max_r:
                        angle = math.atan2(wy1, wx1)
                        bin_idx = int((angle - angle_min) / angle_inc)

                        if 0 <= bin_idx < self.num_beams:
                            if dist < ranges[bin_idx]:
                                ranges[bin_idx] = dist

        # Construct LaserScan message
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.target_frame_id
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_inc
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.timer.timer_period_ns * 1e9
        scan.range_min = min_r
        scan.range_max = max_r
        scan.ranges = ranges

        self.pub_scan.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = SharedObstacleMapScanBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
