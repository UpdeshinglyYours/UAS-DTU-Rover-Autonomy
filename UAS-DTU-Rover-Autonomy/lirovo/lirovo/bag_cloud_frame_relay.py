"""
Bag-replay testing utility ONLY - not part of the real hardware data path.

Some older recorded bags carry PointCloud2.header.frame_id as a bare, unnamespaced string
(e.g. "lidar") along with their own unnamespaced /tf_static (base_link -> ..., lidar -> ...,
a disconnected island from a namespaced pipeline's real TF tree, e.g.
rover1/base_link -> rover1/lidar published by bf_lidar_genz_pipeline.launch.py's own
static_tf_lidar node). A bag-play topic remap can't touch message *content* like frame_id, so
this rewrites just that field before the cloud reaches genz_icp's deskewer/odometry, letting an
old unnamespaced recording feed a namespaced test pipeline.

Real hardware doesn't need this: blickfeld_driver_node already sets the correct namespaced
frame_id itself via its own lidar_frame_id:=<ns>/lidar parameter, so there's no mismatch to fix
outside of bag replay.

Typical use (see also lirovo/README or ask - not yet documented in a launch file):
    # 1. bring up the namespaced pipeline, e.g.:
    ros2 launch genz_icp bf_lidar_genz_pipeline.launch.py namespace:=rover1

    # 2. run this relay for that same namespace:
    ros2 run lirovo bag_cloud_frame_relay --ros-args -p namespace:=rover1 \
        -p input_topic:=/bf_lidar/point_cloud_out_bagraw

    # 3. play the bag, remapping its point cloud topic to the relay's input (a bag-play
    #    remap changes topic NAME only - this is what makes the "_bagraw" intermediate name
    #    necessary, purely as a scratch handoff point, never a topic anything else expects):
    ros2 bag play some_old.bag --clock \
        --topics /bf_lidar/point_cloud_out /mavros/imu/data \
        --remap /bf_lidar/point_cloud_out:=/bf_lidar/point_cloud_out_bagraw \
                /mavros/imu/data:=/rover1/imu/data
    # (exclude the bag's own /tf and /tf_static from --topics entirely - don't let them
    # compete with the pipeline's real namespaced TF broadcaster)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class BagCloudFrameRelay(Node):
    def __init__(self):
        super().__init__('bag_cloud_frame_relay')

        self.declare_parameter('namespace', 'rover1')
        self.declare_parameter('input_topic', '/bf_lidar/point_cloud_out_bagraw')
        # Output defaults to <namespace>/bf_lidar/point_cloud_out if left empty - matches
        # bf_lidar_genz_pipeline.launch.py's raw_cloud_topic default exactly, so the
        # deskewer picks it up with no further config. lirovo_hardware.launch.py expects a
        # DIFFERENT shape instead (/<ns>/point_cloud_out, no "bf_lidar" segment) - pass
        # output_topic explicitly when feeding that launch file.
        self.declare_parameter('output_topic', '')
        self.declare_parameter('target_frame', '')  # empty = "<namespace>/lidar"
        # lirovo_hardware.launch.py hardcodes use_sim_time:False everywhere (real-hardware
        # file, no launch arg for it) - feeding it a bag's original recorded timestamps via
        # --clock does nothing there, and comparing those old timestamps against real
        # wall-clock TF causes the exact "jump back in time"/message-filter-drop chain
        # diagnosed earlier this session. Set true to stamp every relayed message with the
        # current wall time instead of preserving the bag's original header.stamp.
        self.declare_parameter('rewrite_timestamp', False)

        namespace = self.get_parameter('namespace').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value or \
            f'/{namespace}/bf_lidar/point_cloud_out'
        self.target_frame = self.get_parameter('target_frame').value or f'{namespace}/lidar'
        self.rewrite_timestamp = self.get_parameter('rewrite_timestamp').value

        # deskewer's own subscription (ImuRotationDeskewNode.cpp) wants RELIABLE, but a bag
        # replays a point cloud at its ORIGINAL recorded QoS, which for a real sensor is
        # BEST_EFFORT - a RELIABLE subscriber can't receive from a BEST_EFFORT publisher.
        # Subscribe best-effort to match the bag/sensor side, publish reliable for the
        # deskewer side.
        out_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(PointCloud2, output_topic, out_qos)
        self.sub = self.create_subscription(
            PointCloud2, input_topic, self._cb, qos_profile_sensor_data)

        self.count = 0
        self.get_logger().info(
            f'relaying {input_topic} -> {output_topic}, frame_id rewritten to '
            f'"{self.target_frame}"')

    def _cb(self, msg):
        msg.header.frame_id = self.target_frame
        if self.rewrite_timestamp:
            msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        self.count += 1
        if self.count % 50 == 0:
            self.get_logger().info(f'relayed {self.count} clouds')


def main():
    rclpy.init()
    node = BagCloudFrameRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
