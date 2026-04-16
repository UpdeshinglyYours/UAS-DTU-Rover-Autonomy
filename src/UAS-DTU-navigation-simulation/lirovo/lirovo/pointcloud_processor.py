import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class PointCloudTimestampSync(Node):
    def __init__(self):
        super().__init__('pointcloud_timestamp_sync')
        self.sub = self.create_subscription(
            PointCloud2,
            '/bf_lidar/point_cloud_out',
            self.pc_callback,
            10)
        self.pub = self.create_publisher(PointCloud2, '/synced_pointcloud', 10)

    def pc_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

    

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTimestampSync()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
