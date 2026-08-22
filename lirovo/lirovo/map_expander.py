#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
import numpy as np

class ContinuousMapExpander(Node):
    def __init__(self):
        super().__init__('map_expander')
        
        # ROS 2 Parameters
        self.declare_parameter('target_width_m', 400.0)
        self.declare_parameter('target_height_m', 400.0)
        self.declare_parameter('fill_value', 0) # 0 = Free space
        
        # QoS TRANSIENT_LOCAL is crucial for maps. 
        # It ensures "late joiners" (like opening RViz or Nav2 later) instantly get the map.
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Subscribes to the live SLAM map (using matching QoS)
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        
        # Publishes the modified map continuously (using Transient Local QoS)
        self.pub = self.create_publisher(OccupancyGrid, '/map_expanded', map_qos)
        
        self.get_logger().info("Continuous Map Expander online. Listening to /map...")

    def map_callback(self, msg):
        target_width_m = self.get_parameter('target_width_m').value
        target_height_m = self.get_parameter('target_height_m').value
        fill_value = self.get_parameter('fill_value').value

        res = msg.info.resolution
        old_width = msg.info.width
        old_height = msg.info.height
        
        # Calculate the physical size of the incoming SLAM map
        old_physical_width = old_width * res
        old_physical_height = old_height * res

        # CONDITION 1: If the SLAM map is already bigger than our target, pass it through
        if old_physical_width >= target_width_m and old_physical_height >= target_height_m:
            self.get_logger().info(f"Map is {old_physical_width:.1f}x{old_physical_height:.1f}m. Scrubbing -1s and passing through.", once=True)
            
            # Convert to numpy, find all -1s, and replace them with our fill_value (0)
            data_array = np.array(msg.data, dtype=np.int8)
            data_array[data_array == -1] = fill_value
            
            # Pack it back up and publish
            msg.data = data_array.tolist()
            self.pub.publish(msg)
            return

        # CONDITION 2: Map needs expansion
        # 1. Determine new grid dimensions (only expand axes that are too small)
        new_width = max(old_width, int(target_width_m / res))
        new_height = max(old_height, int(target_height_m / res))

        # 2. Calculate symmetrical padding (how many cells to add to each side)
        pad_left = (new_width - old_width) // 2
        pad_bottom = (new_height - old_height) // 2

        # 3. Shift the origin exactly by the padded cells to keep obstacles physically stationary
        new_origin_x = msg.info.origin.position.x - (pad_left * res)
        new_origin_y = msg.info.origin.position.y - (pad_bottom * res)

        # 4. Create the massive free-space grid
        new_data = np.full((new_height, new_width), fill_value, dtype=np.int8)

        # 5. Reshape the original SLAM map into a 2D array
        old_data = np.array(msg.data, dtype=np.int8).reshape((old_height, old_width))
        
        # Scrub out unknown space (-1) from the original SLAM map so it doesn't block Nav2
        old_data[old_data == -1] = fill_value
        
        # 6. Paste the scrubbed SLAM map exactly in the middle
        new_data[pad_bottom : pad_bottom + old_height, pad_left : pad_left + old_width] = old_data

        # 7. Construct the expanded ROS message
        new_msg = OccupancyGrid()
        new_msg.header = msg.header # Keeps the original timestamp and frame_id
        new_msg.info = msg.info
        new_msg.info.width = new_width
        new_msg.info.height = new_height
        new_msg.info.origin.position.x = new_origin_x
        new_msg.info.origin.position.y = new_origin_y
        new_msg.info.origin.orientation = msg.info.origin.orientation
        
        # Flatten back to a standard Python list
        new_msg.data = new_data.ravel().tolist()

        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ContinuousMapExpander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
