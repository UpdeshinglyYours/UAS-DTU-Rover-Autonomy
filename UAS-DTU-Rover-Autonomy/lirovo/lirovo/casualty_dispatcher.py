#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Geometry messages for building our navigation targets
from geometry_msgs.msg import PoseStamped

# Action definition for array-based waypoint navigation
from nav2_msgs.action import NavigateThroughPoses

# Sensor messages for our "Fake Sensor" Keepout Zone
from sensor_msgs.msg import PointCloud2, PointField

import math
import struct

class CasualtyMissionDispatcherMap(Node):
    """
    This node serves TWO distinct purposes:
    1. Action Client: Sends the array of casualty coordinates to Nav2's /navigate_through_poses server.
    2. Fake Sensor: Generates and continuously publishes 3.0m rings around the casualties as a 
       PointCloud2 message so the Global Costmap routes around them.
    """
    def __init__(self):
        super().__init__('casualty_mission_dispatcher_map')

        # 0. GLOBAL FRAME. 2026-08-30: was the bare literal 'map', hardcoded in two places
        # (the keepout cloud header and every goal pose). Under a namespace nav2 runs in
        # <ns>/map (nav2_params.yaml global_costmap.global_frame: <robot_ns_noslash>/map), so a
        # bare "map" frame does not exist in the TF tree at all - the costmap would fail to
        # transform the keepout cloud and the BT navigator would reject the goals. Default is
        # DERIVED from this node's namespace so it tracks the rest of the stack automatically
        # (namespace '/rover1' -> 'rover1/map'); override with -p global_frame:=... if needed.
        # No namespace still yields plain 'map', so single-robot use is unchanged.
        _ns = self.get_namespace().strip('/')
        self.declare_parameter('global_frame', f'{_ns}/map' if _ns else 'map')
        self.global_frame = self.get_parameter('global_frame').value
        self.get_logger().info(f"Using global frame: '{self.global_frame}'")

        # 1. ACTION CLIENT: Connects to the specific Nav2 server that handles multiple waypoints.
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        
        # 2. PUBLISHER: This acts as our fake 3D LiDAR.
        # 2026-08-30: name made RELATIVE (was absolute '/casualty_keepout'). Relative resolves
        # against this node's namespace, so launching with namespace='rover1' publishes
        # /rover1/casualty_keepout - which is what nav2_params.yaml's casualty_layer expects
        # (<robot_namespace>/casualty_keepout, in BOTH costmaps). Absolute meant two rovers
        # shared one topic and neither matched the namespaced layer. Launched with no
        # namespace it still resolves to /casualty_keepout, so single-robot use is unchanged.
        self._keepout_pub = self.create_publisher(PointCloud2, 'casualty_keepout', 10)
        
        # 3. TIMER: The costmap constantly decays old sensor data. 
        # We pulse our static rings at 1 Hz to ensure they stay permanently locked on the map.
        self._keepout_timer = self.create_timer(1.0, self.publish_keepout_rings)
        
        self.casualty_locations = []
        self.keepout_msg = None

    def generate_static_keepout_cloud(self):
        """
        Calculates the 3.0m safety rings mathematically and packs them into a binary PointCloud2 message.
        Because we are using the global (map) frame, we only have to compute the geometry ONCE.
        """
        points = []
        radius = 2.0 #3.0       # The strict keepout distance you defined
        resolution = 0.05  # Place a dot every 5cm to make a solid, impassable wall
        
        # Draw a circle around every single casualty coordinate
        for (cx, cy) in self.casualty_locations:
            angle = 0.0
            while angle < 2 * math.pi:
                x = cx + radius * math.cos(angle)
                y = cy + radius * math.sin(angle)
                points.append((x, y, 0.5))
                angle += resolution

        msg = PointCloud2()
        
        # CRITICAL: We declare these points exist in the absolute global frame.
        # Nav2 will drop them directly onto the map without doing any moving-frame math.
        # Must be the NAMESPACED frame (e.g. 'rover1/map') - see __init__.
        msg.header.frame_id = self.global_frame
        
        msg.height = 1
        msg.width = len(points)
        
        # Define the exact byte layout of an (X, Y, Z) point cloud.
        # Each coordinate is a 32-bit Float (4 bytes).
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12 # 4 bytes * 3 fields (x, y, z)
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        buffer = bytearray()
        for p in points:
            # struct.pack('fff') converts the three python floats into raw C-style bytes.
            buffer.extend(struct.pack('fff', p[0], p[1], p[2]))
        msg.data = bytes(buffer)
        
        return msg

    def publish_keepout_rings(self):
        """
        Fires once a second. 
        It builds the heavy mathematical point cloud on the very first tick, and then just 
        updates the timestamp and blasts it out on all subsequent ticks.
        """
        # Build the cloud only once to save CPU
        if self.keepout_msg is None and self.casualty_locations:
            self.keepout_msg = self.generate_static_keepout_cloud()
            
        if self.keepout_msg is not None:
            # We MUST update the timestamp every second. 
            # If we don't, the Nav2 costmap will reject the message as "stale data".
            self.keepout_msg.header.stamp = self.get_clock().now().to_msg()
            self._keepout_pub.publish(self.keepout_msg)

    def send_mission(self, coordinates):
        """
        Packages the simple (x, y) tuples into ROS 2 PoseStamped objects and sends them to Nav2.
        """
        self.casualty_locations = coordinates 
        self.get_logger().info('Waiting for /navigate_through_poses action server...')
        
        # Block until the Nav2 BT Navigator is fully booted and ready to receive goals
        self._action_client.wait_for_server()

        goal_msg = NavigateThroughPoses.Goal()
        
        for (x, y) in coordinates:
            pose = PoseStamped()
            pose.header.frame_id = self.global_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            
            # The rover needs a valid quaternion to satisfy the message type.
            # Because we set `yaw_goal_tolerance: 6.28` in nav2_params.yaml,
            # the local controller will completely ignore this orientation anyway!
            pose.pose.orientation.w = 1.0 
            
            goal_msg.poses.append(pose)

        self.get_logger().info(f'Dispatching mission with {len(goal_msg.poses)} casualties...')
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Triggered when Nav2 either accepts or rejects our mission request."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Mission rejected by Nav2! Check if BT Navigator is running.')
            return
            
        self.get_logger().info('Mission accepted! Broadcasting 3.0m static Keepout Zones...')
        
        # Now that the mission is accepted, we ask to be notified when it actually finishes.
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Triggered when the entire array is finished or if the mission fatally aborts."""
        status = future.result().status
        
        # In ROS 2 Action Servers, Status 4 corresponds to SUCCEEDED.
        if status == 4:
            self.get_logger().info('Mission Complete! The custom casualty Behavior Tree finished execution.')
        else:
            self.get_logger().warn(f'Mission aborted with status code: {status}')
            
        # Shut down the node since the mission is over.
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    dispatcher = CasualtyMissionDispatcherMap()

    # The actual mission coordinates to investigate.
    # The dispatcher will draw 3.0m Keepout Zones around ALL of these immediately!
    casualty_locations = [ #(225.0, -114.0), #(3.0, 0.0) #, (6.0, 2.0), (3.0, 0.0)
        (-117.0, 29.0),
        #(274.0, -5.0),
        (7.0, 0.0),
    ]

    dispatcher.send_mission(casualty_locations)
    
    # rclpy.spin blocks the script from exiting, keeping our 1Hz Fake Sensor timer alive
    # and allowing our Action Client callbacks to receive data.
    rclpy.spin(dispatcher)

if __name__ == '__main__':
    main()
