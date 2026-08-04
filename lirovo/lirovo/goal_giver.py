#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses

class CasualtyMissionDispatcher(Node):
    def __init__(self):
        super().__init__('casualty_mission_dispatcher')
        
        # Connect to the Nav2 action server that handles goal arrays
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

    def send_mission(self, coordinates):
        self.get_logger().info('Waiting for /navigate_through_poses action server...')
        self._action_client.wait_for_server()

        # Create the action goal message
        goal_msg = NavigateThroughPoses.Goal()

        # Convert simple (x, y) tuples into ROS 2 PoseStamped messages
        for (x, y) in coordinates:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            
            # Default orientation. Because of `yaw_goal_tolerance: 6.28` 
            # in your YAML, the robot will ignore this and just drive to the X/Y.
            pose.pose.orientation.w = 1.0 
            
            goal_msg.poses.append(pose)

        self.get_logger().info(f'Dispatching mission with {len(goal_msg.poses)} casualties...')
        
        # Send the goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Mission rejected by Nav2! Is bt_navigator running?')
            return

        self.get_logger().info('Mission accepted by Nav2! Executing custom Behavior Tree...')
        
        # Request the final result asynchronously
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # This triggers when the entire array is finished (or if it aborts)
        result = future.result().result
        status = future.result().status
        
        # Status code 4 means SUCCEEDED
        if status == 4:
            self.get_logger().info('Mission Complete! All casualties surveyed.')
        else:
            self.get_logger().warn(f'Mission ended with status code: {status}')
            
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        # Nav2 streams continuous feedback while driving. 
        # We can optionally print how far we are from the current target.
        current_distance = feedback_msg.feedback.distance_remaining
        if current_distance < 10.0: # Keep the logs clean by only printing when close
            self.get_logger().debug(f'Distance to current target: {current_distance:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    dispatcher = CasualtyMissionDispatcher()

    # =======================================================
    # EDIT THIS LIST TO ADD/REMOVE CASUALTY COORDINATES (X, Y)
    # =======================================================
    casualty_locations = [
        (2.0, 3.0),
        (5.5, -1.0),
        (8.0, 4.0)
    ]

    # Start the mission
    dispatcher.send_mission(casualty_locations)
    
    # Spin the node to keep it alive and processing callbacks
    rclpy.spin(dispatcher)

if __name__ == '__main__':
    main()
