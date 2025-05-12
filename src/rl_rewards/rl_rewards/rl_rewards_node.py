#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, String, Float32
from nav_msgs.msg import Odometry
import numpy as np
import time
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import json

class RLRewardsNode(Node):
    def __init__(self):
        super().__init__('rl_rewards_node')

        # Parameters
        self.declare_parameter('goals', [])
        self.goals = self.get_parameter('goals').get_parameter_value().string_array_value
        self.goal_index = 0
        self.episode_active = False
        self.episode_start_time = None
        self.last_goal_time = None
        self.total_reward = 0.0
        self.in_track = True

        # Subscribers
        self.create_subscription(Float32MultiArray, '/yolo_cone_tensors', self.yolo_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(String, '/episode_status', self.episode_status_callback, 10)

        # Publishers
        self.in_track_pub = self.create_publisher(Bool, '/in_track_status', 10)
        self.reward_pub = self.create_publisher(Float32, '/episode_rewards', 10)

        # Timer to periodically check and publish in_track status
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("RL Rewards Node has been started.")

        # Load goal data from JSON
        with open('/home/jarred/git/drive_to_survive/src/rl_rewards/goals/goals.json', 'r') as f:  # update path
            self.goals_data = json.load(f)['goals']

        # Publisher for visualization
        self.marker_pub = self.create_publisher(MarkerArray, '/goal_markers', 10)

        # Timer to publish visualization once
        self.visualized = False
        self.create_timer(1.0, self.publish_goal_visuals)


    def yolo_callback(self, msg):
        """
        Process YOLO detections to determine if the vehicle is within the track.
        For simplicity, we assume that if any detection is present, the vehicle is within the track.
        """
        detections = msg.data
        self.in_track = len(detections) > 0

    def odom_callback(self, msg):
        """
        Monitor the vehicle's position to determine if it has reached the next goal.
        """
        if not self.episode_active or not self.goals:
            return

        position = msg.pose.pose.position
        current_goal = self.parse_goal(self.goals[self.goal_index])

        distance = np.sqrt((position.x - current_goal[0])**2 + (position.y - current_goal[1])**2)

        if distance < 1.0:  # Threshold to consider goal reached
            current_time = time.time()
            if self.last_goal_time is not None:
                time_taken = current_time - self.last_goal_time
                self.total_reward -= time_taken  # Penalize time taken
                self.get_logger().info(f"Reached goal {self.goal_index}: Time taken = {time_taken:.2f}s, Total reward = {self.total_reward:.2f}")
            else:
                self.get_logger().info(f"Reached first goal {self.goal_index}")

            self.last_goal_time = current_time
            self.goal_index = (self.goal_index + 1) % len(self.goals)

    def episode_status_callback(self, msg):
        """
        Handle the start and end of episodes.
        """
        if msg.data.lower() == 'start':
            self.episode_active = True
            self.episode_start_time = time.time()
            self.last_goal_time = None
            self.total_reward = 0.0
            self.goal_index = 0
            self.get_logger().info("Episode started.")
        elif msg.data.lower() == 'end':
            self.episode_active = False
            self.publish_reward()
            self.get_logger().info("Episode ended.")

    def timer_callback(self):
        """
        Periodically publish the in_track status and update rewards.
        """
        if self.episode_active:
            if not self.in_track:
                self.total_reward -= 1.0  # Penalize for being out of track
                self.get_logger().warn(f"Out of track! Total reward = {self.total_reward:.2f}")

            # Publish in_track status
            msg = Bool()
            msg.data = self.in_track
            self.in_track_pub.publish(msg)

    def publish_reward(self):
        """
        Publish the total reward at the end of the episode.
        """
        msg = Float32()
        msg.data = self.total_reward
        self.reward_pub.publish(msg)
        self.get_logger().info(f"Total reward published: {self.total_reward:.2f}")

    def parse_goal(self, goal_str):
        """
        Parse a goal string into (x, y) coordinates.
        Expected format: "x,y"
        """
        try:
            x_str, y_str = goal_str.split(',')
            return float(x_str), float(y_str)
        except ValueError:
            self.get_logger().error(f"Invalid goal format: {goal_str}")
            return 0.0, 0.0
        
    def publish_goal_visuals(self):
        if self.visualized:
            return

        marker_array = MarkerArray()
        for i, goal in enumerate(self.goals_data):
            # Goal center marker
            center_marker = Marker()
            center_marker.header.frame_id = "world"
            center_marker.type = Marker.TEXT_VIEW_FACING
            center_marker.action = Marker.ADD
            center_marker.id = i
            center_marker.pose.position.x = goal['centre']['x']
            center_marker.pose.position.y = goal['centre']['y']
            center_marker.pose.position.z = 1.0  # Floating text
            center_marker.scale.z = 0.8  # Text size
            center_marker.color.b = 1.0
            center_marker.color.a = 1.0
            center_marker.text = str(i + 1)
            marker_array.markers.append(center_marker)

            # Prism lines (rectangle connecting the cones)
            line_marker = Marker()
            line_marker.header.frame_id = "world"
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.id = 100 + i
            line_marker.scale.x = 0.1
            line_marker.color.g = 1.0
            line_marker.color.a = 0.4  # Transparent green

            points = []
            for j in range(4):
                cone = goal['cones'][j]
                pt = Point(x=cone['x'], y=cone['y'], z=0.0)
                points.append(pt)
            points.append(points[0])  # Close the loop

            line_marker.points = points
            marker_array.markers.append(line_marker)

        self.marker_pub.publish(marker_array)
        self.visualized = True

def main(args=None):
    rclpy.init(args=args)
    node = RLRewardsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()