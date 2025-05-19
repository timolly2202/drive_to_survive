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
from yolo_msg.msg import YoloDetections  # type: ignore
import threading

class RLRewardsNode(Node):
    def __init__(self):
        """
        Initialize the RLRewardsNode.
        Sets up parameters, subscribers, publishers, goal data, and threads.
        """
        super().__init__('rl_rewards_node')

        # --- Parameters ---
        self.declare_parameter('goals', [])
        self.goals = self.get_parameter('goals').get_parameter_value().string_array_value
        self.goal_index = 0
        self.episode_active = False
        self.episode_start_time = None
        self.last_goal_time = None
        self.total_reward = 0.0
        self.in_track = True
        self.image_width = 640

        self.declare_parameter('min_bbox_height', 20.0)
        self.min_bbox_height = self.get_parameter('min_bbox_height').get_parameter_value().double_value

        # --- Subscribers ---
        self.create_subscription(YoloDetections, '/orange/front_image_yolo/detections', self.yolo_callback_front, 10)
        self.create_subscription(YoloDetections, '/orange/back_image_yolo/detections', self.yolo_callback_back, 10)
        self.create_subscription(YoloDetections, '/orange/left_image_yolo/detections', self.yolo_callback_left, 10)
        self.create_subscription(YoloDetections, '/orange/right_image_yolo/detections', self.yolo_callback_right, 10)
        self.create_subscription(Odometry, '/orange/odom', self.odom_callback, 10)
        self.create_subscription(String, '/episode_status', self.episode_status_callback, 10)

        # --- Publishers ---
        self.in_track_pub = self.create_publisher(Bool, '/in_track_status', 10)
        self.reward_pub = self.create_publisher(Float32, '/episode_rewards', 10)

        # --- Marker Publisher and Visualisation ---
        self.marker_pub = self.create_publisher(MarkerArray, '/goal_markers', 10)
        self.visualized = False
        self.create_timer(1.0, self.publish_goal_visuals)

        # --- Timer to Check In-Track Status ---
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("RL Rewards Node has been started.")

        # --- Load Goal Data ---
        with open('~/drive_to_survive/src/rl_rewards/goals/goals.json', 'r') as f:
            self.goals_data = json.load(f)['goals']

        self.detections = {cam: (False, 0.0) for cam in ['front', 'back', 'left', 'right']}

        self.current_position = None
        self.out_of_track_counter = 0
        self.required_out_count = 10
        self.last_in_track = True
        self.has_looped = False

        self.goal_pub = self.create_publisher(Point, '/current_goal', 10)
        threading.Thread(target=self.goal_publisher_thread, daemon=True).start()
        threading.Thread(target=self.goal_checker_thread, daemon=True).start()

        self.declare_parameter('episode_time_limit', 60.0)
        self.episode_time_limit = self.get_parameter('episode_time_limit').get_parameter_value().double_value
        threading.Thread(target=self.episode_timeout_thread, daemon=True).start()

    def yolo_callback_front(self, msg):
        """Process front camera YOLO detections."""
        self.process_yolo_detections(msg, 'front')

    def yolo_callback_back(self, msg):
        """Process back camera YOLO detections."""
        self.process_yolo_detections(msg, 'back')

    def yolo_callback_left(self, msg):
        """Process left camera YOLO detections."""
        self.process_yolo_detections(msg, 'left')

    def yolo_callback_right(self, msg):
        """Process right camera YOLO detections."""
        self.process_yolo_detections(msg, 'right')

    def odom_callback(self, msg):
        """Update the robot's current position from odometry."""
        position = msg.pose.pose.position
        self.current_position = (position.x, position.y)

    def episode_status_callback(self, msg):
        """Handle the start of an episode and reset all internal state."""
        if msg.data.lower() == 'start':
            self.episode_active = True
            now = self.get_clock().now().nanoseconds / 1e9
            self.episode_start_time = now
            self.last_goal_time = now
            self.total_reward = 0.0
            self.goal_index = 0
            self.has_looped = False
            self.current_position = None
            self.get_logger().info("Episode started.")

    def timer_callback(self):
        """Determine if the robot is in-track based on recent YOLO detections."""
        now = self.get_clock().now().nanoseconds / 1e9
        expiry_seconds = 0.5
        active = {cam: active for cam, (active, t) in self.detections.items() if now - t < expiry_seconds and active}

        left_right = 'left' in active and 'right' in active
        front_back = 'front' in active and 'back' in active
        current_in_track = left_right or front_back

        if current_in_track:
            self.out_of_track_counter = 0
            if not self.last_in_track:
                self.get_logger().info("Back in track.")
            self.in_track = True
        else:
            self.out_of_track_counter += 1
            if self.out_of_track_counter >= self.required_out_count:
                if self.last_in_track:
                    self.get_logger().warn("Out of track confirmed.")
                    self.total_reward -= 100
                    self.get_logger().warn(f"Penalty applied: Out of track! Total reward = {self.total_reward:.2f}")
                self.in_track = False
                self.publish_reward()

        self.last_in_track = self.in_track

        if self.episode_active:
            self.in_track_pub.publish(Bool(data=self.in_track))

    def publish_reward(self):
        """Gracefully end the episode and publish the reward."""
        if not self.episode_active:
            return
        self.episode_active = False
        self.current_position = None

        now = self.get_clock().now().nanoseconds / 1e9
        time_since_last_goal = now - self.last_goal_time

        # Apply penalty only if we haven't just reached a goal
        if time_since_last_goal > 0.5:  # allow a small buffer to avoid double-penalizing
            penalty = 5.0 * time_since_last_goal
            self.total_reward -= penalty
            self.get_logger().warn(f"Penalty for not reaching next goal: -{penalty:.2f} (Time since last goal: {time_since_last_goal:.2f}s)")

        msg = Float32()
        msg.data = self.total_reward
        self.reward_pub.publish(msg)
        self.get_logger().info(f"Total reward published: {self.total_reward:.2f}")

    def goal_checker_thread(self):
        """Check whether the robot is in a goal region and reward accordingly."""
        rate = self.create_rate(10)
        while rclpy.ok():
            if not self.episode_active or self.current_position is None:
                time.sleep(0.1)
                continue

            goal = self.goals_data[self.goal_index]
            quad = [(c['x'], c['y']) for c in goal['cones']]
            px, py = self.current_position

            if self.point_in_quad(px, py, quad):
                current_time = self.get_clock().now().nanoseconds / 1e9
                time_taken = current_time - self.last_goal_time
                self.total_reward -= time_taken
                self.total_reward += 5
                self.get_logger().info(f"Reached goal {self.goal_index}: Time taken = {time_taken:.2f}s, Total reward = {self.total_reward:.2f}")
                self.last_goal_time = current_time

                if self.goal_index == 0 and self.has_looped:
                    self.get_logger().info("Track fully completed by looping to first goal.")
                    self.publish_reward()
                    break

                self.goal_index += 1
                if self.goal_index >= len(self.goals_data):
                    self.goal_index = 0
                    self.has_looped = True

                time.sleep(2.0)
            else:
                time.sleep(0.1)

    def goal_publisher_thread(self):
        """Continuously publish the position of the current goal."""
        rate = self.create_rate(2)
        while rclpy.ok():
            if self.episode_active and self.goal_index < len(self.goals_data):
                goal = self.goals_data[self.goal_index]['centre']
                msg = Point(x=goal['x'], y=goal['y'], z=0.0)
                self.goal_pub.publish(msg)
            time.sleep(0.5)

    def episode_timeout_thread(self):
        """Enforce time limits per episode and end it if time is exceeded."""
        rate = self.create_rate(1)
        while rclpy.ok():
            if self.episode_active and self.episode_start_time is not None:
                current_time = self.get_clock().now().nanoseconds / 1e9
                elapsed = current_time - self.episode_start_time
                if elapsed > self.episode_time_limit:
                    self.get_logger().warn(f"Episode timed out after {elapsed:.2f}s.")
                    self.publish_reward()
            time.sleep(1.0)

    def publish_goal_visuals(self):
        """Publish RViz markers for all defined goal regions."""
        if self.visualized:
            return

        marker_array = MarkerArray()
        for i, goal in enumerate(self.goals_data):
            center_marker = Marker()
            center_marker.header.frame_id = "world"
            center_marker.type = Marker.TEXT_VIEW_FACING
            center_marker.action = Marker.ADD
            center_marker.id = i
            center_marker.pose.position.x = goal['centre']['x']
            center_marker.pose.position.y = goal['centre']['y']
            center_marker.pose.position.z = 1.0
            center_marker.scale.z = 0.8
            center_marker.color.b = 1.0
            center_marker.color.a = 1.0
            center_marker.text = str(i + 1)
            marker_array.markers.append(center_marker)

            line_marker = Marker()
            line_marker.header.frame_id = "world"
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.id = 100 + i
            line_marker.scale.x = 0.1
            line_marker.color.g = 1.0
            line_marker.color.a = 0.4
            points = [Point(x=c['x'], y=c['y'], z=0.0) for c in goal['cones']]
            points.append(points[0])
            line_marker.points = points
            marker_array.markers.append(line_marker)

        self.marker_pub.publish(marker_array)
        self.visualized = True

    def process_yolo_detections(self, msg, cam_name):
        """Process YOLO messages and register detections per camera."""
        in_range = any((bbox.bottom - bbox.top) >= self.min_bbox_height for bbox in msg.bounding_boxes)
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.detections[cam_name] = (in_range, timestamp)

    def point_in_quad(self, px, py, quad):
        """Check if a point is inside a quadrilateral using area comparison."""
        def area(x1, y1, x2, y2, x3, y3):
            return abs((x1*(y2 - y3) + x2*(y3 - y1) + x3*(y1 - y2)) / 2.0)

        x1, y1 = quad[0]
        x2, y2 = quad[1]
        x3, y3 = quad[2]
        x4, y4 = quad[3]
        A = area(x1, y1, x2, y2, x3, y3) + area(x1, y1, x3, y3, x4, y4)
        A1 = area(px, py, x1, y1, x2, y2)
        A2 = area(px, py, x2, y2, x3, y3)
        A3 = area(px, py, x3, y3, x4, y4)
        A4 = area(px, py, x4, y4, x1, y1)
        return abs(A - (A1 + A2 + A3 + A4)) < 1e-1

def main(args=None):
    """Entry point for the node."""
    rclpy.init(args=args)
    node = RLRewardsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()