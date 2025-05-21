# audibot_rl_driver/audibot_rl_driver/rl_agent_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String, Bool, Float32 # Added Bool for in_track, Float32 for episode_rewards
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point # For /current_goal
from yolo_msg.msg import YoloDetections

import numpy as np
import math

# from tf_transformations import euler_from_quaternion # For robust yaw calculation

# Define constants for camera identifiers
CAM_FRONT = 0
CAM_LEFT = 1
CAM_RIGHT = 2
CAM_BACK = 3 # Added back camera
CAM_UNKNOWN = 4 # Should not happen if logic is correct


class RLAgentNode(Node):
    def __init__(self):
        super().__init__('audibot_rl_agent_node')
        self.get_logger().info('RL Agent Node with Jarreds Updates V1 has been started.')

        # --- Parameters ---
        self.declare_parameter('num_closest_cones', 6)
        self.declare_parameter('max_car_speed', 10.0)
        self.declare_parameter('max_car_yaw_rate', math.radians(90)) # Example: 90 deg/s
        self.declare_parameter('max_goal_distance', 50.0)
        self.declare_parameter('max_steering_angle_rad', 0.7) # YOUR_INPUT_REQUIRED: e.g. ~40 degrees
        self.declare_parameter('max_brake_torque', 1000.0)    # YOUR_INPUT_REQUIRED: e.g. 1000 Nm

        self.declare_parameter('front_cam.image_width', 640) # YOUR_INPUT_REQUIRED
        self.declare_parameter('front_cam.image_height', 480) # YOUR_INPUT_REQUIRED
        self.declare_parameter('left_cam.image_width', 640) # YOUR_INPUT_REQUIRED
        self.declare_parameter('left_cam.image_height', 480) # YOUR_INPUT_REQUIRED
        self.declare_parameter('right_cam.image_width', 640) # YOUR_INPUT_REQUIRED
        self.declare_parameter('right_cam.image_height', 480) # YOUR_INPUT_REQUIRED
        self.declare_parameter('back_cam.image_width', 640) # YOUR_INPUT_REQUIRED
        self.declare_parameter('back_cam.image_height', 480) # YOUR_INPUT_REQUIRED

        self.num_closest_cones = self.get_parameter('num_closest_cones').value
        self.max_car_speed = self.get_parameter('max_car_speed').value
        self.max_car_yaw_rate = self.get_parameter('max_car_yaw_rate').value
        self.max_goal_distance = self.get_parameter('max_goal_distance').value
        self.max_steering_angle_rad = self.get_parameter('max_steering_angle_rad').value
        self.max_brake_torque = self.get_parameter('max_brake_torque').value


        self.image_dimensions = {
            CAM_FRONT: (self.get_parameter('front_cam.image_width').value, self.get_parameter('front_cam.image_height').value),
            CAM_LEFT: (self.get_parameter('left_cam.image_width').value, self.get_parameter('left_cam.image_height').value),
            CAM_RIGHT: (self.get_parameter('right_cam.image_width').value, self.get_parameter('right_cam.image_height').value),
            CAM_BACK: (self.get_parameter('back_cam.image_width').value, self.get_parameter('back_cam.image_height').value),
        }

        # norm_center_x, norm_center_y, norm_height, norm_width, present_flag, cam_front, cam_left, cam_right, cam_back
        self.cone_feature_size = 4 + 1 + 4 # (x,y,h,w) + present_flag + (one-hot cam type for 4 cameras)
        # car_vel_x, car_yaw_rate, goal_dist, goal_angle_cos, goal_angle_sin + N * cone_feature_size
        self.observation_size = 2 + 3 + (self.num_closest_cones * self.cone_feature_size)
        self.get_logger().info(f"Cone feature size: {self.cone_feature_size}, Observation space size: {self.observation_size}")

        # --- Publishers ---
        self.throttle_pub = self.create_publisher(Float64, '/orange/throttle_cmd', 10) # [cite: 1]
        self.steering_pub = self.create_publisher(Float64, '/orange/steering_cmd', 10) # [cite: 1]
        self.brake_pub = self.create_publisher(Float64, '/orange/brake_cmd', 10) # [cite: 1]

        # --- Subscribers ---
        # From Jarred's list for observation space [cite: 1]
        self.current_goal_sub = self.create_subscription(Point, '/current_goal', self.current_goal_callback, 10) # [cite: 1]
        self.front_yolo_sub = self.create_subscription(YoloDetections, '/orange/front_image_yolo/detections', self.front_yolo_callback, 10) # [cite: 1]
        self.back_yolo_sub = self.create_subscription(YoloDetections, '/orange/back_image_yolo/detections', self.back_yolo_callback, 10) # [cite: 1]
        self.left_yolo_sub = self.create_subscription(YoloDetections, '/orange/left_image_yolo/detections', self.left_yolo_callback, 10) # [cite: 1]
        self.right_yolo_sub = self.create_subscription(YoloDetections, '/orange/right_image_yolo/detections', self.right_yolo_callback, 10) # [cite: 1]
        self.odom_sub = self.create_subscription(Odometry, '/orange/odom', self.odom_callback, 10) # [cite: 1] (Jarred uses /orange/odom, audibot_gazebo README uses /odom - confirm correct one)
        self.in_track_status_sub = self.create_subscription(Bool, '/in_track_status', self.in_track_status_callback, 10) # [cite: 1]

        # For episode management and monitoring
        self.episode_status_sub = self.create_subscription(String, '/episode_status', self.episode_status_callback, 10) # [cite: 1]
        self.episode_total_reward_sub = self.create_subscription(Float32, '/episode_rewards', self.episode_total_reward_callback, 10) # [cite: 1]


        self.agent_timer_period = 0.1
        self.agent_timer = self.create_timer(self.agent_timer_period, self.agent_step)

        # --- Internal State Variables ---
        self.latest_odometry = None
        self.front_traffic_cones_raw = []
        self.left_traffic_cones_raw = []
        self.right_traffic_cones_raw = []
        self.back_traffic_cones_raw = [] # Added

        self.current_target_goal = None # Will be geometry_msgs/msg/Point [cite: 1]
        self.in_track = True # Assume in track until told otherwise [cite: 1]
        self.latest_episode_total_reward = 0.0

        self.episode_running = False
        self.get_logger().info("RL Agent Node fully initialized based on Jarred's updates.")

    def current_goal_callback(self, msg: Point): # [cite: 1]
       self.current_target_goal = msg
       # self.get_logger().debug(f"Received new current_target_goal: x={msg.x:.2f}, y={msg.y:.2f}")


    def front_yolo_callback(self, msg: YoloDetections): # [cite: 1]
        self.front_traffic_cones_raw = [box for box in msg.bounding_boxes if box.class_name == "traffic-cones"]

    def left_yolo_callback(self, msg: YoloDetections): # [cite: 1]
        self.left_traffic_cones_raw = [box for box in msg.bounding_boxes if box.class_name == "traffic-cones"]

    def right_yolo_callback(self, msg: YoloDetections): # [cite: 1]
        self.right_traffic_cones_raw = [box for box in msg.bounding_boxes if box.class_name == "traffic-cones"]

    def back_yolo_callback(self, msg: YoloDetections): # [cite: 1]
        self.back_traffic_cones_raw = [box for box in msg.bounding_boxes if box.class_name == "traffic-cones"]

    def odom_callback(self, msg: Odometry): # [cite: 1]
        self.latest_odometry = msg

    def in_track_status_callback(self, msg: Bool): # [cite: 1]
        self.in_track = msg.data
        if not self.in_track:
            self.get_logger().warn("Car is OFF TRACK!")
            # This might also be a trigger for episode termination (done=True) for the RL agent.

    def episode_status_callback(self, msg: String): # [cite: 1]
        status = msg.data.lower()
        if status == 'start':
            self.episode_running = True
            self.get_logger().info("Episode started.")
            # Reset agent's internal episodic states here if any (e.g. RNN hidden states)
        elif status == 'end' or status == 'reset': # Assuming 'reset' implies similar handling
            self.episode_running = False
            self.get_logger().info(f"Episode ended/reset by status: {status}.")
            self.publish_control_commands(steering=0.0, throttle=0.0, brake=self.max_brake_torque/2) # Brake on episode end

    def episode_total_reward_callback(self, msg: Float32): # [cite: 1]
        self.latest_episode_total_reward = msg.data
        self.get_logger().info(f"Received total reward for PREVIOUS episode: {self.latest_episode_total_reward:.2f}")


    def _normalize_bounding_box(self, box: BoundingBox, camera_id: int):
        img_width, img_height = self.image_dimensions.get(camera_id, (640.0, 480.0)) # Default to prevent div by zero
        if img_width == 0: img_width = 1.0 # Should not happen with proper params
        if img_height == 0: img_height = 1.0

        center_x = box.left + (box.right - box.left) / 2.0
        center_y = box.top + (box.bottom - box.top) / 2.0
        width = float(box.right - box.left)
        height = float(box.bottom - box.top)

        norm_center_x = np.clip(center_x / img_width, 0.0, 1.0)
        norm_center_y = np.clip(center_y / img_height, 0.0, 1.0)
        norm_width = np.clip(width / img_width, 0.0, 1.0)
        norm_height = np.clip(height / img_height, 0.0, 1.0)

        return norm_center_x, norm_center_y, norm_width, norm_height

    def process_observations(self):
        if not self.latest_odometry:
            # self.get_logger().warn_throttle(throttle_skip_first=True, throttle_time_source_type=rclpy.clock.ClockType.ROS_TIME, duration=5.0, msg="No odometry data yet for observation.")
            return np.zeros(self.observation_size, dtype=np.float32)

        # 1. Car's Kinematic State
        linear_velocity_x = self.latest_odometry.twist.twist.linear.x
        angular_velocity_z = self.latest_odometry.twist.twist.angular.z
        norm_linear_velocity_x = np.clip(linear_velocity_x / self.max_car_speed, -1.0, 1.0)
        norm_angular_velocity_z = np.clip(angular_velocity_z / self.max_car_yaw_rate, -1.0, 1.0)
        car_obs_part = [norm_linear_velocity_x, norm_angular_velocity_z]

        # 2. Target Waypoint Information
        goal_obs_part = [0.0, 0.0, 0.0] # dist, cos(angle), sin(angle) - default if no goal
        if self.current_target_goal: # Check if current_target_goal (Point msg) is available
            car_pos_x = self.latest_odometry.pose.pose.position.x
            car_pos_y = self.latest_odometry.pose.pose.position.y
            orientation_q = self.latest_odometry.pose.pose.orientation
            
            # YOUR_INPUT_REQUIRED: Robust quaternion to yaw conversion.
            # Using a simple approximation for now. Install and use tf_transformations for better results.
            # _, _, car_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            car_yaw = 2.0 * math.atan2(orientation_q.z, orientation_q.w) # Simpler approx.

            target_x = self.current_target_goal.x # From Point message
            target_y = self.current_target_goal.y # From Point message
            
            delta_x_world = target_x - car_pos_x
            delta_y_world = target_y - car_pos_y
            distance_to_goal = math.sqrt(delta_x_world**2 + delta_y_world**2)
            norm_distance_to_goal = np.clip(distance_to_goal / self.max_goal_distance, 0.0, 1.0)
            
            angle_to_goal_world = math.atan2(delta_y_world, delta_x_world)
            angle_to_goal_relative = angle_to_goal_world - car_yaw
            while angle_to_goal_relative > math.pi: angle_to_goal_relative -= 2 * math.pi
            while angle_to_goal_relative < -math.pi: angle_to_goal_relative += 2 * math.pi
            
            goal_angle_cos = math.cos(angle_to_goal_relative)
            goal_angle_sin = math.sin(angle_to_goal_relative)
            goal_obs_part = [norm_distance_to_goal, goal_angle_cos, goal_angle_sin]
        # else:
            # self.get_logger().warn_throttle(throttle_skip_first=True, throttle_time_source_type=rclpy.clock.ClockType.ROS_TIME, duration=5.0, msg="No current target goal for observation.")

        # 3. Cone Data
        all_processed_cones = []
        for cam_id, raw_cone_list in [
            (CAM_FRONT, self.front_traffic_cones_raw),
            (CAM_LEFT, self.left_traffic_cones_raw),
            (CAM_RIGHT, self.right_traffic_cones_raw),
            (CAM_BACK, self.back_traffic_cones_raw), # Added back camera
        ]:
            for box in raw_cone_list:
                norm_cx, norm_cy, norm_w, norm_h = self._normalize_bounding_box(box, cam_id)
                closeness_score = norm_h
                all_processed_cones.append({
                    'cx': norm_cx, 'cy': norm_cy, 'w': norm_w, 'h': norm_h,
                    'cam_id': cam_id, 'score': closeness_score # Score here is just height for sorting
                })
        
        all_processed_cones.sort(key=lambda c: c['score'], reverse=True)
        
        cone_obs_part = []
        for i in range(self.num_closest_cones):
            if i < len(all_processed_cones):
                cone = all_processed_cones[i]
                present_flag = 1.0
                cam_one_hot = [0.0, 0.0, 0.0, 0.0] # For Front, Left, Right, Back
                if cone['cam_id'] == CAM_FRONT: cam_one_hot[0] = 1.0
                elif cone['cam_id'] == CAM_LEFT: cam_one_hot[1] = 1.0
                elif cone['cam_id'] == CAM_RIGHT: cam_one_hot[2] = 1.0
                elif cone['cam_id'] == CAM_BACK: cam_one_hot[3] = 1.0
                
                cone_features = [cone['cx'], cone['cy'], cone['h'], cone['w'], present_flag] + cam_one_hot
                cone_obs_part.extend(cone_features)
            else:
                cone_obs_part.extend([0.0] * self.cone_feature_size) # Pad with zeros
                
        observation = np.array(car_obs_part + goal_obs_part + cone_obs_part, dtype=np.float32)
        
        if len(observation) != self.observation_size:
            self.get_logger().error(f"OBSERVATION LENGTH MISMATCH! Expected {self.observation_size}, Got {len(observation)}. Check feature sizes and concatenation.")
            return np.zeros(self.observation_size, dtype=np.float32) # Critical to return correct shape
            
        return observation

    def select_action(self, observation):
        # YOUR_INPUT_REQUIRED: PPO model's predict() method.
        # action_ppo = self.model.predict(observation, deterministic=True) # Example
        # steering_action = action_ppo[0] # Assuming PPO outputs steering directly
        # throttle_action = action_ppo[1] # Assuming PPO outputs throttle directly
        # brake_action = action_ppo[2]    # Assuming PPO outputs brake directly

        # Placeholder actions
        steering_action = 0.0
        throttle_action = 0.05 # Very gentle throttle for testing
        brake_action = 0.0
        return steering_action, throttle_action, brake_action

    def publish_control_commands(self, steering, throttle, brake):
        # Apply limits and scaling based on PPO output and car's capabilities
        
        # Steering: Assume PPO outputs in [-1, 1], scale to [-max_angle, +max_angle]
        steer_val = np.clip(steering, -1.0, 1.0) * self.max_steering_angle_rad
        
        # Throttle: Assume PPO outputs in [0, 1] (or scale if PPO outputs e.g. [-1,1])
        # (throttle_val = (np.clip(throttle, -1.0, 1.0) + 1.0) / 2.0 # if PPO in [-1,1])
        throttle_val = np.clip(throttle, 0.0, 1.0) # Assuming PPO outputs [0,1] for throttle
        
        # Brake: Assume PPO outputs in [0, 1] (0=no brake, 1=max brake), scale to [0, max_torque]
        brake_val = np.clip(brake, 0.0, 1.0) * self.max_brake_torque
        
        steer_msg = Float64(); steer_msg.data = float(steer_val)
        throttle_msg = Float64(); throttle_msg.data = float(throttle_val)
        brake_msg = Float64(); brake_msg.data = float(brake_val)

        self.steering_pub.publish(steer_msg)
        self.throttle_pub.publish(throttle_msg)
        self.brake_pub.publish(brake_msg)

    def calculate_reward(self, observation, action, next_observation):
        # YOUR_INPUT_REQUIRED: Implement your DENSE reward function here.
        # This is critical for PPO learning.
        # Use self.in_track, progress towards self.current_target_goal, speed, etc.
        dense_reward = 0.0

        # Example components (you MUST refine this):
        # 1. Penalty for being off-track
        if not self.in_track:
            dense_reward -= 100.0 # Large penalty
            self.get_logger().warn("REWARD: Off track penalty applied!")
            # Consider making this an episode termination condition in agent_step

        # 2. Reward for speed (if in track)
        if self.in_track and self.latest_odometry:
            current_speed = self.latest_odometry.twist.twist.linear.x
            dense_reward += current_speed * 0.1 # Small reward for forward speed

        # 3. Reward for getting closer to the current_target_goal (if in track)
        #    You'll need to compare distance_to_goal from current_observation and next_observation
        #    or store previous distance.
        #    (This requires `current_observation` and `next_observation` to be available)
        #    e.g. reward_for_progress = (prev_dist_to_goal - current_dist_to_goal) * factor

        # 4. Penalty for excessive actions (steering, throttle changes) - optional for smoothness

        # 5. Reward for reaching a goal (this might be a larger sparse reward within your dense calculation)
        #    You'd need logic to detect if self.current_target_goal has been reached.

        return dense_reward

    def agent_step(self):
        if not self.episode_running:
            return

        current_observation = self.process_observations()
        if current_observation.shape[0] != self.observation_size :
             self.get_logger().warn_throttle(throttle_skip_first=True, throttle_time_source_type=rclpy.clock.ClockType.ROS_TIME, duration=1.0, msg="Invalid observation in agent_step, skipping.")
             return

        steering, throttle, brake = self.select_action(current_observation)
        self.publish_control_commands(steering, throttle, brake)
        
        # For a full PPO training step, you'd collect (s, a, r, s', done)
        # This 'agent_step' is more like the action selection part.
        # The reward calculation and PPO update would happen in your training loop.
        # If not self.in_track could be a 'done' condition.
        # Reaching the final goal could also be a 'done' condition.
        # Your PPO library (like SB3) will handle how `step` and `reset` are called.

        # self.get_logger().debug(f"Agent Step: S:{steering:.2f} T:{throttle:.2f} B:{brake:.2f}")


def main(args=None):
    rclpy.init(args=args)
    rl_agent_node = RLAgentNode()
    try:
        rclpy.spin(rl_agent_node)
    except KeyboardInterrupt:
        rl_agent_node.get_logger().info('Keyboard interrupt')
    finally:
        rl_agent_node.get_logger().info('Shutting down RL Agent Node.')
        if hasattr(rl_agent_node, 'episode_running') and rl_agent_node.episode_running :
             rl_agent_node.publish_control_commands(0.0, 0.0, rl_agent_node.max_brake_torque) # Full brake on shutdown
        if rclpy.ok():
            rl_agent_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()