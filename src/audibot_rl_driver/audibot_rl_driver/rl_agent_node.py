#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int64, Float32 # For /episode_rewards
from geometry_msgs.msg import Point                 # For /current_goal
from nav_msgs.msg import Odometry                   # For /orange/odom
from yolo_msg.msg import YoloDetections             # For YOLO topics (BoundingBox is part of this)

import numpy as np
import math
import torch
import torch.nn as nn
import torch.optim as optim
import random
from collections import deque # For ReplayBuffer
import time
import os # For saving/loading models
from tf_transformations import euler_from_quaternion # For robust yaw calculation

# --- Constants for camera IDs (ensure these are consistent across your project) ---
CAM_FRONT = 0
CAM_LEFT = 1
CAM_RIGHT = 2
CAM_BACK = 3
CAM_UNKNOWN = 4

# --- DQN Network Definition ---
class DQNNetwork(nn.Module):
    def __init__(self, input_dim: int, output_dim: int):
        super(DQNNetwork, self).__init__()
        # Example MLP architecture
        self.fc1 = nn.Linear(input_dim, 256)
        self.fc2 = nn.Linear(256, 128)
        self.fc3 = nn.Linear(128, output_dim)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # Ensure input 'x' is a FloatTensor. Device handling is done before calling.
        if not isinstance(x, torch.Tensor): x = torch.tensor(x, dtype=torch.float32)
        elif x.dtype != torch.float32: x = x.float()
        # x = x.to(self.device) # This line is usually handled outside the model's forward pass

        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        q_values = self.fc3(x) # Output raw Q-values
        return q_values

# --- Replay Buffer Definition ---
class ReplayBuffer:
    def __init__(self, capacity: int):
        self.buffer = deque(maxlen=capacity)

    def push(self, state, action: int, reward: float, next_state, done: bool):
        self.buffer.append((np.array(state, dtype=np.float32),
                            action, # int
                            reward, # float
                            np.array(next_state, dtype=np.float32),
                            done))   # bool

    def sample(self, batch_size: int):
        state, action, reward, next_state, done = zip(*random.sample(self.buffer, batch_size))
        return np.array(state, dtype=np.float32), \
               np.array(action, dtype=np.int64), \
               np.array(reward, dtype=np.float32), \
               np.array(next_state, dtype=np.float32), \
               np.array(done, dtype=np.float32) # Convert bool to float for (1-done_float)

    def __len__(self) -> int:
        return len(self.buffer)


class AudibotRLDriverDQNNode(Node):
    def __init__(self):
        super().__init__('audibot_rl_driver_dqn_node')
        self.get_logger().info("Audibot RL Driver (DQN) Node: Initializing...")

        # --- Hyperparameters & Configuration ---
        self.BUFFER_SIZE = self.declare_parameter('buffer_size', 50000).value
        self.BATCH_SIZE = self.declare_parameter('batch_size', 64).value
        self.GAMMA = self.declare_parameter('gamma', 0.99).value
        self.EPS_START = self.declare_parameter('eps_start', 1.0).value
        self.EPS_END = self.declare_parameter('eps_end', 0.05).value
        self.EPS_DECAY = self.declare_parameter('eps_decay', 30000).value
        self.TARGET_UPDATE_FREQUENCY = self.declare_parameter('target_update_frequency', 1000).value # In agent learning steps
        self.LEARNING_RATE = self.declare_parameter('learning_rate', 0.0001).value
        self.OPTIMIZE_MODEL_FREQUENCY = self.declare_parameter('optimize_model_frequency', 4).value # Optimize every N agent steps

        self.NUM_ROS_EPISODES_TO_RUN = self.declare_parameter('num_ros_episodes_to_run', 2000).value
        self.MAX_STEPS_PER_RL_TRAJECTORY = self.declare_parameter('max_steps_per_rl_trajectory', 1000).value

        self.max_steering_angle_rad = self.declare_parameter('max_steering_angle_rad', 0.7).value
        self.max_brake_torque = self.declare_parameter('max_brake_torque', 1000.0).value
        self._define_discrete_actions()

        self.num_closest_cones = self.declare_parameter('num_closest_cones', 6).value
        self._define_image_dimensions_and_obs_size() # Defines self.OBSERVATION_SIZE

        # Model Saving Configuration
        self.save_dir = self.declare_parameter('save_dir', 'dqn_models_audibot').value
        self.checkpoint_filename = self.declare_parameter('checkpoint_filename', 'dqn_checkpoint.pth').value
        self.final_model_filename = self.declare_parameter('final_model_filename', 'dqn_final_policy.pth').value
        self.checkpoint_save_frequency = self.declare_parameter('checkpoint_save_frequency', 50).value # Save every N ROS episodes
        
        self.checkpoint_path = os.path.join(self.save_dir, self.checkpoint_filename)
        self.final_model_path = os.path.join(self.save_dir, self.final_model_filename)
        os.makedirs(self.save_dir, exist_ok=True)


        # --- ROS Publishers ---
        self.env_start_request_pub = self.create_publisher(String, 'dqn_agent/start_episode_request', 10)
        self.throttle_pub = self.create_publisher(Float64, '/orange/throttle_cmd', 10)
        self.steering_pub = self.create_publisher(Float64, '/orange/steering_cmd', 10)
        self.brake_pub = self.create_publisher(Float64, '/orange/brake_cmd', 10)
        # Publisher for /episode_status for Jarred's node
        self.episode_status_pub_for_jarred = self.create_publisher(String, '/episode_status', 10)


        # --- ROS Subscribers ---
        self.env_started_confirm_sub = self.create_subscription(Bool, 'rl_gym/environment_started_confirm', self.env_started_callback, 10)
        self.current_goal_sub = self.create_subscription(Point, '/current_goal', self.current_goal_callback, 10)
        self.front_yolo_sub = self.create_subscription(YoloDetections, '/orange/front_image_yolo/detections', self.front_yolo_callback, 10)
        self.back_yolo_sub = self.create_subscription(YoloDetections, '/orange/back_image_yolo/detections', self.back_yolo_callback, 10)
        self.left_yolo_sub = self.create_subscription(YoloDetections, '/orange/left_image_yolo/detections', self.left_yolo_callback, 10)
        self.right_yolo_sub = self.create_subscription(YoloDetections, '/orange/right_image_yolo/detections', self.right_yolo_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/orange/odom', self.odom_callback, 10)
        self.in_track_sub = self.create_subscription(Bool, '/in_track_status', self.in_track_callback, 10)
        self.jarred_episode_over_sub = self.create_subscription(Float32, '/episode_rewards', self.jarred_episode_over_callback, 10)

        # --- DQN Algorithm Components ---
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Using PyTorch device: {self.device}")
        self.policy_net = DQNNetwork(self.OBSERVATION_SIZE, self.num_discrete_actions).to(self.device)
        self.target_net = DQNNetwork(self.OBSERVATION_SIZE, self.num_discrete_actions).to(self.device)
        self.optimizer = optim.AdamW(self.policy_net.parameters(), lr=self.LEARNING_RATE, amsgrad=True)
        self.memory = ReplayBuffer(self.BUFFER_SIZE)
        
        self.total_agent_steps_taken = 0
        self.current_ros_episode_num = 0

        self._load_checkpoint() # Attempt to load checkpoint after initializing networks and optimizer

        # --- State variables ---
        self.env_confirmed_ready_for_rl = False
        self.processing_rl_step_active = False
        self.is_in_track = True
        self.latest_odom = None
        self.latest_front_yolo_cones = []
        self.latest_back_yolo_cones = []
        self.latest_left_yolo_cones = []
        self.latest_right_yolo_cones = []
        self.latest_current_goal = None

        self.s_t_observation_buffer = None
        self.a_t_action_idx_buffer = -1
        self.current_rl_trajectory_step_count = 0

        self.rl_step_timer = self.create_timer(0.1, self.perform_rl_step_and_learn) # 10Hz

        self.get_logger().info("Initialization complete. Requesting first ROS episode from rl_gym.")
        self.request_new_ros_episode_from_rl_gym()

    def _define_discrete_actions(self):
        THROTTLE_LEVELS = {"none": 0.0, "low": 0.35, "high": 0.7}
        STEERING_LEVELS = {
            "hard_left": -self.max_steering_angle_rad, "soft_left": -self.max_steering_angle_rad * 0.5,
            "straight": 0.0,
            "soft_right": self.max_steering_angle_rad * 0.5, "hard_right": self.max_steering_angle_rad
        }
        BRAKE_LEVELS = {"none": 0.0, "moderate": self.max_brake_torque * 0.4}

        self.discrete_actions_map = [
            {'throttle': THROTTLE_LEVELS["none"], 'steering': STEERING_LEVELS["straight"],   'brake': BRAKE_LEVELS["moderate"], 'description': "Brake Straight"},
            {'throttle': THROTTLE_LEVELS["none"], 'steering': STEERING_LEVELS["soft_left"], 'brake': BRAKE_LEVELS["moderate"], 'description': "Brake Soft Left"},
            {'throttle': THROTTLE_LEVELS["none"], 'steering': STEERING_LEVELS["soft_right"],'brake': BRAKE_LEVELS["moderate"], 'description': "Brake Soft Right"},
            {'throttle': THROTTLE_LEVELS["none"], 'steering': STEERING_LEVELS["hard_left"],  'brake': BRAKE_LEVELS["none"], 'description': "Coast Hard Left"},
            {'throttle': THROTTLE_LEVELS["none"], 'steering': STEERING_LEVELS["soft_left"],  'brake': BRAKE_LEVELS["none"], 'description': "Coast Soft Left"},
            {'throttle': THROTTLE_LEVELS["none"], 'steering': STEERING_LEVELS["straight"],   'brake': BRAKE_LEVELS["none"], 'description': "Coast Straight (No-Op)"},
            {'throttle': THROTTLE_LEVELS["none"], 'steering': STEERING_LEVELS["soft_right"], 'brake': BRAKE_LEVELS["none"], 'description': "Coast Soft Right"},
            {'throttle': THROTTLE_LEVELS["none"], 'steering': STEERING_LEVELS["hard_right"], 'brake': BRAKE_LEVELS["none"], 'description': "Coast Hard Right"},
            {'throttle': THROTTLE_LEVELS["low"],  'steering': STEERING_LEVELS["hard_left"],  'brake': BRAKE_LEVELS["none"], 'description': "LowThrottle Hard Left"},
            {'throttle': THROTTLE_LEVELS["low"],  'steering': STEERING_LEVELS["soft_left"],  'brake': BRAKE_LEVELS["none"], 'description': "LowThrottle Soft Left"},
            {'throttle': THROTTLE_LEVELS["low"],  'steering': STEERING_LEVELS["straight"],   'brake': BRAKE_LEVELS["none"], 'description': "LowThrottle Straight"},
            {'throttle': THROTTLE_LEVELS["low"],  'steering': STEERING_LEVELS["soft_right"], 'brake': BRAKE_LEVELS["none"], 'description': "LowThrottle Soft Right"},
            {'throttle': THROTTLE_LEVELS["low"],  'steering': STEERING_LEVELS["hard_right"], 'brake': BRAKE_LEVELS["none"], 'description': "LowThrottle Hard Right"},
            {'throttle': THROTTLE_LEVELS["high"], 'steering': STEERING_LEVELS["hard_left"],  'brake': BRAKE_LEVELS["none"], 'description': "HighThrottle Hard Left"},
            {'throttle': THROTTLE_LEVELS["high"], 'steering': STEERING_LEVELS["soft_left"],  'brake': BRAKE_LEVELS["none"], 'description': "HighThrottle Soft Left"},
            {'throttle': THROTTLE_LEVELS["high"], 'steering': STEERING_LEVELS["straight"],   'brake': BRAKE_LEVELS["none"], 'description': "HighThrottle Straight"},
            {'throttle': THROTTLE_LEVELS["high"], 'steering': STEERING_LEVELS["soft_right"], 'brake': BRAKE_LEVELS["none"], 'description': "HighThrottle Soft Right"},
            {'throttle': THROTTLE_LEVELS["high"], 'steering': STEERING_LEVELS["hard_right"], 'brake': BRAKE_LEVELS["none"], 'description': "HighThrottle Hard Right"}
        ]
        self.num_discrete_actions = len(self.discrete_actions_map)
        self.get_logger().info(f"Defined {self.num_discrete_actions} discrete actions.")

    def _define_image_dimensions_and_obs_size(self):
        default_img_width = 640
        default_img_height = 480
        self.image_dimensions = {
            CAM_FRONT: (self.declare_parameter('front_cam.image_width', default_img_width).value, self.declare_parameter('front_cam.image_height', default_img_height).value),
            CAM_LEFT:  (self.declare_parameter('left_cam.image_width', default_img_width).value, self.declare_parameter('left_cam.image_height', default_img_height).value),
            CAM_RIGHT: (self.declare_parameter('right_cam.image_width', default_img_width).value, self.declare_parameter('right_cam.image_height', default_img_height).value),
            CAM_BACK:  (self.declare_parameter('back_cam.image_width', default_img_width).value, self.declare_parameter('back_cam.image_height', default_img_height).value),
        }
        self.max_car_speed = self.declare_parameter('max_car_speed_norm', 10.0).value
        self.max_car_yaw_rate = self.declare_parameter('max_car_yaw_rate_norm', math.radians(90)).value
        self.max_goal_distance = self.declare_parameter('max_goal_distance_norm', 50.0).value

        self.cone_feature_size = 4 + 1 + 4
        self.OBSERVATION_SIZE = 2 + 3 + (self.num_closest_cones * self.cone_feature_size)
        self.get_logger().info(f"Calculated OBSERVATION_SIZE: {self.OBSERVATION_SIZE}")

    def _load_checkpoint(self):
        if os.path.exists(self.checkpoint_path):
            try:
                self.get_logger().info(f"Loading checkpoint from {self.checkpoint_path}...")
                checkpoint = torch.load(self.checkpoint_path, map_location=self.device)
                self.policy_net.load_state_dict(checkpoint['policy_net_state_dict'])
                self.target_net.load_state_dict(checkpoint['target_net_state_dict'])
                self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
                self.current_ros_episode_num = checkpoint['ros_episode_num']
                self.total_agent_steps_taken = checkpoint['total_agent_steps_taken']
                # self.memory.buffer = checkpoint['replay_buffer'] # Optional: If you save/load buffer
                self.target_net.eval() # Ensure target net is in eval mode
                self.get_logger().info(f"Checkpoint loaded. Resuming from ROS episode {self.current_ros_episode_num}, Total steps {self.total_agent_steps_taken}")
            except Exception as e:
                self.get_logger().error(f"Error loading checkpoint: {e}. Starting from scratch.", exc_info=True)
                self.current_ros_episode_num = 0
                self.total_agent_steps_taken = 0
        else:
            self.get_logger().info("No checkpoint found. Starting training from scratch.")
            self.current_ros_episode_num = 0
            self.total_agent_steps_taken = 0

    def _save_checkpoint(self):
        try:
            checkpoint_data = {
                'ros_episode_num': self.current_ros_episode_num,
                'total_agent_steps_taken': self.total_agent_steps_taken,
                'policy_net_state_dict': self.policy_net.state_dict(),
                'target_net_state_dict': self.target_net.state_dict(),
                'optimizer_state_dict': self.optimizer.state_dict(),
                # 'replay_buffer': self.memory.buffer # Optional: Saving buffer can make files very large
            }
            torch.save(checkpoint_data, self.checkpoint_path)
            self.get_logger().info(f"Checkpoint saved at ROS episode {self.current_ros_episode_num} to {self.checkpoint_path}")
        except Exception as e:
            self.get_logger().error(f"Error saving checkpoint: {e}", exc_info=True)


    def request_new_ros_episode_from_rl_gym(self):
        if self.current_ros_episode_num >= self.NUM_ROS_EPISODES_TO_RUN:
            self.get_logger().info("Maximum training ROS episodes reached. Halting requests.")
            if self.rl_step_timer: self.rl_step_timer.cancel()
            return

        self.get_logger().info(f"DQN Node: Requesting rl_gym to prepare for ROS episode {self.current_ros_episode_num + 1}.")
        start_msg_for_rl_gym = String()
        start_msg_for_rl_gym.data = "start"
        self.env_start_request_pub.publish(start_msg_for_rl_gym)
        
        # Also publish "start" to Jarred's /episode_status topic
        start_msg_for_jarred = String()
        start_msg_for_jarred.data = "start"
        self.episode_status_pub_for_jarred.publish(start_msg_for_jarred)
        self.get_logger().info("Published 'start' to /episode_status for Jarred's node.")

        self.env_confirmed_ready_for_rl = False
        self.current_rl_trajectory_step_count = 0
        self.s_t_observation_buffer = None
        self.a_t_action_idx_buffer = -1

    def env_started_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("DQN Node: Received confirmation from rl_gym - environment IS READY.")
            self.env_confirmed_ready_for_rl = True
        else:
            self.get_logger().error("DQN Node: rl_gym indicated environment FAILED to start. Scheduling retry...")
            self.env_confirmed_ready_for_rl = False
            rclpy.create_timer(5.0, self.request_new_ros_episode_from_rl_gym, self.get_clock(), oneshot=True)

    def current_goal_callback(self, msg: Point): self.latest_current_goal = msg
    def front_yolo_callback(self, msg: YoloDetections): self.latest_front_yolo_cones = [b for b in msg.bounding_boxes if b.class_name == "traffic-cones"]
    def back_yolo_callback(self, msg: YoloDetections): self.latest_back_yolo_cones = [b for b in msg.bounding_boxes if b.class_name == "traffic-cones"]
    def left_yolo_callback(self, msg: YoloDetections): self.latest_left_yolo_cones = [b for b in msg.bounding_boxes if b.class_name == "traffic-cones"]
    def right_yolo_callback(self, msg: YoloDetections): self.latest_right_yolo_cones = [b for b in msg.bounding_boxes if b.class_name == "traffic-cones"]
    def odom_callback(self, msg: Odometry): self.latest_odom = msg
    def in_track_callback(self, msg: Bool):
        if self.is_in_track and not msg.data: self.get_logger().warn("Transitioned to OFF TRACK based on /in_track_status.")
        self.is_in_track = msg.data

    def _normalize_bounding_box(self, box: BoundingBox, camera_id: int):
        img_width, img_height = self.image_dimensions.get(camera_id, (640.0, 480.0))
        img_width = 1.0 if img_width == 0 else img_width
        img_height = 1.0 if img_height == 0 else img_height
        center_x = box.left + (box.right - box.left) / 2.0
        center_y = box.top + (box.bottom - box.top) / 2.0
        width = float(box.right - box.left)
        height = float(box.bottom - box.top)
        norm_cx = np.clip(center_x / img_width, 0.0, 1.0)
        norm_cy = np.clip(center_y / img_height, 0.0, 1.0)
        norm_w = np.clip(width / img_width, 0.0, 1.0)
        norm_h = np.clip(height / img_height, 0.0, 1.0)
        return norm_cx, norm_cy, norm_w, norm_h

    def process_observation_vector(self):
        if not self.latest_odom or not self.latest_current_goal: return None
        lx = self.latest_odom.twist.twist.linear.x; az = self.latest_odom.twist.twist.angular.z
        norm_lx = np.clip(lx / self.max_car_speed, -1.0, 1.0); norm_az = np.clip(az / self.max_car_yaw_rate, -1.0, 1.0)
        car_obs_part = [norm_lx, norm_az]
        goal_obs_part = [0.0, 0.0, 0.0]
        if self.latest_current_goal:
            car_x = self.latest_odom.pose.pose.position.x; car_y = self.latest_odom.pose.pose.position.y
            q = self.latest_odom.pose.pose.orientation
            try:
                (roll, pitch, car_yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
            except NameError: # Fallback if tf_transformations not imported/available
                self.get_logger().warn_throttle(throttle_skip_first=True, throttle_time_source_type=rclpy.clock.ClockType.ROS_TIME, duration=10.0, msg="tf_transformations not available for euler_from_quaternion. Using simplified yaw.")
                car_yaw = 2.0 * math.atan2(q.z, q.w)


            gx, gy = self.latest_current_goal.x, self.latest_current_goal.y
            dx, dy = gx - car_x, gy - car_y
            dist_goal = math.sqrt(dx**2 + dy**2); norm_dist_goal = np.clip(dist_goal / self.max_goal_distance, 0.0, 1.0)
            angle_goal_world = math.atan2(dy, dx); angle_goal_relative = angle_goal_world - car_yaw
            while angle_goal_relative > math.pi: angle_goal_relative -= 2.0 * math.pi
            while angle_goal_relative < -math.pi: angle_goal_relative += 2.0 * math.pi
            goal_obs_part = [norm_dist_goal, math.cos(angle_goal_relative), math.sin(angle_goal_relative)]
        all_cones_processed = []
        yolo_sources = [(CAM_FRONT, self.latest_front_yolo_cones), (CAM_LEFT, self.latest_left_yolo_cones), (CAM_RIGHT, self.latest_right_yolo_cones), (CAM_BACK, self.latest_back_yolo_cones)]
        for cam_id, cone_list in yolo_sources:
            for box in cone_list:
                ncx, ncy, ncw, nch = self._normalize_bounding_box(box, cam_id)
                all_cones_processed.append({'cx': ncx, 'cy': ncy, 'w': ncw, 'h': nch, 'cam_id': cam_id, 'closeness': nch })
        all_cones_processed.sort(key=lambda c: c['closeness'], reverse=True)
        cone_obs_part = []
        for i in range(self.num_closest_cones):
            if i < len(all_cones_processed):
                cone = all_cones_processed[i]; present_flag = 1.0; cam_one_hot = [0.0]*4
                if cone['cam_id'] < 4 : cam_one_hot[cone['cam_id']] = 1.0
                cone_features = [cone['cx'], cone['cy'], cone['h'], cone['w'], present_flag] + cam_one_hot
                cone_obs_part.extend(cone_features)
            else:
                cone_obs_part.extend([0.0] * self.cone_feature_size)
        observation_list = car_obs_part + goal_obs_part + cone_obs_part
        final_observation_vector = np.array(observation_list, dtype=np.float32)
        if len(final_observation_vector) != self.OBSERVATION_SIZE: self.get_logger().error(f"OBS MISMATCH! {len(final_observation_vector)} vs {self.OBSERVATION_SIZE}"); return None
        return final_observation_vector

    def select_dqn_action(self, state_np_array: np.ndarray) -> int:
        sample = random.random()
        eps_threshold = self.EPS_END + (self.EPS_START - self.EPS_END) * \
            math.exp(-1. * self.total_agent_steps_taken / self.EPS_DECAY)

        if sample > eps_threshold: # Exploit
            self.policy_net.eval()
            with torch.no_grad():
                state_tensor = torch.tensor(state_np_array, dtype=torch.float32, device=self.device).unsqueeze(0)
                q_values = self.policy_net(state_tensor)
                action_idx = q_values.max(1)[1].item()
            # self.policy_net.train() # Only if layers like BatchNorm/Dropout are present
        else: # Explore
            action_idx = random.randrange(self.num_discrete_actions)
        return action_idx

    def publish_car_commands(self, discrete_action_index: int):
        if 0 <= discrete_action_index < self.num_discrete_actions:
            action_def = self.discrete_actions_map[discrete_action_index]
            throttle, steering, brake = float(action_def['throttle']), float(action_def['steering']), float(action_def['brake'])
            self.throttle_pub.publish(Float64(data=throttle))
            self.steering_pub.publish(Float64(data=steering))
            self.brake_pub.publish(Float64(data=brake))
        else:
            self.get_logger().error(f"Invalid action index: {discrete_action_index}. Sending brake command.")
            self.throttle_pub.publish(Float64(data=0.0)); self.steering_pub.publish(Float64(data=0.0)); self.brake_pub.publish(Float64(data=self.max_brake_torque * 0.5))

    def calculate_dense_reward(self, s_t_obs_vec, a_t_idx, s_t1_obs_vec, is_rl_trajectory_done: bool) -> float:
        reward = 0.0
        if not self.is_in_track:
            reward -= 50.0 # Large penalty for being off-track
            return reward # Early exit, no further rewards if off-track

        reward += 0.05 # Small survival reward for each step in track

        # Speed reward (from s_t+1, which is s_t1_obs_vec)
        if s_t1_obs_vec is not None and len(s_t1_obs_vec) > 0:
            norm_speed = s_t1_obs_vec[0] # norm_linear_velocity_x
            if norm_speed > 0.05: # If moving forward meaningfully
                reward += norm_speed * 2.0 # Reward proportional to normalized forward speed (tune factor)
            # elif norm_speed < -0.05: # Penalty for reversing if not intended
            #     reward -= abs(norm_speed) * 1.0

        # Goal progress reward
        if s_t_obs_vec is not None and s_t1_obs_vec is not None and \
           len(s_t_obs_vec) > 2 and len(s_t1_obs_vec) > 2:
            dist_s_t_norm = s_t_obs_vec[2]    # norm_dist_to_goal for s_t
            dist_s_t1_norm = s_t1_obs_vec[2]  # norm_dist_to_goal for s_t+1
            
            progress_to_goal_norm = dist_s_t_norm - dist_s_t1_norm
            
            if progress_to_goal_norm > 0.001: # Made some progress
                reward += progress_to_goal_norm * 20.0 # Significant reward (tune factor)
            elif progress_to_goal_norm < -0.001: # Moved away
                reward -= abs(progress_to_goal_norm) * 10.0 # Penalty (tune factor)

            # Goal alignment reward (from s_t+1)
            # s_t1_obs_vec[3] is goal_angle_cos, s_t1_obs_vec[4] is goal_angle_sin
            # cos(0) = 1, so reward if car is pointing towards goal
            if len(s_t1_obs_vec) > 4:
                goal_angle_cos_s_t1 = s_t1_obs_vec[3]
                if goal_angle_cos_s_t1 > 0.7: # Pointing somewhat towards goal (cos(45deg) ~ 0.7)
                    reward += goal_angle_cos_s_t1 * 1.0 # Reward for good alignment (tune factor)
        
        # Penalty for using brake unnecessarily (example)
        # action_def = self.discrete_actions_map[a_t_idx]
        # if action_def['brake'] > 0 and (s_t1_obs_vec is not None and s_t1_obs_vec[0] * self.max_car_speed < 0.1): # Braking at very low speed
        #     reward -= 0.5

        if is_rl_trajectory_done and self.is_in_track:
            # This means MAX_STEPS_PER_RL_TRAJECTORY was reached while still in track.
            # Could be a small penalty for not completing the entire track if there's a final goal.
            # Or, if it's a continuous task, this might not be a penalty.
            # If there's a way to know if the *final* goal of the whole track was reached:
            # if self.all_goals_completed_flag: reward += 500.0 # Very large bonus
            pass # For now, no specific reward/penalty here beyond survival.
        
        return reward

    def optimize_dqn_model(self):
        if len(self.memory) < self.BATCH_SIZE:
            return None
        
        states_np, actions_np, rewards_np, next_states_np, dones_np_float = self.memory.sample(self.BATCH_SIZE)
        
        states_tensor = torch.tensor(states_np, dtype=torch.float32, device=self.device)
        actions_tensor = torch.tensor(actions_np, dtype=torch.long, device=self.device).unsqueeze(-1)
        rewards_tensor = torch.tensor(rewards_np, dtype=torch.float32, device=self.device).unsqueeze(-1)
        next_states_tensor = torch.tensor(next_states_np, dtype=torch.float32, device=self.device)
        dones_tensor = torch.tensor(dones_np_float, dtype=torch.float32, device=self.device).unsqueeze(-1)

        self.policy_net.train()
        q_predicted = self.policy_net(states_tensor).gather(1, actions_tensor)
        
        self.target_net.eval()
        with torch.no_grad():
            q_next_target_values = self.target_net(next_states_tensor).max(1)[0].unsqueeze(-1)
            q_target = rewards_tensor + (self.GAMMA * q_next_target_values * (1.0 - dones_tensor))
            
        loss_fn = nn.SmoothL1Loss()
        loss = loss_fn(q_predicted, q_target)
        
        self.optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(self.policy_net.parameters(), max_norm=1.0) # Clip gradients
        self.optimizer.step()
        return loss.item()

    def perform_rl_step_and_learn(self):
        if not self.env_confirmed_ready_for_rl or self.processing_rl_step_active:
            return
        self.processing_rl_step_active = True

        s_t1_current_observation_vec = self.process_observation_vector()

        if s_t1_current_observation_vec is None:
            self.get_logger().warn_throttle(throttle_skip_first=True, throttle_time_source_type=rclpy.clock.ClockType.ROS_TIME, duration=1.0, msg="Obs data incomplete, skipping RL step.")
            self.processing_rl_step_active = False
            return

        # --- Store Experience (s_t, a_t, r_t, s_t+1, done_t) from PREVIOUS step ---
        if self.s_t_observation_buffer is not None and self.a_t_action_idx_buffer != -1:
            rl_trajectory_done_for_previous_step = not self.is_in_track or \
                                                 self.current_rl_trajectory_step_count >= self.MAX_STEPS_PER_RL_TRAJECTORY
            
            r_t_dense_reward = self.calculate_dense_reward(
                self.s_t_observation_buffer, self.a_t_action_idx_buffer, 
                s_t1_current_observation_vec, rl_trajectory_done_for_previous_step
            )
            self.memory.push(self.s_t_observation_buffer, self.a_t_action_idx_buffer, r_t_dense_reward, s_t1_current_observation_vec, rl_trajectory_done_for_previous_step)
            
            if self.total_agent_steps_taken % self.OPTIMIZE_MODEL_FREQUENCY == 0:
                loss = self.optimize_dqn_model()
                # if loss is not None: self.get_logger().debug(f"DQN Optim Loss: {loss:.4f}")
            
            if self.total_agent_steps_taken > 0 and self.total_agent_steps_taken % self.TARGET_UPDATE_FREQUENCY == 0:
                self.target_net.load_state_dict(self.policy_net.state_dict())
                self.get_logger().info(f"Updated Target Network at total_agent_step {self.total_agent_steps_taken}.")

            if rl_trajectory_done_for_previous_step:
                self.get_logger().info(f"RL Trajectory ended (InTrack: {self.is_in_track}, Steps: {self.current_rl_trajectory_step_count}). Waiting for Jarred's official ROS episode end signal.")
                self.publish_car_commands(self.discrete_actions_map.index(next(a for a in self.discrete_actions_map if a['description'] == "Brake Straight")))
                self.s_t_observation_buffer = None 
                self.a_t_action_idx_buffer = -1    
                self.processing_rl_step_active = False
                return 

        # --- Action Selection for current state s_t1_current_observation_vec ---
        action_to_take_idx = self.select_dqn_action(s_t1_current_observation_vec)
        self.publish_car_commands(action_to_take_idx)

        self.s_t_observation_buffer = s_t1_current_observation_vec 
        self.a_t_action_idx_buffer = action_to_take_idx          
        
        self.current_rl_trajectory_step_count += 1
        self.total_agent_steps_taken += 1
        self.processing_rl_step_active = False

    def jarred_episode_over_callback(self, msg: Float32):
        final_reward_from_jarred = msg.data
        self.get_logger().info(f"DQN Node: OFFICIAL ROS EPISODE END (Jarred's /episode_rewards). Jarred's Reward: {final_reward_from_jarred:.2f}")
        
        if self.s_t_observation_buffer is not None and self.a_t_action_idx_buffer != -1 and \
           not (not self.is_in_track or self.current_rl_trajectory_step_count >= self.MAX_STEPS_PER_RL_TRAJECTORY):
            
            s_t1_final_obs_at_jarred_end = self.process_observation_vector() 
            if s_t1_final_obs_at_jarred_end is not None:
                final_dense_reward_for_buffer = self.calculate_dense_reward(
                    self.s_t_observation_buffer, self.a_t_action_idx_buffer, 
                    s_t1_final_obs_at_jarred_end, True 
                )
                self.memory.push(self.s_t_observation_buffer, self.a_t_action_idx_buffer, final_dense_reward_for_buffer, s_t1_final_obs_at_jarred_end, True)
                # self.get_logger().debug(f"Pushed final experience due to Jarred's episode end. Dense Reward: {final_dense_reward_for_buffer:.2f}")

        self.env_confirmed_ready_for_rl = False 
        self.current_ros_episode_num += 1    
        self.s_t_observation_buffer = None 
        self.a_t_action_idx_buffer = -1    
        self.current_rl_trajectory_step_count = 0 

        if self.current_ros_episode_num > 0 and self.current_ros_episode_num % self.checkpoint_save_frequency == 0:
            self._save_checkpoint()

        self.get_logger().info("Scheduling request for new ROS episode from rl_gym...")
        self.publish_car_commands(self.discrete_actions_map.index(next(a for a in self.discrete_actions_map if a['description'] == "Brake Straight")))
        rclpy.create_timer(2.0, self.request_new_ros_episode_from_rl_gym, self.get_clock(), oneshot=True)


def main(args=None):
    rclpy.init(args=args)
    dqn_node = AudibotRLDriverDQNNode()
    try:
        rclpy.spin(dqn_node)
    except KeyboardInterrupt:
        dqn_node.get_logger().info("Keyboard interrupt received by DQN Node.")
    except Exception as e:
        dqn_node.get_logger().error(f"Unhandled exception in DQN Node: {e}", exc_info=True)
    finally:
        dqn_node.get_logger().info("Shutting down DQN Node...")
        if hasattr(dqn_node, 'throttle_pub'):
            try:
                dqn_node.throttle_pub.publish(Float64(data=0.0))
                dqn_node.steering_pub.publish(Float64(data=0.0))
                dqn_node.brake_pub.publish(Float64(data=dqn_node.max_brake_torque))
                dqn_node.get_logger().info("Sent final brake command.")
            except Exception as e_shutdown:
                dqn_node.get_logger().error(f"Error sending brake command on shutdown: {e_shutdown}")
        
        if hasattr(dqn_node, 'policy_net') and dqn_node.policy_net is not None:
            try:
                final_save_path = os.path.join(dqn_node.save_dir, dqn_node.final_model_filename)
                torch.save(dqn_node.policy_net.state_dict(), final_save_path)
                dqn_node.get_logger().info(f"Saved final policy network to {final_save_path}")
            except Exception as e_save:
                dqn_node.get_logger().error(f"Error saving final model: {e_save}", exc_info=True)

        if rclpy.ok():
            dqn_node.destroy_node()
            rclpy.shutdown()
        dqn_node.get_logger().info("DQN Node shutdown complete.")

if __name__ == '__main__':
    main()
