#!/usr/bin/env python3
# This script defines the main DQN agent node for controlling the Audibot.
# It handles ROS2 communication, observation processing, DQN learning, and action execution.

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.duration import Duration
from std_msgs.msg import String, Bool, Int64, Float32, Float64 # ROS standard message types
from geometry_msgs.msg import Point                 # For receiving current goal coordinates
from nav_msgs.msg import Odometry                   # For receiving car's odometry (position, velocity)
from yolo_msg.msg import YoloDetections             # Custom message for YOLO cone detections
from yolo_msg.msg import BoundingBox

import numpy as np
import math
import torch
import torch.nn as nn
import torch.optim as optim
import random
from collections import deque # For ReplayBuffer implementation
import time
import os # For file path operations (saving/loading models)
import json
from transformations import euler_from_quaternion # For robust yaw calculation from odometry quaternions
# import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy

import subprocess
import signal

# --- Constants for camera IDs ---
# Used to identify the source camera of YOLO detections when processing observations.
CAM_FRONT = 0
CAM_LEFT = 1
CAM_RIGHT = 2
CAM_BACK = 3
CAM_UNKNOWN = 4 # Fallback, should ideally not be used


# --- DQN Network Definition ---
# Defines the neural network architecture for the Q-value function approximation.
# This network takes an observation as input and outputs Q-values for each discrete action.
# NEEDS_TUNING: Network architecture (number of layers, neurons per layer, activation functions)
#               might need adjustment based on learning performance.
class DQNNetwork(nn.Module):
    def __init__(self, input_dim: int, output_dim: int):
        super(DQNNetwork, self).__init__()
        # Standard MLP (Multi-Layer Perceptron) architecture.
        self.fc1 = nn.Linear(input_dim, 256) # First hidden layer
        self.fc2 = nn.Linear(256, 128)       # Second hidden layer
        self.fc3 = nn.Linear(128, output_dim)# Output layer (Q-values)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # Defines the forward pass of the network.
        # Ensures input is a FloatTensor. Device handling is managed externally before calling.
        if not isinstance(x, torch.Tensor): x = torch.tensor(x, dtype=torch.float32)
        elif x.dtype != torch.float32: x = x.float()
        
        x = torch.relu(self.fc1(x)) # ReLU activation for hidden layers
        x = torch.relu(self.fc2(x))
        q_values = self.fc3(x)      # Linear output for Q-values
        return q_values

# --- Replay Buffer Definition ---
# Stores experiences (transitions) for off-policy learning.
# This helps break correlations between consecutive samples and allows for efficient reuse of experiences.
class ReplayBuffer:
    def __init__(self, capacity: int):
        self.buffer = deque(maxlen=capacity) # Uses a deque for efficient appends and pops

    def push(self, state, action: int, reward: float, next_state, done: bool):
        # Adds a new experience tuple to the buffer.
        self.buffer.append((np.array(state, dtype=np.float32),
                            action, # Discrete action index (int)
                            reward, # Dense reward (float)
                            np.array(next_state, dtype=np.float32),
                            done))  # Whether the RL trajectory ended (bool)

    def sample(self, batch_size: int):
        # Randomly samples a batch of experiences from the buffer.
        state, action, reward, next_state, done = zip(*random.sample(self.buffer, batch_size))
        return np.array(state, dtype=np.float32), \
               np.array(action, dtype=np.int64), \
               np.array(reward, dtype=np.float32), \
               np.array(next_state, dtype=np.float32), \
               np.array(done, dtype=np.float32) # 'done' converted to float for (1-done) masking in Bellman eq.

    def __len__(self) -> int:
        # Returns the current number of experiences stored in the buffer.
        return len(self.buffer)

# --- Main DQN Agent ROS2 Node ---
# This class encapsulates all logic for the DQN agent, including ROS interactions,
# observation processing, learning, and action execution.
class AudibotRLDriverDQNNode(Node):
    def __init__(self):
        super().__init__('audibot_rl_driver_dqn_node')
        self.use_env_manager = self.declare_parameter('use_env_manager', True).value

        self.get_logger().info("Audibot RL Driver (DQN) Node: Initializing...")

        # --- Hyperparameters & Configuration ---
        # These parameters control the DQN algorithm's behavior and training process.
        # NEEDS_TUNING: These are critical and will likely require adjustment based on experimental results.
        self.BUFFER_SIZE = self.declare_parameter('buffer_size', 50000).value
        self.BATCH_SIZE = self.declare_parameter('batch_size', 64).value
        self.GAMMA = self.declare_parameter('gamma', 0.99).value # Discount factor for future rewards
        self.EPS_START = self.declare_parameter('eps_start', 1.0).value # Initial epsilon for exploration
        self.EPS_END = self.declare_parameter('eps_end', 0.05).value   # Final epsilon value
        self.EPS_DECAY = self.declare_parameter('eps_decay', 30000).value # Epsilon decay rate (in agent steps)
        self.TARGET_UPDATE_FREQUENCY = self.declare_parameter('target_update_frequency', 1000).value # How often to update target network (in agent steps)
        self.LEARNING_RATE = self.declare_parameter('learning_rate', 0.0001).value # Optimizer learning rate
        self.OPTIMIZE_MODEL_FREQUENCY = self.declare_parameter('optimize_model_frequency', 4).value # How many agent steps per model optimization

        # Simulation/Episode Control Parameters
        self.NUM_ROS_EPISODES_TO_RUN = self.declare_parameter('num_ros_episodes_to_run', 2000).value # Total ROS-level episodes for training
        self.MAX_STEPS_PER_RL_TRAJECTORY = self.declare_parameter('max_steps_per_rl_trajectory', 1000).value # Max steps for an internal RL learning trajectory

        # Action Space Configuration (Car Control Limits)
        # NEEDS_TUNING: Ensure these reflect the car's actual capabilities in Gazebo.
        self.max_steering_angle_rad = self.declare_parameter('max_steering_angle_rad', 0.7).value # Approx 40 degrees
        self.max_brake_torque = self.declare_parameter('max_brake_torque', 1000.0).value # Example Nm
        self._define_discrete_actions() # Helper method to populate self.discrete_actions_map

        # Observation Space Configuration
        self.num_closest_cones = self.declare_parameter('num_closest_cones', 6).value # How many cones to include in observation
        self._define_image_dimensions_and_obs_size() # Defines self.OBSERVATION_SIZE and normalization constants

        # Model Saving Configuration
        self.save_dir = self.declare_parameter('save_dir', 'dqn_models_audibot').value
        self.checkpoint_filename = self.declare_parameter('checkpoint_filename', 'dqn_checkpoint.pth').value
        self.final_model_filename = self.declare_parameter('final_model_filename', 'dqn_final_policy.pth').value
        self.checkpoint_save_frequency = self.declare_parameter('checkpoint_save_frequency', 50).value # Save checkpoint every N ROS episodes
        
        self.checkpoint_path = os.path.join(self.save_dir, self.checkpoint_filename)
        self.final_model_path = os.path.join(self.save_dir, self.final_model_filename)
        os.makedirs(self.save_dir, exist_ok=True) # Create save directory if it doesn't exist

        # --- ROS Publishers ---
        # For interacting with rl_gym and the car controllers.
        self.qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.env_start_request_pub = self.create_publisher(String, 'dqn_agent/start_episode_request', 10) # To rl_gym
        self.throttle_pub = self.create_publisher(Float64, '/orange/throttle_cmd', 10) # To car
        self.steering_pub = self.create_publisher(Float64, '/orange/steering_cmd', 10) # To car
        self.brake_pub = self.create_publisher(Float64, '/orange/brake_cmd', 10)       # To car
        self.episode_status_pub_for_jarred = self.create_publisher(String, '/episode_status', 10) # To Jarred's rewards_node
        self.current_goal_pub = self.create_publisher(Point, '/current_goal', 10)


        # --- ROS Subscribers ---
        # For receiving information from rl_gym, Jarred's rewards_node, and sensors.
        self.env_started_confirm_sub = self.create_subscription(Bool, 'rl_gym/environment_started_confirm', self.env_started_callback, 10) # From rl_gym
        self.current_goal_sub = self.create_subscription(Point, '/current_goal', self.current_goal_callback, 10) # From Jarred
        self.front_yolo_sub = self.create_subscription(YoloDetections, '/orange/front_image_yolo/detections', self.front_yolo_callback, qos_profile=self.qos) # From Tim (YOLO)
        self.back_yolo_sub = self.create_subscription(YoloDetections, '/orange/back_image_yolo/detections', self.back_yolo_callback, qos_profile=self.qos)   # From Tim (YOLO)
        self.left_yolo_sub = self.create_subscription(YoloDetections, '/orange/left_image_yolo/detections', self.left_yolo_callback, qos_profile=self.qos)   # From Tim (YOLO)
        self.right_yolo_sub = self.create_subscription(YoloDetections, '/orange/right_image_yolo/detections', self.right_yolo_callback, qos_profile=self.qos)  # From Tim (YOLO)
        self.odom_sub = self.create_subscription(Odometry, '/orange/odom', self.odom_callback, qos_profile=self.qos) # From Gazebo/car model
        self.in_track_sub = self.create_subscription(Bool, '/in_track_status', self.in_track_callback, 10) # From Jarred
        self.jarred_episode_over_sub = self.create_subscription(Float32, '/episode_rewards', self.jarred_episode_over_callback, 10) # From Jarred

        # --- DQN Algorithm Components ---
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Using PyTorch device: {self.device}")
        self.policy_net = DQNNetwork(self.OBSERVATION_SIZE, self.num_discrete_actions).to(self.device)
        self.target_net = DQNNetwork(self.OBSERVATION_SIZE, self.num_discrete_actions).to(self.device)
        self.optimizer = optim.AdamW(self.policy_net.parameters(), lr=self.LEARNING_RATE, amsgrad=True)
        self.memory = ReplayBuffer(self.BUFFER_SIZE)
        
        self.total_agent_steps_taken = 0 # Global counter for training steps (epsilon decay, target updates)
        self.current_ros_episode_num = 0 # Tracks official ROS episodes requested/run

        self._load_checkpoint() # Attempt to load checkpoint after initializing networks and optimizer

        # --- State variables for interaction and data storage ---
        self.env_confirmed_ready_for_rl = False # Flag: Is rl_gym ready for this ROS episode?
        self.processing_rl_step_active = False  # Mutex to prevent re-entrant RL steps if timer is too fast
        self.is_in_track = True                 # Flag: Is car on track (from Jarred's /in_track_status)?
        self.latest_odom = None                 # Stores latest Odometry message
        self.latest_front_yolo_cones = []       # Stores filtered cone BoundingBoxes from front camera
        self.latest_back_yolo_cones = []
        self.latest_left_yolo_cones = []
        self.latest_right_yolo_cones = []
        self.latest_current_goal = None         # Stores current Point goal from Jarred

        self.s_t_observation_buffer = None      # Stores s_t (previous observation) for replay buffer
        self.a_t_action_idx_buffer = -1         # Stores a_t (last action index taken) for replay buffer
        self.current_rl_trajectory_step_count = 0 # Steps within the current internal RL learning trajectory
        
        # --- Goal / termination helpers ---
        self.GOAL_RADIUS_M        = 1.5      # distance at which we consider the goal “reached”
        self.MAX_GOAL_DISTANCE_M  = 35.0     # > this ⇒ abort episode
        self.ASSIST_PROB_START    = 0.8      # heuristic assistance at the beginning
        self.ASSIST_PROB_END      = 0.0      # fades to 0 as training progresses
        self.ASSIST_DECAY_STEPS   = 20_000   # how fast the assistance probability decays

        # Processes for the environment
        self.gazebo_process = None
        self.camera_process = None

        self.sub = None

        # Timer for the main RL agent step (action selection, learning)
        self.rl_step_timer = self.create_timer(0.1, self.perform_rl_step_and_learn) # Runs at 10Hz

        if self.use_env_manager:
            self.get_logger().info("Initialization complete. Requesting first ROS episode from rl_gym.")
            self.request_new_ros_episode_from_rl_gym()
        else:
            self.get_logger().info("Initialization complete. Running WITHOUT environment manager.")
            self.env_confirmed_ready_for_rl = True

        self.goals_data = []
        self.goal_index = 0
        self.goal_reached_time = None

        goal_file = os.path.expanduser("~/git/drive_to_survive/src/rl_rewards/goals/goals.json")
        try:
            with open(goal_file, 'r') as f:
                self.goals_data = json.load(f)["goals"]
                self.get_logger().info(f"Loaded {len(self.goals_data)} goals.")
        except Exception as e:
            self.get_logger().error(f"Failed to load goal JSON: {e}")

    def _check_if_goal_reached(self, car_x: float, car_y: float) -> bool:
        """Checks if current position is inside the current goal region (quad)."""
        if not self.goals_data:
            return False

        def area(x1, y1, x2, y2, x3, y3):
            return abs((x1*(y2 - y3) + x2*(y3 - y1) + x3*(y1 - y2)) / 2.0)

        quad = [(c['x'], c['y']) for c in self.goals_data[self.goal_index]['cones']]
        px, py = car_x, car_y

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

    def publish_current_goal(self):
        if not self.goals_data:
            self.get_logger().warn("No goals loaded; cannot publish current goal.")
            return
        
        goal = self.goals_data[self.goal_index]['centre']
        goal_msg = Point()
        goal_msg.x = goal['x']
        goal_msg.y = goal['y']
        goal_msg.z = 0.0  # z is unused

        self.current_goal_pub.publish(goal_msg)
        self.latest_current_goal = goal_msg  # Ensure internal obs processing has latest
        self.get_logger().debug(f"📍 Published current goal: ({goal_msg.x:.2f}, {goal_msg.y:.2f})")

    def _define_discrete_actions(self):
        # Defines the mapping from discrete action indices to throttle, steering, and brake commands.
        # NEEDS_TUNING: The specific values for throttle, steering, and brake levels might need adjustment
        #               based on car physics and desired behavior.
        THROTTLE_LEVELS = {"none": 0.0, "low": 0.35, "high": 0.7}
        STEERING_LEVELS = {
            "hard_left": -self.max_steering_angle_rad, "soft_left": -self.max_steering_angle_rad * 0.5,
            "straight": 0.0,
            "soft_right": self.max_steering_angle_rad * 0.5, "hard_right": self.max_steering_angle_rad
        }
        BRAKE_LEVELS = {"none": 0.0, "moderate": self.max_brake_torque * 0.4}

        self.discrete_actions_map = [
            # Braking Actions (Throttle OFF)
            {'throttle': THROTTLE_LEVELS["none"], 'steering': STEERING_LEVELS["straight"],   'brake': BRAKE_LEVELS["moderate"], 'description': "Brake Straight"},
            {'throttle': THROTTLE_LEVELS["none"], 'steering': STEERING_LEVELS["soft_left"], 'brake': BRAKE_LEVELS["moderate"], 'description': "Brake Soft Left"},
            {'throttle': THROTTLE_LEVELS["none"], 'steering': STEERING_LEVELS["soft_right"],'brake': BRAKE_LEVELS["moderate"], 'description': "Brake Soft Right"},
            # Coasting/Steering Only (Throttle OFF, Brake OFF)
            {'throttle': THROTTLE_LEVELS["none"], 'steering': STEERING_LEVELS["hard_left"],  'brake': BRAKE_LEVELS["none"], 'description': "Coast Hard Left"},
            {'throttle': THROTTLE_LEVELS["none"], 'steering': STEERING_LEVELS["soft_left"],  'brake': BRAKE_LEVELS["none"], 'description': "Coast Soft Left"},
            {'throttle': THROTTLE_LEVELS["none"], 'steering': STEERING_LEVELS["straight"],   'brake': BRAKE_LEVELS["none"], 'description': "Coast Straight (No-Op)"},
            {'throttle': THROTTLE_LEVELS["none"], 'steering': STEERING_LEVELS["soft_right"], 'brake': BRAKE_LEVELS["none"], 'description': "Coast Soft Right"},
            {'throttle': THROTTLE_LEVELS["none"], 'steering': STEERING_LEVELS["hard_right"], 'brake': BRAKE_LEVELS["none"], 'description': "Coast Hard Right"},
            # Forward Low Throttle (Brake OFF)
            {'throttle': THROTTLE_LEVELS["low"],  'steering': STEERING_LEVELS["hard_left"],  'brake': BRAKE_LEVELS["none"], 'description': "LowThrottle Hard Left"},
            {'throttle': THROTTLE_LEVELS["low"],  'steering': STEERING_LEVELS["soft_left"],  'brake': BRAKE_LEVELS["none"], 'description': "LowThrottle Soft Left"},
            {'throttle': THROTTLE_LEVELS["low"],  'steering': STEERING_LEVELS["straight"],   'brake': BRAKE_LEVELS["none"], 'description': "LowThrottle Straight"},
            {'throttle': THROTTLE_LEVELS["low"],  'steering': STEERING_LEVELS["soft_right"], 'brake': BRAKE_LEVELS["none"], 'description': "LowThrottle Soft Right"},
            {'throttle': THROTTLE_LEVELS["low"],  'steering': STEERING_LEVELS["hard_right"], 'brake': BRAKE_LEVELS["none"], 'description': "LowThrottle Hard Right"},
            # Forward High Throttle (Brake OFF)
            {'throttle': THROTTLE_LEVELS["high"], 'steering': STEERING_LEVELS["hard_left"],  'brake': BRAKE_LEVELS["none"], 'description': "HighThrottle Hard Left"},
            {'throttle': THROTTLE_LEVELS["high"], 'steering': STEERING_LEVELS["soft_left"],  'brake': BRAKE_LEVELS["none"], 'description': "HighThrottle Soft Left"},
            {'throttle': THROTTLE_LEVELS["high"], 'steering': STEERING_LEVELS["straight"],   'brake': BRAKE_LEVELS["none"], 'description': "HighThrottle Straight"},
            {'throttle': THROTTLE_LEVELS["high"], 'steering': STEERING_LEVELS["soft_right"], 'brake': BRAKE_LEVELS["none"], 'description': "HighThrottle Soft Right"},
            {'throttle': THROTTLE_LEVELS["high"], 'steering': STEERING_LEVELS["hard_right"], 'brake': BRAKE_LEVELS["none"], 'description': "HighThrottle Hard Right"}
        ]
        self.num_discrete_actions = len(self.discrete_actions_map)
        self.get_logger().info(f"Defined {self.num_discrete_actions} discrete actions.")

    def _define_image_dimensions_and_obs_size(self):
        # Defines image dimensions for normalization and calculates the total observation vector size.
        # NEEDS_TUNING: Normalization constants (max_car_speed, etc.) should reflect realistic simulation values.
        default_img_width = 640
        default_img_height = 480
        self.image_dimensions = {
            CAM_FRONT: (self.declare_parameter('front_cam.image_width', default_img_width).value, self.declare_parameter('front_cam.image_height', default_img_height).value),
            CAM_LEFT:  (self.declare_parameter('left_cam.image_width', default_img_width).value, self.declare_parameter('left_cam.image_height', default_img_height).value),
            CAM_RIGHT: (self.declare_parameter('right_cam.image_width', default_img_width).value, self.declare_parameter('right_cam.image_height', default_img_height).value),
            CAM_BACK:  (self.declare_parameter('back_cam.image_width', default_img_width).value, self.declare_parameter('back_cam.image_height', default_img_height).value),
        }
        self.max_car_speed = self.declare_parameter('max_car_speed_norm', 10.0).value # For observation normalization
        self.max_car_yaw_rate = self.declare_parameter('max_car_yaw_rate_norm', math.radians(90)).value # For obs norm
        self.max_goal_distance = self.declare_parameter('max_goal_distance_norm', 50.0).value # For obs norm

        # Observation vector structure:
        # [car_vel_x, car_yaw_rate] (2)
        # + [goal_dist, goal_angle_cos, goal_angle_sin] (3)
        # + N_cones * [cone_cx, cone_cy, cone_h, cone_w, present_flag, one_hot_cam (4 features)] (N_cones * 9)
        self.cone_feature_size = 4 + 1 + 4 # cx,cy,h,w + present_flag + 4cam_one_hot
        self.OBSERVATION_SIZE = 2 + 3 + (self.num_closest_cones * self.cone_feature_size)
        self.get_logger().info(f"Calculated OBSERVATION_SIZE: {self.OBSERVATION_SIZE}")

    def _load_checkpoint(self):
        # Attempts to load a previously saved training checkpoint.
        if os.path.exists(self.checkpoint_path):
            try:
                self.get_logger().info(f"Loading checkpoint from {self.checkpoint_path}...")
                checkpoint = torch.load(self.checkpoint_path, map_location=self.device) # Load to current device
                self.policy_net.load_state_dict(checkpoint['policy_net_state_dict'])
                self.target_net.load_state_dict(checkpoint['target_net_state_dict'])
                self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
                self.current_ros_episode_num = checkpoint['ros_episode_num']
                self.total_agent_steps_taken = checkpoint['total_agent_steps_taken']
                # Optional: Load replay buffer if saved (can be large)
                # if 'replay_buffer' in checkpoint: self.memory.buffer = checkpoint['replay_buffer']
                self.target_net.eval() # Ensure target net is in eval mode after loading
                self.get_logger().info(f"Checkpoint loaded. Resuming from ROS episode {self.current_ros_episode_num + 1}, Total steps {self.total_agent_steps_taken}")
            except Exception as e:
                self.get_logger().error(f"Error loading checkpoint: {e}. Starting from scratch.", exc_info=True)
                self.current_ros_episode_num = 0
                self.total_agent_steps_taken = 0
        else:
            self.get_logger().info("No checkpoint found at specified path. Starting training from scratch.")
            self.current_ros_episode_num = 0
            self.total_agent_steps_taken = 0

    def _save_checkpoint(self):
        # Saves the current training state (networks, optimizer, progress) to a checkpoint file.
        try:
            checkpoint_data = {
                'ros_episode_num': self.current_ros_episode_num, # Save the count of completed ROS episodes
                'total_agent_steps_taken': self.total_agent_steps_taken,
                'policy_net_state_dict': self.policy_net.state_dict(),
                'target_net_state_dict': self.target_net.state_dict(),
                'optimizer_state_dict': self.optimizer.state_dict(),
                # 'replay_buffer': self.memory.buffer # Optional: Saving buffer can make files very large
            }
            torch.save(checkpoint_data, self.checkpoint_path)
            self.get_logger().info(f"Checkpoint saved at ROS episode {self.current_ros_episode_num} (Total steps: {self.total_agent_steps_taken}) to {self.checkpoint_path}")
        except Exception as e:
            self.get_logger().error(f"Error saving checkpoint: {e}", exc_info=True)

    # --- ROS Communication Logic ---
    def request_new_ros_episode_from_rl_gym(self):
        # Publishes a request to rl_gym to start/reset the Gazebo environment for a new episode.
        # Also informs Jarred's rewards_node that a new episode is beginning.
        # DEPENDS_ON_RL_GYM: rl_gym node must be running and listening to 'dqn_agent/start_episode_request'.
        # DEPENDS_ON_REWARDS_PKG: Jarred's node must be listening to '/episode_status'.
        
        if not self.use_env_manager:
            return

        if self.current_ros_episode_num >= self.NUM_ROS_EPISODES_TO_RUN:
            self.get_logger().info("Maximum training ROS episodes reached. Halting further episode requests.")
            if self.rl_step_timer: self.rl_step_timer.cancel() # Stop the RL agent's step timer
            return
        
        if self.gazebo_process is not None:
            self.shutdown_environment()

        self.get_logger().info(f"DQN Node: Requesting rl_gym to prepare for ROS episode {self.current_ros_episode_num + 1}.")

        self.env_confirmed_ready_for_rl = False # Reset flag, wait for rl_gym confirmation
        self.current_rl_trajectory_step_count = 0 # Reset steps for the new RL trajectory
        self.s_t_observation_buffer = None      # Clear previous observation for the new episode
        self.a_t_action_idx_buffer = -1         # Clear previous action
        
        self.start_environment()


        

    def env_started_callback(self, msg: Bool):
        # Callback for rl_gym's confirmation that the environment is ready.
        # DEPENDS_ON_RL_GYM: Expects 'rl_gym/environment_started_confirm' topic.
        if msg.data:
            self.get_logger().info("✅ DQN Node: Received confirmation from rl_gym - environment IS READY.")
            self.env_confirmed_ready_for_rl = True
            # The perform_rl_step_and_learn timer will now be able to proceed fully once sensor data arrives.
        else:
            self.get_logger().error("❌ DQN Node: rl_gym indicated environment FAILED to start. Scheduling retry...")
            self.env_confirmed_ready_for_rl = False
            # Consider a more robust retry mechanism or error handling
            rclpy.create_timer(5.0, self.request_new_ros_episode_from_rl_gym, self.get_clock(), oneshot=True)

    # --- Raw Sensor Data Callbacks ---
    # These callbacks simply store the latest received sensor data.
    # Processing into the observation vector happens in process_observation_vector().
    def current_goal_callback(self, msg: Point): 
        self.latest_current_goal = msg
        self.get_logger().debug(f"✅ Received goal: ({msg.x:.2f}, {msg.y:.2f})")

    def front_yolo_callback(self, msg: YoloDetections): self.latest_front_yolo_cones = [b for b in msg.bounding_boxes if b.class_name == "traffic-cones"]
    def back_yolo_callback(self, msg: YoloDetections): self.latest_back_yolo_cones = [b for b in msg.bounding_boxes if b.class_name == "traffic-cones"]
    def left_yolo_callback(self, msg: YoloDetections): self.latest_left_yolo_cones = [b for b in msg.bounding_boxes if b.class_name == "traffic-cones"]
    def right_yolo_callback(self, msg: YoloDetections): self.latest_right_yolo_cones = [b for b in msg.bounding_boxes if b.class_name == "traffic-cones"]
    def odom_callback(self, msg: Odometry): 
        self.latest_odom = msg
        self.get_logger().info(f"✅ Received odom: pos=({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})")
        self.get_logger().debug(f"✅ Received odom: pos=({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})")

    def in_track_callback(self, msg: Bool):
        # DEPENDS_ON_REWARDS_PKG: Expects '/in_track_status' from Jarred's node.
        if self.is_in_track and not msg.data: self.get_logger().warn("Transitioned to OFF TRACK based on /in_track_status.")
        self.is_in_track = msg.data

    # --- Observation Processing ---
    def _normalize_bounding_box(self, box: BoundingBox, camera_id: int) -> tuple:
        # Normalizes bounding box coordinates and dimensions to the range [0, 1].
        img_width, img_height = self.image_dimensions.get(camera_id, (640.0, 480.0)) # Default if not found
        img_width = 1.0 if img_width == 0 else img_width # Avoid division by zero
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

    def process_observation_vector(self) -> np.ndarray | None:
        # Constructs the fixed-size observation vector from all latest sensor data.
        # This is a critical part for the DQN agent's input.
        # Returns None if essential data (odom, current_goal) is missing.
        if not self.latest_odom or not self.goals_data:
            self.get_logger().warn("Missing odometry or goal list; observation invalid.")
            return None
        # if not self.latest_odom:
        #     self.get_logger().warn("Missing odometry data.")
        # if not self.latest_current_goal:
        #     self.get_logger().warn("Missing current goal data.")
        # if not self.latest_odom or not self.latest_current_goal:
        #     return None

        # Car state part
        lx = self.latest_odom.twist.twist.linear.x
        az = self.latest_odom.twist.twist.angular.z
        norm_lx = np.clip(lx / self.max_car_speed, -1.0, 1.0)
        norm_az = np.clip(az / self.max_car_yaw_rate, -1.0, 1.0)
        car_obs_part = [norm_lx, norm_az]

        # Goal state part
        goal_obs_part = [0.0, 0.0, 0.0] # Default: [dist_norm, cos(angle_rel), sin(angle_rel)]
        car_x = self.latest_odom.pose.pose.position.x
        car_y = self.latest_odom.pose.pose.position.y
        q = self.latest_odom.pose.pose.orientation
        try:
            # Robust yaw calculation using tf_transformations
            (roll, pitch, car_yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        except NameError: # Fallback if tf_transformations is not imported (should not happen if setup correctly)
            self.get_logger().error("tf_transformations.euler_from_quaternion not available! Yaw calculation will be incorrect if car pitches/rolls.")
            car_yaw = 2.0 * math.atan2(q.z, q.w) # Simplified, less robust backup

        # gx, gy = self.latest_current_goal.x, self.latest_current_goal.y
        goal = self.goals_data[self.goal_index]['centre']  # centre from goals.json
        gx, gy = goal['x'], goal['y']
        dx, dy = gx - car_x, gy - car_y
        dist_goal = math.sqrt(dx**2 + dy**2)
        norm_dist_goal = np.clip(dist_goal / self.max_goal_distance, 0.0, 1.0)
        
        if dist_goal > self.MAX_GOAL_DISTANCE_M:
            # Mark a special flag so the main loop knows to end trajectory/episode
            self.too_far_from_goal = True
        else:
            self.too_far_from_goal = False
        
        angle_goal_world = math.atan2(dy, dx)
        angle_goal_relative = angle_goal_world - car_yaw
        # Normalize angle to [-pi, pi]
        while angle_goal_relative > math.pi: angle_goal_relative -= 2.0 * math.pi
        while angle_goal_relative < -math.pi: angle_goal_relative += 2.0 * math.pi
        goal_obs_part = [norm_dist_goal, math.cos(angle_goal_relative), math.sin(angle_goal_relative)]

        # Cone state part
        all_cones_processed = []
        yolo_sources = [
            (CAM_FRONT, self.latest_front_yolo_cones), (CAM_LEFT, self.latest_left_yolo_cones),
            (CAM_RIGHT, self.latest_right_yolo_cones), (CAM_BACK, self.latest_back_yolo_cones)
        ]
        for cam_id, cone_list in yolo_sources:
            for box in cone_list: # box is a BoundingBox message from yolo_msg
                ncx, ncy, ncw, nch = self._normalize_bounding_box(box, cam_id)
                all_cones_processed.append({'cx': ncx, 'cy': ncy, 'w': ncw, 'h': nch,
                                          'cam_id': cam_id, 'closeness': nch}) # Use normalized height as closeness proxy
        
        all_cones_processed.sort(key=lambda c: c['closeness'], reverse=True) # Sort by closeness (largest height first)

        cone_obs_part = []
        for i in range(self.num_closest_cones):
            if i < len(all_cones_processed):
                cone = all_cones_processed[i]
                present_flag = 1.0
                cam_one_hot = [0.0]*4 # For CAM_FRONT, LEFT, RIGHT, BACK
                if cone['cam_id'] < 4 : cam_one_hot[cone['cam_id']] = 1.0 # Ensure cam_id is a valid index
                cone_features = [cone['cx'], cone['cy'], cone['h'], cone['w'], present_flag] + cam_one_hot
                cone_obs_part.extend(cone_features)
            else:
                # Pad with zeros if fewer than num_closest_cones are detected
                cone_obs_part.extend([0.0] * self.cone_feature_size)
        
        observation_list = car_obs_part + goal_obs_part + cone_obs_part
        final_observation_vector = np.array(observation_list, dtype=np.float32)

        if len(final_observation_vector) != self.OBSERVATION_SIZE:
            self.get_logger().error(f"Observation vector length mismatch! Expected {self.OBSERVATION_SIZE}, got {len(final_observation_vector)}. Check processing logic.")
            return None # Return None if size is incorrect
        return final_observation_vector
    
    def _heuristic_action_toward_goal(self, obs: np.ndarray) -> int:
        """
        Very rough steering heuristic just to bootstrap learning:
        - chooses throttle 'low'
        - picks steering bucket based on goal angle (relative).
        """
        _, _, dist_norm, cos_ang, sin_ang = obs[0], obs[1], obs[2], obs[3], obs[4]
        angle = math.atan2(sin_ang, cos_ang)  # [-pi, pi]
        # Choose a steering bucket
        if angle > 0.4:    steering = "hard_left"
        elif angle > 0.15: steering = "soft_left"
        elif angle < -0.4: steering = "hard_right"
        elif angle < -0.15:steering = "soft_right"
        else:              steering = "straight"
        # Find the matching discrete action with 'low' throttle & no brake
        for idx, a in enumerate(self.discrete_actions_map):
            if a['throttle'] > 0 and a['throttle'] <= 0.4 and a['description'].startswith("LowThrottle") and steering in a['description']:
                return idx
        return random.randrange(self.num_discrete_actions)


    # --- DQN Action Selection (Epsilon-Greedy) ---
    def select_dqn_action(self, state_np_array: np.ndarray) -> int:
        """
        1)   Heuristic assist (probability decays with training steps)
        2)   Standard ε-greedy over the learned Q-network
        """
        # ---------- assisted-driving probability ----------
        assist_p = self.ASSIST_PROB_END + (self.ASSIST_PROB_START - self.ASSIST_PROB_END) * \
                math.exp(-1.0 * self.total_agent_steps_taken / self.ASSIST_DECAY_STEPS)

        if random.random() < assist_p:
            return self._heuristic_action_toward_goal(state_np_array)
        # --------------------------------------------------

        # ---------- ε-greedy selection --------------------
        eps_threshold = self.EPS_END + (self.EPS_START - self.EPS_END) * \
                        math.exp(-1.0 * self.total_agent_steps_taken / self.EPS_DECAY)

        if random.random() > eps_threshold:
            # exploit
            with torch.no_grad():
                q_vals = self.policy_net(
                    torch.tensor(state_np_array, dtype=torch.float32, device=self.device).unsqueeze(0)
                )
            return int(q_vals.argmax().item())
        else:
            # explore
            return random.randrange(self.num_discrete_actions)
    # --------------------------------------------------


    # --- Car Command Publishing ---
    def publish_car_commands(self, discrete_action_index: int):
        # Maps the chosen discrete action index to actual throttle, steering, and brake commands.
        if 0 <= discrete_action_index < self.num_discrete_actions:
            action_def = self.discrete_actions_map[discrete_action_index]
            throttle = float(action_def['throttle'])
            steering = float(action_def['steering']) # Assumes map values are already in radians
            brake = float(action_def['brake'])       # Assumes map values are already in Nm (or scaled appropriately)

            self.throttle_pub.publish(Float64(data=throttle))
            self.steering_pub.publish(Float64(data=steering))
            self.brake_pub.publish(Float64(data=brake))
        else:
            self.get_logger().error(f"Invalid action index: {discrete_action_index} received in publish_car_commands. Sending safe brake command.")
            # Fallback safe action
            self.throttle_pub.publish(Float64(data=0.0))
            self.steering_pub.publish(Float64(data=0.0))
            self.brake_pub.publish(Float64(data=self.max_brake_torque * 0.5)) # Moderate brake

    # --- Dense Reward Calculation (CRUCIAL FOR DQN LEARNING) ---
    def calculate_dense_reward(
        self,
        s_t_obs_vec: np.ndarray | None,
        a_t_idx: int,
        s_t1_obs_vec: np.ndarray | None,
        is_rl_trajectory_done: bool
    ) -> float:
        """
        Dense shaping signal used every step.
        Positive ≈ “you’re making progress & driving well”.
        Negative ≈ “you’re off-track or moving away from the goal”.
        """

        # --------------------------------------------------
        reward = 0.0                       # ← always start with zero
        # --------------------------------------------------

        # 1.  Big penalty if the car has left the track
        if not self.is_in_track:
            return -50.0                   # early-exit (nothing else matters)

        # 2.  Tiny survival reward keeps the agent alive & exploring
        reward += 0.05

        # 3.  Forward-speed reward (encourage motion, but only forward)
        if s_t1_obs_vec is not None and len(s_t1_obs_vec) > 0:
            norm_speed = max(0.0, s_t1_obs_vec[0])   # clip reverse
            reward += norm_speed * 1.0               # 1 pt per 0-1 normalised speed

        # 4.  *Progress* towards the active goal (dominant term!)
        if s_t_obs_vec is not None and s_t1_obs_vec is not None:
            d_prev = s_t_obs_vec[2]  * self.max_goal_distance   # metres
            d_curr = s_t1_obs_vec[2] * self.max_goal_distance
            delta  = d_prev - d_curr                            # +ve ⇒ closer
            reward += 5.0 * delta                               # 5 pts per metre

        # 5.  Alignment bonus (points car in the right direction)
        if s_t1_obs_vec is not None and len(s_t1_obs_vec) > 3:
            cos_rel = s_t1_obs_vec[3]                           # already cos(angle)
            if cos_rel > 0.7:                                   # ≈ <45°
                reward += cos_rel                               # up to +1

        # 6.  Bonus for actually entering the goal quad
        if self.latest_odom and self.goals_data:
            p = self.latest_odom.pose.pose.position
            if self._check_if_goal_reached(p.x, p.y):
                now = self.get_clock().now().nanoseconds / 1e9
                if self.goal_reached_time is None or (now - self.goal_reached_time) > 2.0:
                    reward += 50.0                               # big bonus
                    self.goal_reached_time = now
                    self.goal_index = (self.goal_index + 1) % len(self.goals_data)
                    self.publish_current_goal()

        # 7.  Small “trajectory survived” bonus
        if is_rl_trajectory_done and self.is_in_track:
            reward += 10.0

        return float(reward)


    # --- DQN Model Optimization (Adapted from train_agent.py) ---
    def optimize_dqn_model(self):
        # Performs one step of learning on the policy_net using a batch from the replay_buffer.
        if len(self.memory) < self.BATCH_SIZE: # Don't learn until buffer has enough experiences
            return None 
        
        # Sample a batch of experiences
        states_np, actions_np, rewards_np, next_states_np, dones_np_float = self.memory.sample(self.BATCH_SIZE)
        
        # Convert numpy arrays to PyTorch tensors and move to the designated device
        states_tensor = torch.tensor(states_np, dtype=torch.float32, device=self.device)
        actions_tensor = torch.tensor(actions_np, dtype=torch.long, device=self.device).unsqueeze(-1) # Actions are indices
        rewards_tensor = torch.tensor(rewards_np, dtype=torch.float32, device=self.device).unsqueeze(-1)
        next_states_tensor = torch.tensor(next_states_np, dtype=torch.float32, device=self.device)
        dones_tensor = torch.tensor(dones_np_float, dtype=torch.float32, device=self.device).unsqueeze(-1) # Mask for terminal states

        # --- Calculate Q(s_t, a_t) ---
        # Get Q-values from policy_net for actions that were actually taken
        self.policy_net.train() # Set policy_net to training mode (affects layers like Dropout, BatchNorm)
        q_predicted = self.policy_net(states_tensor).gather(1, actions_tensor) # Gathers Q-values for chosen actions
        
        # --- Calculate Target Q-value: r_t + gamma * max_a' Q_target(s_t+1, a') ---
        self.target_net.eval() # Target network in evaluation mode
        with torch.no_grad(): # No gradients needed for target computation
            # Get Q-values for next_states from target_net and select max Q-value for each next_state
            q_next_target_values = self.target_net(next_states_tensor).max(1)[0].unsqueeze(-1)
            # Compute the target Q-value using the Bellman equation
            # If a state is terminal (done), its target Q-value is just the reward received.
            q_target = rewards_tensor + (self.GAMMA * q_next_target_values * (1.0 - dones_tensor))
            
        # --- Compute Loss ---
        loss_fn = nn.SmoothL1Loss() # Huber loss, less sensitive to outliers than MSE
        loss = loss_fn(q_predicted, q_target)
        
        # --- Optimize the policy_net ---
        self.optimizer.zero_grad() # Clear previous gradients
        loss.backward()           # Compute gradients of the loss w.r.t. policy_net parameters
        torch.nn.utils.clip_grad_norm_(self.policy_net.parameters(), max_norm=1.0) # Optional: Gradient clipping
        self.optimizer.step()     # Update policy_net weights
        
        return loss.item() # Return scalar loss value for logging
    
    def wait_for_publishers(self, topic_name: str, timeout_sec: float = 10.0) -> bool:
        """Wait until topic has at least one publisher."""
        start_time = self.get_clock().now()
        while rclpy.ok():
            # topic_info = self.get_topic_names_and_types(topic_name)
            publishers_amount = self.count_publishers(topic_name)
            # self.get_logger().info(f"publishers_info {publishers_amount}")
            if publishers_amount > 0:
                return True
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_sec:
                self.get_logger().warning(f"Timeout waiting for publishers on {topic_name}")
                return False
            time.sleep(0.1)
    
    def wait_for_first_message(self, topic_name: str, msg_type, timeout_sec: float = 120.0) -> bool:
        received_msg = False

        def on_msg(msg):
            nonlocal received_msg
            received_msg = True

        sub = self.create_subscription(msg_type, topic_name, on_msg, qos_profile=self.qos)

        start_time = self.get_clock().now()
        while rclpy.ok() and not received_msg:
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_sec:
                self.get_logger().warning(f"Timeout waiting for first message on {topic_name}")
                self.destroy_subscription(sub)
                return False
            rclpy.spin_once(self, timeout_sec=0.1)  # spin the node to process callbacks
            self.get_logger().info(f"ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', 'Not Set')}")
            self.get_logger().info(f"waiting on topic first message")
            self.get_logger().info(f"publishers_info {self.count_publishers(topic_name)}")

        self.destroy_subscription(sub)
        return True
    
    def wait_for_environment_ready(self):
        topics_and_types = [
            ('/orange/odom', Odometry),
            ('/orange/right_image_yolo/detections', YoloDetections),
            ('/orange/left_image_yolo/detections', YoloDetections),
            ('/orange/front_image_yolo/detections', YoloDetections),
            ('/orange/back_image_yolo/detections', YoloDetections),
        ]

        for topic, msg_type in topics_and_types:
            self.get_logger().info(f"Waiting for publisher and first message on {topic}...")
            if not self.wait_for_publishers(topic, timeout_sec=10):
                return False
            if not self.wait_for_first_message(topic, msg_type):
                return False
        return True

    def resubscribe_to_topics(self):
        if hasattr(self, 'odom_sub') and self.odom_sub is not None:
            self.destroy_subscription(self.odom_sub)

        self.odom_sub = self.create_subscription(Odometry, '/orange/odom', self.odom_callback, qos_profile=self.qos)

        self.get_logger().info("Re-subscribed")
    
    def start_environment(self):
        try:
            # Launch Gazebo
            self.gazebo_process = subprocess.Popen(
                ['ros2', 'launch', 'gazebo_tf', 'drive_to_survive.launch.py', 'gui:=false', 'use_rviz:=true'],
                # stdout=subprocess.DEVNULL,
                # stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
                )

            # Launch Camera nodes
            self.camera_process = subprocess.Popen(
                ['ros2', 'launch', 'audibot_yolo', 'multi_camera.launch.py'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
                )

            self.get_logger().info('Waiting for Gazebo and camera nodes to initialize...')
            
            if self.wait_for_environment_ready():
                self.get_logger().info('Environment ready.')
                self.resubscribe_to_topics()
                # making sure starting opsition is right before confirming ready
                self.wait_for_initial_pose()
                
                self.env_confirmed_ready_for_rl = True

            else:
                self.get_logger().error('Timed out waiting for environment readiness.')
                self.shutdown_environment()
                self.env_confirmed_ready_for_rl = False

        except Exception as e:
            self.get_logger().error(f'Failed to start environment: {e}')
            self.shutdown_environment()

    def wait_for_initial_pose(self, target_x=24.2, target_y=13.2, tolerance=0.1, timeout=60.0):
        self.get_logger().info(f"Waiting for Audibot to reach starting position x={target_x}, y={target_y}...")

        start_time = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.latest_odom is not None:
                position = self.latest_odom.pose.pose.position
                if (
                    abs(position.x - target_x) < tolerance and
                    abs(position.y - target_y) < tolerance
                ):
                    self.get_logger().info("Audibot is at the starting pose.")
                    return True

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            self.resubscribe_to_topics()
            if elapsed > timeout:
                self.get_logger().warning("Timed out waiting for initial pose.")
                return False

    def shutdown_environment(self):
        def terminate_process(p):
            if p:
                self.get_logger().info(f"Terminating process PID={p.pid}...")
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except ProcessLookupError:
                    self.get_logger().warning(f"Process group for PID={p.pid} does not exist (already terminated?)")
                    return

                try:
                    p.wait(timeout=10)
                    self.get_logger().info(f"Process PID={p.pid} terminated (not gracefully...).")
                except subprocess.TimeoutExpired:
                    self.get_logger().warning(f"Process PID={p.pid} did not terminate in time. Forcing kill...")
                    try:
                        os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                        p.wait(timeout=5)
                        self.get_logger().info(f"Process PID={p.pid} force-killed.")
                    except Exception as e:
                        self.get_logger().error(f"Failed to kill process PID={p.pid}: {e}")
        
        terminate_process(self.gazebo_process)
        terminate_process(self.camera_process)

        self.gazebo_process = None
        self.camera_process = None
        

        self.get_logger().info('Simulation processes terminated.')
        self.env_confirmed_ready_for_rl = False

    # --- Main RL Step (called by timer) ---
    def perform_rl_step_and_learn(self):
        # This method is called periodically by self.rl_step_timer.
        # It orchestrates observation processing, action selection, experience storing, and learning.
        self.get_logger().debug("🔁 perform_rl_step_and_learn called")

        # Ensure environment is ready and not already processing a step
        if not self.env_confirmed_ready_for_rl or self.processing_rl_step_active:
            self.get_logger().debug("RL step skipped: Env not ready or already processing step.")
            return
        
        self.processing_rl_step_active = True # Set mutex to prevent re-entry

        # 1. Get current observation (s_t+1 for the previous action)
        s_t1_current_observation_vec = self.process_observation_vector()

        if s_t1_current_observation_vec is None:
            rclpy.logging.get_logger('audibot_rl_driver_dqn_node').warn("Observation data incomplete, cannot perform RL step.")
            self.processing_rl_step_active = False
            return
    
        # Abort entire ROS episode if > 50 m from goal
        if getattr(self, "too_far_from_goal", False):
            self.get_logger().info("🚫 Too far from goal – ending episode early.")
            self.publish_car_commands(self.discrete_actions_map.index(
                next(a for a in self.discrete_actions_map if a['description'] == "Brake Straight")))
            # Pretend rewards-node sent an end-of-episode signal:
            self.jarred_episode_over_callback(Float32(data=0.0))
            self.processing_rl_step_active = False
            return

        # --- If there's a previous state and action, form an experience tuple and learn ---
        if self.s_t_observation_buffer is not None and self.a_t_action_idx_buffer != -1:
            # s_t1_current_observation_vec is s_t+1 for the (s_t, a_t) pair stored in buffers.
            
            # Determine if the RL trajectory ended at s_t+1 due to internal conditions
            # This "done" is for the Bellman equation and replay buffer.
            rl_trajectory_done_at_s_t1 = not self.is_in_track or \
                                         self.current_rl_trajectory_step_count >= self.MAX_STEPS_PER_RL_TRAJECTORY
            
            # Calculate dense reward (r_t) for the transition (s_t, a_t) -> s_t+1
            r_t_dense_reward = self.calculate_dense_reward(
                self.s_t_observation_buffer,    # s_t
                self.a_t_action_idx_buffer,     # a_t
                s_t1_current_observation_vec,   # s_t+1
                rl_trajectory_done_at_s_t1      # done_t (for this transition)
            )
            # Store the experience (s_t, a_t, r_t, s_t+1, done_t)
            self.memory.push(self.s_t_observation_buffer, self.a_t_action_idx_buffer, r_t_dense_reward, s_t1_current_observation_vec, rl_trajectory_done_at_s_t1)
            
            # Perform model optimization periodically
            if self.total_agent_steps_taken > self.BATCH_SIZE and \
               self.total_agent_steps_taken % self.OPTIMIZE_MODEL_FREQUENCY == 0:
                loss = self.optimize_dqn_model()
                # if loss is not None: self.get_logger().debug(f"DQN Optim Loss: {loss:.4f}")
            
            # Update target network periodically
            if self.total_agent_steps_taken > 0 and self.total_agent_steps_taken % self.TARGET_UPDATE_FREQUENCY == 0:
                self.target_net.load_state_dict(self.policy_net.state_dict())
                self.get_logger().info(f"Updated Target Network at total_agent_step {self.total_agent_steps_taken}.")

            # If the RL trajectory ended for s_t (leading to s_t+1 which is terminal for this trajectory)
            if rl_trajectory_done_at_s_t1: # This variable name was from previous logic, should be rl_trajectory_done_at_s_t1
                self.get_logger().info(f"RL Trajectory ended (InTrack: {self.is_in_track}, Steps: {self.current_rl_trajectory_step_count}). Waiting for Jarred's official ROS episode end signal.")
                # Stop the car if RL trajectory is done.
                self.publish_car_commands(self.discrete_actions_map.index(next(a for a in self.discrete_actions_map if a['description'] == "Brake Straight")))
                self.s_t_observation_buffer = None # Clear buffer as this trajectory is done for learning
                self.a_t_action_idx_buffer = -1    # Clear buffer
                # current_rl_trajectory_step_count is reset when new ROS episode starts (via request_new_ros_episode_from_rl_gym)
                self.processing_rl_step_active = False

                self.shutdown_environment()

                return # Wait for Jarred's signal before starting new trajectory logic within a new ROS episode

        # --- Action Selection for current state s_t1_current_observation_vec (which becomes s_t for next step) ---
        # This action will be a_t for the s_t stored in self.s_t_observation_buffer in the next call.
        action_to_take_idx = self.select_dqn_action(s_t1_current_observation_vec)
        self.publish_car_commands(action_to_take_idx)

        # Store current observation and action for the *next* iteration's replay buffer push
        self.s_t_observation_buffer = s_t1_current_observation_vec # This is s_t for the next step
        self.a_t_action_idx_buffer = action_to_take_idx            # This is a_t for the next step
        
        self.current_rl_trajectory_step_count += 1
        self.total_agent_steps_taken += 1 # Increment global training step counter

        self.processing_rl_step_active = False


    def jarred_episode_over_callback(self, msg: Float32):
        # Callback for when Jarred's rewards_node signals the end of the official ROS episode.
        # DEPENDS_ON_REWARDS_PKG: Expects '/episode_rewards' topic.
        final_reward_from_jarred = msg.data
        self.get_logger().info(f"DQN Node: OFFICIAL ROS EPISODE END (Jarred's /episode_rewards). Jarred's Cumulative Reward: {final_reward_from_jarred:.2f}")
        
        # This is the primary signal that the ROS-level episode has ended.
        # If the RL trajectory was still ongoing (i.e., not internally "done"),
        # store the final transition with done=True because the ROS episode ended.
        if self.s_t_observation_buffer is not None and self.a_t_action_idx_buffer != -1 and \
           not (not self.is_in_track or self.current_rl_trajectory_step_count >= self.MAX_STEPS_PER_RL_TRAJECTORY):
            
            s_t1_final_obs_at_jarred_end = self.process_observation_vector() # Get the very final state if possible
            if s_t1_final_obs_at_jarred_end is None: # If sensor data is stale, use previous obs as next_state
                s_t1_final_obs_at_jarred_end = self.s_t_observation_buffer 
            
            final_dense_reward_for_buffer = self.calculate_dense_reward(
                self.s_t_observation_buffer, self.a_t_action_idx_buffer, 
                s_t1_final_obs_at_jarred_end, True # Definitely done from ROS episode perspective
            )
            self.memory.push(self.s_t_observation_buffer, self.a_t_action_idx_buffer, final_dense_reward_for_buffer, s_t1_final_obs_at_jarred_end, True)
            self.get_logger().debug(f"Pushed final experience due to Jarred's episode end. Dense Reward: {final_dense_reward_for_buffer:.2f}")
            # Potentially one last optimization call if desired
            # self.optimize_dqn_model()

        self.env_confirmed_ready_for_rl = False # Mark env as not ready until rl_gym confirms for next episode
        self.current_ros_episode_num += 1    # Increment official ROS episode counter
        
        # Reset buffers for the next ROS episode's RL trajectory
        self.s_t_observation_buffer = None 
        self.a_t_action_idx_buffer = -1    
        self.current_rl_trajectory_step_count = 0 # Reset for the new ROS episode

        # Save checkpoint periodically based on ROS episodes completed
        if self.current_ros_episode_num > 0 and self.current_ros_episode_num % self.checkpoint_save_frequency == 0:
            self._save_checkpoint()

        self.get_logger().info("Scheduling request for new ROS episode from rl_gym...")
        # Ensure car is stopped before requesting new episode
        self.publish_car_commands(self.discrete_actions_map.index(next(a for a in self.discrete_actions_map if a['description'] == "Brake Straight")))
        # Request rl_gym to prepare for a new ROS episode after a short delay
        # rclpy.create_timer(2.0, self.request_new_ros_episode_from_rl_gym, self.get_clock(), oneshot=True)
        self.shutdown_environment()
        self.request_new_ros_episode_from_rl_gym()


def main(args=None):
    rclpy.init(args=args)
    dqn_node = AudibotRLDriverDQNNode()
    try:
        rclpy.spin(dqn_node)
    except KeyboardInterrupt:
        dqn_node.get_logger().info("Keyboard interrupt received by DQN Node.")
        dqn_node.shutdown_environment()
    except Exception as e:
        dqn_node.get_logger().error(f"Unhandled exception in DQN Node: {e}")
    finally:
        dqn_node.get_logger().info("Shutting down DQN Node...")
        if hasattr(dqn_node, 'throttle_pub') and dqn_node.throttle_pub is not None: # Check if publishers are initialized
            try:
                # Send a brake command on shutdown
                dqn_node.throttle_pub.publish(Float64(data=0.0))
                dqn_node.steering_pub.publish(Float64(data=0.0))
                dqn_node.brake_pub.publish(Float64(data=dqn_node.max_brake_torque)) # Apply significant brake
                dqn_node.get_logger().info("Sent final brake command.")
            except Exception as e_shutdown:
                dqn_node.get_logger().error(f"Error sending brake command on shutdown: {e_shutdown}")
        
        # Save final model
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

