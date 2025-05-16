# audibot_rl_driver/audibot_rl_driver/rl_agent_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String # Added String for episode_status
from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TwistStamped # Could be an alternative for odometry
from yolo_msg.msg import YoloDetections # BoundingBox is part of YoloDetections

# import random # For placeholder random actions initially

# --- RL specific imports (you'll add these as you build out PPO) ---
# For example, if using a library like Stable Baselines3 (SB3)
# import gymnasium as gym # Or import gym if using older versions
# from gymnasium import spaces
# import numpy as np
# from stable_baselines3 import PPO
# from stable_baselines3.common.env_checker import check_env


# Placeholder for your custom RL environment if you build one for SB3
# class AudiRLEnv(gym.Env):
#     # ... (implementation of your custom environment) ...
#     pass

class RLAgentNode(Node):
    def __init__(self):
        super().__init__('audibot_rl_agent_node')
        self.get_logger().info('RL Agent Node has been started. V2')

        # --- Parameters ---
        # Example: self.declare_parameter('some_rl_parameter', 0.01)
        #          self.some_rl_param = self.get_parameter('some_rl_parameter').value

        # --- Publishers (for controlling the car) ---
        self.throttle_pub = self.create_publisher(Float64, '/orange/throttle_cmd', 10)
        self.steering_pub = self.create_publisher(Float64, '/orange/steering_cmd', 10)
        self.brake_pub = self.create_publisher(Float64, '/orange/brake_cmd', 10)
        self.get_logger().info('Vehicle control publishers initialized.')

        # --- Subscribers (for getting information from the environment) ---
        # YOLO Detections from different cameras
        self.front_yolo_sub = self.create_subscription(
            YoloDetections,
            '/orange/front_image_yolo/detections', # Confirmed topic
            self.front_yolo_callback,
            10)
        self.left_yolo_sub = self.create_subscription(
            YoloDetections,
            '/orange/left_image_yolo/detections',  # Confirmed topic
            self.left_yolo_callback,
            10)
        self.right_yolo_sub = self.create_subscription(
            YoloDetections,
            '/orange/right_image_yolo/detections', # Confirmed topic
            self.right_yolo_callback,
            10)
        # Optional: Subscribe to back camera if needed for your strategy
        # self.back_yolo_sub = self.create_subscription(
        #     YoloDetections,
        #     '/orange/back_image_yolo/detections',
        #     self.back_yolo_callback,
        #     10)
        
        # Car's Odometry (for position, orientation, speed)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom', # As per README(audibot_gazebo).md
            self.odom_callback,
            10)
        
        # Episode status from Jarred's node (e.g., for resets)
        self.episode_status_sub = self.create_subscription(
            String, # Assuming Jarred uses std_msgs.msg.String
            '/episode_status', # As per rl_rewards_node.py.txt
            self.episode_status_callback,
            10)

        # (Potentially) Jarred's direct reward or in_track status
        # self.reward_sub = self.create_subscription(Float32, '/episode_rewards', self.reward_callback, 10)
        # self.in_track_sub = self.create_subscription(Bool, '/in_track_status', self.in_track_callback, 10)

        self.get_logger().info('Core subscribers initialized.')

        # --- RL Algorithm Setup (PPO) ---
        # This is where you'll integrate your PPO implementation or library.
        # For now, it's a placeholder.
        # Example:
        # self.env = AudiRLEnv(self) # If your Env needs direct access to this node
        # check_env(self.env) # Useful if using SB3 and gymnasium
        # self.model = PPO("MlpPolicy", self.env, verbose=1, tensorboard_log="./ppo_audibot_tensorboard/")
        
        # A timer to periodically trigger the agent's decision-making process (training step or action step)
        # The rate might depend on how fast you want the agent to react or your PPO implementation details.
        self.agent_timer_period = 0.1  # seconds (e.g., 10 Hz)
        self.agent_timer = self.create_timer(self.agent_timer_period, self.agent_step)

        # --- Internal State Variables (to store the latest sensor data and agent state) ---
        self.current_steering_action = 0.0
        self.current_throttle_action = 0.0
        self.current_brake_action = 0.0
        
        self.latest_odometry = None
        self.front_traffic_cones = [] # List to store BoundingBox objects for traffic cones
        self.left_traffic_cones = []
        self.right_traffic_cones = []
        # self.back_traffic_cones = [] # If using back camera

        self.episode_running = False # To track if an episode is active

        self.get_logger().info("RL Agent Node fully initialized.")

    def front_yolo_callback(self, msg: YoloDetections):
        # Filter for "traffic-cones" and store their BoundingBox data
        # Since score is not yet reliable, we don't filter by it for now.
        self.front_traffic_cones = [box for box in msg.bounding_boxes if box.class_name == "traffic-cones"]
        # self.get_logger().debug(f'Front cam: {len(self.front_traffic_cones)} traffic cones detected.')

    def left_yolo_callback(self, msg: YoloDetections):
        self.left_traffic_cones = [box for box in msg.bounding_boxes if box.class_name == "traffic-cones"]
        # self.get_logger().debug(f'Left cam: {len(self.left_traffic_cones)} traffic cones detected.')

    def right_yolo_callback(self, msg: YoloDetections):
        self.right_traffic_cones = [box for box in msg.bounding_boxes if box.class_name == "traffic-cones"]
        # self.get_logger().debug(f'Right cam: {len(self.right_traffic_cones)} traffic cones detected.')

    # def back_yolo_callback(self, msg: YoloDetections): # If you add the back camera subscriber
    #     self.back_traffic_cones = [box for box in msg.bounding_boxes if box.class_name == "traffic-cones"]
    # self.get_logger().debug(f'Back cam: {len(self.back_traffic_cones)} traffic cones detected.')

    def odom_callback(self, msg: Odometry):
        self.latest_odometry = msg
        # Example:
        # position_x = msg.pose.pose.position.x
        # linear_velocity_x = msg.twist.twist.linear.x
        # self.get_logger().debug(f'Odom: X={position_x:.2f}, VelX={linear_velocity_x:.2f}')

    def episode_status_callback(self, msg: String):
        status = msg.data.lower()
        if status == 'start':
            self.episode_running = True
            self.get_logger().info("Episode started.")
            # Reset any episode-specific states for the agent here
        elif status == 'end' or status == 'reset': # Assuming 'reset' also implies end of an attempt
            self.episode_running = False
            self.get_logger().info(f"Episode ended/reset: {status}.")
            # Potentially stop the car or perform other end-of-episode actions
            self.publish_control_commands(steering=0.0, throttle=0.0, brake=1.0) # Example: full brake on episode end

    def process_observations(self):
        """
        Combines all raw sensor data into a structured observation state for the PPO agent.
        This is a CRITICAL and COMPLEX part.
        Output should be a consistent format, e.g., a NumPy array for SB3.
        """
        if not self.latest_odometry:
            self.get_logger().warn("No odometry data yet to process observations.")
            return None # Or return a zeroed/default observation array

        # --- Example elements for your observation state (you'll need to expand and refine) ---
        
        # 1. Car's own state (from odometry)
        # linear_velocity_x = self.latest_odometry.twist.twist.linear.x
        # angular_velocity_z = self.latest_odometry.twist.twist.angular.z
        # Note: For PPO, it's often good to normalize these values.
        
        # 2. Cone data (this is the challenging part: converting pixel BBs to meaningful features)
        # For each camera (front, left, right):
        #   - How many cones?
        #   - For each cone (or a fixed number of closest/most relevant cones):
        #     - Center x, y in pixel space (normalized to image width/height?)
        #     - Width, height of bounding box (normalized?)
        #     - These are proxies for distance and angle. True 3D position estimation is harder.
        
        # Simplistic example: concatenate features from cones.
        # This needs significant thought to create a good state representation.
        # For now, let's just log what we have.
        # self.get_logger().info(
        #     f"Processing Obs: ODEM VelX={linear_velocity_x:.2f}, "
        #     f"FrontCones={len(self.front_traffic_cones)}, "
        #     f"LeftCones={len(self.left_traffic_cones)}, "
        #     f"RightCones={len(self.right_traffic_cones)}"
        # )

        # Placeholder: this would be your actual observation vector/matrix
        # observation = np.array([linear_velocity_x, angular_velocity_z, ... cone features ...])
        # return observation
        return "placeholder_observation" # Replace with actual processed data

    def select_action(self, observation):
        """
        Given an observation, the PPO agent's policy network outputs an action.
        """
        # If using a trained SB3 model:
        # action, _states = self.model.predict(observation, deterministic=True) # Get deterministic action for deployment/testing
        # steering_action = action[0]
        # throttle_action = action[1]
        # brake_action = action[2] # if your action space includes brake
        
        # --- Placeholder for random/dummy actions ---
        # For initial testing before PPO is integrated:
        # steering_action = (random.random() * 2.0) - 1.0  # Range: -1.0 to 1.0
        # throttle_action = random.random() * 0.3          # Range: 0.0 to 0.3 (start slow)
        # brake_action = 0.0                               # Initially no braking
        
        # For now, just a simple forward command for testing connections
        steering_action = 0.0
        throttle_action = 0.1 # Gentle throttle
        brake_action = 0.0

        self.current_throttle_action = throttle_action
        self.current_steering_action = steering_action
        self.current_brake_action = brake_action
        
        return steering_action, throttle_action, brake_action

    def publish_control_commands(self, steering, throttle, brake):
        steer_msg = Float64()
        steer_msg.data = float(steering)
        self.steering_pub.publish(steer_msg)

        throttle_msg = Float64()
        throttle_msg.data = float(throttle)
        self.throttle_pub.publish(throttle_msg)

        brake_msg = Float64()
        brake_msg.data = float(brake)
        self.brake_pub.publish(brake_msg)

    def calculate_reward(self, observation, action, next_observation):
        """
        Calculates the reward based on the agent's action and the resulting state.
        This is another CRITICAL and COMPLEX part.
        """
        reward = 0.0
        
        # --- Example Reward Components (you'll need to define these based on your goals) ---
        # - Reward for speed (e.g., current_forward_velocity from odometry)
        # - Reward for making progress towards the next waypoint (between cones)
        # - Large negative reward if off-track (you'll need a signal for this from Jarred or implement detection)
        # - Reward for passing through a pair of cones successfully
        # - Penalty for time (to encourage speed)
        # - Penalty for collision (if detectable)
        # - Penalty for jerky actions (e.g., large changes in steering/throttle)
        
        # Placeholder
        # if self.latest_odometry:
        #     reward += self.latest_odometry.twist.twist.linear.x * 0.1 # Small reward for forward speed
            
        # self.get_logger().debug(f"Calculated reward: {reward}")
        return reward

    def agent_step(self):
        """
        This function is called periodically by the timer.
        It represents one step of the agent's interaction with the environment.
        """
        if not self.episode_running:
            # self.get_logger().info("Agent step: Episode not running. Idling.")
            # Optionally ensure car is stopped if episode isn't running
            # self.publish_control_commands(0.0, 0.0, 1.0) # Brake
            return

        # 1. Get current observation from processed sensor data
        current_observation = self.process_observations()
        if current_observation is None: # e.g. if odometry isn't available yet
            return

        # 2. Select an action based on the current observation using the PPO policy
        steering, throttle, brake = self.select_action(current_observation)
        # For now, action is just a placeholder.

        # 3. Apply the action: publish control commands to the car
        self.publish_control_commands(steering, throttle, brake)
        
        # --- This part below is more relevant for a full RL training loop (e.g., with SB3) ---
        # 4. Wait for the environment to react (implicitly handled by ROS callbacks updating state)
        #    and then get the next_observation.
        #    For a typical RL loop, you'd often get next_observation, reward, done AFTER applying the action.
        #    In this ROS setup, callbacks update the state asynchronously.
        #    The `self.agent_timer_period` defines how often we take an action based on the latest state.

        # 5. Calculate reward (based on new state, action taken, etc.)
        #    For a full PPO implementation, you'd calculate reward here based on the transition.
        #    next_observation = self.process_observations() # Re-process after action has had time to take effect
        #    reward = self.calculate_reward(current_observation, (steering, throttle, brake), next_observation)

        # 6. Store experience (current_observation, action, reward, next_observation, done_flag)
        #    This is for training the PPO model. `done_flag` would come from `episode_status_callback`
        #    or other conditions (e.g., task completion).
        #    If using SB3, this is often handled within the `env.step()` method.

        # 7. If enough experiences are collected, perform a PPO model update.
        #    If using SB3, `model.learn(total_timesteps=X)` handles this.

        self.get_logger().debug(f"Agent Step: Action Pub -> S:{steering:.2f} T:{throttle:.2f} B:{brake:.2f}")


def main(args=None):
    rclpy.init(args=args)
    rl_agent_node = RLAgentNode()
    try:
        rclpy.spin(rl_agent_node)
    except KeyboardInterrupt:
        rl_agent_node.get_logger().info('Keyboard interrupt, shutting down RL Agent Node.')
    finally:
        # Perform any necessary cleanup here
        rl_agent_node.get_logger().info('Destroying RL Agent Node.')
        if rl_agent_node.episode_running: # Ensure car is stopped if node is killed mid-episode
             rl_agent_node.publish_control_commands(0.0, 0.0, 1.0) # Send a brake command
        rl_agent_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()