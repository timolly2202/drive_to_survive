# audibot_rl_driver/audibot_rl_driver/rl_agent_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry
from yolo_msg.msg import YoloDetections, BoundingBox # BoundingBox is implicitly used by YoloDetections

import numpy as np
import math

# --- RL specific imports (you'll add these as you build out PPO) ---
# For example, if using a library like Stable Baselines3 (SB3)
# import gymnasium as gym # Or import gym if using older versions
# from gymnasium import spaces
# from stable_baselines3 import PPO
# from stable_baselines3.common.env_checker import check_env

# Define constants for camera identifiers
CAM_FRONT = 0
CAM_LEFT = 1
CAM_RIGHT = 2
CAM_BACK = 3 # If you use it
CAM_UNKNOWN = 4


class RLAgentNode(Node):
    def __init__(self):
        super().__init__('audibot_rl_agent_node')
        self.get_logger().info('RL Agent Node with Observation Space V1 has been started.')

        # --- Parameters ---
        # YOUR_INPUT_REQUIRED: Define these parameters for your specific setup
        self.declare_parameter('num_closest_cones', 6)  # N: How many closest cones to consider in observation
        self.declare_parameter('max_car_speed', 10.0) # m/s, for normalizing speed observation
        self.declare_parameter('max_car_yaw_rate', 2.0) # rad/s, for normalizing yaw rate
        self.declare_parameter('max_goal_distance', 50.0) # meters, for normalizing goal distance

        # Image dimensions - crucial for normalization.
        # These might be the same for all cameras or different.
        # Get these from your Gazebo camera sensor configuration or Tim.
        self.declare_parameter('front_cam.image_width', 640) # YOUR_INPUT_REQUIRED
        self.declare_parameter('front_cam.image_height', 480) # YOUR_INPUT_REQUIRED
        self.declare_parameter('left_cam.image_width', 640) # YOUR_INPUT_REQUIRED
        self.declare_parameter('left_cam.image_height', 480) # YOUR_INPUT_REQUIRED
        self.declare_parameter('right_cam.image_width', 640) # YOUR_INPUT_REQUIRED
        self.declare_parameter('right_cam.image_height', 480) # YOUR_INPUT_REQUIRED
        # Add for back camera if used

        self.num_closest_cones = self.get_parameter('num_closest_cones').value
        self.max_car_speed = self.get_parameter('max_car_speed').value
        self.max_car_yaw_rate = self.get_parameter('max_car_yaw_rate').value
        self.max_goal_distance = self.get_parameter('max_goal_distance').value

        self.image_dimensions = {
            CAM_FRONT: (self.get_parameter('front_cam.image_width').value, self.get_parameter('front_cam.image_height').value),
            CAM_LEFT: (self.get_parameter('left_cam.image_width').value, self.get_parameter('left_cam.image_height').value),
            CAM_RIGHT: (self.get_parameter('right_cam.image_width').value, self.get_parameter('right_cam.image_height').value),
            # CAM_BACK: (self.get_parameter('back_cam.image_width').value, self.get_parameter('back_cam.image_height').value),
        }

        # Define the size of one cone's feature set in the observation space
        # norm_center_x, norm_center_y, norm_height, norm_width, present_flag, cam_front, cam_left, cam_right
        self.cone_feature_size = 4 + 1 + 3 # (x,y,h,w) + present_flag + (one-hot cam type)

        # Calculate total observation space size
        # car_vel_x, car_yaw_rate, goal_dist, goal_angle_cos, goal_angle_sin + N * cone_feature_size
        self.observation_size = 2 + 3 + (self.num_closest_cones * self.cone_feature_size)
        self.get_logger().info(f"Observation space size: {self.observation_size}")


        # --- Publishers (for controlling the car) ---
        self.throttle_pub = self.create_publisher(Float64, '/orange/throttle_cmd', 10)
        self.steering_pub = self.create_publisher(Float64, '/orange/steering_cmd', 10)
        self.brake_pub = self.create_publisher(Float64, '/orange/brake_cmd', 10)

        # --- Subscribers (for getting information from the environment) ---
        self.front_yolo_sub = self.create_subscription(YoloDetections, '/orange/front_image_yolo/detections', self.front_yolo_callback, 10)
        self.left_yolo_sub = self.create_subscription(YoloDetections, '/orange/left_image_yolo/detections', self.left_yolo_callback, 10)
        self.right_yolo_sub = self.create_subscription(YoloDetections, '/orange/right_image_yolo/detections', self.right_yolo_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.episode_status_sub = self.create_subscription(String, '/episode_status', self.episode_status_callback, 10)

        self.agent_timer_period = 0.1
        self.agent_timer = self.create_timer(self.agent_timer_period, self.agent_step)

        # --- Internal State Variables ---
        self.latest_odometry = None
        self.front_traffic_cones_raw = [] # Stores raw BoundingBox objects
        self.left_traffic_cones_raw = []
        self.right_traffic_cones_raw = []
        # self.back_traffic_cones_raw = []

        self.current_target_goal = None # Stores (x, y) of the current goal from Jarred's system
        # YOUR_INPUT_REQUIRED: How will you get the list of goals and current_target_goal?
        # Option A: Replicate Jarred's 'goals' parameter and logic for goal_index
        # self.declare_parameter('track_goals', ['0.0,0.0']) # Example
        # self.track_goals_str = self.get_parameter('track_goals').value
        # self.track_goals = [self.parse_goal_str(g) for g in self.track_goals_str]
        # self.current_goal_index = 0
        # if self.track_goals:
        #     self.current_target_goal = self.track_goals[self.current_goal_index]

        # Option B: Subscribe to a topic from Jarred's node that publishes the current target goal.
        # self.current_goal_sub = self.create_subscription(Point, '/current_target_goal', self.current_goal_callback, 10)


        self.episode_running = False
        self.get_logger().info("RL Agent Node fully initialized.")

    # def parse_goal_str(self, goal_str): # If using Option A for goals
    #     try:
    #         x_str, y_str = goal_str.split(',')
    #         return float(x_str), float(y_str)
    #     except ValueError:
    #         self.get_logger().error(f"Invalid goal format in parameter: {goal_str}")
    #         return 0.0, 0.0

    # def current_goal_callback(self, msg): # If using Option B for goals
    #    self.current_target_goal = (msg.x, msg.y)

    def front_yolo_callback(self, msg: YoloDetections):
        self.front_traffic_cones_raw = [box for box in msg.bounding_boxes if box.class_name == "traffic-cones"]

    def left_yolo_callback(self, msg: YoloDetections):
        self.left_traffic_cones_raw = [box for box in msg.bounding_boxes if box.class_name == "traffic-cones"]

    def right_yolo_callback(self, msg: YoloDetections):
        self.right_traffic_cones_raw = [box for box in msg.bounding_boxes if box.class_name == "traffic-cones"]

    def odom_callback(self, msg: Odometry):
        self.latest_odometry = msg

    def episode_status_callback(self, msg: String):
        status = msg.data.lower()
        if status == 'start':
            self.episode_running = True
            self.get_logger().info("Episode started.")
            # YOUR_INPUT_REQUIRED: Reset agent's internal episodic states if any
            # For example, if using Option A for goals:
            # self.current_goal_index = 0
            # if self.track_goals:
            #     self.current_target_goal = self.track_goals[self.current_goal_index]
        elif status == 'end' or status == 'reset':
            self.episode_running = False
            self.get_logger().info(f"Episode ended/reset: {status}.")
            self.publish_control_commands(steering=0.0, throttle=0.0, brake=1.0)


    def _normalize_bounding_box(self, box: BoundingBox, camera_id: int):
        img_width, img_height = self.image_dimensions.get(camera_id, (1.0, 1.0)) # Default to 1.0 to avoid division by zero if misconfigured
        if img_width == 0: img_width = 1.0
        if img_height == 0: img_height = 1.0

        center_x = box.left + (box.right - box.left) / 2.0
        center_y = box.top + (box.bottom - box.top) / 2.0
        width = float(box.right - box.left)
        height = float(box.bottom - box.top)

        # Normalize to [0, 1]
        norm_center_x = center_x / img_width
        norm_center_y = center_y / img_height
        norm_width = width / img_width
        norm_height = height / img_height
        
        # Clamping to ensure they are within [0,1] robustly, or [-0.5, 0.5] if you change normalization
        norm_center_x = np.clip(norm_center_x, 0.0, 1.0)
        norm_center_y = np.clip(norm_center_y, 0.0, 1.0)
        norm_width = np.clip(norm_width, 0.0, 1.0)
        norm_height = np.clip(norm_height, 0.0, 1.0)

        return norm_center_x, norm_center_y, norm_width, norm_height
    

    '''
You're at a crucial point: translating raw sensor data (YOLO bounding boxes) into a meaningful "observation" for your RL agent. The goal is to give the agent the information it needs to decide how to steer towards the center of the two closest cones.

Here are several ideas for implementing the observation space, ranging from simpler to more complex. You'll likely need to experiment to see what works best.

Core Idea: Relative Information is Key

The RL agent doesn't necessarily need to know the absolute pixel coordinates of cones. It needs to know where the cones are relative to the car and relative to each other to find that midpoint.

General Preprocessing for Bounding Boxes (BBs):

For each detected cone (box from self.front_traffic_cones, etc.):

Calculate Center:
    bb_center_x = box.left + (box.right - box.left) / 2
    bb_center_y = box.top + (box.bottom - box.top) / 2

Calculate Size:
    bb_width = box.right - box.left
    bb_height = box.bottom - box.top

Normalization (Highly Recommended): 

Normalize these values to a consistent range (e.g., 0 to 1, or -1 to 1). This helps the neural network learn more effectively.
You'll need the image dimensions (width, height) for each camera. Let's assume you have image_width and image_height.
    norm_center_x = bb_center_x / image_width (range 0 to 1)
    norm_center_y = bb_center_y / image_height (range 0 to 1)
    norm_width = bb_width / image_width (range 0 to 1)
    norm_height = bb_height / image_height (range 0 to 1)

Alternatively, for norm_center_x, you might want it in the range -0.5 to 0.5 (by doing (bb_center_x / image_width) - 0.5) to represent left/right of the image center.

Observation Space Ideas:

Option 2: Fixed Number of "Closest" Cones

This is often more robust than trying to explicitly find pairs.

How it Works:

    1) Gather All Cones: Collect all normalized cone features (center x, y, height, width) from all relevant cameras.

    2) Sort by "Closeness": Use a heuristic like sort_key = (1 - norm_height) (prioritizing taller cones). You might also factor in norm_center_y (lower in image = closer).

    3)Select Top N: Take the top N (e.g., N=4 or N=6) "closest" cones.

Observation Vector Features:

For each of the N selected cones:
    cone_i_norm_center_x
    cone_i_norm_center_y
    cone_i_norm_height
    cone_i_norm_width (optional, height might be a better distance proxy)
    A flag indicating which camera detected it (e.g., one-hot encode: [1,0,0] for front, [0,1,0] for left, [0,0,1] for right). This can be important if normalization doesn't fully account for camera perspective differences.

If fewer than N cones are detected, pad the remaining slots with special values (e.g., 0 or -1).

Pros: Simpler to implement, more robust to noisy detections than explicit pairing. The RL agent learns to interpret the spatial arrangement of these N cones.

Cons: The agent has to implicitly learn the "gate" concept. The order of cones in the observation vector matters (sorting is crucial).

'''

    def process_observations(self):
        if not self.latest_odometry:
            self.get_logger().warn_throttle(throttle_skip_first=True, throttle_time_source_type=rclpy.clock.ClockType.ROS_TIME, duration=5.0, msg="No odometry data yet.")
            return np.zeros(self.observation_size, dtype=np.float32) # Return a default observation


        # 1. Car's Kinematic State
        linear_velocity_x = self.latest_odometry.twist.twist.linear.x
        angular_velocity_z = self.latest_odometry.twist.twist.angular.z

        norm_linear_velocity_x = np.clip(linear_velocity_x / self.max_car_speed, -1.0, 1.0)
        norm_angular_velocity_z = np.clip(angular_velocity_z / self.max_car_yaw_rate, -1.0, 1.0)
        
        car_obs_part = [norm_linear_velocity_x, norm_angular_velocity_z]

        # 2. Target Waypoint Information
        goal_obs_part = [0.0, 0.0, 0.0] # dist, cos(angle), sin(angle) - default if no goal
        if self.current_target_goal and self.latest_odometry:
            car_pos_x = self.latest_odometry.pose.pose.position.x
            car_pos_y = self.latest_odometry.pose.pose.position.y
            
            # Orientation of the car (quaternion to yaw)
            orientation_q = self.latest_odometry.pose.pose.orientation
            # Simple conversion assuming yaw is around Z axis
            # For more robust conversion: from tf_transformations import euler_from_quaternion
            # qx, qy, qz, qw = orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
            # _, _, car_yaw = euler_from_quaternion([qx, qy, qz, qw]) # Needs tf_transformations
            # Temporary simpler yaw (less robust, assumes car is mostly flat)
            car_yaw = 2.0 * math.atan2(orientation_q.z, orientation_q.w)


            target_x, target_y = self.current_target_goal
            
            delta_x_world = target_x - car_pos_x
            delta_y_world = target_y - car_pos_y
            
            distance_to_goal = math.sqrt(delta_x_world**2 + delta_y_world**2)
            norm_distance_to_goal = np.clip(distance_to_goal / self.max_goal_distance, 0.0, 1.0)
            
            # Angle to goal in world frame
            angle_to_goal_world = math.atan2(delta_y_world, delta_x_world)
            # Angle to goal relative to car's heading
            angle_to_goal_relative = angle_to_goal_world - car_yaw
            # Normalize angle to be within -pi to pi
            while angle_to_goal_relative > math.pi: angle_to_goal_relative -= 2 * math.pi
            while angle_to_goal_relative < -math.pi: angle_to_goal_relative += 2 * math.pi
            
            # Using cos and sin is often better for NNs than raw angle
            goal_angle_cos = math.cos(angle_to_goal_relative)
            goal_angle_sin = math.sin(angle_to_goal_relative)
            
            goal_obs_part = [norm_distance_to_goal, goal_angle_cos, goal_angle_sin]
        else:
            self.get_logger().warn_throttle(throttle_skip_first=True, throttle_time_source_type=rclpy.clock.ClockType.ROS_TIME, duration=5.0, msg="No current target goal set for observation.")


        # 3. Cone Data
        all_processed_cones = []
        for cam_id, raw_cone_list in [
            (CAM_FRONT, self.front_traffic_cones_raw),
            (CAM_LEFT, self.left_traffic_cones_raw),
            (CAM_RIGHT, self.right_traffic_cones_raw),
            # (CAM_BACK, self.back_traffic_cones_raw) # If using back camera
        ]:
            for box in raw_cone_list:
                norm_cx, norm_cy, norm_w, norm_h = self._normalize_bounding_box(box, cam_id)
                # Closeness heuristic: taller cones are considered closer.
                # You might want to refine this, e.g. cones lower in FRONT camera view.
                closeness_score = norm_h # Larger height = closer
                all_processed_cones.append({
                    'cx': norm_cx, 'cy': norm_cy, 'w': norm_w, 'h': norm_h,
                    'cam_id': cam_id, 'score': closeness_score
                })
        
        # Sort cones by closeness_score (descending, so closest first)
        all_processed_cones.sort(key=lambda c: c['score'], reverse=True)
        
        cone_obs_part = []
        for i in range(self.num_closest_cones):
            if i < len(all_processed_cones):
                cone = all_processed_cones[i]
                present_flag = 1.0
                cam_one_hot = [0.0, 0.0, 0.0] # For Front, Left, Right
                if cone['cam_id'] == CAM_FRONT: cam_one_hot[0] = 1.0
                elif cone['cam_id'] == CAM_LEFT: cam_one_hot[1] = 1.0
                elif cone['cam_id'] == CAM_RIGHT: cam_one_hot[2] = 1.0
                #elif cone['cam_id'] == CAM_BACK: # Add if using back camera
                
                cone_features = [cone['cx'], cone['cy'], cone['h'], cone['w'], present_flag] + cam_one_hot
                cone_obs_part.extend(cone_features)
            else:
                # Pad with zeros if fewer than num_closest_cones are detected
                cone_obs_part.extend([0.0] * self.cone_feature_size)
                
        # Combine all parts into a single observation vector
        observation = np.array(car_obs_part + goal_obs_part + cone_obs_part, dtype=np.float32)
        
        if len(observation) != self.observation_size:
            self.get_logger().error(f"Observation length mismatch! Expected {self.observation_size}, Got {len(observation)}")
            # Return a correctly sized zero array to prevent PPO errors, but log the issue.
            return np.zeros(self.observation_size, dtype=np.float32)
            
        return observation


    def select_action(self, observation):
        # YOUR_INPUT_REQUIRED: This is where your PPO model's predict() method will go
        # action, _states = self.model.predict(observation, deterministic=True)
        # steering_action = action[0]
        # throttle_action = action[1]
        # brake_action = 0.0 # Or action[2] if brake is part of PPO output
        
        # Placeholder: simple forward action
        steering_action = 0.0
        throttle_action = 0.1 # Gentle throttle for testing
        brake_action = 0.0
        return steering_action, throttle_action, brake_action

    def publish_control_commands(self, steering, throttle, brake):
        # YOUR_INPUT_REQUIRED: Add clipping or scaling if PPO outputs actions in a different range
        # e.g., PPO might output steering in [-1, 1], which is fine.
        # PPO might output throttle in [-1, 1], needs scaling to [0, 1] or [0, MAX_THROTTLE]
        
        steer_msg = Float64()
        steer_msg.data = float(steering)
        self.steering_pub.publish(steer_msg)

        throttle_msg = Float64()
        # Example: scale throttle if PPO outputs [-1,1] and you want [0,1]
        # throttle_msg.data = float((throttle + 1.0) / 2.0) 
        throttle_msg.data = float(throttle) # Assuming PPO outputs throttle in desired range for now
        self.throttle_pub.publish(throttle_msg)

        brake_msg = Float64()
        brake_msg.data = float(brake)
        self.brake_pub.publish(brake_msg)

    def calculate_reward(self, observation, action, next_observation):
        # YOUR_INPUT_REQUIRED: Implement your reward function here.
        # This is a placeholder.
        reward = 0.0
        # Example: reward for forward speed
        # if self.latest_odometry:
        #    reward += self.latest_odometry.twist.twist.linear.x * 0.01 # Small reward
        return reward

    def agent_step(self):
        if not self.episode_running:
            return

        current_observation = self.process_observations()
        if current_observation is None or current_observation.shape[0] != self.observation_size : # check for valid observation
             self.get_logger().warn_throttle(throttle_skip_first=True, throttle_time_source_type=rclpy.clock.ClockType.ROS_TIME, duration=1.0, msg="Invalid observation received in agent_step, skipping.")
             return


        steering, throttle, brake = self.select_action(current_observation)
        self.publish_control_commands(steering, throttle, brake)
        
        # For a full RL loop, you would also:
        # 1. Get the `next_observation` after the action has taken effect.
        #    (In ROS, this happens asynchronously via callbacks; you might need to wait briefly or use the next `agent_step`'s `current_observation` as the `next_observation` for the previous step's experience tuple)
        # 2. Call `reward = self.calculate_reward(current_observation, (steering, throttle, brake), next_observation)`
        # 3. Determine if the episode is `done` (e.g., from `episode_status_callback` or other conditions).
        # 4. Store the experience `(current_observation, action, reward, next_observation, done)` for PPO training.
        # 5. Periodically trigger `model.learn()` if using SB3, or your custom PPO update logic.

        # self.get_logger().debug(f"Agent Step: Action Pub -> S:{steering:.2f} T:{throttle:.2f} B:{brake:.2f}")


def main(args=None):
    rclpy.init(args=args)
    rl_agent_node = RLAgentNode()
    try:
        rclpy.spin(rl_agent_node)
    except KeyboardInterrupt:
        rl_agent_node.get_logger().info('Keyboard interrupt, shutting down RL Agent Node.')
    finally:
        rl_agent_node.get_logger().info('Destroying RL Agent Node.')
        if hasattr(rl_agent_node, 'episode_running') and rl_agent_node.episode_running:
             rl_agent_node.publish_control_commands(0.0, 0.0, 1.0)
        rl_agent_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()