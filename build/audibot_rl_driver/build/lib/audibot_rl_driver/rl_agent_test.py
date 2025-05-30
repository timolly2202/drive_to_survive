#!/usr/bin/env python3
"""
RL Agent *test‑time* node for Audibot.

Differences from the training node (``rl_agent_train.py``)
-----------------------------------------------------------
* **No learning** – replay‑buffer, optimiser and checkpoint saving code removed.
* **Deterministic policy** – epsilon‑greedy is fixed to 0 (or a tiny value if
  you pass the ROS param ``eval_eps``).
* **Loads a frozen `.pth` model** that was produced during training.
* **Simpler stats logging** – prints episode reward & distance‑to‑goal, but
  never alters network weights.

Usage
-----
.. code-block:: bash

   ros2 run audibot_rl audibot_rl_test \
       --ros-args -p checkpoint_path:=/path/to/dqn_final_policy.pth

If you have the same directory layout suggested in the README this works out of
 the box:
 ``~/git/drive_to_survive/src/audibot_rl/dqn_models_audibot/dqn_final_policy.pth``

``eval_eps`` (default 0.02) lets you keep a *tiny* amount of exploration to
avoid getting stuck in local optima during evaluation runs.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Float64
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from yolo_msg.msg import YoloDetections, BoundingBox

import torch
import numpy as np
import math
import json
import os
import random

from transformations import euler_from_quaternion

# -----------------------------------------------------------------------------
CAM_FRONT, CAM_LEFT, CAM_RIGHT, CAM_BACK = range(4)

# The network definition is identical to training -----------------------------
class DQNNetwork(torch.nn.Module):
    def __init__(self, input_dim: int, output_dim: int):
        super().__init__()
        self.fc1 = torch.nn.Linear(input_dim, 256)
        self.fc2 = torch.nn.Linear(256, 128)
        self.fc3 = torch.nn.Linear(128, output_dim)

    def forward(self, x):
        if not isinstance(x, torch.Tensor):
            x = torch.tensor(x, dtype=torch.float32)
        elif x.dtype != torch.float32:
            x = x.float()
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return self.fc3(x)

# -----------------------------------------------------------------------------
class AudibotRLDriverDQNTestNode(Node):
    """Inference‑only DQN driver."""

    # ----------------------------- INITIALISATION ----------------------------
    def __init__(self):
        super().__init__('audibot_rl_driver_dqn_test_node')
        self.get_logger().info('Audibot RL **TEST** node initialising…')

        # --------- parameters we care about in evaluation mode ----------------
        self.checkpoint_path = self.declare_parameter(
            'checkpoint_path',
            os.path.expanduser('~/git/drive_to_survive/src/audibot_rl/dqn_models_audibot/dqn_final_policy.pth')
        ).value
        self.eval_eps = float(self.declare_parameter('eval_eps', 0.0).value)  # exploratory epsilon at test‑time
        self.num_closest_cones = self.declare_parameter('num_closest_cones', 6).value

        # Very small set of hard‑coded normalisation constants (same as train)
        self.max_car_speed = 10.0
        self.max_car_yaw_rate = math.radians(90.0)
        self.max_goal_distance = 50.0

        # ---------------- discretised action table (copied) -------------------
        self.max_steering_angle_rad = 0.7
        self.max_brake_torque = 1000.0
        self._define_discrete_actions()

        # ---------------------------------------------------------------------
        self._define_image_dimensions_and_obs_size()

        # ---------------- create network & load weights -----------------------
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.policy_net = DQNNetwork(self.OBSERVATION_SIZE, self.num_discrete_actions).to(self.device)

        try:
            state_dict = torch.load(self.checkpoint_path, map_location=self.device)
            # checkpoint might be full dict – accept both cases
            if 'policy_net_state_dict' in state_dict:
                state_dict = state_dict['policy_net_state_dict']
            self.policy_net.load_state_dict(state_dict)
            self.get_logger().info(f'Loaded policy weights from {self.checkpoint_path}')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Failed to load checkpoint: {e}')
            raise SystemExit(1)

        self.policy_net.eval()

        # ------------------------- ROS pub / sub -----------------------------
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.throttle_pub = self.create_publisher(Float64, '/orange/throttle_cmd', 10)
        self.steering_pub = self.create_publisher(Float64, '/orange/steering_cmd', 10)
        self.brake_pub = self.create_publisher(Float64, '/orange/brake_cmd', 10)

        self.create_subscription(YoloDetections, '/orange/front_image_yolo/detections',
                                 lambda m: setattr(self, 'front_cones', m.bounding_boxes), qos)
        self.create_subscription(YoloDetections, '/orange/back_image_yolo/detections',
                                 lambda m: setattr(self, 'back_cones', m.bounding_boxes), qos)
        self.create_subscription(YoloDetections, '/orange/left_image_yolo/detections',
                                 lambda m: setattr(self, 'left_cones', m.bounding_boxes), qos)
        self.create_subscription(YoloDetections, '/orange/right_image_yolo/detections',
                                 lambda m: setattr(self, 'right_cones', m.bounding_boxes), qos)
        self.create_subscription(Odometry, '/orange/odom', self._odom_cb, qos)

        # goal list (static JSON) ---------------------------------------------
        goal_file = os.path.expanduser('~/git/drive_to_survive/src/rl_rewards/goals/goals.json')
        with open(goal_file, 'r') as f:
            self.goals_data = json.load(f)['goals']
        self.goal_index = 0

        # initialise buffers ---------------------------------------------------
        self.front_cones = []
        self.back_cones = []
        self.left_cones = []
        self.right_cones = []
        self.latest_odom = None

        # run main loop @10 Hz -------------------------------------------------
        self.timer = self.create_timer(0.1, self._step)

    # ------------------------- helper definitions ----------------------------
    def _define_image_dimensions_and_obs_size(self):
        w, h = 640, 480
        self.image_dimensions = {CAM_FRONT: (w, h), CAM_LEFT: (w, h), CAM_RIGHT: (w, h), CAM_BACK: (w, h)}
        self.cone_feature_size = 9  # cx, cy, h, w, present + 4×cam one‑hot
        self.OBSERVATION_SIZE = 2 + 3 + self.num_closest_cones * self.cone_feature_size

    def _define_discrete_actions(self):
        THR = {'none': 0.0, 'low': 0.35, 'high': 0.7}
        STEER = {
            'hard_left': -self.max_steering_angle_rad,
            'soft_left': -self.max_steering_angle_rad * 0.5,
            'straight': 0.0,
            'soft_right': self.max_steering_angle_rad * 0.5,
            'hard_right': self.max_steering_angle_rad,
        }
        BRK = {'none': 0.0, 'moderate': self.max_brake_torque * 0.4}
        self.discrete_actions_map = [
            {'throttle': THR['low'], 'steering': STEER['straight'], 'brake': BRK['none'], 'description': 'DriveStraight'},
            {'throttle': THR['low'], 'steering': STEER['soft_left'], 'brake': BRK['none'], 'description': 'SoftLeft'},
            {'throttle': THR['low'], 'steering': STEER['soft_right'], 'brake': BRK['none'], 'description': 'SoftRight'},
            {'throttle': THR['low'], 'steering': STEER['hard_left'], 'brake': BRK['none'], 'description': 'HardLeft'},
            {'throttle': THR['low'], 'steering': STEER['hard_right'], 'brake': BRK['none'], 'description': 'HardRight'},
            {'throttle': THR['none'], 'steering': STEER['straight'], 'brake': BRK['moderate'], 'description': 'Brake'},
        ]
        self.num_discrete_actions = len(self.discrete_actions_map)

    # ------------------------- callbacks & obs ------------------------------
    def _odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def _bbox_norm(self, box: BoundingBox, cam_id: int):
        w, h = self.image_dimensions[cam_id]
        cx = (box.left + box.right) / 2 / w
        cy = (box.top + box.bottom) / 2 / h
        bw = (box.right - box.left) / w
        bh = (box.bottom - box.top) / h
        return cx, cy, bw, bh

    def _observation(self):
        if self.latest_odom is None:
            return None
        odom = self.latest_odom
        # car state -----------------------------------------------------------
        norm_speed = np.clip(odom.twist.twist.linear.x / self.max_car_speed, -1, 1)
        norm_yaw_rate = np.clip(odom.twist.twist.angular.z / self.max_car_yaw_rate, -1, 1)
        obs = [norm_speed, norm_yaw_rate]

        # goal ---------------------------------------------------------------
        goal = self.goals_data[self.goal_index]['centre']
        car_x, car_y = odom.pose.pose.position.x, odom.pose.pose.position.y
        dx, dy = goal['x'] - car_x, goal['y'] - car_y
        dist = math.hypot(dx, dy)
        norm_dist = np.clip(dist / self.max_goal_distance, 0, 1)
        _, _, yaw = euler_from_quaternion([odom.pose.pose.orientation.x,
                                           odom.pose.pose.orientation.y,
                                           odom.pose.pose.orientation.z,
                                           odom.pose.pose.orientation.w])
        rel = math.atan2(dy, dx) - yaw
        # wrap to [-pi, pi]
        while rel > math.pi:
            rel -= 2 * math.pi
        while rel < -math.pi:
            rel += 2 * math.pi
        obs += [norm_dist, math.cos(rel), math.sin(rel)]

        # cones --------------------------------------------------------------
        all_boxes = []
        for cam_id, bb_list in ((CAM_FRONT, self.front_cones),
                                (CAM_LEFT, self.left_cones),
                                (CAM_RIGHT, self.right_cones),
                                (CAM_BACK, self.back_cones)):
            for b in bb_list:
                if b.class_name != 'traffic-cones':
                    continue
                cx, cy, bw, bh = self._bbox_norm(b, cam_id)
                all_boxes.append({'cx': cx, 'cy': cy, 'bw': bw, 'bh': bh,
                                   'cam': cam_id, 'closeness': bh})
        all_boxes.sort(key=lambda d: d['closeness'], reverse=True)
        for i in range(self.num_closest_cones):
            if i < len(all_boxes):
                c = all_boxes[i]
                present = 1.0
                cam_oh = [0.0] * 4
                cam_oh[c['cam']] = 1.0
                obs += [c['cx'], c['cy'], c['bh'], c['bw'], present] + cam_oh
            else:
                obs += [0.0] * self.cone_feature_size
        return np.array(obs, dtype=np.float32)

    # ------------------------------ main loop -------------------------------
    def _step(self):
        obs = self._observation()
        if obs is None:
            return
        # epsilon‑greedy (mostly greedy at test time) -------------------------
        if random.random() < self.eval_eps:
            act_idx = random.randrange(self.num_discrete_actions)
        else:
            with torch.no_grad():
                q = self.policy_net(torch.tensor(obs, device=self.device).unsqueeze(0))
            act_idx = int(q.argmax().item())
        self._publish_action(act_idx)

    # ------------------------------ actuation --------------------------------
    def _publish_action(self, idx: int):
        a = self.discrete_actions_map[idx]
        self.throttle_pub.publish(Float64(data=float(a['throttle'])))
        self.steering_pub.publish(Float64(data=float(a['steering'])))
        self.brake_pub.publish(Float64(data=float(a['brake'])))


# -----------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(AudibotRLDriverDQNTestNode())
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
