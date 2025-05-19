
# RL Rewards Node

## 🧠 Overview

The `rl_rewards` package provides real-time reward calculation for Reinforcement Learning (RL) training in a simulated driving environment. It listens for a start signal from the RL training package and tracks goal progression, time elapsed, and track position status. When an episode concludes due to completion, timeout, or the vehicle going out of bounds, the system publishes the final reward score for use in RL training.

---

## 📡 Subscribers & Publishers

### Subscribers:
- `/orange/front_image_yolo/detections` – `YoloDetections` (Front camera)
- `/orange/back_image_yolo/detections` – `YoloDetections` (Back camera)
- `/orange/left_image_yolo/detections` – `YoloDetections` (Left camera)
- `/orange/right_image_yolo/detections` – `YoloDetections` (Right camera)
- `/orange/odom` – `nav_msgs/msg/Odometry` (Robot position)
- `/episode_status` – `std_msgs/msg/String` (Start signal)

### Publishers:
- `/in_track_status` – `std_msgs/msg/Bool` (True/False whether robot is in track)
- `/episode_rewards` – `std_msgs/msg/Float32` (Final reward score)
- `/goal_markers` – `visualization_msgs/msg/MarkerArray` (Goal visualization in RViz)
- `/current_goal` – `geometry_msgs/msg/Point` (Current goal location)

---

## 🔁 System Diagram

```
RL Training Script (Publisher) ---> /episode_status
                                     |
                                     v
                          +----------------------+
                          |     rl_rewards       |
                          +----------------------+
        Camera Feeds ---> | /yolo/...detections  |
         Odometry ------> | /orange/odom         |
                          |                      |
                          | Goal Tracking &      |
                          | Reward Computation   |
                          |                      |
                          +----------------------+
                                     |
               +---------------------+----------------------+
               |                     |                      |
         /in_track_status     /episode_rewards       /current_goal
```

---

## 🧰 Installing Dependencies

Ensure you also have the `yolo_msg` custom message package built and sourced.

---

## 🔨 Compiling the Package

From the root of your ROS 2 workspace:

```bash
colcon build --packages-select rl_rewards
source install/setup.bash
```

---

## 🚀 Running the System

Launch each component in separate terminals:

### 1. World
```bash
ros2 launch gazebo_tf drive_to_survive.launch.py
```

### 2. YOLO Detector
```bash
ros2 launch audibot_yolo multi_camera.launch.py
```

### 3. RL Rewards Node
```bash
ros2 run rl_rewards rl_rewards
```

### 4. Start Episode (Simulated)
```bash
ros2 topic pub /episode_status std_msgs/msg/String "data: 'start'"
```

--- 

## 🧮 How Rewards Are Calculated

| Condition                       | Reward Impact             |
|--------------------------------|---------------------------|
| Reaching a goal                | `+5` points               |
| Time taken between goals       | `-1` per second           |
| Going out of the track         | `-100` points + episode end |
| Timeout (exceeding time limit)	Episode ends `+ -5` × time since last goal penalty applied |
| Completing all goals and returning to start	Episode ends, no penalty applied |
| Episode ends before next goal is reached (any reason)	`-5 × time since last goal penalty applied` |

### Reward Publication Triggers:
- When **all goals are reached** (looped back to first goal).
- When **robot is out of bounds** for an extended period.
- When **episode time limit is exceeded**.

---

## 📂 File Dependencies

- `goals.json` – must be present at:
  ```
  ~/drive_to_survive/src/rl_rewards/goals/goals.json
  ```

---

## ✅ Notes

- RViz visualization will display goal markers and regions.
- Camera detections must be live to determine in-track status.
- Ensure simulation clock is active if running with Gazebo.