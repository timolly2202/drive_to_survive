# audibot_yolo

`rl_gym` is a ROS 2 Python package which runs specified nodes in preparation for Reinforcement Learning from the RL agent.

## What it does

Subscribes to:

- `dqn_agent/start_episode_request` (`std_msgs/msg/Bool`) to know when to start up the environment
- `/episode_rewards` (`std_msgs/msg/Float32`) to know when the RL has finished

Publishes to:

- `rl_gym/environment_started_confirm` (`std_msgs/msg/Bool`) to let the RL agent know when the environment is running.

When the episode request is received, the node launches gazebo and the camera nodes, and then publishes to let the RL agent know everything is started. Once the rewards are received, it then kills the nodes and waits for the next episode request.

## Build Instructions

Navigate to your ROS 2 workspace and clone the full repository:

```bash
colcon build --packages-select rl_gym
source install/setup.bash
```

## Running the Node

Launch with:

```bash
ros2 run rl_gym environment_manager
```
