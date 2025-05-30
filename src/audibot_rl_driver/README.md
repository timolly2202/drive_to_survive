To train the environment:
```bash
# default launch config is RVIZ only, no gazebo:
ros2 run rl_gym environment_manager use_rviz:=false

ros2 run audibot_rl_driver rl_agent_node
```