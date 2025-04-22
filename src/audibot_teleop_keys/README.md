ros2 topic pub /cluster_label std_msgs/msg/Int32 "data: 1" -1

ros2 topic pub /cluster_label std_msgs/msg/Int32 "data: 0" -1

ros2 run audibot_teleop_keys teleop_keys
