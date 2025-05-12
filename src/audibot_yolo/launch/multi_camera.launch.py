from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='audibot_yolo',
            executable='front_camera_node',
            name='front_camera_node',
            output='screen'
        ),
        Node(
            package='audibot_yolo',
            executable='back_camera_node',
            name='back_camera_node',
            output='screen'
        ),
        Node(
            package='audibot_yolo',
            executable='left_camera_node',
            name='left_camera_node',
            output='screen'
        ),
        Node(
            package='audibot_yolo',
            executable='right_camera_node',
            name='right_camera_node',
            output='screen'
        ),
    ])