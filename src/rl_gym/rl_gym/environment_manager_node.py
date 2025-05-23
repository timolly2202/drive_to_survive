import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32
import subprocess
import signal
import os

class EnvironmentManager(Node):

    def __init__(self):
        super().__init__('environment_manager')

        self.start_sub = self.create_subscription(
            Bool,
            'dqn_agent/start_episode_request',
            self.start_callback,
            10
        )

        self.reward_sub = self.create_subscription(
            Float32,
            '/episode_rewards',
            self.reward_callback,
            10
        )

        self.status_pub = self.create_publisher(Bool, 'rl_gym/environment_started_confirm', 10)

        self.gazebo_process = None
        self.camera_process = None
        self.get_logger().info('Node started')

    def start_callback(self, msg):
        if msg.data:
            self.get_logger().info('Received start episode request.')
            self.start_environment()
        else:
            self.get_logger().info('Received false start request, ignoring.')

    def reward_callback(self, msg):
        self.get_logger().info('Episode complete. Shutting down simulation.')
        self.shutdown_environment()

    def start_environment(self):
        try:
            # Launch Gazebo
            self.gazebo_process = subprocess.Popen([
                'ros2', 'launch', 'gazebo_tf', 'drive_to_survive.launch.py',
                'gui:=false', 'use_rviz:=false'
            ])

            # Launch Camera nodes
            self.camera_process = subprocess.Popen([
                'ros2', 'launch', 'audibot_yolo', 'multi_camera.launch.py'
            ])

            self.get_logger().info('Waiting for Gazebo and camera nodes to initialize...')
            self.status_pub.publish(Bool(data=True))

        except Exception as e:
            self.get_logger().error(f'Failed to start environment: {e}')
            self.status_pub.publish(Bool(data=False))

    def shutdown_environment(self):
        def terminate_process(p):
            if p:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        
        terminate_process(self.gazebo_process)
        terminate_process(self.camera_process)

        self.gazebo_process = None
        self.camera_process = None

        self.get_logger().info('Simulation processes terminated.')

def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()