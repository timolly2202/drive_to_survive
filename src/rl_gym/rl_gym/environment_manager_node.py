import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool 
from nav_msgs.msg import Odometry
import time
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy
import subprocess
import signal
import threading

class EnvironmentManager(Node):

    def __init__(self):
        super().__init__('environment_manager')

        self.use_rviz = self.declare_parameter('use_rviz', False).value

        self.get_logger().info(f'use rviz: {self.use_rviz}')

        self.qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.start_sub = self.create_subscription(
            Bool,
            '/dqn_agent/start_episode_request',
            self.start_callback,
            10
        )
        self.end_sub = self.create_subscription(
            Bool,
            '/dqn_agent/end_episode_request',
            self.end_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/orange/odom',
            self.odom_callback,
            10
        )

        self.status_pub = self.create_publisher(Bool, 'rl_gym/environment_started_confirm', 10)

        self.gazebo_process = None
        self.camera_process = None

        self.latest_odom = None

        self.get_logger().info('Node started')

    def odom_callback(self, msg):
        self.latest_odom = msg

    def start_callback(self, msg):
        if msg.data:
            self.get_logger().info('Received start episode request.')

            if self.gazebo_process is not None:
                self.shutdown_environment()
                time.sleep(3)

            self.start_environment()
        else:
            self.get_logger().info('Received false start request, ignoring.')

    def end_callback(self, msg):
        if msg.data:
            self.get_logger().info('Received end episode request.')

            self.shutdown_environment()
        else:
            self.get_logger().info('Received false start request, ignoring.')

    # def reward_callback(self, msg):
    #     self.get_logger().info('Episode complete. Shutting down simulation.')
    #     self.shutdown_environment()

    # def wait_for_publishers(self, topic_name: str, timeout_sec: float = 120.0) -> bool:
    #     """Wait until topic has at least one publisher."""
    #     start_time = self.get_clock().now()
    #     while rclpy.ok():
    #         # topic_info = self.get_topic_names_and_types(topic_name)
    #         publishers_amount = self.count_publishers(topic_name)
    #         # self.get_logger().info(f"publishers_info {publishers_amount}")
    #         if publishers_amount > 0:
    #             return True
    #         if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_sec:
    #             self.get_logger().warning(f"Timeout waiting for publishers on {topic_name}")
    #             return False
    #         time.sleep(0.1)
    
    # def wait_for_first_message(self, topic_name: str, msg_type, timeout_sec: float = 120.0) -> bool:
    #     start_time = self.get_clock().now()
    #     self.message_received = False
    #     while rclpy.ok() and not self.message_received:
    #         if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_sec:
    #             self.get_logger().warning(f"Timeout waiting for first message on {topic_name}")
    #             return False
    #         rclpy.spin_once(self, timeout_sec=0.1)  # spin the node to process callbacks
    #         # self.get_logger().info(f"ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', 'Not Set')}")
    #         self.get_logger().info(f"waiting on topic first message")
    #         self.get_logger().info(f"publishers_info {self.count_publishers(topic_name)}")

    #     return True
    
    # def wait_for_environment_ready(self):
    #     topics_and_types = [
    #         ('/orange/odom', Odometry),
    #     ]

    #     for topic, msg_type in topics_and_types:
    #         self.get_logger().info(f"Waiting for publisher and first message on {topic}...")
            
    #         if not self.wait_for_publishers(topic, timeout_sec=50):
    #             return False
    #         if not self.wait_for_first_message(topic, msg_type):
    #             return False
    #     return True

    # def wait_for_initial_pose(self, target_x=24.2, target_y=13.2, tolerance=0.1, timeout=60.0):
    #     self.get_logger().info(f"Waiting for Audibot to reach starting position x={target_x}, y={target_y}...")

    #     start_time = self.get_clock().now()
    #     while rclpy.ok():
    #         rclpy.spin_once(self, timeout_sec=0.1)

    #         if self.latest_odom is not None:
    #             position = self.latest_odom.pose.pose.position
    #             if (
    #                 abs(position.x - target_x) < tolerance and
    #                 abs(position.y - target_y) < tolerance
    #             ):
    #                 self.get_logger().info("Audibot is at the starting pose.")
    #                 return True

    #         elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
    #         if elapsed > timeout:
    #             self.get_logger().warning("Timed out waiting for initial pose.")
    #             return False
    
    def start_environment(self):
        try:
            # Launch Gazebo
            self.gazebo_process = subprocess.Popen(
                ['ros2', 'launch', 'gazebo_tf', 'drive_to_survive.launch.py', 'gui:=false', f'use_rviz:={str(self.use_rviz)}'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
                )

            # Launch Camera nodes
            self.camera_process = subprocess.Popen(
                ['ros2', 'launch', 'audibot_yolo', 'multi_camera.launch.py'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
                )
            
            # return True
            if self.use_rviz:
                time_wait = 12
            else:
                time_wait = 10
            self.get_logger().info(f'Waiting for Gazebo and camera nodes to initialize ({time_wait} seconds)')
            
            time.sleep(time_wait)
            
            self.confirm_env_status(True)


        except Exception as e:
            self.get_logger().error(f'Failed to start environment: {e}')
            self.confirm_env_status(False)
            self.shutdown_environment()

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
                    p.wait(timeout=50)
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

    def confirm_env_status(self, bool=True):
        status_msg = Bool()
        status_msg.data = bool
        self.status_pub.publish(status_msg)
        self.get_logger().info('Published env status')

def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentManager()

    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()
    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received on Environment Manager.")
        node.shutdown_environment()
    finally:
        node.get_logger().info("Shutting down Environment Manager...")
        node.shutdown_environment()

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()