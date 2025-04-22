import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty
import threading

class AudibotTeleop(Node):
    def __init__(self):
        super().__init__('audibot_teleop_keys')
        self.throttle_pub = self.create_publisher(Float64, '/orange/throttle_cmd', 10)
        self.steering_pub = self.create_publisher(Float64, '/orange/steering_cmd', 10)
        self.brake_pub = self.create_publisher(Float64, '/orange/brake_cmd', 10)

        self.throttle = 0.0
        self.steering = 0.0
        self.brake = 0.0

        self.print_instructions()
        self.timer = self.create_timer(0.1, self.send_commands)

        # 🔁 Run keyboard loop in background
        self.thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.thread.start()

    def print_instructions(self):
        print("""
--- Audibot Keyboard Teleop ---
Throttle: W/S
Steering: A/D
Brake:    Spacebar
Reset:    R
Quit:     Q
------------------------------
        """)

    def send_commands(self):
        self.throttle_pub.publish(Float64(data=self.throttle))
        self.steering_pub.publish(Float64(data=self.steering))
        self.brake_pub.publish(Float64(data=self.brake))

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def keyboard_loop(self):
        try:
            while rclpy.ok():
                key = self.get_key().lower()
                if key == 'w':
                    self.throttle += 0.1
                elif key == 's':
                    self.throttle = max(0.0, self.throttle - 0.1)
                elif key == 'a':
                    self.steering = max(-1.0, self.steering - 0.1)
                elif key == 'd':
                    self.steering = min(1.0, self.steering + 0.1)
                elif key == ' ':
                    self.brake = 1.0
                elif key == 'r':
                    self.throttle = 0.0
                    self.steering = 0.0
                    self.brake = 0.0
                elif key == 'q':
                    print("Exiting...")
                    rclpy.shutdown()
                    break

                print(f"Throttle: {self.throttle:.2f} | Steering: {self.steering:.2f} | Brake: {self.brake:.2f}")
        except Exception as e:
            self.get_logger().error(f"Keyboard loop error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AudibotTeleop()
    rclpy.spin(node)  # 🔁 Now the timer works
    node.destroy_node()
    rclpy.shutdown()
