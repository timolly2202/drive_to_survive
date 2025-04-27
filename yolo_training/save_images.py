import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time 

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.bridge = CvBridge()
        self.saved_images = {}
        timestamp = int(time.time() * 1000)
        # Define your image topics here
        self.topics = {
            '/orange/front_camera_sensor/image_raw': f'{timestamp}front.jpg',
            '/orange/back_camera_sensor/image_raw': f'{timestamp}back.jpg',
            '/orange/left_camera_sensor/image_raw': f'{timestamp}left.jpg',
            '/orange/right_camera_sensor/image_raw': f'{timestamp}right.jpg'
        }

        self.subs = []

        for topic in self.topics:
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, t=topic: self.image_callback(msg, t),
                10
            )
            self.subs.append(sub)
        
    def image_callback(self, msg, topic_name):
            if topic_name not in self.saved_images:
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                    filename = self.topics[topic_name]
                    cv2.imwrite(filename, cv_image)
                    self.saved_images[topic_name] = filename
                    self.get_logger().info(f"Saved {filename}")
                except Exception as e:
                    self.get_logger().error(f"Error converting image: {e}")

            # Exit after all 4 images have been saved
            if len(self.saved_images) == len(self.topics):
                self.get_logger().info("All images saved. Shutting down.")
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()