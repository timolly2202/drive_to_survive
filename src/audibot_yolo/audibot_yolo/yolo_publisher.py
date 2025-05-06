from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

"""
@software{yolov8_ultralytics,
  author = {Glenn Jocher and Ayush Chaurasia and Jing Qiu},
  title = {Ultralytics YOLOv8},
  version = {8.0.0},
  year = {2023},
  url = {https://github.com/ultralytics/ultralytics},
  orcid = {0000-0001-5950-6979, 0000-0002-7603-6750, 0000-0003-3783-7069},
  license = {AGPL-3.0}
}
"""

"""
SUBSCRIPTIONS:
/orange/front_camera_sensor/image_raw
/orange/back_camera_sensor/image_raw
/orange/left_camera_sensor/image_raw
/orange/right_camera_sensor/image_raw

PUBLISHERS:
/orange/front_image_yolo
/orange/back_image_yolo
/orange/left_image_yolo
/orange/right_image_yolo
"""

class Camera_subscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.model = YOLO('resources/yolo/weights.pt')

        self.front_cam_subscription = self.create_subscription(
            Image,
            '/orange/front_camera_sensor/image_raw',
            self.front_camera_callback,
            10
        )
        self.front_cam_subscription

        self.back_cam_subscription = self.create_subscription(
            Image,
            '/orange/back_camera_sensor/image_raw',
            self.back_camera_callback,
            10
        )
        self.back_cam_subscription

        self.left_cam_subscription = self.create_subscription(
            Image,
            '/orange/left_camera_sensor/image_raw',
            self.left_camera_callback,
            10
        )
        self.left_cam_subscription

        self.right_cam_subscription = self.create_subscription(
            Image,
            '/orange/right_camera_sensor/image_raw',
            self.right_camera_callback,
            10
        )
        self.right_cam_subscription

        self.front_img_pub = self.create_publisher(Image, "/orange/front_image_yolo",1)
        self.back_img_pub = self.create_publisher(Image, "/orange/back_image_yolo",1)
        self.left_img_pub = self.create_publisher(Image, "/orange/left_image_yolo",1)
        self.right_img_pub = self.create_publisher(Image, "/orange/right_image_yolo",1)
    
    def front_camera_callback(self, input):
        image = bridge.imgmsg_to_cv2(input,"bgr8")
        output = self.model(image)

        annotated_frame = output[0].plot()
        image_msg = bridge.cv2_to_imgmsg(annotated_frame)
        self.front_img_pub.publish(image_msg)
    
    def back_camera_callback(self, input):
        image = bridge.imgmsg_to_cv2(input,"bgr8")
        output = self.model(image)

        annotated_frame = output[0].plot()
        image_msg = bridge.cv2_to_imgmsg(annotated_frame)
        self.back_img_pub.publish(image_msg)
    
    def left_camera_callback(self, input):
        image = bridge.imgmsg_to_cv2(input,"bgr8")
        output = self.model(image)

        annotated_frame = output[0].plot()
        image_msg = bridge.cv2_to_imgmsg(annotated_frame)
        self.left_img_pub.publish(image_msg)

    def right_camera_callback(self, input):
        image = bridge.imgmsg_to_cv2(input,"bgr8")
        output = self.model(image)

        annotated_frame = output[0].plot()
        image_msg = bridge.cv2_to_imgmsg(annotated_frame)
        self.right_img_pub.publish(image_msg)


def main (args=None):
    rclpy.init(args=args)
    node = Camera_subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()