from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolo_msg.msg import BoundingBox
from yolo_msg.msg import YoloDetections

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
/orange/left_camera_sensor/image_raw

PUBLISHERS:
/orange/left_image_yolo/image
/orange/left_image_yolo/detections
"""

class Camera_subscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.model = YOLO('resources/yolo/weights.pt')

        self.yolo_detections_left = YoloDetections()

        self.left_cam_subscription = self.create_subscription(
            Image,
            '/orange/left_camera_sensor/image_raw',
            self.left_camera_callback,
            10
        )
        self.left_cam_subscription

        self.left_img_pub = self.create_publisher(Image, "/orange/left_image_yolo/image",1)

        self.left_detections_pub = self.create_publisher(YoloDetections, "/orange/left_image_yolo/detections",1)
    
    def left_camera_callback(self, input):
        image = bridge.imgmsg_to_cv2(input,"bgr8")
        output = self.model(image)

        self.yolo_detections_left.header.frame_id = "left_detections"
        self.yolo_detections_left.header.stamp = self.get_clock().now().to_msg()

        for i in output:
            boxes = i.boxes
            for box in boxes:
                bounding_box = BoundingBox()
                coords = box.xyxy[0].to('cpu').detach().numpy().copy()
                box_class = box.cls
                bounding_box.class_name = self.model.names[int(box_class)]
                bounding_box.top = int(coords[0])
                bounding_box.left = int(coords[1])
                bounding_box.bottom = int(coords[2])
                bounding_box.right = int(coords[3])

                self.yolo_detections_left.bounding_boxes.append(bounding_box)

        annotated_frame = output[0].plot()
        image_msg = bridge.cv2_to_imgmsg(annotated_frame)
        self.left_img_pub.publish(image_msg)

        self.left_detections_pub.publish(self.yolo_detections_left)
        self.yolo_detections_left.bounding_boxes.clear()


def main (args=None):
    rclpy.init(args=args)
    node = Camera_subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()