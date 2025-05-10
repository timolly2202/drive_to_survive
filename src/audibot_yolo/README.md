# audibot_yolo

`audibot_yolo` is a ROS 2 Python package designed for real-time detection of traffic cones and firetrucks, and publishing the images with bounding boxes on detected objects. It has yolo_publisher which just adds bounding boxes to the images, and separate nodes for each camera which publishes the bounding box msgs as well.

## What it does

- It has nodes subscribes to the raw images
    - `/orange/front_camera_sensor/image_raw` (`sensor_msgs/msg/Image`)
    - `/orange/back_camera_sensor/image_raw` (`sensor_msgs/msg/Image`)
    - `/orange/left_camera_sensor/image_raw` (`sensor_msgs/msg/Image`)
    - `/orange/right_camera_sensor/image_raw` (`sensor_msgs/msg/Image`)
- Uses yolov8 to determine what is a cone or a firetruck
- It then publishes the images with bounding boxes to
    - `/orange/front_image_yolo/image` (`sensor_msgs/msg/Image`)
    - `/orange/back_image_yolo/image` (`sensor_msgs/msg/Image`)
    - `/orange/left_image_yolo/image` (`sensor_msgs/msg/Image`)
    - `/orange/right_image_yolo/image` (`sensor_msgs/msg/Image`)
    - `/orange/front_image_yolo/detections` (`yolo_msg/msg/YoloDetections`)
    - `/orange/back_image_yolo/detections` (`yolo_msg/msg/YoloDetections`)
    - `/orange/left_image_yolo/detections` (`yolo_msg/msg/YoloDetections`)
    - `/orange/right_image_yolo/detections` (`yolo_msg/msg/YoloDetections`)

## Dependencies

### Python

```bash
pip install ultralytics cv-bridge
```

## Build Instructions

Navigate to your ROS 2 workspace and clone the full repository:

```bash
colcon build --packages-select audibot_yolo
source install/setup.bash
```

## Running the Node

Turn on the audibot dimulation:

```bash
ros2 launch gazebo_tf drive_to_survive.launch.py 
```

Launch the yolo publisher:

```bash
ros2 run audibot_yolo audibot_yolo
```

OR you can launch all the separate camera nodes:
```bash
ros2 launch audibot_yolo multi_camera.launch.py
```

## Visualizing in RViz

Change the camera sensor images to the yolo images.