# audibot_yolo

`audibot_yolo` is a ROS 2 Python package designed for real-time detection of traffic cones and firetrucks, and publishing the images with bounding boxes on detected objects.

## What it does

- It subscribes to
    - `/orange/front_camera_sensor/image_raw` (`sensor_msgs/msg/Image`)
    - `/orange/back_camera_sensor/image_raw` (`sensor_msgs/msg/Image`)
    - `/orange/left_camera_sensor/image_raw` (`sensor_msgs/msg/Image`)
    - `/orange/right_camera_sensor/image_raw` (`sensor_msgs/msg/Image`)
- Uses yolov8 to determine what is a cone or a firetruck
- It then publishes the images with bounding boxes to
    - `/orange/front_image_yolo` (`sensor_msgs/msg/Image`)
    - `/orange/back_image_yolo` (`sensor_msgs/msg/Image`)
    - `/orange/left_image_yolo` (`sensor_msgs/msg/Image`)
    - `/orange/right_image_yolo` (`sensor_msgs/msg/Image`)

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

## Visualizing in RViz

Change the camera sensor images to the yolo images.