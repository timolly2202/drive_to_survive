# yolo_training

This trains the yolo to detect cones and the firetruck using a manually labelled dataset.

Dataset: https://universe.roboflow.com/computer-vision-wppi7/drive-to-survive

## Training

To train, unzip the data, cd into `yolo_training`, and run the train_yolo script.

If you'd like to make more datapoints, run the save_images script while gazebo is running with the audi, while also moving it around with teleop.