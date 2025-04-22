# cluster_puck

`cluster_puck` is a ROS 2 Python package designed for real-time detection and visualization of object clusters (e.g., cones) using 2D laser scan data and odometry. It estimates the number of clusters, applies KMeans clustering, and publishes interactive markers to RViz for visualization.

---

## 🔍 What It Does

- Subscribes to:
  - `/orange/laserscan` (`sensor_msgs/msg/LaserScan`)
  - `/orange/odom` (`nav_msgs/msg/Odometry`)
- Transforms each scan point into the global frame using odometry and known static offsets from the robot's URDF.
- Estimates the number of clusters based on spatial gaps.
- Applies **KMeans clustering** to group points.
- Publishes:
  - All scan points as **green dots**
  - Cluster centers as **blue spheres**

This is useful for robotic applications involving perception, mapping, navigation, or object recognition.

---

## 📦 Dependencies

### 🐍 Python Dependencies

Install the following using `pip`:

```bash
pip install numpy==1.23.5 scikit-learn
```

⚠️ **Note**: `transforms3d` used by `tf_transformations` is **incompatible with `numpy >= 1.24`**. That’s why we fix it to `1.23.5`.

### 🧩 ROS 2 Packages

Install the following using `apt`:

```bash
sudo apt install ros-$ROS_DISTRO-tf-transformations
```

Replace `$ROS_DISTRO` with your current ROS 2 distribution (e.g., `humble`, `foxy`, `galactic`).

---

## 🛠️ Build Instructions

Navigate to your ROS 2 workspace and clone the repository:

```bash
cd ~/drive_to_survive
colcon build --packages-select cluster_puck
source install/setup.bash
```

---

## 🚀 Running the Node

Turn on the audibot simulation

```bash
ros2 launch gazebo_tf drive_to_survive.launch.py 
```

Make sure your robot simulation or hardware is publishing the following topics:

- `/orange/laserscan` (`sensor_msgs/msg/LaserScan`)
- `/orange/odom` (`nav_msgs/msg/Odometry`)

Then launch the clustering node:

```bash
ros2 run cluster_puck lidar_processing
```

---

## 🖥️ Visualizing in RViz

1. Set the **Fixed Frame** to `world`.
2. Add the following marker topics:

- `/cluster_puck/points`  
  ➤ Visualizes laser scan points as green dots.

- `/cluster_puck/clusters`  
  ➤ Visualizes cluster centers as blue spheres.

---

## 👤 Author

**Jarred Deluca**  
📧 `jarred.g.deluca@student.uts.edu.au`

---

## 🧠 Future Work

Planned improvements include:

- Extracting PCA-based shape descriptors
- Training ML classifiers to differentiate cones from other obstacles
- Tracking clusters over time using IDs or filters
- Exporting clusters to `.csv` for offline training or analysis