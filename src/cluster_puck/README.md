
# cluster_puck

`cluster_puck` is a ROS 2 Python package designed for real-time detection, classification, and visualization of object clusters (e.g., cones) using 2D laser scan data and odometry. It estimates the number of clusters, applies PCA and KMeans clustering, classifies clusters using an SVM model, and tracks confirmed cones across the map using a dynamic ConeMap.

---

## 🔍 What It Does

- Subscribes to:
  - `/orange/laserscan` (`sensor_msgs/msg/LaserScan`)
  - `/orange/odom` (`nav_msgs/msg/Odometry`)
  - `/cluster_label` (`std_msgs/msg/Int32`) — for manual labeling (if recording is enabled)
- Transforms each scan point into the global frame using odometry and known static offsets from the robot's URDF
- Estimates the number of clusters based on spatial gaps
- Applies **KMeans clustering** to group points
- Uses **PCA** to determine the shape and orientation of each cluster
- Classifies clusters using a **trained SVM model**
- Maintains a persistent **ConeMap** of confirmed cone positions and shape descriptors

---

## 🖼️ Visualization Topics

| Topic                   | Description                                         |
|------------------------|-----------------------------------------------------|
| `/cluster_puck/points` | Green dots for all laser scan points               |
| `/cluster_puck/clusters` | Blue spheres for cluster centers                  |
| `/cluster_puck/pca_ellipsoids` | Purple ellipsoids based on PCA              |
| `/cone_ellipsoids`     | Orange ellipsoids for confirmed cone shapes        |
| `/cone_centres`        | PoseArray of all confirmed cone centers            |

---

## 📦 Dependencies

### 🐍 Python Dependencies

Install using `pip`:

```bash
pip install numpy==1.23.5 scikit-learn pandas joblib
```

> ⚠️ `tf_transformations` is based on `transforms3d`, which is **incompatible with numpy ≥ 1.24**, so we lock `numpy` to 1.23.5.

### 🧩 ROS 2 Dependencies

Install using:

```bash
sudo apt install ros-$ROS_DISTRO-tf-transformations
```

Replace `$ROS_DISTRO` with your ROS 2 distro (`humble`, `foxy`, etc).

---

## 🛠️ Build Instructions

From the root of your workspace:

```bash
cd ~/drive_to_survive
colcon build --packages-select cluster_puck
source install/setup.bash
```

---

## 🚀 Running the Node

Start the Audibot simulation:

```bash
ros2 launch gazebo_tf drive_to_survive.launch.py
```

Run the Lidar processing node:

```bash
ros2 run cluster_puck lidar_processing
```

---

## 🖥️ Visualizing in RViz

1. Set **Fixed Frame** to `world`
2. Add the following markers:
   - `/cluster_puck/points` – green (raw lidar)
   - `/cluster_puck/clusters` – blue spheres (KMeans centers)
   - `/cluster_puck/pca_ellipsoids` – purple (PCA ellipsoids)
   - `/cone_ellipsoids` – orange (confirmed cones)
   - `/cone_centres` – cone positions as PoseArray

---

## 📊 SVM Training (Classification)

The classifier is trained on the following features:

- `extent_x`, `extent_y`
- `aspect_ratio`, `area`
- `num_points`

To retrain:

```bash
python3 cluster_puck/train_svm.py
```

This reads `training_data/cluster_training_data.csv` and saves the model to `svm_weights/svm_cone_classifier.pkl`.

---

## ✍️ Labeling Clusters

Enable `recording_enabled = True` in `lidar_processing.py` to pause at each cluster for labeling.

Then use:

```bash
ros2 topic pub /cluster_label std_msgs/msg/Int32 "data: 1"   # Cone
ros2 topic pub /cluster_label std_msgs/msg/Int32 "data: 0"   # Not a cone
```

You’ll see an orange ellipsoid + points around the cluster currently being labeled.

Labeled data will be appended to:

```
training_data/cluster_training_data.csv
```

---

## 🗺️ Cone Map

The ConeMap class:

- Tracks cone locations across time
- Updates their position and shape using a running average
- Prevents duplicate cones using a spatial threshold
- Is published live to:
  - `/cone_ellipsoids` (visual)
  - `/cone_centres` (PoseArray)

---

## 👤 Author

**Jarred Deluca**  
📧 `jarred.g.deluca@student.uts.edu.au`

---

## 🧠 Future Ideas

- Real-time cone confidence scoring
- Path planning integration
- ConeMap export to JSON or CSV
- Integration with perception & planning stacks

---