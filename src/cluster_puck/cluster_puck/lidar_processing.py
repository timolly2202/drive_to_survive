import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import numpy as np
import tf_transformations
from geometry_msgs.msg import Point
from sklearn.cluster import KMeans
from sklearn.decomposition import PCA
import math
import csv
import os
from std_msgs.msg import Int32

class LidarProcessing(Node):
    def __init__(self):
        super().__init__('lidar_processing')
        self.scan_sub = self.create_subscription(LaserScan, '/orange/laserscan', self.laser_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/orange/odom', self.odom_callback, 10)
        self.label_sub = self.create_subscription(Int32, '/cluster_label', self.label_callback, 10)

        self.points_pub = self.create_publisher(Marker, 'cluster_puck/points', 10)
        self.clusters_pub = self.create_publisher(Marker, 'cluster_puck/clusters', 10)
        self.ellipsoids_pub = self.create_publisher(Marker, 'cluster_puck/pca_ellipsoids', 10)

        self.latest_odom = None
        self.static_laser_offset = np.array([1.378, 0.0, 0.56])  # From odom to laser_link
        self.marker_id = 0

        # Recording classification
        self.recording_enabled = True
        self.waiting_for_label = False
        self.current_cluster_features = None
        self.dataset_path = '/home/jarred/git/drive_to_survive/src/cluster_puck/training_data/cluster_training_data.csv'
        self.cluster_queue = []
        self.processing_cluster = False


        if self.recording_enabled and not os.path.exists(self.dataset_path):
            with open(self.dataset_path, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['extent_x', 'extent_y', 'aspect_ratio', 'area', 'num_points', 'label'])

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.latest_odom = {
            'position': np.array([pos.x, pos.y, pos.z]),
            'orientation': np.array([ori.x, ori.y, ori.z, ori.w])
        }

    def laser_callback(self, msg):
        if self.latest_odom is None or self.waiting_for_label:
            return

        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        valid_indices = np.isfinite(ranges)
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        zs = np.zeros_like(xs)
        local_points = np.stack((xs, ys, zs), axis=1)
        local_points += self.static_laser_offset

        cluster_estimate = self.estimate_cluster_count(local_points)
        self.get_logger().info(f"Estimated Clusters: {cluster_estimate}")

        global_points = self.transform_to_global(local_points)
        self.publish_markers(global_points, color='green', marker_type='points')

        if cluster_estimate > 0 and len(local_points) >= cluster_estimate:
            try:
                kmeans = KMeans(n_clusters=cluster_estimate, n_init='auto')
                kmeans.fit(local_points[:, :2])
                labels = kmeans.labels_

                for i in range(cluster_estimate):
                    cluster_pts = local_points[labels == i]
                    if len(cluster_pts) < 3:
                        continue

                    pca = PCA(n_components=2)
                    pca.fit(cluster_pts[:, :2])
                    center_local = pca.mean_
                    axes = pca.components_
                    variances = pca.explained_variance_
                    lengths = 2.0 * np.sqrt(variances)

                    z = self.latest_odom['position'][2] + self.static_laser_offset[2]
                    center_global = self.transform_to_global(np.array([[center_local[0], center_local[1], z]]))[0]

                    self.publish_sphere(center_global, scale=0.1, color='blue')

                    yaw = math.atan2(axes[0][1], axes[0][0])
                    quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
                    self.publish_ellipsoid(center_global, quat, lengths, height=0.5)

                    if self.recording_enabled:
                        extent_x, extent_y = lengths
                        aspect_ratio = extent_x / extent_y if extent_y != 0 else 0.0
                        area = extent_x * extent_y
                        num_points = len(cluster_pts)

                        if not self.processing_cluster:
                            self.current_cluster_features = [extent_x, extent_y, aspect_ratio, area, num_points]
                            cluster_pts_global = self.transform_to_global(cluster_pts)
                            self.publish_orange_highlight(cluster_pts_global)
                            self.waiting_for_label = True
                            self.processing_cluster = True

            except Exception as e:
                self.get_logger().warn(f"KMeans/PCA failed: {e}")

    def label_callback(self, msg):
        if self.waiting_for_label and self.current_cluster_features is not None:
            label = msg.data
            if label in [0, 1]:
                with open(self.dataset_path, mode='a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(self.current_cluster_features + [label])
                self.get_logger().info(f"✅ Saved label from topic: {label}")
            else:
                self.get_logger().info("❌ Invalid label, must be 0 or 1")
            self.clear_highlight()
            self.waiting_for_label = False
            self.processing_cluster = False  # ✅ Ready for the next cluster

    def transform_to_global(self, points):
        pos = self.latest_odom['position']
        quat = self.latest_odom['orientation']
        rotation_matrix = tf_transformations.quaternion_matrix(quat)[:3, :3]
        return np.dot(points, rotation_matrix.T) + pos

    def publish_markers(self, points, color='green', marker_type='points'):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "laser_points" if color == 'green' else "cluster_shapes"
        marker.id = self.marker_id
        marker.action = Marker.ADD

        if marker_type == 'points':
            marker.type = Marker.POINTS
            marker.scale.x = 0.05
            marker.scale.y = 0.05
        elif marker_type == 'line_strip':
            marker.type = Marker.LINE_STRIP
            marker.scale.x = 0.03

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0 if color == 'green' else 1.0 if color == 'yellow' else 0.0
        marker.color.b = 0.0 if color == 'yellow' else 1.0 if color == 'blue' else 0.0

        for p in points:
            pt = Point()
            pt.x, pt.y, pt.z = p.tolist()
            marker.points.append(pt)

        pub = self.points_pub if color == 'green' else self.clusters_pub
        pub.publish(marker)
        self.marker_id += 1

    def publish_orange_highlight(self, points):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "highlight_cluster"
        marker.id = 9999
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        for p in points:
            pt = Point()
            pt.x, pt.y, pt.z = p.tolist()
            marker.points.append(pt)
        self.clusters_pub.publish(marker)

    def clear_highlight(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "highlight_cluster"
        marker.id = 9999
        marker.action = Marker.DELETE
        self.clusters_pub.publish(marker)

    def publish_sphere(self, position, scale=0.1, color='blue'):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cluster_centers"
        marker.id = self.marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.clusters_pub.publish(marker)
        self.marker_id += 1

    def publish_ellipsoid(self, position, orientation, lengths, height):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pca_ellipsoids"
        marker.id = self.marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]
        marker.scale.x = lengths[0]
        marker.scale.y = lengths[1]
        marker.scale.z = height
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.ellipsoids_pub.publish(marker)
        self.marker_id += 1

    def estimate_cluster_count(self, points, threshold=0.4):
        if len(points) == 0:
            return 0
        cluster_count = 1
        for i in range(1, len(points)):
            distance = np.linalg.norm(points[i] - points[i - 1])
            if distance > threshold:
                cluster_count += 1
        return cluster_count

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
