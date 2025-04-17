import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import numpy as np
import tf_transformations
from geometry_msgs.msg import Point
from sklearn.cluster import KMeans

class LaserToGlobal(Node):
    def __init__(self):
        super().__init__('laser_to_global')
        self.scan_sub = self.create_subscription(LaserScan, '/orange/laserscan', self.laser_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/orange/odom', self.odom_callback, 10)

        # Separate publishers
        self.points_pub = self.create_publisher(Marker, 'cluster_puck/points', 10)
        self.clusters_pub = self.create_publisher(Marker, 'cluster_puck/clusters', 10)

        self.latest_odom = None
        self.static_laser_offset = np.array([1.378, 0.0, 0.56])  # From odom to laser_link

        self.marker_id = 0

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.latest_odom = {
            'position': np.array([pos.x, pos.y, pos.z]),
            'orientation': np.array([ori.x, ori.y, ori.z, ori.w])
        }

    def laser_callback(self, msg):
        if self.latest_odom is None:
            self.get_logger().info("Waiting for odometry...")
            return

        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        valid_indices = np.isfinite(ranges)
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]

        # Convert from polar to local Cartesian (in laser frame)
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        zs = np.zeros_like(xs)
        local_points = np.stack((xs, ys, zs), axis=1)

        # Offset to base_link
        local_points += self.static_laser_offset

        # Estimate number of clusters
        cluster_estimate = self.estimate_cluster_count(local_points)
        self.get_logger().info(f"Estimated Clusters: {cluster_estimate}")

        # Transform local points to global
        global_points = self.transform_to_global(local_points)

        # Visualize laser points
        self.publish_markers(global_points, color='green', marker_type='points')

        # Apply KMeans in local frame, then transform centers
        if cluster_estimate > 0 and len(local_points) >= cluster_estimate:
            try:
                kmeans = KMeans(n_clusters=cluster_estimate, n_init='auto')
                kmeans.fit(local_points[:, :2])  # Only X, Y
                centers_local = np.hstack((kmeans.cluster_centers_, np.zeros((cluster_estimate, 1))))
                # Add Z = odom_z + lidar offset (i.e., absolute Z in global frame)
                odom_z = self.latest_odom['position'][2]
                z_offset = self.static_laser_offset[2]
                centers_z = np.full((cluster_estimate, 1), odom_z + z_offset)
                centers_local = np.hstack((kmeans.cluster_centers_, centers_z))

                # Transform XY to global using the same function
                centers_global = self.transform_to_global(centers_local)

                self.publish_markers(centers_global, color='blue', marker_type='spheres')
            except Exception as e:
                self.get_logger().warn(f"KMeans failed: {e}")

    def transform_to_global(self, points):
        pos = self.latest_odom['position']
        quat = self.latest_odom['orientation']
        rotation_matrix = tf_transformations.quaternion_matrix(quat)[:3, :3]
        return np.dot(points, rotation_matrix.T) + pos

    def publish_markers(self, points, color='green', marker_type='points'):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "laser_points" if color == 'green' else "cluster_centers"
        marker.id = self.marker_id
        marker.action = Marker.ADD

        if marker_type == 'points':
            marker.type = Marker.POINTS
            marker.scale.x = 0.05
            marker.scale.y = 0.05
        elif marker_type == 'spheres':
            marker.type = Marker.SPHERE_LIST
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0 if color == 'green' else 0.0
        marker.color.b = 1.0 if color == 'blue' else 0.0

        for p in points:
            pt = Point()
            pt.x, pt.y, pt.z = p.tolist()
            marker.points.append(pt)

        if color == 'green':
            self.points_pub.publish(marker)
        else:
            self.clusters_pub.publish(marker)

        self.marker_id += 1

    def estimate_cluster_count(self, points, threshold=0.4):
        """
        Estimate number of clusters by counting gaps larger than threshold
        between consecutive laser scan points.
        """
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
    node = LaserToGlobal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
