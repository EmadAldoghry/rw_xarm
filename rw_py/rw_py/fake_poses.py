# FILE: rw/rw/fake_poses.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
import numpy as np
from scipy.spatial import KDTree
import tf_transformations
import math
import open3d as o3d

class PathGeneratorNode(Node):
    def __init__(self):
        super().__init__('path_generator_publisher_node')

        self.declare_parameter('input_topic', '/projected_non_ground_points')
        self.declare_parameter('output_topic', '/arm_path_poses')
        # <<< FIX 1: Increase the Z-offset to a safer height
        self.declare_parameter('z_offset', 0.15) # From 2.5cm to 15cm
        self.declare_parameter('voxel_size', 0.02)
        # <<< FIX 2: Add a parameter to filter points too close to the robot
        self.declare_parameter('min_distance_from_base', 0.25) # Min 25cm from robot base
        self.declare_parameter('target_roll_deg', 180.0)
        self.declare_parameter('target_pitch_deg', 0.0)
        self.declare_parameter('target_yaw_deg', 0.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.z_offset = self.get_parameter('z_offset').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.min_dist = self.get_parameter('min_distance_from_base').value
        
        target_roll = math.radians(self.get_parameter('target_roll_deg').value)
        target_pitch = math.radians(self.get_parameter('target_pitch_deg').value)
        target_yaw = math.radians(self.get_parameter('target_yaw_deg').value)

        q = tf_transformations.quaternion_from_euler(target_roll, target_pitch, target_yaw)
        self.target_orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.output_frame_id = "base_link"

        self._is_processing = False

        pose_array_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pose_array_publisher = self.create_publisher(
            PoseArray,
            self.output_topic,
            pose_array_qos)

        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info("PathGeneratorNode initialized.")
        self.get_logger().info(f"Publishing to: {self.output_topic} (Frame: {self.output_frame_id})")
        self.get_logger().info(f"Z-offset: {self.z_offset}, Min distance: {self.min_dist}")

    def pointcloud_callback(self, cloud_msg: PointCloud2):
        if self._is_processing:
            return

        self._is_processing = True
        try:
            points_generator = pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
            points_list = [[x, y, z] for x, y, z in points_generator]

            if not points_list:
                return

            points_np = np.array(points_list, dtype=np.float32)
            points_np = points_np[np.all(np.isfinite(points_np), axis=1)]
            
            # <<< FIX 2: Filter points that are too close to the robot's base (0,0,0 in base_link frame)
            distances = np.linalg.norm(points_np[:, :2], axis=1) # Check only XY distance
            keep_mask = distances >= self.min_dist
            points_np = points_np[keep_mask]
            
            if points_np.shape[0] < 2:
                self.get_logger().warn("Not enough valid points after filtering for path generation.")
                return

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points_np)
            pcd_down = pcd.voxel_down_sample(voxel_size=self.voxel_size)
            downsampled_points = np.asarray(pcd_down.points)

            if downsampled_points.shape[0] < 2:
                self.get_logger().warn(f"Downsampled point cloud too small ({downsampled_points.shape[0]} points).")
                return

            # --- Sort points by distance from one end to the other ---
            # Find the two points that are farthest apart to define the start and end of the path
            from scipy.spatial.distance import pdist, squareform
            dist_matrix = squareform(pdist(downsampled_points, 'euclidean'))
            start_idx, end_idx = np.unravel_index(np.argmax(dist_matrix), dist_matrix.shape)

            # Use KDTree for efficient nearest neighbor search
            kdtree = KDTree(downsampled_points)
            path_indices = [start_idx]
            visited = {start_idx}
            current_index = start_idx

            while len(path_indices) < len(downsampled_points):
                _, indices = kdtree.query(downsampled_points[current_index], k=len(downsampled_points))
                found_next = False
                for idx in indices:
                    if idx not in visited:
                        path_indices.append(idx)
                        visited.add(idx)
                        current_index = idx
                        found_next = True
                        break
                if not found_next:
                    self.get_logger().warn("Could not find next unvisited neighbor. Path may be incomplete.")
                    break # Break if no unvisited neighbor is found

            sorted_points = downsampled_points[path_indices]
            # <<< FIX 1: Apply the Z-offset
            sorted_points[:, 2] = self.z_offset

            pose_array_msg = PoseArray()
            pose_array_msg.header.stamp = self.get_clock().now().to_msg()
            pose_array_msg.header.frame_id = self.output_frame_id

            for point in sorted_points:
                pose = Pose()
                pose.position = Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
                pose.orientation = self.target_orientation
                pose_array_msg.poses.append(pose)

            self.pose_array_publisher.publish(pose_array_msg)
            self.get_logger().info(f"Published PoseArray with {len(pose_array_msg.poses)} poses.")

        except Exception as e:
            self.get_logger().error(f"Error during processing: {e}")
        finally:
            self._is_processing = False


def main(args=None):
    rclpy.init(args=args)
    node = PathGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()