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
import open3d as o3d  # For voxel downsampling


class PathGeneratorNode(Node):
    def __init__(self):
        super().__init__('path_generator_publisher_node')

        # --- Parameters ---
        self.declare_parameter('input_topic', '/projected_non_ground_points')
        self.declare_parameter('output_topic', '/arm_path_poses')
        self.declare_parameter('z_offset', 0.025)
        self.declare_parameter('voxel_size', 0.02)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.z_offset = self.get_parameter('z_offset').value
        self.voxel_size = self.get_parameter('voxel_size').value

        self.target_roll = math.pi
        self.target_pitch = 0.0
        self.target_yaw = 0.0
        self.target_orientation_quat = tf_transformations.quaternion_from_euler(
            self.target_roll, self.target_pitch, self.target_yaw
        )
        self.target_orientation = Quaternion(
            x=self.target_orientation_quat[0],
            y=self.target_orientation_quat[1],
            z=self.target_orientation_quat[2],
            w=0.0
        )
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
        self.get_logger().info(f"Subscribed to: {self.input_topic}")
        self.get_logger().info(f"Publishing to: {self.output_topic} (Frame: {self.output_frame_id})")
        self.get_logger().info(f"Z-offset: {self.z_offset}, Voxel size: {self.voxel_size}")
        self.get_logger().info(f"Fixed orientation RPY: ({self.target_roll:.2f}, {self.target_pitch:.2f}, {self.target_yaw:.2f})")

    def pointcloud_callback(self, cloud_msg: PointCloud2):
        if self._is_processing:
            self.get_logger().warn("Still processing previous cloud. Skipping new message.")
            return

        self._is_processing = True
        try:
            points_generator = pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
            points_list = [[x, y, z] for x, y, z in points_generator]

            if not points_list:
                self.get_logger().warn("Received empty point cloud.")
                return

            points_np = np.array(points_list, dtype=np.float32)
            points_np = points_np[np.all(np.isfinite(points_np), axis=1)]

            if points_np.shape[0] < 2:
                self.get_logger().warn("Not enough valid points for path generation.")
                return

            # --- Downsample using voxel grid ---
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points_np)
            pcd_down = pcd.voxel_down_sample(voxel_size=self.voxel_size)
            downsampled_points = np.asarray(pcd_down.points)

            if downsampled_points.shape[0] < 2:
                self.get_logger().warn(f"Downsampled point cloud too small ({downsampled_points.shape[0]} points).")
                return

            self.get_logger().info(f"Downsampled from {points_np.shape[0]} to {downsampled_points.shape[0]} points.")

            # --- Nearest Neighbor Path Generation ---
            num_points = len(downsampled_points)
            distances_from_origin = np.linalg.norm(downsampled_points, axis=1)
            start_index = np.argmax(distances_from_origin)

            kdtree = KDTree(downsampled_points)
            visited = np.zeros(num_points, dtype=bool)
            path_indices = [start_index]
            visited[start_index] = True
            current_index = start_index

            for _ in range(num_points - 1):
                distances, indices = kdtree.query(downsampled_points[current_index], k=num_points)
                next_index = -1
                for idx in indices:
                    if 0 <= idx < num_points and not visited[idx]:
                        next_index = idx
                        break
                if next_index == -1:
                    self.get_logger().warn("Could not find next unvisited neighbor. Ending path early.")
                    break
                path_indices.append(next_index)
                visited[next_index] = True
                current_index = next_index

            sorted_points = downsampled_points[path_indices]
            sorted_points[:, 2] += self.z_offset

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
            self.get_logger().error(f"Error during processing: {e}", exc_info=True)
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
