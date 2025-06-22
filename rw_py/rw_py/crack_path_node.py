#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct
import threading
import time

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped
import tf_transformations

def nothing(x):
    pass

class ColorPointCloudFilterNode(Node):
    def __init__(self):
        super().__init__('color_pointcloud_filter_node')

        # --- Parameters ---
        self.declare_parameter('input_cloud_topic', '/camera2/points')
        self.declare_parameter('output_cloud_topic', '/projected_non_ground_points')
        self.declare_parameter('debug_image_topic', '/centerline_debug_image')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('black_threshold', 1)
        self.declare_parameter('slice_width', 0.05)
        self.declare_parameter('show_debug_window', False)

        # Get parameters
        input_cloud_topic = self.get_parameter('input_cloud_topic').get_parameter_value().string_value
        output_cloud_topic = self.get_parameter('output_cloud_topic').get_parameter_value().string_value
        debug_image_topic = self.get_parameter('debug_image_topic').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.black_thresh = self.get_parameter('black_threshold').get_parameter_value().integer_value
        self.slice_width = self.get_parameter('slice_width').get_parameter_value().double_value
        self.show_window = self.get_parameter('show_debug_window').get_parameter_value().bool_value

        # --- TF2 & CV Bridge ---
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()
        
        # --- Subscriber and Publisher ---
        self.sub = self.create_subscription(PointCloud2, input_cloud_topic, self.pointcloud_callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, output_cloud_topic, 10)
        self.debug_image_pub = self.create_publisher(Image, debug_image_topic, 10)

        # *** FIX: Define the PointField structure here ***
        self.xyzrgb_point_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # --- State and Threading for Window ---
        self.latest_bgr_image = None
        self.image_lock = threading.Lock()
        
        # --- Conditionally create OpenCV Window ---
        if self.show_window:
            self.cv_window_name = "Threshold Tuner"
            cv2.namedWindow(self.cv_window_name)
            cv2.createTrackbar('Threshold', self.cv_window_name, self.black_thresh, 255, self.on_trackbar_change)
            self.get_logger().info("Debug window is ENABLED.")
        else:
            self.get_logger().info("Debug window is DISABLED. Publishing debug images to /centerline_debug_image topic.")
        
        self.get_logger().info(f"Color Point Cloud Filter initialized. Subscribed to '{input_cloud_topic}'.")

    def on_trackbar_change(self, val):
        self.black_thresh = val

    def pointcloud_callback(self, cloud_msg: PointCloud2):
        source_frame = cloud_msg.header.frame_id
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=0.2)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not get transform from '{source_frame}' to '{self.target_frame}': {e}", throttle_duration_sec=2.0)
            return

        black_points_cam_frame = []
        
        for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
            rgb_bytes = struct.pack('f', point[3])
            b, g, r, a = struct.unpack('BBBB', rgb_bytes)
            if r < self.black_thresh and g < self.black_thresh and b < self.black_thresh:
                black_points_cam_frame.append([point[0], point[1], point[2]])

        if not black_points_cam_frame:
            return

        points_np = np.array(black_points_cam_frame, dtype=np.float32)
        
        t = transform.transform.translation
        q = transform.transform.rotation
        trans_matrix = tf_transformations.translation_matrix([t.x, t.y, t.z])
        rot_matrix = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        transform_matrix = np.dot(trans_matrix, rot_matrix)
        
        points_homogeneous = np.hstack((points_np, np.ones((points_np.shape[0], 1))))
        transformed_homogeneous = points_homogeneous @ transform_matrix.T
        points_in_target_frame = transformed_homogeneous[:, :3]

        if points_in_target_frame.shape[0] == 0:
            return
            
        centerline_points = self.calculate_centerline_from_3d_points(points_in_target_frame)

        if not centerline_points:
            return

        self.get_logger().info(f"Publishing {len(centerline_points)} centerline points.", throttle_duration_sec=2.0)
        self.publish_filtered_cloud(centerline_points, cloud_msg.header)

        if self.show_window:
            with self.image_lock:
                self.latest_bgr_image = self.create_image_from_points(cloud_msg)

    def run_vision_loop(self):
        if not self.show_window:
            self.get_logger().warn("run_vision_loop() called, but debug window is disabled. Node will spin normally.")
            return

        while rclpy.ok():
            with self.image_lock:
                if self.latest_bgr_image is None:
                    time.sleep(0.01)
                    continue
                display_image = self.latest_bgr_image.copy()

            gray = cv2.cvtColor(display_image, cv2.COLOR_BGR2GRAY)
            _, mask = cv2.threshold(gray, self.black_thresh, 255, cv2.THRESH_BINARY_INV)
            debug_display = np.hstack([display_image, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
            cv2.imshow(self.cv_window_name, debug_display)

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                break
        
        cv2.destroyAllWindows()

    def create_image_from_points(self, cloud_msg):
        height = cloud_msg.height
        width = cloud_msg.width
        if height == 0 or width == 0: return None
        image = np.zeros((height, width, 3), dtype=np.uint8)
        points = pc2.read_points(cloud_msg, field_names=("x", "y", "z", "rgb"), skip_nans=False)
        for i, point in enumerate(points):
            row = i // width
            col = i % width
            rgb_bytes = struct.pack('f', point[3])
            b, g, r, a = struct.unpack('BBBB', rgb_bytes)
            image[row, col] = [b, g, r]
        return image

    def calculate_centerline_from_3d_points(self, points_3d):
        centerline = []
        if points_3d.shape[0] < 2: return centerline
        min_x = np.min(points_3d[:, 0])
        max_x = np.max(points_3d[:, 0])
        current_x = min_x
        while current_x < max_x:
            slice_mask = (points_3d[:, 0] >= current_x) & (points_3d[:, 0] < current_x + self.slice_width)
            points_in_slice = points_3d[slice_mask]
            if points_in_slice.shape[0] > 0:
                centroid_y = np.mean(points_in_slice[:, 1])
                centroid_x = current_x + self.slice_width / 2.0
                centerline.append([centroid_x, centroid_y, 0.0])
            current_x += self.slice_width
        return centerline

    def publish_filtered_cloud(self, points, original_header):
        r, g, b = 255, 165, 0
        rgb_int = (r << 16) | (g << 8) | b
        packed = struct.pack('I', rgb_int)
        rgb_float = struct.unpack('f', packed)[0]
        cloud_points = [[p[0], p[1], p[2], rgb_float] for p in points]
        output_header = Header()
        output_header.stamp = original_header.stamp
        output_header.frame_id = self.target_frame
        cloud_msg = pc2.create_cloud(output_header, self.xyzrgb_point_fields, cloud_points)
        self.pub.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ColorPointCloudFilterNode()
    if node.show_window:
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        try:
            node.run_vision_loop()
        except KeyboardInterrupt:
            pass
        finally:
            node.get_logger().info("Shutting down...")
            rclpy.shutdown()
            spin_thread.join()
    else:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.get_logger().info("Shutting down...")
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()