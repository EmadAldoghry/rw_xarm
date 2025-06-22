#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from rclpy.duration import Duration as RclpyDuration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PointStamped, Point, Quaternion
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import tf_transformations
from std_srvs.srv import SetBool
import traceback

class GoalCalculatorNode(Node):
    def __init__(self):
        super().__init__('goal_calculator_node')
        
        self.declare_parameter('input_pc_topic', '/target_points')
        self.declare_parameter('output_corrected_goal_topic', '/corrected_local_goal')
        self.declare_parameter('navigation_frame', 'map')

        self.navigation_frame_ = self.get_parameter('navigation_frame').value
        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)
        
        self.is_active = False

        # The service name must match what the orchestrator is calling.
        # The default name is 'goal_calculator_node/activate_segmentation'
        self.activation_service_ = self.create_service(SetBool, '~/activate_segmentation', self.handle_activation_request)

        self.target_points_sub_ = None
        self.input_pc_topic_ = self.get_parameter('input_pc_topic').value

        _qos_transient = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.corrected_goal_pub_ = self.create_publisher(PoseStamped, self.get_parameter('output_corrected_goal_topic').value, _qos_transient)
        
        self.get_logger().info('Goal Calculator Node started. Waiting for activation.')

    def handle_activation_request(self, request: SetBool.Request, response: SetBool.Response):
        self.is_active = request.data
        if self.is_active:
            if self.target_points_sub_ is None:
                qos_sensor = rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
                self.target_points_sub_ = self.create_subscription(PointCloud2, self.input_pc_topic_, self.target_points_callback, qos_sensor)
                response.message = "Goal calculation ACTIVATED."
            else:
                response.message = "Goal calculation was already active."
        else:
            if self.target_points_sub_ is not None:
                self.destroy_subscription(self.target_points_sub_)
                self.target_points_sub_ = None
                self._publish_invalid_goal("Deactivated by service call.")
                response.message = "Goal calculation DEACTIVATED."
            else:
                response.message = "Goal calculation was already inactive."

        self.get_logger().info(response.message)
        response.success = True
        return response
    
    def _publish_invalid_goal(self, reason=""):
        invalid_goal_pose = PoseStamped(header=Header(stamp=self.get_clock().now().to_msg(), frame_id=""))
        self.corrected_goal_pub_.publish(invalid_goal_pose)
        self.get_logger().debug(f"Published invalid goal. Reason: {reason}")

    def target_points_callback(self, msg: PointCloud2):
        if not self.is_active:
            return
        
        # --- NEW LOGGING ---
        self.get_logger().info("Received a point cloud on /target_points")

        try:
            target_points_np = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            
            # --- NEW LOGGING ---
            num_points = len(target_points_np)
            self.get_logger().info(f"Point cloud contains {num_points} points.")

            if num_points == 0:
                self._publish_invalid_goal("Received empty target point cloud.")
                return
            
            centroid_lidar_frame = np.mean(target_points_np.reshape(-1, 3), axis=0)
            
            pt_stamped_lidar = PointStamped(
                header=msg.header, 
                point=Point(x=float(centroid_lidar_frame[0]), y=float(centroid_lidar_frame[1]), z=float(centroid_lidar_frame[2]))
            )

            transform_to_nav = self.tf_buffer_.lookup_transform(
                self.navigation_frame_,
                pt_stamped_lidar.header.frame_id,
                RclpyTime(),
                timeout=RclpyDuration(seconds=0.5)
            )
            
            pt_stamped_nav_frame = do_transform_point(pt_stamped_lidar, transform_to_nav)
            
            corrected_goal_pose = PoseStamped(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.navigation_frame_)
            )
            corrected_goal_pose.pose.position = pt_stamped_nav_frame.point
            
            q = tf_transformations.quaternion_from_euler(0, 0, 0) 
            corrected_goal_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

            self.get_logger().info(f"Successfully calculated and publishing corrected goal in {self.navigation_frame_} frame.")
            self.corrected_goal_pub_.publish(corrected_goal_pose)

        except Exception as ex:
            self.get_logger().warn(f"Could not calculate/publish goal. TF lookup or other error: {ex}", throttle_duration_sec=2.0)
            self.get_logger().debug(f"Traceback: {traceback.format_exc()}", throttle_duration_sec=2.0)
            self._publish_invalid_goal(f"TF Error: {ex}")

def main(args=None):
    rclpy.init(args=args)
    node = GoalCalculatorNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Unhandled exception in GoalCalculatorNode: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()