#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time as RclpyTime
from rclpy.duration import Duration as RclpyDuration
from functools import partial
import traceback
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import SetBool
from action_msgs.msg import GoalStatus
from enum import Enum, auto

from rw_interfaces.srv import GetWaypoints
from rw_interfaces.msg import ProximityStatus, NavigationStatus

class NavState(Enum):
    IDLE = 0
    FETCHING_WAYPOINTS = 1
    NAVIGATING_GLOBAL = 2
    AWAITING_LOCAL_CORRECTION = 3
    NAVIGATING_LOCAL = 4
    WAYPOINT_COMPLETE = 5
    MISSION_COMPLETE = 6
    MISSION_FAILED = 7

class WaypointFollowerCorrected(Node):
    def __init__(self):
        super().__init__('waypoint_follower_corrected_node')
        
        # Parameters
        self.declare_parameter('local_target_arrival_threshold', 0.35)
        self.declare_parameter('local_goal_update_threshold', 0.25)
        self.declare_parameter('correction_wait_timeout', 20.0)
        self.declare_parameter('segmentation_node_name', 'goal_calculator_node')
        self.declare_parameter('corrected_local_goal_topic', '/corrected_local_goal')
        self.declare_parameter('global_frame', 'map')

        # Get Parameters
        self.local_arrival_thresh_sq_ = self.get_parameter('local_target_arrival_threshold').value ** 2
        self.local_goal_update_threshold_sq_ = self.get_parameter('local_goal_update_threshold').value ** 2
        self.correction_wait_timeout_ = self.get_parameter('correction_wait_timeout').value
        self.goal_calc_service_name_ = f"/{self.get_parameter('segmentation_node_name').value}/activate_segmentation"
        self.corrected_goal_topic_ = self.get_parameter('corrected_local_goal_topic').value
        self.global_frame_ = self.get_parameter('global_frame').value
        
        # State variables
        self.state_ = NavState.IDLE
        self.all_waypoints_ = []
        self.current_global_wp_index_ = -1
        self.latest_corrected_goal_ = None
        self.last_sent_local_goal_pose_ = None
        self.correction_wait_timer_ = None
        self._nav_to_pose_goal_handle = None

        # ROS Communications
        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)
        self._navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_calc_activation_client_ = self.create_client(SetBool, self.goal_calc_service_name_)
        self.segmenter_activation_client_ = self.create_client(SetBool, '/image_segmenter_node/activate_pipeline')
        self.get_waypoints_client_ = self.create_client(GetWaypoints, 'get_waypoints')
        self.proximity_sub_ = self.create_subscription(ProximityStatus, 'proximity_status', self.proximity_callback, 10)
        self.corrected_goal_sub_ = None
        self.nav_status_publisher_ = self.create_publisher(NavigationStatus, 'navigation_status', 10)

        self.get_logger().info('Orchestrator Initialized (Direct NavigateToPose Control).')
        self._change_state_and_process(NavState.FETCHING_WAYPOINTS)

    def _activate_pipeline(self, activate: bool):
        """Activates or deactivates the entire sensor fusion pipeline."""
        self.get_logger().info(f"Requesting full pipeline activation: {activate}")
        if self.segmenter_activation_client_.service_is_ready():
            self.segmenter_activation_client_.call_async(SetBool.Request(data=activate))
        else:
            self.get_logger().warn("Image segmenter activation service not ready.")
        
        if self.goal_calc_activation_client_.service_is_ready():
            self.goal_calc_activation_client_.call_async(SetBool.Request(data=activate))
        else:
            self.get_logger().warn("Goal calculator activation service not ready.")

    def _change_state_and_process(self, new_state: NavState):
        if self.state_ == new_state: return
        self.get_logger().info(f"STATE: {self.state_.name} -> {new_state.name}")
        old_state = self.state_
        self.state_ = new_state
        
        if old_state == NavState.AWAITING_LOCAL_CORRECTION:
            self._destroy_correction_wait_timer()
        
        if new_state not in [NavState.NAVIGATING_LOCAL, NavState.AWAITING_LOCAL_CORRECTION]:
            self._destroy_corrected_goal_subscriber()
        
        if self.state_ == NavState.FETCHING_WAYPOINTS:
            self.fetch_waypoints_from_server()
        elif self.state_ == NavState.NAVIGATING_GLOBAL:
            self._send_nav_to_pose_goal(self.all_waypoints_[self.current_global_wp_index_])
        elif self.state_ == NavState.AWAITING_LOCAL_CORRECTION:
            self._cancel_current_nav_goal()
            self._create_corrected_goal_subscriber()
            self._start_correction_wait_timer()
        elif self.state_ == NavState.WAYPOINT_COMPLETE:
            self._cancel_current_nav_goal()
            self._activate_pipeline(False)
            self.navigate_to_next_global_waypoint()
        elif self.state_ == NavState.MISSION_FAILED or self.state_ == NavState.MISSION_COMPLETE:
            self._cancel_current_nav_goal()
            self._activate_pipeline(False)
        
        self.publish_nav_status()

    def publish_nav_status(self):
        status_msg = NavigationStatus()
        status_msg.status_code = self.state_.value
        status_msg.status_message = self.state_.name
        status_msg.current_waypoint_index = self.current_global_wp_index_
        if self.state_ == NavState.MISSION_COMPLETE:
            status_msg.last_completed_waypoint_index = len(self.all_waypoints_) - 1
        else:
            status_msg.last_completed_waypoint_index = self.current_global_wp_index_ - 1
        is_local = self.state_ in [NavState.AWAITING_LOCAL_CORRECTION, NavState.NAVIGATING_LOCAL]
        status_msg.is_using_corrected_goal = is_local
        if is_local and self.latest_corrected_goal_:
            status_msg.current_goal_pose = self.latest_corrected_goal_
        elif self.current_global_wp_index_ != -1 and self.current_global_wp_index_ < len(self.all_waypoints_):
             status_msg.current_goal_pose = self.all_waypoints_[self.current_global_wp_index_]
        self.nav_status_publisher_.publish(status_msg)

    def fetch_waypoints_from_server(self):
        while not self.get_waypoints_client_.wait_for_service(timeout_sec=2.0): self.get_logger().info('Waypoint service not available, waiting...')
        self.get_waypoints_client_.call_async(GetWaypoints.Request()).add_done_callback(self.waypoints_response_callback)

    def waypoints_response_callback(self, future):
        try:
            response = future.result()
            if response.success and response.waypoints:
                self.all_waypoints_ = response.waypoints
                self.get_logger().info(f"Successfully fetched {len(self.all_waypoints_)} waypoints.")
                self.navigate_to_next_global_waypoint()
            else:
                self.get_logger().error(f"Failed to fetch waypoints: {response.message}")
                self._change_state_and_process(NavState.MISSION_FAILED)
        except Exception as e:
            self.get_logger().error(f"Waypoint service call exception: {e}")
            self._change_state_and_process(NavState.MISSION_FAILED)

    def navigate_to_next_global_waypoint(self):
        self.current_global_wp_index_ += 1
        if self.current_global_wp_index_ >= len(self.all_waypoints_):
            self._change_state_and_process(NavState.MISSION_COMPLETE)
        else:
            self._change_state_and_process(NavState.NAVIGATING_GLOBAL)

    def proximity_callback(self, msg: ProximityStatus):
        if self.state_ == NavState.NAVIGATING_GLOBAL and msg.is_within_activation_distance and msg.waypoint_index == self.current_global_wp_index_:
            self._activate_pipeline(True)
            self._change_state_and_process(NavState.AWAITING_LOCAL_CORRECTION)

    def corrected_goal_callback(self, msg: PoseStamped):
        if not msg.header.frame_id:
            self.latest_corrected_goal_ = None
            return
        self.latest_corrected_goal_ = msg
        self.publish_nav_status()
        if self.state_ == NavState.AWAITING_LOCAL_CORRECTION:
            self._change_state_and_process(NavState.NAVIGATING_LOCAL)
            self._send_nav_to_pose_goal(self.latest_corrected_goal_)
        elif self.state_ == NavState.NAVIGATING_LOCAL:
            is_new_goal_different = True
            if self.last_sent_local_goal_pose_:
                dist_sq = (msg.pose.position.x - self.last_sent_local_goal_pose_.pose.position.x)**2 + \
                          (msg.pose.position.y - self.last_sent_local_goal_pose_.pose.position.y)**2
                if dist_sq < self.local_goal_update_threshold_sq_:
                    is_new_goal_different = False
            if is_new_goal_different:
                self._send_nav_to_pose_goal(self.latest_corrected_goal_)

    def local_nav_feedback_cb(self, feedback: NavigateToPose.Feedback):
        if self.state_ != NavState.NAVIGATING_LOCAL or self.latest_corrected_goal_ is None:
            return
        robot_pose_odom = feedback.feedback.current_pose
        try:
            transform = self.tf_buffer_.lookup_transform(self.global_frame_, robot_pose_odom.header.frame_id, RclpyTime(), RclpyDuration(seconds=0.1))
            robot_pose_map = do_transform_pose_stamped(robot_pose_odom, transform)
            target_pos = self.latest_corrected_goal_.pose.position
            robot_pos = robot_pose_map.pose.position
            dist_sq = (robot_pos.x - target_pos.x)**2 + (robot_pos.y - target_pos.y)**2
            if dist_sq < self.local_arrival_thresh_sq_:
                self.get_logger().info(f"Local target arrival threshold met ({self.local_arrival_thresh_sq_**0.5:.2f}m).")
                self._change_state_and_process(NavState.WAYPOINT_COMPLETE)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Could not transform robot pose for arrival check: {e}", throttle_duration_sec=2.0)

    def _send_nav_to_pose_goal(self, target_pose: PoseStamped):
        if not self._navigate_to_pose_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("NavigateToPose server not available!")
            self._change_state_and_process(NavState.MISSION_FAILED)
            return
        self.last_sent_local_goal_pose_ = target_pose if self.state_ in [NavState.NAVIGATING_LOCAL, NavState.AWAITING_LOCAL_CORRECTION] else None
        goal_msg = NavigateToPose.Goal(pose=target_pose)
        self.get_logger().info(f"Sending goal to NavigateToPose server for target index {self.current_global_wp_index_}.")
        feedback_cb = self.local_nav_feedback_cb if self.state_ == NavState.NAVIGATING_LOCAL else None
        send_future = self._navigate_to_pose_client.send_goal_async(goal_msg, feedback_callback=feedback_cb)
        send_future.add_done_callback(self.nav_to_pose_goal_response_cb)

    def nav_to_pose_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("NavigateToPose goal was rejected.")
            self._change_state_and_process(NavState.MISSION_FAILED)
            return
        self._nav_to_pose_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(partial(self.nav_to_pose_result_cb, goal_id=goal_handle.goal_id))

    def nav_to_pose_result_cb(self, future, goal_id):
        if self._nav_to_pose_goal_handle and goal_id != self._nav_to_pose_goal_handle.goal_id:
            self.get_logger().debug(f"Received result for a superseded goal. Ignoring.")
            return

        status = future.result().status
        self._nav_to_pose_goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Goal for waypoint {self.current_global_wp_index_} succeeded.")
            self._change_state_and_process(NavState.WAYPOINT_COMPLETE)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Goal was canceled.")
            if self.state_ != NavState.WAYPOINT_COMPLETE and self.state_ != NavState.AWAITING_LOCAL_CORRECTION:
                 self.get_logger().warn("Goal Canceled unexpectedly! Treating as waypoint complete to be safe.")
                 self._change_state_and_process(NavState.WAYPOINT_COMPLETE)
            else:
                self.get_logger().info("Cancellation was expected due to a state transition. Normal operation.")
        elif status == GoalStatus.STATUS_ABORTED:
             self.get_logger().error(f"Navigation was aborted by the server. Mission failed.")
             self._change_state_and_process(NavState.MISSION_FAILED)
        else:
            self.get_logger().error(f"Navigation failed with unhandled status: {status}. Mission failed.")
            self._change_state_and_process(NavState.MISSION_FAILED)

    def _cancel_current_nav_goal(self):
        if self._nav_to_pose_goal_handle:
            self.get_logger().info(f"Requesting to cancel goal {self._nav_to_pose_goal_handle.goal_id.uuid}")
            self._nav_to_pose_goal_handle.cancel_goal_async()
        
    def _create_corrected_goal_subscriber(self):
        if self.corrected_goal_sub_ is None:
            qos = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
            self.corrected_goal_sub_ = self.create_subscription(PoseStamped, self.corrected_goal_topic_, self.corrected_goal_callback, qos)
    
    def _destroy_corrected_goal_subscriber(self):
        if self.corrected_goal_sub_ is not None:
            self.destroy_subscription(self.corrected_goal_sub_)
            self.corrected_goal_sub_ = None

    def _start_correction_wait_timer(self):
        self._destroy_correction_wait_timer()
        self.correction_wait_timer_ = self.create_timer(self.correction_wait_timeout_, self.correction_wait_timeout_cb)

    def _destroy_correction_wait_timer(self):
        if self.correction_wait_timer_:
            self.correction_wait_timer_.cancel()
            self.correction_wait_timer_ = None
    
    def correction_wait_timeout_cb(self):
        if self.state_ == NavState.AWAITING_LOCAL_CORRECTION:
            self.get_logger().warn("Timed out waiting for a corrected goal. Marking waypoint as complete and moving on.")
            self._change_state_and_process(NavState.WAYPOINT_COMPLETE)

    def destroy_node(self):
        self._activate_pipeline(False)
        self._cancel_current_nav_goal()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointFollowerCorrected()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if node:
            node.get_logger().error(f"Unhandled exception in orchestrator:\n{traceback.format_exc()}")
    finally:
        if node and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()