# FILE: rw_py/rw_py/follow_waypoints.py
# Corrected and Final Version v3

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

from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import SetBool, Trigger
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import PointCloud2
from enum import Enum, auto

# Custom interfaces
from rw_interfaces.srv import GetWaypoints
from rw_interfaces.msg import ProximityStatus, NavigationStatus
from rw_interfaces.action import Manipulate

# Arm control interfaces
from xarm_msgs.srv import PlanJoint, PlanExec

class NavState(Enum):
    IDLE = 0
    FETCHING_WAYPOINTS = 1
    NAVIGATING_GLOBAL = 2
    AWAITING_LOCAL_CORRECTION = 3
    NAVIGATING_LOCAL = 4
    WAYPOINT_COMPLETE = 5
    MISSION_COMPLETE = 6
    MISSION_FAILED = 7
    PAUSING_FOR_MANIPULATION = 8
    PERFORMING_MANIPULATION = 9
    HOMING_ARM = 10
    RESUMING_NAVIGATION = 11

class WaypointFollowerCorrected(Node):
    def __init__(self):
        super().__init__('waypoint_follower_corrected_node')
        
        self.declare_parameter('local_target_arrival_threshold', 0.35)
        self.declare_parameter('local_goal_update_threshold', 0.25)
        self.declare_parameter('correction_wait_timeout', 20.0)
        self.declare_parameter('segmentation_node_name', 'goal_calculator_node')

        self.local_arrival_thresh_sq_ = self.get_parameter('local_target_arrival_threshold').value ** 2
        self.local_goal_update_threshold_sq_ = self.get_parameter('local_goal_update_threshold').value ** 2
        self.correction_wait_timeout_ = self.get_parameter('correction_wait_timeout').value
        self.goal_calc_service_name_ = f"/{self.get_parameter('segmentation_node_name').value}/activate_segmentation"
        self.global_frame_ = 'map'
        
        self.state_ = NavState.IDLE
        self.all_waypoints_ = []
        self.current_global_wp_index_ = -1
        self.latest_corrected_goal_ = None
        self.last_sent_local_goal_pose_ = None
        self.correction_wait_timer_ = None
        self._nav_to_pose_goal_handle = None
        self.manipulation_done_ = False
        self.latest_arm_poses_ = None

        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)
        
        self._navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_waypoints_client_ = self.create_client(GetWaypoints, 'get_waypoints')
        self.proximity_sub_ = self.create_subscription(ProximityStatus, 'proximity_status', self.proximity_callback, 10)
        self.nav_status_publisher_ = self.create_publisher(NavigationStatus, 'navigation_status', 10)
        self.corrected_goal_sub_ = None

        self.nav2_pause_client_ = self.create_client(Trigger, '/bt_navigator/pause')
        self.nav2_resume_client_ = self.create_client(Trigger, '/bt_navigator/resume')
        
        self.arm_poses_sub_ = self.create_subscription(PoseArray, '/arm_path_poses', self.arm_poses_callback, 10)
        self._manipulate_action_client = ActionClient(self, Manipulate, 'manipulate_path')
        self.arm_joint_plan_client_ = self.create_client(PlanJoint, '/xarm_joint_plan')
        self.arm_exec_plan_client_ = self.create_client(PlanExec, '/xarm_exec_plan')

        self.crack_path_sub_ = self.create_subscription(PointCloud2, '/projected_non_ground_points', self.crack_path_callback, 10)
        
        # <<< NEW/RE-INTEGRATED: Service clients for activating the correction pipeline
        self.goal_calc_activation_client_ = self.create_client(SetBool, self.goal_calc_service_name_)
        self.segmenter_activation_client_ = self.create_client(SetBool, '/image_segmenter_node/activate_pipeline')

        self.get_logger().info('Orchestrator Initialized (With Manipulation Logic).')
        self._change_state_and_process(NavState.FETCHING_WAYPOINTS)

    # <<< NEW/RE-INTEGRATED: Method to activate/deactivate correction pipeline
    def _activate_correction_pipeline(self, activate: bool):
        self.get_logger().info(f"Requesting correction pipeline activation: {activate}")
        if self.segmenter_activation_client_.wait_for_service(timeout_sec=1.0):
            self.segmenter_activation_client_.call_async(SetBool.Request(data=activate))
        else: self.get_logger().warn("Image segmenter activation service not ready.")
        
        if self.goal_calc_activation_client_.wait_for_service(timeout_sec=1.0):
            self.goal_calc_activation_client_.call_async(SetBool.Request(data=activate))
        else: self.get_logger().warn("Goal calculator activation service not ready.")

    def crack_path_callback(self, msg: PointCloud2):
        is_at_waypoint = self.state_ == NavState.WAYPOINT_COMPLETE
        if not self.manipulation_done_ and is_at_waypoint and msg.data:
            self.get_logger().info("Robot is at a waypoint and a crack path is available. Initiating manipulation.")
            self.manipulation_done_ = True
            self._change_state_and_process(NavState.PAUSING_FOR_MANIPULATION)

    def arm_poses_callback(self, msg: PoseArray):
        if msg.poses: self.latest_arm_poses_ = msg

    def proximity_callback(self, msg: ProximityStatus):
        if self.state_ == NavState.NAVIGATING_GLOBAL and msg.is_within_activation_distance and msg.waypoint_index == self.current_global_wp_index_:
            self._activate_correction_pipeline(True)
            self._change_state_and_process(NavState.AWAITING_LOCAL_CORRECTION)

    def corrected_goal_callback(self, msg: PoseStamped):
        # ... this function remains unchanged ...
        if not msg.header.frame_id: self.latest_corrected_goal_ = None; return
        self.latest_corrected_goal_ = msg; self.publish_nav_status()
        if self.state_ == NavState.AWAITING_LOCAL_CORRECTION:
            self._change_state_and_process(NavState.NAVIGATING_LOCAL)
            self._send_nav_to_pose_goal(self.latest_corrected_goal_)
        elif self.state_ == NavState.NAVIGATING_LOCAL:
            is_new_goal_different = True
            if self.last_sent_local_goal_pose_:
                dist_sq = (msg.pose.position.x - self.last_sent_local_goal_pose_.pose.position.x)**2 + (msg.pose.position.y - self.last_sent_local_goal_pose_.pose.position.y)**2
                if dist_sq < self.local_goal_update_threshold_sq_: is_new_goal_different = False
            if is_new_goal_different: self._send_nav_to_pose_goal(self.latest_corrected_goal_)

    def _change_state_and_process(self, new_state: NavState):
        if self.state_ == new_state: return
        self.get_logger().info(f"STATE: {self.state_.name} -> {new_state.name}")
        old_state = self.state_; self.state_ = new_state
        if old_state == NavState.AWAITING_LOCAL_CORRECTION: self._destroy_correction_wait_timer()
        if new_state not in [NavState.NAVIGATING_LOCAL, NavState.AWAITING_LOCAL_CORRECTION]: self._destroy_corrected_goal_subscriber()
        if new_state != NavState.AWAITING_LOCAL_CORRECTION: self._activate_correction_pipeline(False)
        
        if self.state_ == NavState.FETCHING_WAYPOINTS: self.fetch_waypoints_from_server()
        elif self.state_ == NavState.NAVIGATING_GLOBAL: self._send_nav_to_pose_goal(self.all_waypoints_[self.current_global_wp_index_])
        elif self.state_ == NavState.AWAITING_LOCAL_CORRECTION: self._cancel_current_nav_goal(); self._create_corrected_goal_subscriber(); self._start_correction_wait_timer()
        elif self.state_ == NavState.WAYPOINT_COMPLETE: self._cancel_current_nav_goal(); self.check_for_manipulation_or_proceed()
        elif self.state_ == NavState.MISSION_FAILED or self.state_ == NavState.MISSION_COMPLETE: self._cancel_current_nav_goal()
        elif self.state_ == NavState.PAUSING_FOR_MANIPULATION: self.pause_navigation()
        elif self.state_ == NavState.PERFORMING_MANIPULATION: self.perform_manipulation()
        elif self.state_ == NavState.HOMING_ARM: self.home_arm()
        elif self.state_ == NavState.RESUMING_NAVIGATION: self.resume_navigation()
        self.publish_nav_status()

    def check_for_manipulation_or_proceed(self):
        if not self.manipulation_done_ and self.latest_arm_poses_:
            self.get_logger().info("Waypoint complete and arm poses are available. Starting manipulation.")
            self.manipulation_done_ = True
            self._change_state_and_process(NavState.PAUSING_FOR_MANIPULATION)
        else:
            self.get_logger().info("Waypoint complete. No arm poses or manipulation already done. Proceeding.")
            self.navigate_to_next_global_waypoint()

    def fetch_waypoints_from_server(self):
        # ... remains unchanged ...
        while not self.get_waypoints_client_.wait_for_service(timeout_sec=2.0): self.get_logger().info('Waypoint service waiting...')
        self.get_waypoints_client_.call_async(GetWaypoints.Request()).add_done_callback(self.waypoints_response_callback)

    def waypoints_response_callback(self, future):
        # ... remains unchanged ...
        try:
            response = future.result()
            if response.success and response.waypoints: self.all_waypoints_ = response.waypoints; self.get_logger().info(f"Successfully fetched {len(self.all_waypoints_)} waypoints."); self.navigate_to_next_global_waypoint()
            else: self.get_logger().error(f"Failed to fetch waypoints: {response.message}"); self._change_state_and_process(NavState.MISSION_FAILED)
        except Exception as e: self.get_logger().error(f"Error in waypoint response: {e}"); self._change_state_and_process(NavState.MISSION_FAILED)

    def navigate_to_next_global_waypoint(self):
        self.current_global_wp_index_ += 1
        if self.current_global_wp_index_ >= len(self.all_waypoints_): self._change_state_and_process(NavState.MISSION_COMPLETE)
        else: self._change_state_and_process(NavState.NAVIGATING_GLOBAL)
    
    def pause_navigation(self):
        self.get_logger().info("Requesting to pause navigation...")
        if not self.nav2_pause_client_.wait_for_service(timeout_sec=5.0): # Increased timeout
            self.get_logger().error("Nav2 pause service not available. Skipping manipulation."); self._change_state_and_process(NavState.HOMING_ARM); return
        self.nav2_pause_client_.call_async(Trigger.Request()).add_done_callback(self.pause_nav_response_callback)
    
    def pause_nav_response_callback(self, future):
        try:
            response = future.result()
            if response.success: self.get_logger().info("Navigation paused successfully."); self._change_state_and_process(NavState.PERFORMING_MANIPULATION)
            else: self.get_logger().error(f"Failed to pause navigation: {response.message}"); self._change_state_and_process(NavState.HOMING_ARM)
        except Exception as e: self.get_logger().error(f"Exception while pausing navigation: {e}"); self._change_state_and_process(NavState.HOMING_ARM)

    def perform_manipulation(self):
        if self.latest_arm_poses_ is None:
            self.get_logger().warn("No arm poses available to perform manipulation. Homing arm and resuming."); self._change_state_and_process(NavState.HOMING_ARM); return
        
        self.get_logger().info("Sending goal to manipulation action server..."); goal_msg = Manipulate.Goal(pose_array=self.latest_arm_poses_)
        if not self._manipulate_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Manipulation action server not available."); self._change_state_and_process(NavState.HOMING_ARM); return
        self._manipulate_action_client.send_goal_async(goal_msg).add_done_callback(self.manipulation_goal_response_callback)

    def manipulation_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Manipulation goal rejected.'); self._change_state_and_process(NavState.HOMING_ARM); return
        self.get_logger().info('Manipulation goal accepted.'); goal_handle.get_result_async().add_done_callback(self.manipulation_result_callback)

    def manipulation_result_callback(self, future):
        result = future.result().result
        if result.success: self.get_logger().info(f"Manipulation succeeded: {result.message}")
        else: self.get_logger().error(f"Manipulation failed: {result.message}")
        self._change_state_and_process(NavState.HOMING_ARM)

    def home_arm(self):
        self.get_logger().info("Homing the arm..."); home_joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if not self.arm_joint_plan_client_.wait_for_service(timeout_sec=2.0): self.get_logger().error("Arm joint plan service not available."); self._change_state_and_process(NavState.RESUMING_NAVIGATION); return
        self.arm_joint_plan_client_.call_async(PlanJoint.Request(target=home_joint_state)).add_done_callback(self.arm_home_plan_callback)

    def arm_home_plan_callback(self, future):
        try:
            if future.result().success:
                if not self.arm_exec_plan_client_.wait_for_service(timeout_sec=2.0): self.get_logger().error("Arm exec service not available."); self._change_state_and_process(NavState.RESUMING_NAVIGATION); return
                self.arm_exec_plan_client_.call_async(PlanExec.Request(wait=True)).add_done_callback(self.arm_home_exec_callback)
            else: self.get_logger().error("Failed to plan arm move to home."); self._change_state_and_process(NavState.RESUMING_NAVIGATION)
        except Exception as e: self.get_logger().error(f"Exception in arm home planning: {e}"); self._change_state_and_process(NavState.RESUMING_NAVIGATION)

    def arm_home_exec_callback(self, future):
        try:
            if future.result().success: self.get_logger().info("Arm successfully moved to home position.")
            else: self.get_logger().error("Execution of arm move to home failed.")
        except Exception as e: self.get_logger().error(f"Exception in arm home execution: {e}")
        finally: self._change_state_and_process(NavState.RESUMING_NAVIGATION)

    def resume_navigation(self):
        self.get_logger().info("Requesting to resume navigation...")
        if not self.nav2_resume_client_.wait_for_service(timeout_sec=5.0): # Increased timeout
            self.get_logger().error("Nav2 resume service not available."); self._change_state_and_process(NavState.MISSION_FAILED); return
        self.nav2_resume_client_.call_async(Trigger.Request()).add_done_callback(self.resume_nav_response_callback)
    
    def resume_nav_response_callback(self, future):
        try:
            if future.result().success:
                self.get_logger().info("Navigation resumed successfully.")
                self.navigate_to_next_global_waypoint() # <<< CHANGE: always proceed to next
            else: self.get_logger().error(f"Failed to resume navigation: {future.result().message}"); self._change_state_and_process(NavState.MISSION_FAILED)
        except Exception as e: self.get_logger().error(f"Exception while resuming navigation: {e}"); self._change_state_and_process(NavState.MISSION_FAILED)

    def _send_nav_to_pose_goal(self, target_pose: PoseStamped):
        if not self._navigate_to_pose_client.wait_for_server(timeout_sec=3.0): self._change_state_and_process(NavState.MISSION_FAILED); return
        self.last_sent_local_goal_pose_ = target_pose if self.state_ in [NavState.NAVIGATING_LOCAL, NavState.AWAITING_LOCAL_CORRECTION] else None
        self._navigate_to_pose_client.send_goal_async(NavigateToPose.Goal(pose=target_pose), feedback_callback=self.local_nav_feedback_cb).add_done_callback(self.nav_to_pose_goal_response_cb)

    def local_nav_feedback_cb(self, feedback: NavigateToPose.Feedback):
        if self.state_ != NavState.NAVIGATING_LOCAL or self.latest_corrected_goal_ is None: return
        try:
            transform = self.tf_buffer_.lookup_transform(self.global_frame_, feedback.feedback.current_pose.header.frame_id, RclpyTime(), RclpyDuration(seconds=0.1))
            robot_pose_map = do_transform_pose_stamped(feedback.feedback.current_pose, transform)
            dist_sq = (robot_pose_map.pose.position.x - self.latest_corrected_goal_.pose.position.x)**2 + (robot_pose_map.pose.position.y - self.latest_corrected_goal_.pose.position.y)**2
            if dist_sq < self.local_arrival_thresh_sq_: self._change_state_and_process(NavState.WAYPOINT_COMPLETE)
        except Exception as e: self.get_logger().warn(f"TF error in local_nav_feedback: {e}", throttle_duration_sec=2.0)

    def nav_to_pose_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted: self._change_state_and_process(NavState.MISSION_FAILED); return
        self._nav_to_pose_goal_handle = goal_handle; goal_handle.get_result_async().add_done_callback(partial(self.nav_to_pose_result_cb, goal_id=goal_handle.goal_id))

    def nav_to_pose_result_cb(self, future, goal_id):
        if self._nav_to_pose_goal_handle and goal_id != self._nav_to_pose_goal_handle.goal_id: return
        status = future.result().status; self._nav_to_pose_goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED: self._change_state_and_process(NavState.WAYPOINT_COMPLETE)
        elif status == GoalStatus.STATUS_ABORTED: self._change_state_and_process(NavState.MISSION_FAILED)
        elif status == GoalStatus.STATUS_CANCELED: pass
        else: self._change_state_and_process(NavState.MISSION_FAILED)

    def _cancel_current_nav_goal(self):
        if self._nav_to_pose_goal_handle:
            status = self._nav_to_pose_goal_handle.status
            if status in [GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING]:
                self.get_logger().info(f"Canceling active Nav2 goal.")
                self._nav_to_pose_goal_handle.cancel_goal_async()
    
    def publish_nav_status(self):
        status_msg = NavigationStatus(); status_msg.status_code = self.state_.value; status_msg.status_message = self.state_.name; status_msg.current_waypoint_index = self.current_global_wp_index_
        status_msg.last_completed_waypoint_index = (len(self.all_waypoints_) - 1) if self.state_ == NavState.MISSION_COMPLETE else (self.current_global_wp_index_ - 1)
        self.nav_status_publisher_.publish(status_msg)

    def _create_corrected_goal_subscriber(self):
        if self.corrected_goal_sub_ is None: qos = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL); self.corrected_goal_sub_ = self.create_subscription(PoseStamped, '/corrected_local_goal', self.corrected_goal_callback, qos)
    
    def _destroy_corrected_goal_subscriber(self):
        if self.corrected_goal_sub_ is not None: self.destroy_subscription(self.corrected_goal_sub_); self.corrected_goal_sub_ = None

    def _start_correction_wait_timer(self): self._destroy_correction_wait_timer(); self.correction_wait_timer_ = self.create_timer(self.correction_wait_timeout_, self.correction_wait_timeout_cb)

    def _destroy_correction_wait_timer(self):
        if self.correction_wait_timer_: self.correction_wait_timer_.cancel(); self.correction_wait_timer_ = None
    
    def correction_wait_timeout_cb(self):
        if self.state_ == NavState.AWAITING_LOCAL_CORRECTION: self.get_logger().warn("Timed out waiting for a corrected goal. Marking waypoint as complete."); self._change_state_and_process(NavState.WAYPOINT_COMPLETE)

    def destroy_node(self): self._cancel_current_nav_goal(); super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointFollowerCorrected()
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    except Exception:
        if node: node.get_logger().error(f"Unhandled exception in orchestrator:\n{traceback.format_exc()}")
    finally:
        if node and rclpy.ok(): node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()