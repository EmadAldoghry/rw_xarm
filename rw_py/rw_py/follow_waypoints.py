#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time as RclpyTime
from rclpy.duration import Duration as RclpyDuration
from functools import partial
import traceback

from geometry_msgs.msg import PoseStamped, PoseArray
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import SetBool, Empty
from action_msgs.msg import GoalStatus
from enum import Enum, auto

from rw_interfaces.srv import GetWaypoints
from rw_interfaces.msg import ProximityStatus, NavigationStatus
from rw_interfaces.action import Manipulate
from xarm_msgs.srv import PlanExec, PlanJoint

# *** FIX: Removed PAUSING_FOR_MANIPULATION and RESUMING_NAVIGATION states ***
class NavState(Enum):
    IDLE = auto()
    FETCHING_WAYPOINTS = auto()
    NAVIGATING_GLOBAL = auto()
    AWAITING_LOCAL_CORRECTION = auto()
    NAVIGATING_LOCAL = auto()
    AWAITING_MANIPULATION_TRIGGER = auto()
    # PAUSING_FOR_MANIPULATION = auto() # Removed
    MANIPULATING = auto()
    HOMING_ARM = auto()
    # RESUMING_NAVIGATION = auto() # Removed
    WAYPOINT_COMPLETE = auto()
    MISSION_COMPLETE = auto()
    MISSION_FAILED = auto()

class WaypointFollowerCorrected(Node):
    def __init__(self):
        super().__init__('waypoint_follower_corrected_node')
        
        self.declare_parameter('local_target_arrival_threshold', 0.35)
        self.declare_parameter('local_goal_update_threshold', 0.2)
        self.declare_parameter('correction_wait_timeout', 20.0)
        self.declare_parameter('min_arm_poses', 9)

        self.local_arrival_thresh_sq_ = self.get_parameter('local_target_arrival_threshold').value ** 2
        self.local_goal_update_threshold_sq_ = self.get_parameter('local_goal_update_threshold').value ** 2
        self.correction_wait_timeout_ = self.get_parameter('correction_wait_timeout').value
        self.min_arm_poses_ = self.get_parameter('min_arm_poses').value
        
        self.state_ = NavState.IDLE
        self.all_waypoints_ = []
        self.current_global_wp_index_ = -1
        self.latest_corrected_goal_ = None
        self.last_sent_local_goal_pose_ = None
        self.correction_wait_timer_ = None
        self._nav_to_pose_goal_handle = None
        self.arm_poses_for_current_wp_ = None
        self.current_global_goal_pose_ = None

        self._navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._manipulation_client = ActionClient(self, Manipulate, 'do_manipulation')
        
        self.goal_calc_activation_client_ = self.create_client(SetBool, '/goal_calculator_node/activate_segmentation')
        self.segmenter_activation_client_ = self.create_client(SetBool, '/image_segmenter_node/activate_pipeline')
        self.get_waypoints_client_ = self.create_client(GetWaypoints, 'get_waypoints')
        
        # *** FIX: Remove unused pause/resume clients ***
        # self.nav2_pause_client_ = self.create_client(Empty, '/bt_navigator/pause_navigation')
        # self.nav2_resume_client_ = self.create_client(Empty, '/bt_navigator/resume_navigation')
        self.xarm_exec_client_ = self.create_client(PlanExec, '/xarm_exec_plan')
        self.xarm_home_plan_client_ = self.create_client(PlanJoint, '/xarm_joint_plan')

        self.proximity_sub_ = self.create_subscription(ProximityStatus, 'proximity_status', self.proximity_callback, 10)
        self.nav_status_publisher_ = self.create_publisher(NavigationStatus, 'navigation_status', 10)
        
        self.corrected_goal_sub_ = None
        self.arm_poses_sub_ = None

        self.get_logger().info('Orchestrator Initialized (With Manipulation Logic).')
        self._change_state_and_process(NavState.FETCHING_WAYPOINTS)

    def _change_state_and_process(self, new_state: NavState):
        if self.state_ == new_state: return
        self.get_logger().info(f"STATE: {self.state_.name} -> {new_state.name}")
        old_state = self.state_
        self.state_ = new_state
        
        if old_state == NavState.AWAITING_LOCAL_CORRECTION:
            self._destroy_correction_wait_timer()
        
        if new_state != NavState.AWAITING_LOCAL_CORRECTION:
            self._activate_correction_pipeline(False)
        
        if new_state not in [NavState.AWAITING_LOCAL_CORRECTION, NavState.NAVIGATING_LOCAL]:
            self._destroy_corrected_goal_subscriber()
        
        # *** FIX: Simplified logic for destroying arm pose subscriber ***
        if new_state not in [NavState.NAVIGATING_GLOBAL, NavState.NAVIGATING_LOCAL, NavState.AWAITING_MANIPULATION_TRIGGER]:
            self._destroy_arm_pose_subscriber()

        try:
            if self.state_ == NavState.FETCHING_WAYPOINTS:
                self.fetch_waypoints_from_server()
            elif self.state_ == NavState.NAVIGATING_GLOBAL:
                self._create_arm_pose_subscriber() 
                self._send_nav_to_pose_goal(self.all_waypoints_[self.current_global_wp_index_])
            elif self.state_ == NavState.AWAITING_LOCAL_CORRECTION:
                self._cancel_current_nav_goal()
                self._activate_correction_pipeline(True)
                self._create_corrected_goal_subscriber()
                self._start_correction_wait_timer()
            elif self.state_ == NavState.NAVIGATING_LOCAL:
                self._create_arm_pose_subscriber()
            elif self.state_ == NavState.AWAITING_MANIPULATION_TRIGGER:
                self.get_logger().info("Arrived at corrected location. Triggering manipulation.")
                # *** FIX: Go directly to manipulation. Current nav goal is already complete. ***
                self._cancel_current_nav_goal() # Ensure it's stopped
                self._change_state_and_process(NavState.MANIPULATING)
            elif self.state_ == NavState.MANIPULATING:
                self.send_manipulation_goal()
            elif self.state_ == NavState.HOMING_ARM:
                self.home_the_arm()
            elif self.state_ == NavState.WAYPOINT_COMPLETE:
                self.navigate_to_next_global_waypoint()
            elif self.state_ in [NavState.MISSION_FAILED, NavState.MISSION_COMPLETE]:
                self._cancel_current_nav_goal()

        except Exception as e:
            self.get_logger().error(f"Error processing new state {new_state.name}: {e}\n{traceback.format_exc()}")
            self._change_state_and_process(NavState.MISSION_FAILED)
        
        self.publish_nav_status()

    def proximity_callback(self, msg: ProximityStatus):
        if self.state_ == NavState.NAVIGATING_GLOBAL and msg.is_within_activation_distance and msg.waypoint_index == self.current_global_wp_index_:
            self._change_state_and_process(NavState.AWAITING_LOCAL_CORRECTION)

    def corrected_goal_callback(self, msg: PoseStamped):
        if not msg.header.frame_id: return
        
        if self.current_global_goal_pose_:
            msg.pose.orientation = self.current_global_goal_pose_.pose.orientation
        
        self.latest_corrected_goal_ = msg
        if self.state_ == NavState.AWAITING_LOCAL_CORRECTION:
            self._change_state_and_process(NavState.NAVIGATING_LOCAL)
            self._send_nav_to_pose_goal(self.latest_corrected_goal_)
            
    def arm_poses_callback(self, msg: PoseArray):
        if self.state_ not in [NavState.NAVIGATING_GLOBAL, NavState.NAVIGATING_LOCAL]:
            return

        if len(msg.poses) >= self.min_arm_poses_:
            self.get_logger().info(f"Received {len(msg.poses)} arm poses, meeting threshold. Stopping navigation and triggering manipulation.")
            self.arm_poses_for_current_wp_ = msg.poses
            self._destroy_arm_pose_subscriber()
            self._cancel_current_nav_goal()  # Stop navigation immediately
            self._change_state_and_process(NavState.MANIPULATING)

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

    def nav_to_pose_result_cb(self, future, goal_id):
        if self._nav_to_pose_goal_handle and goal_id != self._nav_to_pose_goal_handle.goal_id:
            return
        
        result = future.result()
        if not result:
            self.get_logger().error("Navigation goal future came back with no result. Mission failed.")
            self._change_state_and_process(NavState.MISSION_FAILED)
            return

        status = result.status
        self._nav_to_pose_goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.state_ in [NavState.NAVIGATING_GLOBAL, NavState.NAVIGATING_LOCAL]:
                # If arm poses have not been received yet, wait for them.
                if self.arm_poses_for_current_wp_ is None:
                    self._change_state_and_process(NavState.AWAITING_MANIPULATION_TRIGGER)
                else: # If poses were received while navigating, go straight to manipulating.
                    self._change_state_and_process(NavState.MANIPULATING)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Navigation goal was canceled, as expected for state transition.")
        else:
            self.get_logger().error(f"Navigation failed with status {status}. Mission failed.")
            self._change_state_and_process(NavState.MISSION_FAILED)

    def manipulation_done_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("Manipulation action succeeded.")
            self._change_state_and_process(NavState.HOMING_ARM)
        else:
            self.get_logger().error(f"Manipulation action failed: {result.message}")
            self._change_state_and_process(NavState.MISSION_FAILED)
            
    def fetch_waypoints_from_server(self):
        while not self.get_waypoints_client_.wait_for_service(timeout_sec=2.0): self.get_logger().info('Waypoint service not available, waiting...')
        self.get_waypoints_client_.call_async(GetWaypoints.Request()).add_done_callback(self.waypoints_response_callback)

    def navigate_to_next_global_waypoint(self):
        self.arm_poses_for_current_wp_ = None
        self.current_global_wp_index_ += 1
        if self.current_global_wp_index_ >= len(self.all_waypoints_):
            self._change_state_and_process(NavState.MISSION_COMPLETE)
        else:
            self._change_state_and_process(NavState.NAVIGATING_GLOBAL)

    def send_manipulation_goal(self):
        if not self._manipulation_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Manipulation action server not available.")
            self._change_state_and_process(NavState.MISSION_FAILED)
            return

        goal_msg = Manipulate.Goal(poses=self.arm_poses_for_current_wp_)
        send_goal_future = self._manipulation_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.manipulation_goal_response_callback)
        
    def manipulation_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Manipulation goal was rejected.")
            self._change_state_and_process(NavState.MISSION_FAILED)
            return
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.manipulation_done_callback)

    def home_the_arm(self):
        self.get_logger().info("Homing the arm...")
        home_joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        req = PlanJoint.Request(target=home_joint_state)
        
        if not self.xarm_home_plan_client_.wait_for_service(timeout_sec=2.0) or \
           not self.xarm_exec_client_.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Arm planning/execution services not available for homing.")
            self._change_state_and_process(NavState.MISSION_FAILED)
            return
        
        plan_future = self.xarm_home_plan_client_.call_async(req)
        rclpy.spin_until_future_complete(self, plan_future)
        if not plan_future.result() or not plan_future.result().success:
            self.get_logger().error("Failed to plan homing motion.")
            self._change_state_and_process(NavState.MISSION_FAILED)
            return
            
        exec_future = self.xarm_exec_client_.call_async(PlanExec.Request(wait=True))
        rclpy.spin_until_future_complete(self, exec_future)
        if not exec_future.result() or not exec_future.result().success:
            self.get_logger().error("Failed to execute homing motion.")
            self._change_state_and_process(NavState.MISSION_FAILED)
            return

        self.get_logger().info("Arm successfully moved to home position.")
        # *** FIX: Go directly to the next waypoint ***
        self._change_state_and_process(NavState.WAYPOINT_COMPLETE)
        
    def _send_nav_to_pose_goal(self, target_pose: PoseStamped):
        if not self._navigate_to_pose_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("NavigateToPose server not available!")
            self._change_state_and_process(NavState.MISSION_FAILED)
            return
        
        self.last_sent_local_goal_pose_ = target_pose if self.state_ == NavState.NAVIGATING_LOCAL else None
        
        goal_msg = NavigateToPose.Goal(pose=target_pose)
        self.get_logger().info(f"Sending goal to Nav2 for waypoint {self.current_global_wp_index_}.")
        send_future = self._navigate_to_pose_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.nav_to_pose_goal_response_cb)

    def nav_to_pose_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("NavigateToPose goal rejected.")
            self._change_state_and_process(NavState.MISSION_FAILED)
            return
        self._nav_to_pose_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(partial(self.nav_to_pose_result_cb, goal_id=goal_handle.goal_id))

    def _cancel_current_nav_goal(self):
        if self._nav_to_pose_goal_handle:
            status = self._nav_to_pose_goal_handle.status
            if status == GoalStatus.STATUS_ACCEPTED or status == GoalStatus.STATUS_EXECUTING:
                self.get_logger().info(f"Requesting to cancel active Nav2 goal (Status: {status}).")
                self._nav_to_pose_goal_handle.cancel_goal_async()
        self._nav_to_pose_goal_handle = None
        
    def _activate_correction_pipeline(self, activate: bool):
        self.get_logger().info(f"Requesting correction pipeline activation: {activate}")
        if self.segmenter_activation_client_.service_is_ready():
            self.segmenter_activation_client_.call_async(SetBool.Request(data=activate))
        if self.goal_calc_activation_client_.service_is_ready():
            self.goal_calc_activation_client_.call_async(SetBool.Request(data=activate))

    def _create_corrected_goal_subscriber(self):
        if self.corrected_goal_sub_ is None:
            qos = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
            self.corrected_goal_sub_ = self.create_subscription(PoseStamped, '/corrected_local_goal', self.corrected_goal_callback, qos)
    
    def _destroy_corrected_goal_subscriber(self):
        if self.corrected_goal_sub_ is not None:
            self.destroy_subscription(self.corrected_goal_sub_); self.corrected_goal_sub_ = None

    def _create_arm_pose_subscriber(self):
        if self.arm_poses_sub_ is None:
            self.get_logger().info("Listening for arm poses to trigger manipulation...")
            self.arm_poses_sub_ = self.create_subscription(PoseArray, '/arm_path_poses', self.arm_poses_callback, 10)
    
    def _destroy_arm_pose_subscriber(self):
        if self.arm_poses_sub_ is not None:
            self.get_logger().info("Stopping listener for arm poses.")
            self.destroy_subscription(self.arm_poses_sub_); self.arm_poses_sub_ = None

    def _start_correction_wait_timer(self):
        self._destroy_correction_wait_timer()
        self.correction_wait_timer_ = self.create_timer(self.correction_wait_timeout_, self.correction_wait_timeout_cb)

    def _destroy_correction_wait_timer(self):
        if self.correction_wait_timer_:
            self.correction_wait_timer_.cancel(); self.correction_wait_timer_ = None
    
    def correction_wait_timeout_cb(self):
        if self.state_ == NavState.AWAITING_LOCAL_CORRECTION:
            self.get_logger().warn("Timed out waiting for corrected goal. Marking waypoint as complete and moving on.")
            self._change_state_and_process(NavState.WAYPOINT_COMPLETE)
    
    def publish_nav_status(self):
        status_msg = NavigationStatus(status_code=self.state_.value, status_message=self.state_.name, current_waypoint_index=self.current_global_wp_index_)
        if self.state_ == NavState.MISSION_COMPLETE:
            status_msg.last_completed_waypoint_index = len(self.all_waypoints_) - 1 if self.all_waypoints_ else -1
        else:
            status_msg.last_completed_waypoint_index = self.current_global_wp_index_ - 1
        is_local = self.state_ in [NavState.AWAITING_LOCAL_CORRECTION, NavState.NAVIGATING_LOCAL, NavState.AWAITING_MANIPULATION_TRIGGER]
        status_msg.is_using_corrected_goal = is_local
        
        if self.state_ == NavState.NAVIGATING_GLOBAL:
            self.current_global_goal_pose_ = self.all_waypoints_[self.current_global_wp_index_]
        
        if is_local and self.latest_corrected_goal_:
            status_msg.current_goal_pose = self.latest_corrected_goal_
        elif self.current_global_goal_pose_:
             status_msg.current_goal_pose = self.current_global_goal_pose_
        self.nav_status_publisher_.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerCorrected()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        node.get_logger().info("Orchestrator shutting down.")
    except Exception:
        node.get_logger().error(f"Unhandled exception in orchestrator:\n{traceback.format_exc()}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()