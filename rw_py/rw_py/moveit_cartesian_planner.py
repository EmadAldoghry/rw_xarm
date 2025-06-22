# FILE: rw/rw/moveit_cartesian_planner.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from xarm_msgs.srv import PlanPose, PlanJoint, PlanExec, PlanMultiStraight
import sys
import time

CARTESIAN_PLAN_SERVICE = '/xarm_cartesian_plan' 
POSE_PLAN_SERVICE = '/xarm_pose_plan'
JOINT_PLAN_SERVICE = '/xarm_joint_plan'
EXEC_PLAN_SERVICE = '/xarm_exec_plan'
POSE_ARRAY_TOPIC = '/arm_path_poses'

class CartesianPlannerClient(Node):

    def __init__(self):
        super().__init__('xarm6_cartesian_planner_client')
        self.get_logger().info("Initializing Cartesian Planner Client...")

        self.cartesian_plan_client = self.create_client(PlanMultiStraight, CARTESIAN_PLAN_SERVICE)
        self.pose_plan_client = self.create_client(PlanPose, POSE_PLAN_SERVICE)
        self.joint_plan_client = self.create_client(PlanJoint, JOINT_PLAN_SERVICE)
        self.exec_plan_client = self.create_client(PlanExec, EXEC_PLAN_SERVICE)

        services_to_wait_for = {
            self.cartesian_plan_client: CARTESIAN_PLAN_SERVICE,
            self.pose_plan_client: POSE_PLAN_SERVICE,
            self.joint_plan_client: JOINT_PLAN_SERVICE,
            self.exec_plan_client: EXEC_PLAN_SERVICE,
        }
        for client, name in services_to_wait_for.items():
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'Service {name} not available.')
                rclpy.shutdown()
                sys.exit(1)

        self.get_logger().info("Service clients created and ready.")

        self.pose_array_sub = self.create_subscription(PoseArray, POSE_ARRAY_TOPIC, self.pose_array_callback, 10)
        self.pose_array = []
        self.is_new_path_available = False

    def pose_array_callback(self, msg):
        if self.is_new_path_available:
            return
        self.pose_array = msg.poses
        self.is_new_path_available = True
        self.get_logger().info(f"Received a new path with {len(self.pose_array)} poses from topic '{POSE_ARRAY_TOPIC}'")
        self.destroy_subscription(self.pose_array_sub)

    def plan_and_execute(self):
        if not self.pose_array:
            self.get_logger().warn("Pose array is empty. Cannot plan.")
            return False

        # --- STEP 1: Move to a known, collision-free "ready" position first.
        # Find these values using the RViz MotionPlanning panel interactively!
        ready_joint_state = [0.0, 0.2, -1.2, 0.0, 1.0, 0.0] 
        
        self.get_logger().info(f"Planning joint move to 'ready' state: {ready_joint_state}")
        if not self.plan_and_exec_joint(ready_joint_state):
             self.get_logger().error("Failed to move to 'ready' joint state.")
             return False
        
        self.get_logger().info("Successfully moved to 'ready' state. Waiting a moment...")
        time.sleep(1.0) 

        # --- STEP 2: Plan a non-Cartesian (joint-space) move to the first waypoint of the path.
        # This allows MoveIt to find a flexible path to the start of the crack.
        first_waypoint = self.pose_array[0]
        self.get_logger().info("Planning move to the first path waypoint...")
        if not self.plan_and_exec_pose(first_waypoint):
            self.get_logger().error("Failed to plan to the first waypoint of the path.")
            return False

        self.get_logger().info("Successfully at the start of the crack path. Waiting a moment...")
        time.sleep(1.0)

        # --- STEP 3: Plan and execute the rest of the path as a single Cartesian trajectory.
        self.get_logger().info("Planning the full Cartesian path...")
        cartesian_req = PlanMultiStraight.Request()
        # Pass the whole array. MoveIt will start the path from its current position (which is the first waypoint).
        cartesian_req.targets = self.pose_array
        
        cartesian_plan_future = self.cartesian_plan_client.call_async(cartesian_req)
        rclpy.spin_until_future_complete(self, cartesian_plan_future)
        
        if not cartesian_plan_future.result() or not cartesian_plan_future.result().success:
            self.get_logger().error("Cartesian path planning failed for the full sequence!")
            return False
        
        self.get_logger().info("Cartesian path planning successful.")
        
        self.get_logger().info("Executing the Cartesian path...")
        if not self.execute_plan():
            self.get_logger().error("Execution of the Cartesian path failed.")
            return False

        self.get_logger().info("Successfully executed the crack-filling path.")
        return True

    def plan_and_exec_joint(self, joint_state):
        joint_plan_req = PlanJoint.Request()
        joint_plan_req.target = joint_state
        joint_plan_future = self.joint_plan_client.call_async(joint_plan_req)
        rclpy.spin_until_future_complete(self, joint_plan_future)

        if not joint_plan_future.result() or not joint_plan_future.result().success:
            self.get_logger().error(f"Failed to plan to joint state: {joint_state}")
            return False
        
        return self.execute_plan()

    # <<< NEW HELPER FUNCTION for planning to a pose target
    def plan_and_exec_pose(self, pose):
        pose_plan_req = PlanPose.Request()
        pose_plan_req.target = pose
        pose_plan_future = self.pose_plan_client.call_async(pose_plan_req)
        rclpy.spin_until_future_complete(self, pose_plan_future)

        if not pose_plan_future.result() or not pose_plan_future.result().success:
            self.get_logger().error(f"Failed to plan to pose: {pose}")
            return False

        return self.execute_plan()

    def execute_plan(self, wait=True):
        exec_req = PlanExec.Request()
        exec_req.wait = wait
        exec_future = self.exec_plan_client.call_async(exec_req)
        rclpy.spin_until_future_complete(self, exec_future)
        if exec_future.result() and exec_future.result().success:
            return True
        else:
            self.get_logger().error(f'Exception or failure during plan execution: {exec_future.exception()}')
            return False

def main(args=None):
    rclpy.init(args=args)
    planner_client_node = CartesianPlannerClient()
    try:
        planner_client_node.get_logger().info("Waiting for a path on /arm_path_poses...")
        while not planner_client_node.is_new_path_available and rclpy.ok():
            rclpy.spin_once(planner_client_node, timeout_sec=0.1)
        
        if rclpy.ok():
            planner_client_node.get_logger().info("Path received. Starting planning and execution.")
            planner_client_node.plan_and_execute()
        
    except Exception as e:
        planner_client_node.get_logger().error(f"An exception occurred during execution: {e}")
    finally:
        planner_client_node.get_logger().info("Shutting down node.")
        planner_client_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()