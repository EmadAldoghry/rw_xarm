#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from xarm_msgs.srv import PlanPose, PlanSingleStraight, PlanExec
import sys
import time

# Service names provided by xarm_planner_node (adjust if namespace/prefix is used)
POSE_PLAN_SERVICE = '/xarm_pose_plan'
JOINT_PLAN_SERVICE = '/xarm_joint_plan'  # Not used here, but good to know
STRAIGHT_PLAN_SERVICE = '/xarm_straight_plan'
EXEC_PLAN_SERVICE = '/xarm_exec_plan'
POSE_ARRAY_TOPIC = '/arm_path_poses'


class CartesianPlannerClient(Node):

    def __init__(self):
        super().__init__('xarm6_cartesian_planner_client')
        self.get_logger().info("Initializing Cartesian Planner Client...")

        # Create Service Clients
        self.pose_plan_client = self.create_client(PlanPose, POSE_PLAN_SERVICE)
        self.straight_plan_client = self.create_client(PlanSingleStraight, STRAIGHT_PLAN_SERVICE)
        self.exec_plan_client = self.create_client(PlanExec, EXEC_PLAN_SERVICE)

        # Wait for services to be available
        if not self.straight_plan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {STRAIGHT_PLAN_SERVICE} not available.')
            rclpy.shutdown()
            sys.exit(1)
        if not self.exec_plan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {EXEC_PLAN_SERVICE} not available.')
            rclpy.shutdown()
            sys.exit(1)

        self.get_logger().info("Service clients created and ready.")

        # Subscribe to PoseArray topic
        self.pose_array_sub = self.create_subscription(PoseArray, POSE_ARRAY_TOPIC, self.pose_array_callback, 10)
        self.poses_received = False
        self.pose_array = []

    def pose_array_callback(self, msg):
        self.pose_array = msg.poses
        self.poses_received = True
        self.get_logger().info(f"Received {len(self.pose_array)} poses from topic '{POSE_ARRAY_TOPIC}'")

    def plan_and_execute_straight(self, target_pose, wait_for_execution=True):
        self.get_logger().info(f"Planning straight path to: {target_pose}")

        plan_req = PlanSingleStraight.Request()
        plan_req.target = target_pose

        plan_future = self.straight_plan_client.call_async(plan_req)
        rclpy.spin_until_future_complete(self, plan_future)

        if plan_future.result() is None:
            self.get_logger().error(f'Exception while calling service {STRAIGHT_PLAN_SERVICE}: {plan_future.exception()}')
            return False

        plan_res = plan_future.result()
        if not plan_res.success:
            self.get_logger().error("Cartesian planning failed!")
            return False

        self.get_logger().info("Cartesian planning successful.")

        exec_req = PlanExec.Request()
        exec_req.wait = wait_for_execution

        exec_future = self.exec_plan_client.call_async(exec_req)
        rclpy.spin_until_future_complete(self, exec_future)

        if exec_future.result() is None:
            self.get_logger().error(f'Exception while calling service {EXEC_PLAN_SERVICE}: {exec_future.exception()}')
            return False

        exec_res = exec_future.result()
        if not exec_res.success:
            self.get_logger().error("Plan execution failed!")
            return False

        self.get_logger().info("Plan execution successful.")
        return True


def main(args=None):
    rclpy.init(args=args)

    planner_client_node = CartesianPlannerClient()

    try:
        # Wait until PoseArray is received
        planner_client_node.get_logger().info("Waiting for PoseArray message...")
        while not planner_client_node.poses_received:
            rclpy.spin_once(planner_client_node)

        time.sleep(1.0)  # Optional: Let planner settle

        # Define Start Pose (e.g., Home position)
        start_pose = Pose()
        start_pose.position.x = 0.4
        start_pose.position.y = 0.0
        start_pose.position.z = 0.2
        start_pose.orientation.x = 1.0
        start_pose.orientation.y = 0.0
        start_pose.orientation.z = 0.0
        start_pose.orientation.w = 0.0

        # Define End Pose (e.g., Safe park position)
        end_pose = Pose()
        end_pose.position.x = 0.3
        end_pose.position.y = 0.1
        end_pose.position.z = 0.3
        end_pose.orientation.x = 1.0
        end_pose.orientation.y = 0.0
        end_pose.orientation.z = 0.0
        end_pose.orientation.w = 0.0

        # Combine full sequence: start → poses → end
        full_pose_sequence = [start_pose] + planner_client_node.pose_array + [end_pose]

        total = len(full_pose_sequence)
        for idx, pose in enumerate(full_pose_sequence):
            pose_number = idx + 1
            planner_client_node.get_logger().info(f"Attempting to reach Pose {pose_number}/{total}...")
            success = planner_client_node.plan_and_execute_straight(pose)

            if success:
                planner_client_node.get_logger().info(f"Successfully reached Pose {pose_number}")
                time.sleep(1.0)
            else:
                planner_client_node.get_logger().warn(f"Skipping Pose {pose_number} due to planning/execution failure.")

        planner_client_node.get_logger().info("Completed sequence (with skips if any).")

    except Exception as e:
        planner_client_node.get_logger().error(f"An exception occurred during execution: {e}")
    finally:
        planner_client_node.get_logger().info("Shutting down node.")
        planner_client_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
