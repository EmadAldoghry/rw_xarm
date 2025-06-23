#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import Pose
from xarm_msgs.srv import PlanPose, PlanJoint, PlanExec, PlanMultiStraight
from rw_interfaces.action import Manipulate
import sys
import time
import asyncio  # <-- [FIX] Add this missing import

# Service names from the xarm_planner node
CARTESIAN_PLAN_SERVICE = '/xarm_cartesian_plan'
POSE_PLAN_SERVICE = '/xarm_pose_plan'
JOINT_PLAN_SERVICE = '/xarm_joint_plan'
EXEC_PLAN_SERVICE = '/xarm_exec_plan'

class ManipulationActionServer(Node):
    def __init__(self):
        super().__init__('manipulation_action_server')
        self.get_logger().info('Manipulation Action Server starting...')

        # Connect to xarm_planner services
        self.cartesian_plan_client = self.create_client(PlanMultiStraight, CARTESIAN_PLAN_SERVICE)
        self.pose_plan_client = self.create_client(PlanPose, POSE_PLAN_SERVICE)
        self.joint_plan_client = self.create_client(PlanJoint, JOINT_PLAN_SERVICE)
        self.exec_plan_client = self.create_client(PlanExec, EXEC_PLAN_SERVICE)

        # Wait for all services to be available
        services = {
            self.cartesian_plan_client: CARTESIAN_PLAN_SERVICE,
            self.pose_plan_client: POSE_PLAN_SERVICE,
            self.joint_plan_client: JOINT_PLAN_SERVICE,
            self.exec_plan_client: EXEC_PLAN_SERVICE,
        }
        for client, name in services.items():
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'Service {name} not available. Shutting down.')
                rclpy.shutdown()
                sys.exit(1)
        
        # Create the Action Server
        self._action_server = ActionServer(
            self,
            Manipulate,
            'do_manipulation',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info('Manipulation Action Server is ready.')

    def goal_callback(self, goal_request):
        self.get_logger().info('Received manipulation goal request.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request.')
        # Here you could try to stop the arm execution if it's running
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing manipulation goal...')
        feedback_msg = Manipulate.Feedback()
        result = Manipulate.Result()

        poses = goal_handle.request.poses
        if not poses:
            result.success = False
            result.message = "Goal failed: poses list is empty."
            goal_handle.abort()
            return result

        # --- STEP 1: Move to a known, collision-free "ready" position first.
        ready_joint_state = [0.0, 0.2, -1.2, 0.0, 1.0, 0.0]
        self.get_logger().info(f"Moving to 'ready' state: {ready_joint_state}")
        if not await self.plan_and_exec_joint(ready_joint_state):
             result.success = False
             result.message = "Failed to move to 'ready' joint state."
             goal_handle.abort()
             return result
        time.sleep(1.0)

        # --- STEP 2: Plan a non-Cartesian move to the first waypoint.
        first_waypoint = poses[0]
        self.get_logger().info("Moving to the first path waypoint...")
        if not await self.plan_and_exec_pose(first_waypoint):
            result.success = False
            result.message = "Failed to plan to the first waypoint."
            goal_handle.abort()
            return result
        time.sleep(1.0)
        
        # --- STEP 3: Plan and execute the rest of the path as a single Cartesian trajectory.
        self.get_logger().info("Planning the full Cartesian path...")
        cartesian_req = PlanMultiStraight.Request(targets=poses)
        cartesian_plan_future = self.cartesian_plan_client.call_async(cartesian_req)
        
        await self.wait_for_future(cartesian_plan_future)
        
        if not cartesian_plan_future.result() or not cartesian_plan_future.result().success:
            result.success = False
            result.message = "Cartesian path planning failed."
            goal_handle.abort()
            return result
        
        self.get_logger().info("Executing the Cartesian path...")
        if not await self.execute_plan():
            result.success = False
            result.message = "Execution of the Cartesian path failed."
            goal_handle.abort()
            return result

        self.get_logger().info("Manipulation task completed successfully.")
        goal_handle.succeed()
        result.success = True
        result.message = "Manipulation completed."
        return result

    async def plan_and_exec_joint(self, joint_state):
        joint_plan_req = PlanJoint.Request(target=joint_state)
        joint_plan_future = self.joint_plan_client.call_async(joint_plan_req)
        await self.wait_for_future(joint_plan_future)
        if not joint_plan_future.result() or not joint_plan_future.result().success:
            return False
        return await self.execute_plan()

    async def plan_and_exec_pose(self, pose):
        pose_plan_req = PlanPose.Request(target=pose)
        pose_plan_future = self.pose_plan_client.call_async(pose_plan_req)
        await self.wait_for_future(pose_plan_future)
        if not pose_plan_future.result() or not pose_plan_future.result().success:
            return False
        return await self.execute_plan()

    async def execute_plan(self, wait=True):
        exec_req = PlanExec.Request(wait=wait)
        exec_future = self.exec_plan_client.call_async(exec_req)
        await self.wait_for_future(exec_future)
        return exec_future.result() and exec_future.result().success
    
    async def wait_for_future(self, future):
        while rclpy.ok():
            if future.done():
                return
            await asyncio.sleep(0.01)

import asyncio
def main(args=None):
    rclpy.init(args=args)
    node = ManipulationActionServer()
    try:
        # Use a MultiThreadedExecutor to handle callbacks concurrently
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()