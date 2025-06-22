# FILE: rw_py/rw_py/manipulation_action_server.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import Pose, PoseArray
from xarm_msgs.srv import PlanPose, PlanMultiStraight, PlanExec
import sys
import time

# <<< NEW: Import the custom action
from rw_interfaces.action import Manipulate

class ManipulationActionServer(Node):
    def __init__(self):
        super().__init__('manipulation_action_server')
        self.get_logger().info("Manipulation Action Server starting...")

        # Service clients for the xarm_planner
        self.cartesian_plan_client = self.create_client(PlanMultiStraight, '/xarm_cartesian_plan')
        self.pose_plan_client = self.create_client(PlanPose, '/xarm_pose_plan')
        self.exec_plan_client = self.create_client(PlanExec, '/xarm_exec_plan')

        services_to_wait_for = [
            self.cartesian_plan_client, self.pose_plan_client, self.exec_plan_client
        ]
        for client in services_to_wait_for:
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'Service {client.srv_name} not available. Shutting down.')
                sys.exit(1)

        # Create the Action Server
        self._action_server = ActionServer(
            self,
            Manipulate,
            'manipulate_path',  # The action name
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info("Manipulation Action Server is ready.")

    def goal_callback(self, goal_request):
        self.get_logger().info('Received a manipulation goal request.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received a cancel request.')
        # Here you could try to stop the arm motion if possible
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing manipulation goal...')
        feedback_msg = Manipulate.Feedback()
        result = Manipulate.Result()
        
        pose_array = goal_handle.request.pose_array

        if not pose_array.poses:
            result.success = False
            result.message = "Goal failed: Received an empty pose array."
            self.get_logger().warn(result.message)
            goal_handle.abort()
            return result

        # --- The core planning and execution logic ---
        feedback_msg.status = "Planning move to the first path waypoint..."
        goal_handle.publish_feedback(feedback_msg)
        if not await self.plan_and_exec_pose(pose_array.poses[0]):
            result.success = False
            result.message = "Failed to plan/execute move to the first waypoint."
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result
        
        time.sleep(1.0) # Pause briefly at the start

        feedback_msg.status = "Planning the full Cartesian path..."
        goal_handle.publish_feedback(feedback_msg)

        cartesian_req = PlanMultiStraight.Request()
        cartesian_req.targets = pose_array.poses
        
        cartesian_plan_future = self.cartesian_plan_client.call_async(cartesian_req)
        rclpy.spin_until_future_complete(self, cartesian_plan_future)
        
        if not cartesian_plan_future.result() or not cartesian_plan_future.result().success:
            result.success = False
            result.message = "Cartesian path planning failed."
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result
        
        feedback_msg.status = "Executing the Cartesian path..."
        goal_handle.publish_feedback(feedback_msg)
        if not await self.execute_plan():
            result.success = False
            result.message = "Execution of the Cartesian path failed."
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        goal_handle.succeed()
        result.success = True
        result.message = "Manipulation task completed successfully."
        self.get_logger().info(result.message)
        return result

    async def plan_and_exec_pose(self, pose: Pose):
        pose_plan_req = PlanPose.Request(target=pose)
        pose_plan_future = self.pose_plan_client.call_async(pose_plan_req)
        rclpy.spin_until_future_complete(self, pose_plan_future)
        if not pose_plan_future.result() or not pose_plan_future.result().success:
            self.get_logger().error(f"Failed to plan to pose: {pose}")
            return False
        return await self.execute_plan()

    async def execute_plan(self, wait=True):
        exec_req = PlanExec.Request(wait=wait)
        exec_future = self.exec_plan_client.call_async(exec_req)
        rclpy.spin_until_future_complete(self, exec_future)
        if exec_future.result() and exec_future.result().success:
            return True
        else:
            self.get_logger().error(f'Exception or failure during plan execution: {exec_future.exception()}')
            return False

def main(args=None):
    rclpy.init(args=args)
    action_server = ManipulationActionServer()
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()