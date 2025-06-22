/* FILE: xarm_planner/src/xarm_planner.cpp */

#include "xarm_planner/xarm_planner.h"
#include <iterator> // Required for std::ostream_iterator

namespace xarm_planner
{
const double eef_step = 0.005;
const double max_velocity_scaling_factor = 0.3;
const double max_acceleration_scaling_factor = 0.1;

XArmPlanner::XArmPlanner(const rclcpp::Node::SharedPtr& node, const std::string& group_name)
    : node_(node)
{
    init(group_name);
}

XArmPlanner::XArmPlanner(const std::string& group_name)
{
    node_ = rclcpp::Node::make_shared("xarm_planner_move_group_node");
    init(group_name);
}

void XArmPlanner::init(const std::string& group_name) 
{
    is_trajectory_ = false;
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);
    RCLCPP_INFO(node_->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(node_->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
    RCLCPP_INFO(node_->get_logger(), "Available Planning Groups:");
    std::copy(move_group_->getJointModelGroupNames().begin(), move_group_->getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    std::cout << std::endl;
    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
    move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
}

bool XArmPlanner::planJointTarget(const std::vector<double>& joint_target)
{
    bool success = move_group_->setJointValueTarget(joint_target);
    if (!success)
        RCLCPP_WARN(node_->get_logger(), "setJointValueTarget: out of bounds");
    success = (move_group_->plan(xarm_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "planJointTarget: plan failed");
    is_trajectory_ = false;
    return success;
}

bool XArmPlanner::planPoseTarget(const geometry_msgs::msg::Pose& pose_target)
{
    bool success = move_group_->setPoseTarget(pose_target);
    if (!success)
        RCLCPP_WARN(node_->get_logger(), "setPoseTarget: out of bounds");
    success = (move_group_->plan(xarm_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "planPoseTarget: plan failed");
    is_trajectory_ = false;
    return success;
}

bool XArmPlanner::planPoseTargets(const std::vector<geometry_msgs::msg::Pose>& pose_target_vector)
{
    bool success = move_group_->setPoseTargets(pose_target_vector);
    if (!success)
        RCLCPP_WARN(node_->get_logger(), "setPoseTargets: out of bounds");
    success = (move_group_->plan(xarm_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "planPoseTargets: plan failed");
    is_trajectory_ = false;
    return success;
}

bool XArmPlanner::planCartesianPath(const std::vector<geometry_msgs::msg::Pose>& pose_target_vector)
{   
    // FIX 1: The deprecated `jump_threshold` argument is removed from this call.
    double fraction = move_group_->computeCartesianPath(pose_target_vector, eef_step, trajectory_);
    
    bool success = (fraction > 0.9); // Consider plans that cover at least 90% of the path as successful

    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "planCartesianPath: plan failed, fraction = %lf", fraction);
    }
    
    is_trajectory_ = success; 
    
    if (success) {
        // FIX 2: Corrected `trajectory_` to `trajectory`
        xarm_plan_.trajectory = trajectory_;
    }

    return success;
}

bool XArmPlanner::executePath(bool wait)
{
    moveit::core::MoveItErrorCode code;
    // FIX 3: Correctly use the plan's trajectory member for execution
    if (is_trajectory_) {
        // This is a cartesian path
        code = wait ? move_group_->execute(xarm_plan_.trajectory) : move_group_->asyncExecute(xarm_plan_.trajectory);
    } else {
        // This is a joint or pose goal plan
        code = wait ? move_group_->execute(xarm_plan_) : move_group_->asyncExecute(xarm_plan_);
    }
    
    bool success = (code == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "executePath: execute failed, wait=%d, MoveItErrorCode=%d", wait, code.val);
    
    return success;
}
}