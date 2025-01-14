#include "moveit_planning/MoveitPlanner.hpp"
#include <vector>
#include <cmath>

MoveitPlanner::MoveitPlanner() : Node("moveit_planner")
{
    this->subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "ur5_control", 10, std::bind(&MoveitPlanner::performMotion, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "moveit_planner node initialized successfully!");
}

void MoveitPlanner::setMoveGroup()
{
    this->move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");

    RCLCPP_INFO(this->get_logger(), "Node initialized successfully!");

}

void MoveitPlanner::getCurrentPose()
{
    this->current_pose = this->move_group_->getCurrentPose();
    RCLCPP_INFO(this->get_logger(), "Current position: [x: %f, y: %f, z: %f]",
                this->current_pose.pose.position.x,
                this->current_pose.pose.position.y,
                this->current_pose.pose.position.z);

    RCLCPP_INFO(this->get_logger(), "Current orientation: [x: %f, y: %f, z: %f, w: %f]",
                this->current_pose.pose.orientation.x,
                this->current_pose.pose.orientation.y,
                this->current_pose.pose.orientation.z,
                this->current_pose.pose.orientation.w);

    this->last_target_pose.position.x = this->current_pose.pose.position.x;
    this->last_target_pose.position.y = this->current_pose.pose.position.y;
    this->last_target_pose.position.z = this->current_pose.pose.position.z;
}

void MoveitPlanner::performMotion(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    double eef_step = 0.05;  // 1 cm spacing between waypoints
    double jump_threshold = 0.0; // set to 0 to disable the jump distance check (not recommended)
    moveit_msgs::msg::RobotTrajectory trajectory;
    std::vector<geometry_msgs::msg::Pose> waypoints;

    RCLCPP_INFO(this->get_logger(), "Received command - Linear: %f, Angular: %f",
              msg->linear.x, msg->angular.z);

    this->getCurrentPose();

    geometry_msgs::msg::Pose target_pose = this->current_pose.pose;
    
    //target_pose.orientation.w = 1.0;
    // target_pose.position.x = this->last_target_pose.position.x + msg->linear.x;
    // target_pose.position.y = this->last_target_pose.position.y + msg->linear.y;
    // target_pose.position.z = this->last_target_pose.position.z + msg->linear.z;

    target_pose.position.x = this->current_pose.pose.position.x + msg->linear.x;
    target_pose.position.y = this->current_pose.pose.position.y + msg->linear.y;
    target_pose.position.z = this->current_pose.pose.position.z + msg->linear.z;

    RCLCPP_INFO(this->get_logger(), "Target pose: [x: %f, y: %f, z: %f]",
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z);


    bool targets_equal = target_pose.position.x == this->last_target_pose.position.x &&
                         target_pose.position.y == this->last_target_pose.position.y &&
                         target_pose.position.z == this->last_target_pose.position.z &&
                         target_pose.orientation.w == this->last_target_pose.orientation.w;

    if(!targets_equal)
    {
        waypoints.push_back(target_pose);
        double fraction = this->move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        RCLCPP_INFO(this->get_logger(), "Target fraction: %f", fraction);
        if (fraction > 0.0) { // Check if a sufficient part of the path was planned
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            my_plan.trajectory_ = trajectory;
            this->move_group_->execute(my_plan);

            this->last_target_pose.position.x = target_pose.position.x;
            this->last_target_pose.position.y = target_pose.position.y;
            this->last_target_pose.position.z = target_pose.position.z;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }
    }
}
