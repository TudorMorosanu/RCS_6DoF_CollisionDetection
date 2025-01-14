#ifndef MOVEIT_PLANNER_HPP
#define MOVEIT_PLANNER_HPP

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>


class MoveitPlanner : public rclcpp::Node {

private:

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    geometry_msgs::msg::PoseStamped current_pose;
    geometry_msgs::msg::Pose last_target_pose;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

public:

    MoveitPlanner();
    void setMoveGroup();
    void getCurrentPose();
    void performMotion(const geometry_msgs::msg::Twist::SharedPtr msg);

};

#endif
