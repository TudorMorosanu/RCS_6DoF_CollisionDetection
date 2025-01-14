#ifndef MOVEIT_PLANNER_HPP
#define MOVEIT_PLANNER_HPP

#include <chrono>
using namespace std::chrono_literals;

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>


struct Point {
    double x, y, z;
};


class MoveitPlanner : public rclcpp::Node {

private:

    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseStamped current_pose;
    geometry_msgs::msg::Pose last_target_pose;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::vector<Point> circle_trajectory;

    void timer_callback()
    {
        getCurrentPose();
        performMotion();
        RCLCPP_INFO(this->get_logger(), "Performing motion");
    }

public:

    MoveitPlanner();
    void setMoveGroup();
    void getCurrentPose();
    void performMotion();

};

#endif
