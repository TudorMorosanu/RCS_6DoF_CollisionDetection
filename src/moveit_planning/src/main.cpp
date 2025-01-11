#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "moveit_planning/MoveitPlanner.hpp"

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    
    auto planner = std::make_shared<MoveitPlanner>();

    // static const std::string PLANNING_GROUP = "ur_manipulator";
    // auto move_group = moveit::planning_interface::MoveGroupInterface(planner, "ur_manipulator");

    planner->setMoveGroup();
    // planner->getInitialPose();

    rclcpp::spin(planner);

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
