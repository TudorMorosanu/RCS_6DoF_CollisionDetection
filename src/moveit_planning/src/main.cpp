#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "moveit_planning/MoveitPlanner.hpp"

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    
    auto planner = std::make_shared<MoveitPlanner>();

    planner->setMoveGroup();

    rclcpp::spin(planner);

    rclcpp::shutdown();
    return 0;
}
