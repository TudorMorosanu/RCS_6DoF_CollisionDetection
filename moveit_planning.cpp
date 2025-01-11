#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ur5_move");

    // Create MoveIt MoveGroupInterface for the planning group
    static const std::string PLANNING_GROUP = "ur_manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
    RCLCPP_INFO(node->get_logger(), "Current position: [x: %f, y: %f, z: %f]",
                current_pose.pose.position.x,
                current_pose.pose.position.y,
                current_pose.pose.position.z);

    RCLCPP_INFO(node->get_logger(), "Current orientation: [x: %f, y: %f, z: %f, w: %f]",
                current_pose.pose.orientation.x,
                current_pose.pose.orientation.y,
                current_pose.pose.orientation.z,
                current_pose.pose.orientation.w);

    // Set the planning frame and reference frame
    RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Define target pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = current_pose.pose.position.x + 0.2;  // Desired X coordinate
    target_pose.position.y = current_pose.pose.position.y + 0.2;  // Desired Y coordinate
    target_pose.position.z = current_pose.pose.position.x + 0.2;  // Desired Z coordinate

    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(node->get_logger(), "Plan successful, executing...");
        move_group.execute(my_plan);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
