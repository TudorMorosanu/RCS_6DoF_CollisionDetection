#include "moveit_planning/MoveitPlanner.hpp"
#include <vector>
#include <cmath>

// THIS IS ONLY FOR TESTING
std::vector<Point> generateCircleTrajectory(double radius, Point center, double height, int num_points) 
{
    std::vector<Point> trajectory;
    double angle_step = 2 * M_PI / num_points;  // Divide the circle into num_points parts

    for (int i = 0; i < num_points; ++i) {
        double angle = i * angle_step;
        Point point;
        point.x = center.x + radius * cos(angle);
        point.y = center.y + radius * sin(angle);
        point.z = height;  // Constant height
        trajectory.push_back(point);
    }

    return trajectory;
}
// END OF TESTING STUFF

MoveitPlanner::MoveitPlanner() : Node("moveit_planner")
{
    timer_ = this->create_wall_timer(
      5000ms, std::bind(&MoveitPlanner::timer_callback, this));

    Point center = {-0.1, 0.5, 0.7};  // Center of the circle at origin
    double radius = 0.1;
    double height = 0.7;  // Example height
    int num_points = 10;  // Number of points on the circle

    this->circle_trajectory = generateCircleTrajectory(radius, center, height, num_points);

}

void MoveitPlanner::setMoveGroup()
{
    this->move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
    std::string planner_id = "RRTConnectkConfigDefault";
    this->move_group_->setPlannerId(planner_id);
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

    RCLCPP_INFO(this->get_logger(), "Node initialized successfully!");
}

void MoveitPlanner::performMotion()
{
    static int point_num = 0;

    // Set the planning frame and reference frame
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", this->move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "End effector link: %s", this->move_group_->getEndEffectorLink().c_str());

    // Define target pose
    geometry_msgs::msg::Pose target_pose;
    //target_pose.orientation.w = 1.0;
    // target_pose.position.x = this->current_pose.pose.position.x + 0.1;  // Desired X coordinate
    // target_pose.position.y = this->current_pose.pose.position.y + 0.1;  // Desired Y coordinate
    // target_pose.position.z = this->current_pose.pose.position.x + 0.1;  // Desired Z coordinate
    // target_pose.position.x = -0.1;  // Desired X coordinate
    // target_pose.position.y = 0.6;  // Desired Y coordinate
    // target_pose.position.z = 0.5;  // Desired Z coordinate

    // target_pose.position.x = -0.1;  // Desired X coordinate
    // target_pose.position.y = 0.7;  // Desired Y coordinate
    // target_pose.position.z = 0.7;  // Desired Z coordinate

    target_pose.position.x = this->circle_trajectory[point_num].x;  // Desired X coordinate
    target_pose.position.y = this->circle_trajectory[point_num].y;  // Desired Y coordinate
    target_pose.position.z = this->circle_trajectory[point_num].z;  // Desired Z coordinate
    RCLCPP_INFO(this->get_logger(), "X: %f, Y: %f, Z: %f, Num: %d", this->circle_trajectory[point_num].x, 
                this->circle_trajectory[point_num].y, this->circle_trajectory[point_num].z, point_num);
    point_num = (point_num + 1) % 10;

    // bool targets_equal = target_pose.position.x == this->last_target_pose.position.x &&
    //                      target_pose.position.y == this->last_target_pose.position.y &&
    //                      target_pose.position.z == this->last_target_pose.position.z &&
    //                      target_pose.orientation.w == this->last_target_pose.orientation.w;

    if(true)
    {
        this->move_group_->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (this->move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "Plan successful, executing...");
            this->move_group_->execute(my_plan);
            this->last_target_pose.position.x = target_pose.position.x;
            this->last_target_pose.position.y = target_pose.position.y;
            this->last_target_pose.position.z = target_pose.position.z;
            this->last_target_pose.orientation.w = target_pose.orientation.w;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }
    }
}