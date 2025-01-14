cmake_minimum_required(VERSION 3.5)
project(keyboard_control)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Install Python scripts
install(
  PROGRAMS
    src/ur5_keyboard_control.py
    topic_read/read_twist_topic.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
