from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moveit_planning',
            executable='moveit_planning',
            name='moveit_planning',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        ),
    ])
