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
        Node(
            package='keyboard_control',
            executable='ur5_keyboard_control.py',
            name='ur5_keyboard_control',
            output='screen',
        ),
        Node(
            package='keyboard_control',
            executable='read_twist_topic.py',
            name='read_twist_topic',
            output='screen',
        ),
    ])
