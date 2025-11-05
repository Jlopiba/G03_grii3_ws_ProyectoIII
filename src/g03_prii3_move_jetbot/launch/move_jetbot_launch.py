from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g03_prii3_move_jetbot',
            executable='move_jetbot',
            name='move_jetbot',
            output='screen'
        )
    ])

