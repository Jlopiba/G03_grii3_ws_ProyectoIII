from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g03_prii3_move_jetbot',
            executable='draw_and_avoid',
            name='draw_and_avoid_node',
            output='screen',
            parameters=[{
                'safe_distance': 0.55,
                'v_line': 0.15,
                'v_arc': 0.12,
                'w_arc': -0.6,
            }],
        )
    ])

