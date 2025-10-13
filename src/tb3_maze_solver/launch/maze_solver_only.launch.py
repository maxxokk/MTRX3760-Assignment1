from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tb3_maze_solver',
            executable='maze_solver_node',
            name='tb3_maze_solver',
            output='screen',
            parameters=[
                {'linear_speed': 0.15},
                {'angular_speed': 0.6},
                {'wall_distance': 0.5},
                {'front_stop_distance': 0.3},
                {'control_rate_hz': 20},
            ]
        )
    ])
