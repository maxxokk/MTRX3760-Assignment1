from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    remaps_nav = [
        ('/cmd_vel', '/model/tb3/cmd_vel'),
        ('/odom', '/model/tb3/odometry'),
    ]
    remaps_traj = [
        ('/odom', '/model/tb3/odometry'),
    ]

    return LaunchDescription([
        Node(
            package='mtrx3760_maze_nav',
            executable='maze_navigator',
            name='maze_navigator',
            output='screen',
            remappings=remaps_nav,
        ),
        Node(
            package='mtrx3760_maze_nav',
            executable='trajectory_node',
            name='trajectory_node',
            output='screen',
            remappings=remaps_traj,
        ),
    ])
