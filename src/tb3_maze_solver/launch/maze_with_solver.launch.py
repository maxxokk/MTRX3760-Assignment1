from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory('tb3_maze_solver'), 'worlds', 'maze_with_tb3.world'),
        description='Path to Gazebo world file'
    )

    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    tb3_world_launch = os.path.join(tb3_gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')

    tb3_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_world_launch),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'world_name': LaunchConfiguration('world')
        }.items()
    )

    solver_node = Node(
        package='tb3_maze_solver',
        executable='maze_solver_node',
        name='tb3_maze_solver',
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        tb3_include,
        solver_node
    ])
