from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    with_rviz = LaunchConfiguration('with_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('with_rviz', default_value='true'),

        # Keep discovery local and fixed
        SetEnvironmentVariable(name='ROS_LOCALHOST_ONLY', value='1'),
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='30'),
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger'),

        # Launch TurtleBot3 classic world (this starts its own parameter_bridge)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_gazebo'),
                    'launch',
                    'turtlebot3_world.launch.py'
                ])
            )
        ),

        # Optional RViz
        Node(
            package='rviz2',
            executable='rviz2',
            condition=IfCondition(with_rviz)
        ),
    ])
