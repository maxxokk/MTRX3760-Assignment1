from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_arg = DeclareLaunchArgument('world', default_value='maze_open.world')
    use_gui_arg = DeclareLaunchArgument('use_gui', default_value='true')

    world = LaunchConfiguration('world')
    world_path = PathJoinSubstitution([
        get_package_share_directory('maze_simulation'),
        'worlds',
        world
    ])

    # Launch Gazebo (GZ 8) via the CLI
    gz_proc = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v2', world_path],
        output='screen'
    )

    # Spawn TurtleBot3 model
    spawn_tb3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_tb3',
        output='screen',
        arguments=[
            '-entity', 'tb3',  # Name of the entity in Gazebo
            '-file', PathJoinSubstitution([get_package_share_directory('maze_simulation'), 'models/turtlebot3_burger/model.sdf']),  # Path to model SDF file
            '-x', '0', '-y', '0', '-z', '0.1',  # Initial position of the robot
            '-Y', '0'  # Orientation (yaw)
        ]
    )

    # Dedicated parameter bridge on model topics + scan + clock
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/model/tb3/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/tb3/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
    )

    return LaunchDescription([world_arg, use_gui_arg, gz_proc, spawn_tb3, bridge])
