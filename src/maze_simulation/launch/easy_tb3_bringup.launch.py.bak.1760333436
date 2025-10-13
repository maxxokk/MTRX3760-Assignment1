import os, glob
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def _pick_world():
    # Look in install share (what ros2 launch uses)
    share = get_package_share_directory('maze_simulation')
    candidates = []
    for pat in ['maze.world','maze.sdf','*.world','*.sdf']:
        candidates += glob.glob(os.path.join(share, 'worlds', pat))
    return candidates[0] if candidates else os.path.join(share, 'worlds', 'maze.sdf')

def generate_launch_description():
    use_gui    = LaunchConfiguration('use_gui')
    with_rviz  = LaunchConfiguration('with_rviz')
    robot_name = LaunchConfiguration('robot_name')

    maze_share = get_package_share_directory('maze_simulation')
    tb3_desc   = FindPackageShare('turtlebot3_description')
    tb3_gz     = FindPackageShare('turtlebot3_gazebo')

    world_path = _pick_world()
    rviz_cfg   = os.path.join(maze_share, 'rviz', 'maze_tb3.rviz')
    robot_urdf = PathJoinSubstitution([tb3_desc, 'urdf', 'turtlebot3_burger.urdf'])
    robot_sdf  = PathJoinSubstitution([tb3_gz, 'models', 'turtlebot3_burger', 'model.sdf'])

    # Ensure Gazebo can resolve local resources referenced by the world/models
    gz_res_path = os.pathsep.join(filter(None, [
        maze_share,
        os.environ.get('GZ_SIM_RESOURCE_PATH',''),
    ]))
    ign_res_path = os.pathsep.join(filter(None, [
        maze_share,
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH',''),
    ]))

    env = [
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '30'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_res_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_res_path),
    ]

    # Start Gazebo via CLI
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen',
        condition=IfCondition(use_gui)
    )
    gz_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world_path],
        output='screen',
        condition=UnlessCondition(use_gui)
    )

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', 'default', '-name', robot_name, '-file', robot_sdf]
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/TwistStamped@gz.msgs.Twist',
        ]
    )

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', robot_urdf])
        }]
    )

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(with_rviz)
    )

    return LaunchDescription([
        *env,
        LogInfo(msg=['[maze_simulation] Using world: ', world_path]),
        DeclareLaunchArgument('use_gui', default_value=TextSubstitution(text='true')),
        DeclareLaunchArgument('with_rviz', default_value=TextSubstitution(text='true')),
        DeclareLaunchArgument('robot_name', default_value=TextSubstitution(text='tb3')),
        gz_gui, gz_headless, spawn, bridge, rsp, rviz
    ])
