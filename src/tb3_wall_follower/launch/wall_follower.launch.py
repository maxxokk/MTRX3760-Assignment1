from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ]
    )

    follower = Node(
        package='tb3_wall_follower',
        executable='wall_follower',
        output='screen',
        parameters=[{
            'desired': 0.35,
            'front_clear': 0.45,
            'v_forward': 0.18,
            'kp': 1.6,
            'kd': 0.25,
            'max_w': 1.8
        }]
    )

    return LaunchDescription([bridge, follower])
