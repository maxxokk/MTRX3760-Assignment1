#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch maze_simulation easy_tb3_bringup.launch.py use_gui:=true with_rviz:=true
