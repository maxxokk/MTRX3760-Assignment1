# MTRX3760-Assignment1
 
# Quick Start (ROS 2 Jazzy + Gazebo Sim + ros_gz_bridge)

## Prereqs (Ubuntu 24.04 / ROS 2 Jazzy)
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz git python3-colcon-common-extensions

# optional but useful
sudo apt install -y ros-jazzy-rviz2 ros-jazzy-ros2launch

## Clone & Build
git clone git@github.com:maxxokk/MTRX3760-Assignment1.git

cd MTRX3760-Assignment1 

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash

## Run (GUI + RViz by default)
./run_tb3_stack.sh              # or: ./run_tb3_stack.sh headless no_rviz

## Expected topics
ros2 topic list | egrep '/scan|/odom|/tf|/cmd_vel|/camera'
