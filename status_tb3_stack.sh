#!/usr/bin/env bash
echo "=== PIDs ==="
ps -eo pid,cmd | egrep "gz sim|parameter_bridge|robot_state_publisher|rviz2" | grep -v egrep || echo "(none)"
echo
echo "=== Key topics ==="
ros2 topic list | egrep '^/(scan|odom|tf|cmd_vel|clock)' || true
