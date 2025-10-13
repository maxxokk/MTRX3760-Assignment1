#!/usr/bin/env bash
# Stop all processes started by run_tb3_stack.sh
LOGDIR=/tmp/tb3_stack
for p in gz rviz bridge rsp; do
  if [ -f "$LOGDIR/$p.pid" ]; then
    kill "$(cat "$LOGDIR/$p.pid")" 2>/dev/null || true
  fi
done
# fallback pkill
pkill -f "gz sim" 2>/dev/null || true
pkill -f "ros_gz_bridge/parameter_bridge" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true
pkill -f "[r]viz2" 2>/dev/null || true
echo "✅ Stopped TB3 stack."
