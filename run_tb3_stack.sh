#!/usr/bin/env bash
# Bring up Gazebo+TB3+bridge+RSP+RViz without a launch file.
# Usage: ./run_tb3_stack.sh [gui|headless] [rviz|no_rviz]
# No 'set -e' or 'set -u' so errors won't kill the terminal.
set -o pipefail

MODE_GUI="${1:-gui}"       # gui | headless
MODE_RVIZ="${2:-rviz}"     # rviz | no_rviz

# --- Env: source ROS setups (no nounset!) ---
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
else
  echo "ERROR: /opt/ros/jazzy/setup.bash not found"; exit 1
fi
[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash || true

export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# --- Locate shares / resources ---
maze_share="$(ros2 pkg prefix maze_simulation 2>/dev/null)/share/maze_simulation"
tb3_gz_share="$(ros2 pkg prefix turtlebot3_gazebo 2>/dev/null)/share/turtlebot3_gazebo"
tb3_desc_share="$(ros2 pkg prefix turtlebot3_description 2>/dev/null)/share/turtlebot3_description"

if [ ! -d "$maze_share" ] || [ ! -d "$tb3_gz_share" ] || [ ! -d "$tb3_desc_share" ]; then
  echo "ERROR: required packages not found."
  echo "maze_share=$maze_share"
  echo "tb3_gz_share=$tb3_gz_share"
  echo "tb3_desc_share=$tb3_desc_share"
  exit 1
fi

world="$maze_share/worlds/maze_open.world"
[ -f "$world" ] || world="$maze_share/worlds/maze.world"
[ -f "$world" ] || { echo "ERROR: world file not found in $maze_share/worlds"; exit 1; }

robot_sdf="$tb3_gz_share/models/turtlebot3_burger/model.sdf"
[ -f "$robot_sdf" ] || { echo "ERROR: TB3 model not found at $robot_sdf"; exit 1; }

rviz_cfg="$maze_share/rviz/maze_tb3.rviz"
[ -f "$rviz_cfg" ] || rviz_cfg=""

# Gazebo resources so it can find worlds/models
export GZ_SIM_RESOURCE_PATH="$maze_share:$tb3_gz_share/models:${GZ_SIM_RESOURCE_PATH:-}"

# --- Logs ---
LOGDIR=/tmp/tb3_stack; mkdir -p "$LOGDIR"
echo "[maze_simulation] Using world: $world"

# --- Clean old processes (best-effort) ---
pkill -f "gz sim" 2>/dev/null || true
pkill -f "ros_gz_bridge/parameter_bridge" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true
pkill -f "[r]viz2" 2>/dev/null || true
sleep 0.3

# --- 1) Gazebo sim (GUI or headless) ---
if [ "$MODE_GUI" = "headless" ]; then
  nohup gz sim -s -r "$world" >"$LOGDIR/gz.log" 2>&1 & echo $! > "$LOGDIR/gz.pid"
else
  nohup gz sim -r "$world"   >"$LOGDIR/gz.log" 2>&1 & echo $! > "$LOGDIR/gz.pid"
fi
echo "gz sim PID: $(cat "$LOGDIR/gz.pid")"

# Give server a moment
sleep 1

# --- 2) Spawn TB3 into world ---
nohup ros2 run ros_gz_sim create -world default -name tb3 -file "$robot_sdf" >"$LOGDIR/create.log" 2>&1 &
echo "spawn PID: $!"

# --- 3) Bridge (clock, tf, odom, scan, cmd_vel) ---
nohup ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock \
  /odom@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V \
  /cmd_vel@geometry_msgs/msg/TwistStamped@gz.msgs.Twist \
  >"$LOGDIR/bridge.log" 2>&1 & echo $! > "$LOGDIR/bridge.pid"
echo "bridge PID: $(cat "$LOGDIR/bridge.pid")"

# --- 4) Robot State Publisher ---
URDF_TMP="$LOGDIR/tb3.urdf"
if command -v xacro >/dev/null 2>&1; then
  xacro "$tb3_desc_share/urdf/turtlebot3_burger.urdf" > "$URDF_TMP" 2>"$LOGDIR/xacro.err" || true
  if [ -s "$URDF_TMP" ]; then
    nohup ros2 run robot_state_publisher robot_state_publisher \
      --ros-args -p use_sim_time:=true -p robot_description:="$(cat "$URDF_TMP")" \
      >"$LOGDIR/rsp.log" 2>&1 & echo $! > "$LOGDIR/rsp.pid"
  else
    nohup ros2 run robot_state_publisher robot_state_publisher \
      --ros-args -p use_sim_time:=true \
      >"$LOGDIR/rsp.log" 2>&1 & echo $! > "$LOGDIR/rsp.pid"
  fi
else
  nohup ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p use_sim_time:=true \
    >"$LOGDIR/rsp.log" 2>&1 & echo $! > "$LOGDIR/rsp.pid"
fi
echo "robot_state_publisher PID: $(cat "$LOGDIR/rsp.pid" 2>/dev/null || echo "n/a")"

# --- 5) RViz (optional) ---
if [ "$MODE_RVIZ" = "rviz" ]; then
  if [ -n "$rviz_cfg" ]; then
    nohup rviz2 -d "$rviz_cfg" --ros-args -p use_sim_time:=true >"$LOGDIR/rviz.log" 2>&1 & echo $! > "$LOGDIR/rviz.pid"
  else
    nohup rviz2 --ros-args -p use_sim_time:=true >"$LOGDIR/rviz.log" 2>&1 & echo $! > "$LOGDIR/rviz.pid"
  fi
  echo "rviz PID: $(cat "$LOGDIR/rviz.pid")"
fi

echo
echo "✅ TB3 stack started. Logs in $LOGDIR"
echo "Quick checks:"
echo "  ros2 topic list | egrep '^/(scan|odom|tf|cmd_vel)'"
echo "  ros2 topic echo /odom --once"
