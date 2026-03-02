#!/usr/bin/env bash
# Simple headless integration test: launches the system, publishes a single target point,
# waits for metrics to be written and checks for a successful grasp entry.
set -e
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/foxy/setup.bash
source /home/ros/ros2_domino_ws/install/setup.bash || true

ros2 launch domino_project final_system_moveit.launch.py &
LAUNCH_PID=$!
sleep 20
# Publish a test Point to /domino_position
python3 - <<'PY'
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

rclpy.init()
node = Node('test_publisher')
pub = node.create_publisher(Point, '/domino_position', 10)
pt = Point()
pt.x = 0.5
pt.y = 0.0
pt.z = 1.0
# publish few times
for _ in range(5):
    pub.publish(pt)
    node.get_logger().info('Published test point')
    rclpy.spin_once(node, timeout_sec=0.5)
node.destroy_node()
rclpy.shutdown()
PY

# wait for metrics file and inspect for a successful grasp_attempt
for i in {1..30}; do
  if [ -f /tmp/domino_metrics.csv ]; then
    if grep -q "grasp_attempt,1" /tmp/domino_metrics.csv; then
      echo "Integration test: grasp recorded. PASS"
      kill $LAUNCH_PID || true
      exit 0
    fi
  fi
  sleep 1
done

echo "Integration test: timeout or no successful grasp recorded. FAIL"
kill $LAUNCH_PID || true
exit 2
