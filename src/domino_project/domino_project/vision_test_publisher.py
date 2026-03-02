#!/usr/bin/env python3
"""Test publisher for /domino_detections — mimics SmartDominoVision output.

Publishes a MarkerArray with exactly two markers:
  ns='center', id=0  → the piece that stays on the table
  ns='target', id=1  → the piece the robot should pick and place

Encoding (must match robot_mover.cpp expectations):
  pose.position.{x,y}       → world X,Y on the table
  pose.orientation.{z,w}    → half-angle quaternion encoding piece yaw
  scale.z                   → match_angle: world-frame angle (rad) from piece
                               centre to its matching-colour half
  color.{r,g,b}             → 1/0 flags for ROSSO/VERDE/BLU
  color.a                   → matching colour code (1=ROSSO, 2=VERDE, 3=BLU)
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import math

# Must stay in sync with robot_mover.cpp / vision_processor.py
TABLE_TOP_Z   = 1.30
DOMINO_H      = 0.0075
DOMINO_REST_Z = TABLE_TOP_Z + DOMINO_H / 2.0   # 1.30375 m


class TestPublisher(Node):
    def __init__(self):
        super().__init__('vision_test_publisher')
        self.pub = self.create_publisher(MarkerArray, '/domino_detections', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.published = False
        self.get_logger().info('Test detection publisher started')

    def timer_callback(self):
        if self.published:
            return  # publish once — robot_mover ignores re-sends while busy

        stamp = self.get_clock().now().to_msg()
        ma = MarkerArray()

        # --- Center piece (stays on table) ---
        # Simulates a red+green domino at (0.5, 0.0) oriented along X
        center = Marker()
        center.header = Header(frame_id='world', stamp=stamp)
        center.ns = 'center'
        center.id = 0
        center.type = Marker.CUBE
        center.action = Marker.ADD
        center.pose.position.x = 0.5
        center.pose.position.y = 0.0
        center.pose.position.z = DOMINO_REST_Z
        center_yaw = 0.0
        center.pose.orientation.z = math.sin(center_yaw / 2.0)
        center.pose.orientation.w = math.cos(center_yaw / 2.0)
        center.scale.x = 0.048
        center.scale.y = 0.024
        # scale.z = match_angle: green half is at +x direction (angle 0 rad)
        center.scale.z = 0.0
        # color flags: r=1 g=1 → has red & green; a=2.0 → matching = VERDE
        center.color.r = 1.0
        center.color.g = 1.0
        center.color.b = 0.0
        center.color.a = 2.0
        ma.markers.append(center)

        # --- Target piece (to be picked) ---
        # Simulates a green+blue domino at (0.5, 0.2) oriented along X
        target = Marker()
        target.header = Header(frame_id='world', stamp=stamp)
        target.ns = 'target'
        target.id = 1
        target.type = Marker.CUBE
        target.action = Marker.ADD
        target.pose.position.x = 0.5
        target.pose.position.y = 0.2
        target.pose.position.z = DOMINO_REST_Z
        target_yaw = 0.0
        target.pose.orientation.z = math.sin(target_yaw / 2.0)
        target.pose.orientation.w = math.cos(target_yaw / 2.0)
        target.scale.x = 0.048
        target.scale.y = 0.024
        # scale.z = match_angle: green half at −x direction (angle π)
        target.scale.z = math.pi
        # color flags: g=1 b=1 → has green & blue; a=2.0 → matching = VERDE
        target.color.r = 0.0
        target.color.g = 1.0
        target.color.b = 1.0
        target.color.a = 2.0
        ma.markers.append(target)

        self.pub.publish(ma)
        self.published = True
        self.get_logger().info('Published test center + target detection')


def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
