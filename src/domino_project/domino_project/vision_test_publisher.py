#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math


class TestPublisher(Node):
    def __init__(self):
        super().__init__('vision_test_publisher')
        self.pub = self.create_publisher(MarkerArray, '/domino_detections', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.count = 0
        self.get_logger().info('Test detection publisher started')

    def timer_callback(self):
        ma = MarkerArray()
        base_x = 0.5
        zs = 0.235
        # three sample dominos with different colors and orientations
        colors = [ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0), ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)]
        y_offsets = [0.0, 0.2, -0.2]
        y_shift = 0.06 * (self.count % 4)
        for i in range(3):
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'domino'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = base_x
            m.pose.position.y = y_offsets[i] + y_shift
            m.pose.position.z = zs
            # orientation: rotate yaw to show different faces (0,90,180)
            yaw = [0.0, math.pi/2.0, math.pi][i]
            qz = math.sin(yaw/2.0)
            qw = math.cos(yaw/2.0)
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = qz
            m.pose.orientation.w = qw
            # domino scale (thin rectangle)
            m.scale.x = 0.06
            m.scale.y = 0.02
            m.scale.z = 0.01
            m.color = colors[i]
            ma.markers.append(m)

        self.pub.publish(ma)
        self.count += 1


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
