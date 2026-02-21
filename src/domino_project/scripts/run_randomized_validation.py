#!/usr/bin/env python3
"""
Run a sequence of randomized pick-and-place attempts using the existing nodes.
This script assumes the system is launched externally (headless) and it will publish
random targets to /domino_position and collect basic stats from /tmp/domino_metrics.csv.
"""
import time
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class RandPublisher(Node):
    def __init__(self):
        super().__init__('rand_validator')
        self.pub = self.create_publisher(Point, '/domino_position', 10)

    def publish_random(self):
        p = Point()
        p.x = 0.45 + random.uniform(-0.05, 0.05)
        p.y = random.uniform(-0.2, 0.2)
        p.z = random.choice([1.0, 2.0, 3.0])
        self.pub.publish(p)

if __name__ == '__main__':
    rclpy.init()
    node = RandPublisher()
    tries = 20
    successes = 0
    for i in range(tries):
        node.get_logger().info(f'Publishing random target {i+1}/{tries}')
        node.publish_random()
        rclpy.spin_once(node, timeout_sec=0.5)
        time.sleep(5)
        # inspect metrics
        try:
            with open('/tmp/domino_metrics.csv','r') as f:
                data = f.read()
                if 'grasp_attempt,1' in data:
                    successes += 1
        except FileNotFoundError:
            pass
    node.get_logger().info(f'Validation done: {successes}/{tries} successes')
    node.destroy_node()
    rclpy.shutdown()
