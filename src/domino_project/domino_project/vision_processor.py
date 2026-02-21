#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose  # <--- Usiamo Pose invece di Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class SmartDominoVision(Node):
    def __init__(self):
        super().__init__('smart_domino_vision')
        self.declare_parameter('camera.cam_x', 0.7)
        self.declare_parameter('camera.cam_y', 0.0)
        self.declare_parameter('camera.img_center_x', 400)
        self.declare_parameter('camera.img_center_y', 400)
        self.declare_parameter('camera.scale_x', -0.0020)
        self.declare_parameter('camera.scale_y', -0.0020)
        self.CAMERA_X = self.get_parameter('camera.cam_x').value
        self.CAMERA_Y = self.get_parameter('camera.cam_y').value
        self.IMG_CENTER_X = int(self.get_parameter('camera.img_center_x').value)
        self.IMG_CENTER_Y = int(self.get_parameter('camera.img_center_y').value)
        self.SCALE_X = self.get_parameter('camera.scale_x').value
        self.SCALE_Y = self.get_parameter('camera.scale_y').value

        self.COLORS = {
            "ROSSO": [
                (np.array([0, 70, 50]), np.array([10, 255, 255])),
                (np.array([160, 70, 50]), np.array([180, 255, 255]))
            ],
            "BLU": [
                (np.array([90, 50, 50]), np.array([140, 255, 255]))
            ],
            "VERDE": [
                (np.array([35, 50, 50]), np.array([85, 255, 255]))
            ]
        }
        
        self.COLOR_CODES = { "ROSSO": 1.0, "VERDE": 2.0, "BLU": 3.0 }

        qos_policy = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.subscription = self.create_subscription(Image, '/table_camera/image_raw', self.image_callback, qos_profile=qos_policy)
        
        # NUOVO TOPIC: Pubblica una Pose completa (Posizione + Orientamento)
        self.coord_pub = self.create_publisher(Pose, '/domino_pose', 10)
        self.detections_pub = self.create_publisher(MarkerArray, '/domino_detections', 10)
        
        self.bridge = CvBridge()
        self.last_print_time = 0
        self.get_logger().info('VISION: Pronta. Calcolo angolo (Yaw) ATTIVO.')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        blobs = [] 
        
        for color_name, ranges in self.COLORS.items():
            mask = np.zeros(hsv_image.shape[:2], dtype="uint8")
            for (lower, upper) in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv_image, lower, upper))
            
            kernel = np.ones((3,3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                if cv2.contourArea(c) > 80: 
                    # 1. Trova il centro
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        rx, ry = self.pixel_to_real(cx, cy)
                        
                        # 2. Calcola il rettangolo e l'angolo (Orientamento del pezzo)
                        rect = cv2.minAreaRect(c)
                        (rect_cx, rect_cy), (width, height), angle = rect
                        
                        # OpenCV calcola l'angolo in modo particolare.
                        # Vogliamo allinearci al lato lungo del domino
                        if width < height:
                            angle = angle - 90
                            
                        # Convertiamo da gradi a radianti per MoveIt/ROS
                        yaw = math.radians(angle)
                        
                        blobs.append({
                            'colore': color_name, 'cx': cx, 'cy': cy, 
                            'rx': rx, 'ry': ry, 'yaw': yaw, 'rect': rect
                        })

        if not blobs:
            cv2.imshow("Smart Vision", cv_image)
            cv2.waitKey(1)
            return

        blobs.sort(key=lambda b: math.sqrt((b['rx'] - 0.50)**2 + (b['ry'] - 0.00)**2))
        central_blob = blobs[0]
        
        # Disegna il box orientato per il pezzo centrale
        box_c = np.int0(cv2.boxPoints(central_blob['rect']))
        cv2.drawContours(cv_image, [box_c], 0, (0, 255, 255), 2)

        target_blob = None
        for b in blobs:
            if b == central_blob: continue
            if b['colore'] == central_blob['colore']:
                if math.sqrt((b['rx'] - central_blob['rx'])**2 + (b['ry'] - central_blob['ry'])**2) > 0.04:
                    target_blob = b
                    break
        
        if target_blob:
            # Disegna il box orientato per il pezzo target
            box_t = np.int0(cv2.boxPoints(target_blob['rect']))
            cv2.drawContours(cv_image, [box_t], 0, (0, 255, 0), 2)
            cv2.putText(cv_image, f"TGT {target_blob['colore']} ({math.degrees(target_blob['yaw']):.1f} deg)", 
                        (target_blob['cx'] - 40, target_blob['cy'] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
            
            # Crea e pubblica la Pose
            p = Pose()
            p.position.x = target_blob['rx']
            p.position.y = target_blob['ry']
            p.position.z = self.COLOR_CODES.get(target_blob['colore'], 0.0) 
            
            # Converti lo Yaw in Quaternione per ROS
            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = math.sin(target_blob['yaw'] / 2.0)
            p.orientation.w = math.cos(target_blob['yaw'] / 2.0)
            self.coord_pub.publish(p)

            # Marker visivi per RViz
            marker = Marker()
            marker.header = Header(frame_id='world')
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'domino_detections'
            marker.id = 0
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = p # Assegniamo la posa calcolata!
            marker.pose.position.z = 1.32 # Altezza visiva sul tavolo
            marker.scale.x = 0.06; marker.scale.y = 0.03; marker.scale.z = 0.02
            
            col = {'ROSSO': (1.0,0.0,0.0), 'BLU': (0.0,0.0,1.0), 'VERDE': (0.0,1.0,0.0)}.get(target_blob['colore'], (1.0,1.0,1.0))
            marker.color.r = col[0]; marker.color.g = col[1]; marker.color.b = col[2]; marker.color.a = 0.9
            
            ma = MarkerArray()
            ma.markers.append(marker)
            self.detections_pub.publish(ma)
            
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_print_time > 2.0:
                self.get_logger().info(f"Target {target_blob['colore']}: X={p.position.x:.3f}, Y={p.position.y:.3f}, Yaw={math.degrees(target_blob['yaw']):.1f}°")
                self.last_print_time = now

        cv2.imshow("Smart Vision", cv_image)
        cv2.waitKey(1)

    def pixel_to_real(self, u, v):
        real_x = self.CAMERA_X + (v - self.IMG_CENTER_Y) * self.SCALE_X
        real_y = self.CAMERA_Y + (u - self.IMG_CENTER_X) * self.SCALE_Y
        return real_x, real_y

def main(args=None):
    rclpy.init(args=args)
    node = SmartDominoVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()