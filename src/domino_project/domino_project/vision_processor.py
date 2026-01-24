import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class SmartDominoVision(Node):
    def __init__(self):
        super().__init__('smart_domino_vision')
        
        # --- CALIBRAZIONE ---
        self.CAMERA_X = 0.7      
        self.CAMERA_Y = 0.0      
        self.IMG_CENTER_X = 400  
        self.IMG_CENTER_Y = 400  
        self.SCALE_X = -0.0020  
        self.SCALE_Y = -0.0020  

        # --- COLORI & CODICI ---
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
        
        # Mappiamo i colori su numeri per inviarli al robot tramite la Z
        self.COLOR_CODES = { "ROSSO": 1.0, "VERDE": 2.0, "BLU": 3.0 }

        # QoS Gazebo
        qos_policy = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.subscription = self.create_subscription(Image, '/table_camera/image_raw', self.image_callback, qos_profile=qos_policy)
        self.coord_pub = self.create_publisher(Point, '/domino_position', 10)
        
        self.bridge = CvBridge()
        self.last_print_time = 0
        self.get_logger().info('VISION: Avviato. Invio codice colore su Z.')

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
                if cv2.contourArea(c) > 300: 
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        rx, ry = self.pixel_to_real(cx, cy)
                        blobs.append({'colore': color_name, 'cx': cx, 'cy': cy, 'rx': rx, 'ry': ry})

        if not blobs:
            cv2.imshow("Smart Vision", cv_image)
            cv2.waitKey(1)
            return

        # Trova Centrale
        blobs.sort(key=lambda b: math.sqrt((b['rx'] - 0.50)**2 + (b['ry'] - 0.00)**2))
        central_blob = blobs[0]
        cv2.circle(cv_image, (central_blob['cx'], central_blob['cy']), 15, (0, 255, 255), 3)

        # Cerca Target
        target_blob = None
        for b in blobs:
            if b == central_blob: continue
            if b['colore'] == central_blob['colore']:
                if math.sqrt((b['rx'] - central_blob['rx'])**2 + (b['ry'] - central_blob['ry'])**2) > 0.05:
                    target_blob = b
                    break
        
        if target_blob:
            cv2.line(cv_image, (central_blob['cx'], central_blob['cy']), (target_blob['cx'], target_blob['cy']), (0, 255, 0), 2)
            cv2.putText(cv_image, f"TGT {target_blob['colore']}", (target_blob['cx'], target_blob['cy']), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
            
            p = Point()
            p.x = target_blob['rx']
            p.y = target_blob['ry']
            # --- MODIFICA: Inviamo il codice colore su Z ---
            p.z = self.COLOR_CODES.get(target_blob['colore'], 0.0) 
            self.coord_pub.publish(p)
            
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_print_time > 2.0:
                self.get_logger().info(f"Target {target_blob['colore']} a X={p.x:.3f}, Y={p.y:.3f}")
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