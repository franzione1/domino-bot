import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class DominoDetector(Node):
    def __init__(self):
        super().__init__('domino_detector')
        
        # PARAMETRI CALIBRAZIONE
        self.CAMERA_X = 0.7      
        self.CAMERA_Y = 0.0      
        self.IMG_CENTER_X = 400  
        self.IMG_CENTER_Y = 400  
        self.SCALE_X = -0.00183  
        self.SCALE_Y = -0.00073  

        # QoS
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            Image,
            '/table_camera/image_raw',
            self.image_callback,
            qos_policy)
            
        # PUBLISHER (Reliable per essere sicuri che arrivi)
        self.coord_pub = self.create_publisher(Point, '/domino_position', 10)
            
        self.bridge = CvBridge()
        self.get_logger().info('Visione Debug Attiva! Cerco tasselli ROSSI...')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Maschera ROSSO (Ampia)
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 70, 50])
        upper_red2 = np.array([180, 255, 255])
        mask_red = cv2.bitwise_or(cv2.inRange(hsv_image, lower_red1, upper_red1), 
                                  cv2.inRange(hsv_image, lower_red2, upper_red2))

        # Passiamo solo il rosso al processore
        self.process_color(cv_image, mask_red, (0, 0, 255), "ROSSO")
        
        cv2.imshow("Robot View - Debug", cv_image)
        cv2.waitKey(1)

    def process_color(self, image, mask, color, label):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return # Nessun contorno trovato

        # Prendo il contorno più grande per evitare rumore
        largest_contour = max(contours, key=cv2.contourArea)
            
        if cv2.contourArea(largest_contour) > 500: # Filtro rumore aumentato
            cv2.drawContours(image, [largest_contour], -1, color, 2)
            
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                real_x = self.CAMERA_X + (cY - self.IMG_CENTER_Y) * self.SCALE_X
                real_y = self.CAMERA_Y + (cX - self.IMG_CENTER_X) * self.SCALE_Y
                
                coord_text = f"X:{real_x:.2f} Y:{real_y:.2f}"
                cv2.putText(image, coord_text, (cX - 50, cY - 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # --- INVIO MESSAGGIO ---
                point_msg = Point()
                point_msg.x = real_x
                point_msg.y = real_y
                point_msg.z = 1.32
                
                self.coord_pub.publish(point_msg)
                
                # LOG DI DEBUG: Se non vedi questo, non sta inviando!
                self.get_logger().info(f'[PUBBLICO] {label} a X={real_x:.2f}, Y={real_y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = DominoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()