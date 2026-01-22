import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class SmartDominoVision(Node):
    def __init__(self):
        super().__init__('smart_domino_vision')
        
        # --- CALIBRAZIONE ---
        self.CAMERA_X = 0.7      
        self.CAMERA_Y = 0.0      
        self.IMG_CENTER_X = 400  
        self.IMG_CENTER_Y = 400  
        self.SCALE_X = -0.00183  
        self.SCALE_Y = -0.00073  

        # --- COLORI (HSV) ---
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

        self.subscription = self.create_subscription(
            Image, '/table_camera/image_raw', self.image_callback, 10)
            
        self.coord_pub = self.create_publisher(Point, '/domino_position', 10)
        self.bridge = CvBridge()
        self.last_print_time = 0
        self.get_logger().info('Visione Debug: Parametri Rilassati...')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        blobs = [] 
        
        # 1. RILEVAZIONE
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
                        
                        # Disegno solo per debug
                        color_rgb = (0, 255, 0) if color_name == "VERDE" else ((255, 0, 0) if color_name == "BLU" else (0, 0, 255))
                        cv2.drawContours(cv_image, [c], -1, color_rgb, 2)
                        cv2.putText(cv_image, f"{color_name}", (cx-20, cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)

                        blobs.append({'colore': color_name, 'cx': cx, 'cy': cy, 'rx': rx, 'ry': ry, 'contour': c})

        # --- LOGICA CENTRO ---
        # Definiamo dove ci aspettiamo sia il centro del tavolo.
        # Se ti prende il pezzo a destra, prova a modificare questi valori leggermente.
        centro_atteso_x = 0.50
        centro_atteso_y = 0.00
        
        # Cerchiamo blob entro 2cm da questo punto
        blobs_centrali = [b for b in blobs if math.sqrt((b['rx'] - centro_atteso_x)**2 + (b['ry'] - centro_atteso_y)**2) < 0.02]

        if not blobs_centrali:
            cv2.imshow("Smart Vision", cv_image)
            cv2.waitKey(1)
            return

        # Ordiniamo per Y pixel (chi ha Y minore è "sopra" nell'immagine)
        blobs_centrali.sort(key=lambda b: b['cy'])
        blob_superiore = blobs_centrali[0] # Questo è il mezzo-domino "in alto"
        colore_target = blob_superiore['colore']

        # Evidenziamo chi abbiamo scelto come centro
        cv2.circle(cv_image, (blob_superiore['cx'], blob_superiore['cy']), 20, (0, 255, 255), 3)
        cv2.putText(cv_image, "CENTRO", (blob_superiore['cx']-30, blob_superiore['cy']-30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # 3. CERCA TARGET
        blob_match = None
        
        # --- MODIFICA FONDAMENTALE: RIDOTTA DISTANZA MINIMA ---
        distanza_minima = 0.05 # 5 cm (Prima era 20cm!)
        
        match_candidates = [] # Per debug

        for b in blobs:
            if b['colore'] == colore_target:
                # Calcola distanza dal blob centrale che abbiamo scelto
                d = math.sqrt((b['rx'] - blob_superiore['rx'])**2 + (b['ry'] - blob_superiore['ry'])**2)
                
                # Se è abbastanza lontano da non essere lo stesso pezzo, ma è dello stesso colore
                if d > distanza_minima:
                    blob_match = b
                    match_candidates.append(f"Trovato a dist {d:.2f}")
                    break
                else:
                    if d > 0.01: # Se non è se stesso (distanza > 1cm) ma < 8cm
                         match_candidates.append(f"Scartato per dist {d:.2f} < {distanza_minima}")

        # LOG DEBUG PERIODICO
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_print_time > 2.0:
            self.get_logger().info(f"Target: {colore_target} | Candidati visti: {match_candidates}")
            self.last_print_time = now

        if blob_match:
            # DISEGNA LINEA DI COLLEGAMENTO
            cv2.line(cv_image, (blob_superiore['cx'], blob_superiore['cy']), (blob_match['cx'], blob_match['cy']), (0, 255, 0), 3)
            
            p = Point()
            p.x = blob_match['rx']
            p.y = blob_match['ry']
            self.coord_pub.publish(p)
        else:
            cv2.putText(cv_image, f"CERCO {colore_target}...", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Smart Vision", cv_image)
        cv2.waitKey(1)

    def pixel_to_real(self, u, v):
        real_x = self.CAMERA_X + (v - self.IMG_CENTER_Y) * self.SCALE_X
        real_y = self.CAMERA_Y + (u - self.IMG_CENTER_X) * self.SCALE_Y
        return real_x, real_y

def main(args=None):
    rclpy.init(args=args)
    node = SmartDominoVision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()