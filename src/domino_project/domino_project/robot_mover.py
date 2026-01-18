import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.arm_publisher = self.create_publisher(JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        
        self.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        
        # --- CONFIGURAZIONE FISICA ---
        self.L1 = 0.316
        self.L2 = 0.384
        self.OFFSET_SPALLA = 0.33
        self.LUNGHEZZA_MANO = 0.12 
        
        # --- ALTEZZE ---
        # Conferma utente: Tavolo e Base Robot sono alla stessa altezza (Z=0 relativa)
        self.Z_TAVOLO_RISPETTO_BASE = 0.00 
        
        # Hover: 20 cm sopra il tavolo
        self.ALTEZZA_VOLO = 0.20   
        
        # --- TARGET ---
        self.target_x = 0.55
        self.target_y = -0.07
        
        self.get_logger().info('Step 7: FIX MAPPING GIUNTO 2...')
        self.timer = self.create_timer(4.0, self.esegui_hover_corretto)

    def esegui_hover_corretto(self):
        self.timer.cancel()

        # 1. Coordinate Target
        # Z Polso = Altezza Tavolo + Volo + Mano
        z_polso_target = self.Z_TAVOLO_RISPETTO_BASE + self.ALTEZZA_VOLO + self.LUNGHEZZA_MANO
        
        # Z relativa alla spalla (che è a 0.33 da terra)
        z_rel = z_polso_target - self.OFFSET_SPALLA
        
        # Raggio (Arretriamo per la mano)
        r_totale = math.sqrt(self.target_x**2 + self.target_y**2)
        r_polso = r_totale - self.LUNGHEZZA_MANO

        self.get_logger().info(f'Target Z Polso: {z_polso_target:.3f}m')
        self.get_logger().info(f'Z Relativa Spalla: {z_rel:.3f}m')

        # 2. Cinematica Inversa Geometrica
        # Calcoliamo theta1 (Base)
        theta1 = math.atan2(self.target_y, self.target_x)
        
        # Calcoliamo distanza ipotenusa spalla-polso
        d = math.sqrt(r_polso**2 + z_rel**2)
        
        # Protezione lunghezza
        if d > (self.L1 + self.L2):
            d = self.L1 + self.L2 - 0.001
            self.get_logger().warn("Target limitato al raggio massimo")

        # Angolo Gomito (Theta 4)
        cos_theta4 = (self.L1**2 + self.L2**2 - d**2) / (2 * self.L1 * self.L2)
        cos_theta4 = max(-1.0, min(1.0, cos_theta4))
        theta4 = -1.0 * (math.pi - math.acos(cos_theta4)) # Gomito "su" (negativo per Panda)

        # Angolo Spalla Geometrico (rispetto all'orizzontale)
        val_acos = (self.L1**2 + d**2 - self.L2**2) / (2 * self.L1 * d)
        val_acos = max(-1.0, min(1.0, val_acos))
        theta2_geom = math.atan2(z_rel, r_polso) + math.acos(val_acos)
        
        # --- FIX CRUCIALE ---
        # Il Panda ha J2=0 in VERTICALE. La nostra formula dava l'angolo dall'ORIZZONTALE.
        # Conversione: J2 = 90° - Angolo_Calcolato
        theta2_robot = (math.pi / 2) - theta2_geom

        # Angolo Polso (Theta 6)
        # Deve compensare la rotazione di spalla e gomito per puntare giù
        # Usiamo l'angolo geometrico per il calcolo dell'orientamento
        theta6 = (theta2_geom + theta4) + 1.57 
        
        self.get_logger().info(f'J2 Calcolato (geom): {theta2_geom:.2f} -> J2 Robot: {theta2_robot:.2f}')

        # Comando
        target_pos = [theta1, theta2_robot, 0.0, theta4, 0.0, theta6, 0.78]
        self.muovi_braccio(target_pos, durata=4.0)

    def muovi_braccio(self, posizioni, durata):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = posizioni
        point.time_from_start.sec = int(durata)
        point.time_from_start.nanosec = int((durata - int(durata)) * 1e9)
        msg.points.append(point)
        self.arm_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()