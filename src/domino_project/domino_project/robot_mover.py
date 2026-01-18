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
        
        # --- MISURE FISICHE ---
        self.L1 = 0.316
        self.L2 = 0.384
        self.OFFSET_SPALLA = 0.33
        self.LUNGHEZZA_MANO = 0.12 # Distanza Polso -> Punta Dita
        
        # --- PARAMETRI CRITICI DA VERIFICARE ---
        # "Quanto è più alto il tavolo rispetto alla base del robot?"
        # Se il robot è appoggiato SOPRA il tavolo, questo è 0.0.
        # Se il robot sfonda, PROBABILMENTE questo valore deve essere maggiore (es. 0.05, 0.10...)
        self.Z_TAVOLO_RISPETTO_BASE = 0.0 
        
        self.ALTEZZA_VOLO = 0.20 # Volo 20cm SOPRA il tavolo
        
        # --- TARGET ---
        self.target_x = 0.55
        self.target_y = -0.07
        
        self.get_logger().info('Step 5: Calibrazione Altezza Tavolo...')
        self.timer = self.create_timer(4.0, self.esegui_hover_calibrato)

    def esegui_hover_calibrato(self):
        self.timer.cancel()

        # 1. Calcolo Z Obiettivo (Assoluta rispetto alla base del robot)
        # Z_Target = (Altezza del Tavolo) + (Aria che vogliamo lasciare) + (Lunghezza pinza)
        z_polso_target = self.Z_TAVOLO_RISPETTO_BASE + self.ALTEZZA_VOLO + self.LUNGHEZZA_MANO
        
        # 2. Calcolo Z Relativa alla Spalla (J2)
        z_rel = z_polso_target - self.OFFSET_SPALLA
        
        # 3. Raggio (arretriamo per la mano)
        r_totale = math.sqrt(self.target_x**2 + self.target_y**2)
        r_polso = r_totale - self.LUNGHEZZA_MANO

        self.get_logger().info(f'--- CALCOLO ALTEZZE ---')
        self.get_logger().info(f'Tavolo stimato a Z: {self.Z_TAVOLO_RISPETTO_BASE:.3f}m')
        self.get_logger().info(f'Voglio volare a Z (totale): {z_polso_target:.3f}m')
        self.get_logger().info(f'Altezza rispetto alla spalla: {z_rel:.3f}m')

        # Controllo di sicurezza: Se la Z è troppo bassa per la spalla
        if z_rel < -0.3:
            self.get_logger().warn("ATTENZIONE: Target troppo basso! Rischio collisione base.")

        # Cinematica
        theta1 = math.atan2(self.target_y, self.target_x)
        d = math.sqrt(r_polso**2 + z_rel**2)
        
        # Check Reach
        if d > (self.L1 + self.L2):
            self.get_logger().error(f"NON CI ARRIVO! Distanza richiesta {d:.2f}m > Max {self.L1+self.L2:.2f}m")
            return 

        # IK
        cos_theta4 = (self.L1**2 + self.L2**2 - d**2) / (2 * self.L1 * self.L2)
        theta4 = -1.0 * (math.pi - math.acos(cos_theta4))

        val_acos = (self.L1**2 + d**2 - self.L2**2) / (2 * self.L1 * d)
        val_acos = max(-1.0, min(1.0, val_acos))
        theta2 = math.atan2(z_rel, r_polso) + math.acos(val_acos)
        
        theta6 = (theta2 + theta4) + 1.57 
        
        target_pos = [theta1, theta2, 0.0, theta4, 0.0, theta6, 0.78]
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