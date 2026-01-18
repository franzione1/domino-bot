import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
import math
import time
import threading

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        
        self.arm_publisher = self.create_publisher(JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        self.gripper_client = ActionClient(self, GripperCommand, '/panda_hand_controller/gripper_cmd')
        
        self.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        
        # --- CONFIGURAZIONE ---
        self.HOME_POS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        self.L1 = 0.316
        self.L2 = 0.384
        self.OFFSET_SPALLA = 0.33
        
        self.LUNGHEZZA_MANO = 0.22 
        self.Z_MINIMA_SICUREZZA = 0.24 
        
        # Target (Coordinate Mattoncino)
        self.target_x = 0.55
        self.target_y = -0.15 
        
        # Correzione Mira Base
        self.CORREZIONE_J1 = -0.05 
        
        # --- PARAMETRO DI ALLINEAMENTO AL DOMINO ---
        # Questo è l'angolo che il polso dovrebbe avere se il robot fosse dritto (J1=0).
        # Solitamente 1.57 (90°) o 0.78 (45°) o 2.35 (135°) a seconda di come è girato il pezzo.
        # Prova 1.57. Se arriva ancora storto (es. parallelo al pezzo invece che perpendicolare),
        # cambia questo valore aggiungendo o togliendo 90 gradi (+/- 1.57).
        self.ALLINEAMENTO_BASE_POLSO = 1.57 
        
        self.get_logger().info('Avvio: MISSIONE CON COMPENSAZIONE ROTAZIONE')
        threading.Thread(target=self.missione_completa).start()

    def missione_completa(self):
        time.sleep(1.0)

        # 0. HOME
        self.get_logger().info('--> FASE 0: HOME')
        self.muovi_home()
        time.sleep(4.0)
        
        # 1. APERTURA
        self.get_logger().info('--> FASE 1: APERTURA')
        self.muovi_pinza(0.04)
        time.sleep(2.0)

        # CALCOLO J1
        theta1 = math.atan2(self.target_y, self.target_x) + self.CORREZIONE_J1
        
        # --- CALCOLO J7 (Compensazione) ---
        # La rotazione finale del polso deve cancellare la rotazione della base (theta1)
        # per mantenere l'orientamento assoluto rispetto al tavolo.
        theta7_compensato = self.ALLINEAMENTO_BASE_POLSO - theta1
        
        self.get_logger().info(f'Base J1: {theta1:.2f} -> Polso J7 Compensato: {theta7_compensato:.2f}')
        
        # 2. APPROCCIO ALTO
        self.get_logger().info('--> FASE 2: AVVICINAMENTO')
        # Arriviamo già con il polso ruotato correttamente
        self.muovi_orizzontale(theta1, z_altezza_presa=0.35, rotazione_j7=theta7_compensato)
        time.sleep(4.0)
        
        # 3. DISCESA
        self.get_logger().info(f'--> FASE 3: DISCESA (Z={self.Z_MINIMA_SICUREZZA})')
        self.muovi_orizzontale(theta1, z_altezza_presa=self.Z_MINIMA_SICUREZZA, rotazione_j7=theta7_compensato) 
        time.sleep(4.0)
        
        # 4. PRESA
        self.get_logger().info('--> FASE 4: CHIUSURA')
        self.muovi_pinza(0.01) 
        time.sleep(2.0)
        
        # 5. SOLLEVAMENTO
        self.get_logger().info('--> FASE 5: SOLLEVAMENTO')
        self.muovi_orizzontale(theta1, z_altezza_presa=0.35, rotazione_j7=theta7_compensato)
        time.sleep(4.0)
        
        # 6. RITORNO
        self.get_logger().info('--> FASE 6: RITORNO HOME')
        self.muovi_home()
        self.get_logger().info('Missione Completata!')

    def muovi_home(self):
        self.muovi_braccio(self.HOME_POS, durata=4.0)

    def muovi_orizzontale(self, theta1, z_altezza_presa, rotazione_j7):
        if z_altezza_presa < self.Z_MINIMA_SICUREZZA:
            z_altezza_presa = self.Z_MINIMA_SICUREZZA

        z_rel = z_altezza_presa - self.OFFSET_SPALLA
        r_polso = math.sqrt(self.target_x**2 + self.target_y**2) - self.LUNGHEZZA_MANO
        d = math.sqrt(r_polso**2 + z_rel**2)
        
        if d > (self.L1 + self.L2): d = self.L1 + self.L2 - 0.001
            
        cos_t4 = (self.L1**2 + self.L2**2 - d**2) / (2 * self.L1 * self.L2)
        theta4 = -1.0 * (math.pi - math.acos(max(-1.0, min(1.0, cos_t4))))

        val_acos = (self.L1**2 + d**2 - self.L2**2) / (2 * self.L1 * d)
        theta2_geom = math.atan2(z_rel, r_polso) + math.acos(max(-1.0, min(1.0, val_acos)))
        theta2_robot = (math.pi / 2) - theta2_geom

        theta6 = -1.0 * (theta2_geom + theta4)
        theta6 = max(0.01, min(3.75, theta6))

        # Assicuriamoci che J7 rimanga nei limiti fisici del Panda (-2.89 a +2.89)
        # Se la matematica lo manda fuori, lo riportiamo nel range corretto
        if rotazione_j7 > 2.89: rotazione_j7 -= 3.14
        if rotazione_j7 < -2.89: rotazione_j7 += 3.14

        target_pos = [theta1, theta2_robot, 0.0, theta4, 0.0, theta6, rotazione_j7] 
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

    def muovi_pinza(self, apertura):
        goal = GripperCommand.Goal()
        goal.command.position = apertura
        goal.command.max_effort = 100.0
        
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            return
        self.gripper_client.send_goal_async(goal)

def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()