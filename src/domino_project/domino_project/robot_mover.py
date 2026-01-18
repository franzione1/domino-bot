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
        
        # --- POSIZIONE HOME (DEFAULT GAZEBO) ---
        # Questa è la posizione "rannicchiata" standard di avvio
        self.HOME_POS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

        # --- MISURE FISICHE ---
        self.L1 = 0.316
        self.L2 = 0.384
        self.OFFSET_SPALLA = 0.33
        
        # Distanza orizzontale Polso -> Punta Dita
        self.LUNGHEZZA_MANO = 0.22 
        
        self.Z_TAVOLO_RISPETTO_BASE = 0.00 
        
        # --- SAFETY Z ---
        # Limite minimo per non toccare il tavolo con il polso
        self.Z_MINIMA_SICUREZZA = 0.24
        
        # --- TARGET ---
        self.target_x = 0.55
        self.target_y = -0.15 
        
        self.get_logger().info('Avvio: MISSIONE COMPLETA (HOME -> PRESA -> HOME)')
        threading.Thread(target=self.missione_parallela).start()

    def missione_parallela(self):
        time.sleep(1.0) # Breve attesa avvio

        # 0. VAI A HOME (Posizione Iniziale)
        self.get_logger().info('--> FASE 0: POSIZIONAMENTO HOME')
        self.muovi_home()
        time.sleep(5.0) # Tempo per arrivare a home
        
        # 1. APRIRE PINZA
        self.get_logger().info('--> FASE 1: APERTURA PINZA')
        self.muovi_pinza(0.04) 
        time.sleep(1.0)

        # 2. CALCOLO ANGOLO
        theta1 = math.atan2(self.target_y, self.target_x)
        self.get_logger().info(f'J1 Orientamento: {theta1:.3f} rad')

        # 3. APPROCCIO ALTO (Volo)
        self.get_logger().info('--> FASE 2: VOLO DI APPROCCIO')
        self.muovi_orizzontale(theta1, z_altezza_presa=0.30) # Stiamo alti
        time.sleep(5.0)
        
        # 4. DISCESA ALLA QUOTA DI PRESA
        self.get_logger().info(f'--> FASE 3: DISCESA PRESA (Z={self.Z_MINIMA_SICUREZZA}m)')
        self.muovi_orizzontale(theta1, z_altezza_presa=self.Z_MINIMA_SICUREZZA) 
        time.sleep(5.0)
        
        # 5. CHIUSURA
        self.get_logger().info('--> FASE 4: CHIUSURA PINZA')
        self.muovi_pinza(0.00) 
        time.sleep(2.0)
        
        # 6. SOLLEVAMENTO CON PEZZO
        self.get_logger().info('--> FASE 5: SOLLEVAMENTO')
        self.muovi_orizzontale(theta1, z_altezza_presa=0.35)
        time.sleep(4.0)
        
        # 7. RITORNO A HOME
        self.get_logger().info('--> FASE 6: RITORNO A HOME')
        self.muovi_home()
        time.sleep(5.0)
        
        self.get_logger().info('Missione Completata!')

    def muovi_home(self):
        # Muove il robot alla configurazione predefinita
        self.muovi_braccio(self.HOME_POS, durata=4.0)

    def muovi_orizzontale(self, theta1, z_altezza_presa):
        # PROTEZIONE
        if z_altezza_presa < self.Z_MINIMA_SICUREZZA:
            self.get_logger().warn(f'Richiesta Z={z_altezza_presa} troppo bassa! Forzo a {self.Z_MINIMA_SICUREZZA}')
            z_altezza_presa = self.Z_MINIMA_SICUREZZA

        # 1. Z Target
        z_polso_target = self.Z_TAVOLO_RISPETTO_BASE + z_altezza_presa
        z_rel = z_polso_target - self.OFFSET_SPALLA
        
        # 2. Raggio Target
        r_totale = math.sqrt(self.target_x**2 + self.target_y**2)
        r_polso = r_totale - self.LUNGHEZZA_MANO
        
        # --- CINEMATICA ---
        d = math.sqrt(r_polso**2 + z_rel**2)
        
        if d > (self.L1 + self.L2):
            self.get_logger().error("Target fuori portata!")
            d = self.L1 + self.L2 - 0.001
            
        cos_theta4 = (self.L1**2 + self.L2**2 - d**2) / (2 * self.L1 * self.L2)
        cos_theta4 = max(-1.0, min(1.0, cos_theta4))
        theta4 = -1.0 * (math.pi - math.acos(cos_theta4)) 

        val_acos = (self.L1**2 + d**2 - self.L2**2) / (2 * self.L1 * d)
        val_acos = max(-1.0, min(1.0, val_acos))
        theta2_geom = math.atan2(z_rel, r_polso) + math.acos(val_acos)
        theta2_robot = (math.pi / 2) - theta2_geom

        # --- FORMULA PARALLELA ---
        theta6 = -1.0 * (theta2_geom + theta4)
        
        if theta6 < 0.0: theta6 = 0.05
        if theta6 > 3.75: theta6 = 3.75

        target_pos = [theta1, theta2_robot, 0.0, theta4, 0.0, theta6, 1.57] 
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
        self.gripper_client.wait_for_server()
        self.gripper_client.send_goal_async(goal)

def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()