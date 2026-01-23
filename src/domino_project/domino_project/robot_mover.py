import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point 
import math
import time
import threading

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        
        self.arm_publisher = self.create_publisher(JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        self.gripper_client = ActionClient(self, GripperCommand, '/panda_hand_controller/gripper_cmd')
        
        self.vision_subscription = self.create_subscription(Point, '/domino_position', self.vision_callback, 10)
        
        self.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        
        # =========================================================================
        #                         SEZIONE CALIBRAZIONE (TUNING)
        # =========================================================================
        
        # 1. POSIZIONE HOME (Neutra, in alto)
        # J7 = 0.785 (45 gradi) è solo estetico per la posa di riposo, non influenza la presa.
        self.HOME_POS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        
        # 2. CALIBRAZIONE FINALE
        # Dato che il tuo robot parte da 0.0 (allineato), lasciamo questo a 0.
        self.OFFSET_J7_POLSO = 1.57
        
        # Se il braccio arriva corto o lungo, modifica questo (es. 0.02 per +2cm)
        self.OFFSET_DISTANZA_REACH = 0.0
        
        # Se il braccio è storto rispetto al pezzo (sinistra/destra)
        self.OFFSET_J1_BASE = 0.0 
        
        # Angolo target nel mondo (1.57 = 90 gradi = Perpendicolare ai pezzi su X)
        self.TARGET_WORLD_ANGLE = 1.57
        
        # =========================================================================
        
        self.L1 = 0.316
        self.L2 = 0.384
        self.OFFSET_SPALLA = 0.33
        self.LUNGHEZZA_MANO = 0.22 
        self.Z_SICUREZZA = 0.235
        self.Z_ALTA = 0.35        
        
        self.CENTER_X = 0.50
        self.CENTER_Y = 0.00
        
        self.is_busy = False 
        
        # Reset iniziale verso Home
        self.muovi_braccio(self.HOME_POS, durata=5.0)
        self.get_logger().info('ROBOT PRONTO. Parametri calibrati per start a 0.0.')

    def vision_callback(self, msg):
        if self.is_busy: return 
        if msg.x == 0.0 and msg.y == 0.0: return
        if math.sqrt(msg.x**2 + msg.y**2) > 0.85: return 

        self.is_busy = True
        threading.Thread(target=self.esegui_missione, args=(msg.x, msg.y)).start()

    def esegui_missione(self, target_x, target_y):
        self.get_logger().info(f'--- MISSIONE: Target X={target_x:.3f}, Y={target_y:.3f} ---')
        
        # --- CALCOLO ANGOLI ---
        theta1 = math.atan2(target_y, target_x) + self.OFFSET_J1_BASE
        
        # Compensazione Polso: 90° - RotazioneBase + Offset
        theta7 = (self.TARGET_WORLD_ANGLE - theta1) + self.OFFSET_J7_POLSO
        
        self.get_logger().info(f'Calcoli: J1={theta1:.2f}, J7={theta7:.2f}')

        # 1. APPROCCIO ALTO
        self.muovi_cinematica(target_x, target_y, self.Z_ALTA, theta1, theta7)
        time.sleep(4.0)
        
        # 2. DISCESA
        self.muovi_pinza(0.04)
        time.sleep(1.0)
        self.muovi_cinematica(target_x, target_y, self.Z_SICUREZZA, theta1, theta7)
        time.sleep(3.0)
        
        # 3. PRESA
        self.muovi_pinza(0.01) 
        time.sleep(1.5)
        
        # 4. SOLLEVAMENTO
        self.muovi_cinematica(target_x, target_y, self.Z_ALTA, theta1, theta7)
        time.sleep(3.0)
        
        # 5. PIAZZAMENTO (a sinistra del centro)
        dest_x = self.CENTER_X
        dest_y = self.CENTER_Y + 0.06
        
        theta1_dest = math.atan2(dest_y, dest_x) + self.OFFSET_J1_BASE
        theta7_dest = (self.TARGET_WORLD_ANGLE - theta1_dest) + self.OFFSET_J7_POLSO
        
        self.muovi_cinematica(dest_x, dest_y, self.Z_ALTA, theta1_dest, theta7_dest)
        time.sleep(4.0)
        
        self.muovi_cinematica(dest_x, dest_y, 0.25, theta1_dest, theta7_dest)
        time.sleep(3.0)
        
        self.muovi_pinza(0.04)
        time.sleep(1.5)
        
        self.muovi_braccio(self.HOME_POS, durata=4.0)
        time.sleep(4.0)
        
        self.is_busy = False 

    def muovi_cinematica(self, x, y, z, theta1_cmd, theta7_cmd):
        if z < self.Z_SICUREZZA: z = self.Z_SICUREZZA
        z_rel = z - self.OFFSET_SPALLA
        
        # Calcolo raggio con correzione distanza
        r_totale = math.sqrt(x**2 + y**2) + self.OFFSET_DISTANZA_REACH
        r_polso = r_totale - self.LUNGHEZZA_MANO
        d = math.sqrt(r_polso**2 + z_rel**2)
        
        if d > (self.L1 + self.L2): d = self.L1 + self.L2 - 0.001
            
        cos_t4 = (self.L1**2 + self.L2**2 - d**2) / (2 * self.L1 * self.L2)
        theta4 = -1.0 * (math.pi - math.acos(max(-1.0, min(1.0, cos_t4))))

        val_acos = (self.L1**2 + d**2 - self.L2**2) / (2 * self.L1 * d)
        theta2_geom = math.atan2(z_rel, r_polso) + math.acos(max(-1.0, min(1.0, val_acos)))
        theta2_robot = (math.pi / 2) - theta2_geom

        theta6 = -1.0 * (theta2_geom + theta4)
        
        # Normalizzazione J7 (-PI a +PI)
        while theta7_cmd > 3.14: theta7_cmd -= 6.28
        while theta7_cmd < -3.14: theta7_cmd += 6.28

        target_pos = [theta1_cmd, theta2_robot, 0.0, theta4, 0.0, theta6, theta7_cmd] 
        self.muovi_braccio(target_pos, durata=3.0)

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
        if not self.gripper_client.wait_for_server(timeout_sec=1.0): return
        self.gripper_client.send_goal_async(goal)

def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()