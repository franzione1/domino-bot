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
        
        # --- CONFIGURAZIONE BRACCIO E PINZA ---
        self.arm_publisher = self.create_publisher(JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        self.gripper_client = ActionClient(self, GripperCommand, '/panda_hand_controller/gripper_cmd')
        
        # --- SUBSCRIBER VISIONE ---
        self.vision_subscription = self.create_subscription(
            Point, 
            '/domino_position', 
            self.vision_callback, 
            10
        )
        
        self.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        
        # --- POSIZIONI PREDEFINITE ---
        
        # 1. HOME NEUTRA (Alta e indietro): 
        # Serve per non impallare la telecamera mentre cerca i pezzi.
        self.HOME_POS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

        # 2. READY POS (Verticale sul tavolo): 
        # Il robot si mette qui appena inizia la missione, pronto a scendere.
        # J4=-1.57 (gomito 90), J6=1.57 (polso dritto), J7=1.57 (dita per lato lungo)
        self.READY_POS = [0.0, 0.0, 0.0, -1.571, 0.0, 1.571, 1.571]

        # --- PARAMETRI GEOMETRICI ---
        self.L1 = 0.316
        self.L2 = 0.384
        self.OFFSET_SPALLA = 0.33
        self.LUNGHEZZA_MANO = 0.22 
        self.Z_MINIMA_SICUREZZA = 0.24 
        
        # Tuning
        self.CORREZIONE_J1 = 0.0
        self.ALLINEAMENTO_BASE_POLSO = 1.57 
        
        self.is_busy = False 
        
        # All'avvio, andiamo subito in posizione neutra per lasciar vedere la cam
        self.muovi_braccio(self.HOME_POS, durata=5.0)
        self.get_logger().info('NODO PRONTO: Posizione Home raggiunta. In attesa di coordinate...')

    def vision_callback(self, msg):
        if self.is_busy:
            return 

        self.get_logger().info(f'!!! TROVATO DOMINO !!! Coordinate: X={msg.x:.3f}, Y={msg.y:.3f}')
        
        if msg.x == 0.0 and msg.y == 0.0:
            return
            
        distanza = math.sqrt(msg.x**2 + msg.y**2)
        if distanza > 0.85: 
            self.get_logger().warn('Target troppo lontano! Ignorato.')
            return

        self.is_busy = True
        threading.Thread(target=self.esegui_missione, args=(msg.x, msg.y)).start()

    def esegui_missione(self, target_x, target_y):
        self.get_logger().info(f'--> AVVIO PIPELINE per X={target_x}, Y={target_y}')
        
        # FASE 1: POSIZIONAMENTO VERTICALE (READY)
        # Il robot si porta al centro, braccio verticale verso il tavolo
        self.get_logger().info('1. Vado in posizione di PRONTI (Verticale)...')
        self.muovi_braccio(self.READY_POS, durata=3.0)
        time.sleep(3.5) 
        
        # FASE 2: PREPARAZIONE PINZA
        # "Si ferma, apre le dita"
        self.get_logger().info('2. Apro la pinza...')
        self.muovi_pinza(0.04)
        time.sleep(1.5)

        # FASE 3: CALCOLO E ROTAZIONE VERSO IL TARGET
        theta1 = math.atan2(target_y, target_x) + self.CORREZIONE_J1
        theta7_compensato = self.ALLINEAMENTO_BASE_POLSO - theta1
        
        # Approccio Alto (Ruota la base e si estende sopra il pezzo)
        self.get_logger().info('3. Ruoto verso il pezzo e avanzo...')
        self.muovi_orizzontale(target_x, target_y, 0.35, theta1, theta7_compensato)
        time.sleep(3.5)
        
        # FASE 4: DISCESA E PRESA
        self.get_logger().info('4. Scendo per la presa...')
        self.muovi_orizzontale(target_x, target_y, self.Z_MINIMA_SICUREZZA, theta1, theta7_compensato) 
        time.sleep(3.0)
        
        self.muovi_pinza(0.01) # Chiude
        time.sleep(1.0)
        
        # FASE 5: SOLLEVAMENTO
        self.get_logger().info('5. Sollevo...')
        self.muovi_orizzontale(target_x, target_y, 0.35, theta1, theta7_compensato)
        time.sleep(2.5)
        
        # FASE 6: RITORNO A HOME (NEUTRA)
        # Torna indietro in alto per liberare la visuale alla telecamera
        self.get_logger().info('6. Missione finita. Torno in HOME (Alta) per liberare la visuale.')
        self.muovi_braccio(self.HOME_POS, durata=4.0)
        time.sleep(4.0)
        
        self.is_busy = False 

    # --- FUNZIONI DI MOVIMENTO ---
    def muovi_orizzontale(self, x, y, z, theta1, rotazione_j7):
        # Logica cinematica inversa
        if z < self.Z_MINIMA_SICUREZZA: z = self.Z_MINIMA_SICUREZZA
        z_rel = z - self.OFFSET_SPALLA
        r_polso = math.sqrt(x**2 + y**2) - self.LUNGHEZZA_MANO
        d = math.sqrt(r_polso**2 + z_rel**2)
        
        if d > (self.L1 + self.L2): d = self.L1 + self.L2 - 0.001
            
        cos_t4 = (self.L1**2 + self.L2**2 - d**2) / (2 * self.L1 * self.L2)
        theta4 = -1.0 * (math.pi - math.acos(max(-1.0, min(1.0, cos_t4))))

        val_acos = (self.L1**2 + d**2 - self.L2**2) / (2 * self.L1 * d)
        theta2_geom = math.atan2(z_rel, r_polso) + math.acos(max(-1.0, min(1.0, val_acos)))
        theta2_robot = (math.pi / 2) - theta2_geom

        theta6 = -1.0 * (theta2_geom + theta4)
        theta6 = max(0.01, min(3.75, theta6))

        # Normalizzazione J7
        if rotazione_j7 > 2.89: rotazione_j7 -= 3.14
        if rotazione_j7 < -2.89: rotazione_j7 += 3.14

        target_pos = [theta1, theta2_robot, 0.0, theta4, 0.0, theta6, rotazione_j7] 
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
        if not self.gripper_client.wait_for_server(timeout_sec=5.0): return
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