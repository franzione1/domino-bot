import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point # <--- Importiamo il tipo di messaggio per le coordinate
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
        # Il robot si mette in ascolto sul topic che hai trovato
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
        
        # --- PARAMETRI GEOMETRICI ---
        self.HOME_POS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        self.L1 = 0.316
        self.L2 = 0.384
        self.OFFSET_SPALLA = 0.33
        self.LUNGHEZZA_MANO = 0.22 
        self.Z_MINIMA_SICUREZZA = 0.24 
        
        # Tuning
        self.CORREZIONE_J1 = 0.0
        self.ALLINEAMENTO_BASE_POLSO = 1.57 
        
        # Stato del Robot
        self.is_busy = False # Per evitare che parta 10 volte se arrivano 10 messaggi
        
        self.get_logger().info('NODO PRONTO: In attesa di coordinate su /domino_position ...')

    def vision_callback(self, msg):
        """
        Questa funzione scatta AUTOMATICAMENTE ogni volta che 
        la telecamera pubblica un messaggio.
        """
        if self.is_busy:
            return # Se stiamo già prendendo un pezzo, ignoriamo i nuovi messaggi

        self.get_logger().info(f'!!! TROVATO DOMINO !!! Coordinate ricevute: X={msg.x:.3f}, Y={msg.y:.3f}')
        
        # Controllo di sicurezza: se la cam dà 0,0 (errore) o valori assurdi, ignoriamo
        if msg.x == 0.0 and msg.y == 0.0:
            return
            
        distanza = math.sqrt(msg.x**2 + msg.y**2)
        if distanza > 0.85: # Fuori dalla portata del braccio
            self.get_logger().warn('Target troppo lontano! Ignorato.')
            return

        # Impostiamo il flag occupato e avviamo la missione in un thread separato
        self.is_busy = True
        
        # Passiamo le coordinate al thread
        threading.Thread(target=self.esegui_missione, args=(msg.x, msg.y)).start()

    def esegui_missione(self, target_x, target_y):
        self.get_logger().info(f'--> AVVIO MOVIMENTO verso X={target_x}, Y={target_y}')
        
        # 0. HOME
        self.muovi_home()
        time.sleep(4.0)
        
        # 1. APERTURA PINZA
        self.muovi_pinza(0.04)
        time.sleep(2.0)

        # --- CALCOLI CINEMATICA ---
        theta1 = math.atan2(target_y, target_x) + self.CORREZIONE_J1
        
        # Compensazione rotazione polso (per mantenerlo perpendicolare)
        theta7_compensato = self.ALLINEAMENTO_BASE_POLSO - theta1
        
        # 2. APPROCCIO ALTO
        self.muovi_orizzontale(target_x, target_y, 0.35, theta1, theta7_compensato)
        time.sleep(4.0)
        
        # 3. DISCESA
        self.muovi_orizzontale(target_x, target_y, self.Z_MINIMA_SICUREZZA, theta1, theta7_compensato) 
        time.sleep(4.0)
        
        # 4. PRESA (Chiude a 1cm per sicurezza)
        self.muovi_pinza(0.01) 
        time.sleep(2.0)
        
        # 5. SOLLEVAMENTO
        self.muovi_orizzontale(target_x, target_y, 0.35, theta1, theta7_compensato)
        time.sleep(4.0)
        
        # 6. RITORNO
        self.muovi_home()
        self.get_logger().info('MISSIONE COMPLETATA! Torno in ascolto...')
        
        # Rilasciamo il flag: ora il robot è pronto per un nuovo messaggio
        self.is_busy = False 

    # --- FUNZIONI DI MOVIMENTO ---
    def muovi_home(self):
        self.muovi_braccio(self.HOME_POS, durata=4.0)

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
        if not self.gripper_client.wait_for_server(timeout_sec=5.0): return
        self.gripper_client.send_goal_async(goal)

def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    # rclpy.spin è fondamentale: mantiene il programma vivo per ascoltare i messaggi
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()