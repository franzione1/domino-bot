import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
import math
import time
import threading

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        
        # --- CONFIGURAZIONE ---
        self.MOLTIPLICATORE_X = 1.0  
        self.MOLTIPLICATORE_Y = -1.0 
        
        # Altezze calibrate
        self.ALTEZZA_BASE = 1.305      
        self.Z_TASELLO = 1.32          
        self.ALTEZZA_PRESA = self.Z_TASELLO + 0.14 # Presa decisa
        self.ALTEZZA_SICUREZZA = self.Z_TASELLO + 0.35 
        
        # Lunghezze braccio
        self.L1 = 0.316
        self.L2 = 0.384
        
        # Posizione HOME (Stabile)
        self.HOME_POS = [0.0, -0.78, 0.0, -2.35, 0.0, 1.57, 0.78]

        # Ultimo target visitato (per evitare loop)
        self.last_target_x = 0.0
        self.last_target_y = 0.0

        # Sottoscrizione Visione
        self.subscription = self.create_subscription(
            Point, '/domino_position', self.listener_callback, 10)
            
        # Publisher Braccio
        self.arm_publisher = self.create_publisher(
            JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        
        # --- CLIENT PINZA MULTIPLI (Per gestire il tuo setup) ---
        # Proviamo a connetterci a tutti i possibili controller della pinza
        self.gripper_client_main = ActionClient(self, GripperCommand, '/panda_hand_controller/gripper_cmd')
        self.gripper_client_left = ActionClient(self, GripperCommand, '/panda_handleft_controller/gripper_cmd')
        self.gripper_client_right = ActionClient(self, GripperCommand, '/panda_handright_controller/gripper_cmd')
            
        self.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        
        self.target_locked = True 
        
        # Timer avvio
        self.timer_init = self.create_timer(1.0, self.mossa_di_risveglio)
        self.get_logger().info('Robot Mover: In attesa di avvio...')

    def mossa_di_risveglio(self):
        self.get_logger().info('>>> RESET HOME E PINZA <<<')
        self.muovi_braccio(self.HOME_POS, durata=4.0)
        self.muovi_pinza(0.04) # Apre
        self.timer_init.cancel()
        threading.Timer(5.0, self.sblocca_target).start()

    def sblocca_target(self):
        self.target_locked = False
        self.get_logger().info('>>> SISTEMA PRONTO: IN ATTESA DI TASSELLI ROSSI <<<')

    def listener_callback(self, msg):
        if self.target_locked:
            return
        
        # --- FILTRO ANTI-LOOP ---
        # Se il nuovo target è vicinissimo all'ultimo eseguito, ignoralo
        distanza = math.sqrt((msg.x - self.last_target_x)**2 + (msg.y - self.last_target_y)**2)
        if distanza < 0.05: # 5 cm di tolleranza
            return

        self.target_locked = True
        self.get_logger().info(f'NUOVO TARGET VALIDO: X={msg.x:.2f} Y={msg.y:.2f}')
        
        # Aggiorna ultimo target
        self.last_target_x = msg.x
        self.last_target_y = msg.y

        mission = threading.Thread(target=self.esegui_missione, args=(msg.x, msg.y))
        mission.start()

    def esegui_missione(self, target_x, target_y):
        tx = target_x * self.MOLTIPLICATORE_X
        ty = target_y * self.MOLTIPLICATORE_Y
        drop_x = 0.5 * self.MOLTIPLICATORE_X
        drop_y = 0.0 
        
        self.get_logger().info(f'--> 1. AVVICINAMENTO a {tx:.2f}, {ty:.2f}')
        q_appr = self.calcola_ik_stable(tx, ty, self.ALTEZZA_SICUREZZA)
        if q_appr: self.muovi_braccio(q_appr, 4.0)
        time.sleep(5.0)
        
        self.get_logger().info('--> 2. APERTURA PINZA')
        self.muovi_pinza(0.04) # 4cm per lato (totale 8cm)
        time.sleep(1.0)
        
        self.get_logger().info('--> 3. DISCESA')
        q_down = self.calcola_ik_stable(tx, ty, self.ALTEZZA_PRESA)
        if q_down: self.muovi_braccio(q_down, 3.0)
        time.sleep(3.5)
        
        self.get_logger().info('--> 4. PRESA (CHIUSURA)')
        self.muovi_pinza(0.0) 
        time.sleep(1.5)
        
        self.get_logger().info('--> 5. RISALITA')
        if q_appr: self.muovi_braccio(q_appr, 3.0)
        time.sleep(3.5)
        
        self.get_logger().info(f'--> 6. SPOSTAMENTO')
        q_drop = self.calcola_ik_stable(drop_x, drop_y, self.ALTEZZA_SICUREZZA)
        if q_drop:
            # Ruotiamo il polso di 90 gradi per allineamento
            q_drop[6] += 1.57 
            self.muovi_braccio(q_drop, 5.0)
        time.sleep(5.5)
        
        self.get_logger().info('--> 7. DEPOSITO')
        q_drop_low = self.calcola_ik_stable(drop_x, drop_y, self.ALTEZZA_PRESA)
        if q_drop_low:
            q_drop_low[6] += 1.57
            self.muovi_braccio(q_drop_low, 3.0)
        time.sleep(3.5)
        
        self.muovi_pinza(0.04) # Rilascia
        time.sleep(1.0)
        
        self.get_logger().info('--> 8. FINE MISSIONE')
        self.muovi_braccio(self.HOME_POS, 4.0)
        time.sleep(4.0)
        
        self.get_logger().info('Missione conclusa. Attendo nuovi target (diversi dal precedente)...')
        self.target_locked = False

    def muovi_braccio(self, posizioni, durata):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = posizioni
        point.time_from_start.sec = int(durata)
        msg.points.append(point)
        self.arm_publisher.publish(msg)

    def muovi_pinza(self, apertura):
        """Manda il comando a TUTTI i possibili controller della pinza"""
        goal = GripperCommand.Goal()
        goal.command.position = apertura
        goal.command.max_effort = 100.0
        
        sent = False
        # Tenta il controller Left (per il tuo yaml)
        if self.gripper_client_left.wait_for_server(timeout_sec=0.2):
            self.gripper_client_left.send_goal_async(goal)
            sent = True
        
        # Tenta il controller Right (per il tuo yaml)
        if self.gripper_client_right.wait_for_server(timeout_sec=0.2):
            self.gripper_client_right.send_goal_async(goal)
            sent = True
            
        # Tenta il controller Standard (caso fallback)
        if not sent and self.gripper_client_main.wait_for_server(timeout_sec=0.2):
            self.gripper_client_main.send_goal_async(goal)
            sent = True
            
        if not sent:
            self.get_logger().warn('ATTENZIONE: Nessun controller pinza trovato! (Ho provato main, left e right)')

    def calcola_ik_stable(self, x, y, z_desiderata):
        """IK Semplificato e Stabile per evitare rotazioni pazze"""
        z_rel = z_desiderata - 0.33 - self.ALTEZZA_BASE
        
        theta1 = math.atan2(y, x)
        r = math.sqrt(x**2 + y**2) - 0.05 
        d = math.sqrt(r**2 + z_rel**2)
        
        if d > (self.L1 + self.L2):
            d = self.L1 + self.L2 - 0.001

        cos_theta4 = (self.L1**2 + self.L2**2 - d**2) / (2 * self.L1 * self.L2)
        cos_theta4 = max(-1.0, min(1.0, cos_theta4))
        theta4 = -1.0 * (math.pi - math.acos(cos_theta4)) # Gomito alto

        alpha = math.atan2(z_rel, r)
        beta = math.acos((self.L1**2 + d**2 - self.L2**2) / (2 * self.L1 * d))
        theta2 = alpha + beta

        # --- FIX STABILITÀ ---
        # Invece di calcolare un theta6 "perfetto" che fa impazzire il robot,
        # usiamo un valore fisso che punta verso il basso in modo naturale.
        # 2.0 - 2.5 radianti è un buon compromesso per il Panda.
        theta6 = 2.2 
        
        return [theta1, theta2, 0.0, theta4, 0.0, theta6, 0.79]

def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()fi