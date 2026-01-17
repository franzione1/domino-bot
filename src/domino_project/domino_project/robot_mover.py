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
        
        # --- PARAMETRI DI CALIBRAZIONE ---
        self.MOLTIPLICATORE_X = 1.0  
        self.MOLTIPLICATORE_Y = -1.0 
        self.OFFSET_X = 0.00  
        self.OFFSET_Y = 0.00  
        
        # --- ALTEZZE ---
        self.ALTEZZA_BASE = 1.305      
        self.Z_TASELLO = 1.32          
        self.Z_VOLO = self.Z_TASELLO + 0.35 
        
        # Altezza presa (modifica se non tocca)
        self.Z_PRESA = self.Z_TASELLO + 0.11
        
        # --- CONFIGURAZIONE DEBUG ---
        # Tempo di attesa tra uno step e l'altro per permetterti di controllare
        self.TEMPO_PAUSA_DEBUG = 5.0 
        
        self.L1 = 0.316
        self.L2 = 0.384
        self.HOME_POS = [0.0, -0.78, 0.0, -2.35, 0.0, 1.57, 0.78]
        
        self.last_target_x = 0.0
        self.last_target_y = 0.0

        self.subscription = self.create_subscription(Point, '/domino_position', self.listener_callback, 10)
        self.arm_publisher = self.create_publisher(JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        self.gripper_client = ActionClient(self, GripperCommand, '/panda_hand_controller/gripper_cmd')
            
        self.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        
        self.target_locked = True 
        self.timer_init = self.create_timer(1.0, self.mossa_di_risveglio)
        self.get_logger().info('Robot Mover: In attesa di avvio...')

    def mossa_di_risveglio(self):
        self.get_logger().info('>>> RESET HOME <<<')
        self.muovi_braccio(self.HOME_POS, durata=4.0)
        self.muovi_pinza(0.04) 
        self.timer_init.cancel()
        threading.Timer(5.0, self.sblocca_target).start()

    def sblocca_target(self):
        self.target_locked = False
        self.get_logger().info('>>> SISTEMA PRONTO <<<')

    def listener_callback(self, msg):
        if self.target_locked: return
        
        distanza = math.sqrt((msg.x - self.last_target_x)**2 + (msg.y - self.last_target_y)**2)
        if distanza < 0.05: return

        self.target_locked = True
        self.last_target_x = msg.x
        self.last_target_y = msg.y
        
        self.get_logger().info(f'TARGET ACQUISITO: X={msg.x:.3f} Y={msg.y:.3f}')
        
        mission = threading.Thread(target=self.esegui_presa_diretta, args=(msg.x, msg.y))
        mission.start()

    def pausa_debug(self):
        """Funzione helper per fermare il robot tra gli step"""
        self.get_logger().warn(f'[PAUSA ISPEZIONE] Fermo per {self.TEMPO_PAUSA_DEBUG} secondi...')
        time.sleep(self.TEMPO_PAUSA_DEBUG)

    def esegui_presa_diretta(self, raw_x, raw_y):
        tx = (raw_x * self.MOLTIPLICATORE_X) + self.OFFSET_X
        ty = (raw_y * self.MOLTIPLICATORE_Y) + self.OFFSET_Y
        
        drop_x = 0.5 * self.MOLTIPLICATORE_X
        drop_y = 0.0 
        
        # --- STEP 1: APPROCCIO ---
        self.get_logger().info(f'--> 1. VADO SOPRA IL TASSELLO (X={tx:.3f}, Y={ty:.3f})')
        q_approccio = self.calcola_ik_stable(tx, ty, self.Z_VOLO)
        if q_approccio: self.muovi_braccio(q_approccio, 4.0)
        time.sleep(4.5) # Aspetta fine movimento
        self.pausa_debug() # <--- PAUSA QUI
        
        # --- STEP 2: APERTURA ---
        self.get_logger().info('--> 2. APERTURA PINZA')
        self.muovi_pinza(0.04) 
        time.sleep(0.5)
        # (Qui non metto pausa lunga perchè la pinza è veloce)
        
        # --- STEP 3: DISCESA ---
        self.get_logger().info(f'--> 3. DISCESA FINALE A Z={self.Z_PRESA:.3f}')
        q_giu = self.calcola_ik_stable(tx, ty, self.Z_PRESA)
        if q_giu: self.muovi_braccio(q_giu, 3.0) 
        time.sleep(3.5)
        self.pausa_debug() # <--- PAUSA CRUCIALE: Controlla se tocca il tassello!
        
        # --- STEP 4: CHIUSURA ---
        self.get_logger().info('--> 4. CHIUSURA (GRASP)')
        self.muovi_pinza(0.0) 
        time.sleep(1.0) 
        self.pausa_debug() # <--- PAUSA: Controlla se l'ha preso
        
        # --- STEP 5: RISALITA ---
        self.get_logger().info('--> 5. RISALITA')
        if q_approccio: self.muovi_braccio(q_approccio, 2.0)
        time.sleep(2.5)
        
        # --- STEP 6: TRASPORTO ---
        self.get_logger().info(f'--> 6. TRASPORTO E DEPOSITO')
        q_drop = self.calcola_ik_stable(drop_x, drop_y, self.Z_VOLO)
        if q_drop:
            q_drop[6] += 1.57
            self.muovi_braccio(q_drop, 4.0)
        time.sleep(4.5)
        self.pausa_debug() # <--- PAUSA
        
        # Discesa deposito
        q_drop_low = self.calcola_ik_stable(drop_x, drop_y, self.Z_PRESA + 0.02)
        if q_drop_low:
            q_drop_low[6] += 1.57
            self.muovi_braccio(q_drop_low, 2.0)
        time.sleep(2.5)
        
        self.muovi_pinza(0.04) # Rilascia
        time.sleep(1.0)
        
        # --- STEP 7: HOME ---
        self.get_logger().info('--> 7. RITORNO HOME')
        self.muovi_braccio(self.HOME_POS, 4.0)
        time.sleep(4.0)
        
        self.target_locked = False
        self.get_logger().info('Missione completata.')

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
        goal.command.max_effort = 200.0 
        if self.gripper_client.wait_for_server(timeout_sec=0.5):
            self.gripper_client.send_goal_async(goal)

    def calcola_ik_stable(self, x, y, z_desiderata):
        z_rel = z_desiderata - 0.33 - self.ALTEZZA_BASE
        theta1 = math.atan2(y, x)
        r = math.sqrt(x**2 + y**2) - 0.05 
        d = math.sqrt(r**2 + z_rel**2)
        if d > (self.L1 + self.L2): d = self.L1 + self.L2 - 0.001
        
        cos_theta4 = (self.L1**2 + self.L2**2 - d**2) / (2 * self.L1 * self.L2)
        cos_theta4 = max(-1.0, min(1.0, cos_theta4))
        theta4 = -1.0 * (math.pi - math.acos(cos_theta4))
        
        val_acos = (self.L1**2 + d**2 - self.L2**2) / (2 * self.L1 * d)
        val_acos = max(-1.0, min(1.0, val_acos))
        theta2 = math.atan2(z_rel, r) + math.acos(val_acos)
        
        return [theta1, theta2, 0.0, theta4, 0.0, 2.2, 0.79]

def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()