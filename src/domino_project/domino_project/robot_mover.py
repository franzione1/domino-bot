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
        self.OFFSET_X = -0.03 
        self.OFFSET_Y = 0.00  
        
        # --- ALTEZZE GEOMETRICHE ---
        self.ALTEZZA_BASE = 1.305      
        self.Z_TASELLO = 1.32          
        self.LUNGHEZZA_MANO = 0.22 
        
        # Altezze target del POLSO (Wrist)
        self.Z_POLSO_VOLO = (self.Z_TASELLO + self.LUNGHEZZA_MANO) + 0.25 
        self.Z_POLSO_PRESA = (self.Z_TASELLO + self.LUNGHEZZA_MANO) + 0.11
        
        self.TEMPO_PAUSA_DEBUG = 3.0 
        
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
        self.get_logger().warn(f'[DEBUG] Pausa {self.TEMPO_PAUSA_DEBUG}s...')
        time.sleep(self.TEMPO_PAUSA_DEBUG)

    def esegui_presa_diretta(self, raw_x, raw_y):
        tx = (raw_x * self.MOLTIPLICATORE_X) + self.OFFSET_X
        ty = (raw_y * self.MOLTIPLICATORE_Y) + self.OFFSET_Y
        drop_x = 0.5 * self.MOLTIPLICATORE_X
        drop_y = 0.0 
        
        # 1. VOLO
        self.get_logger().info(f'--> 1. VOLO (Z_Polso={self.Z_POLSO_VOLO:.2f})')
        q_approccio = self.calcola_ik_stable(tx, ty, self.Z_POLSO_VOLO)
        if q_approccio: 
            self.muovi_braccio(q_approccio, 4.0)
            time.sleep(4.5)
            self.pausa_debug() 
        
        # 2. APERTURA
        self.get_logger().info('--> 2. APERTURA')
        self.muovi_pinza(0.04) 
        time.sleep(0.5)
        
        # 3. DISCESA
        self.get_logger().info(f'--> 3. DISCESA (Z_Polso={self.Z_POLSO_PRESA:.2f})')
        q_giu = self.calcola_ik_stable(tx, ty, self.Z_POLSO_PRESA)
        if q_giu: 
            self.muovi_braccio(q_giu, 3.0) 
            time.sleep(3.5)
            self.pausa_debug() # ORA LA MANO DOVREBBE ESSERE DRITTA!
        
        # 4. PRESA
        self.get_logger().info('--> 4. CHIUSURA')
        self.muovi_pinza(0.0) 
        time.sleep(1.0) 
        self.pausa_debug() 
        
        # 5. RISALITA
        self.get_logger().info('--> 5. RISALITA')
        if q_approccio: self.muovi_braccio(q_approccio, 2.0)
        time.sleep(2.5)
        
        # 6. DEPOSITO
        self.get_logger().info(f'--> 6. DEPOSITO')
        q_drop = self.calcola_ik_stable(drop_x, drop_y, self.Z_POLSO_VOLO)
        if q_drop:
            q_drop[6] += 1.57 # Ruota polso
            self.muovi_braccio(q_drop, 4.0)
        time.sleep(4.5)
        self.pausa_debug() 
        
        self.muovi_pinza(0.04) 
        time.sleep(1.0)
        
        self.get_logger().info('--> 7. HOME')
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

    def calcola_ik_stable(self, x, y, z_wrist_abs):
        z_rel = z_wrist_abs - (self.ALTEZZA_BASE + 0.33)
        theta1 = math.atan2(y, x)
        r = math.sqrt(x**2 + y**2) 
        d = math.sqrt(r**2 + z_rel**2)
        
        if d > (self.L1 + self.L2): d = self.L1 + self.L2 - 0.001
        
        cos_theta4 = (self.L1**2 + self.L2**2 - d**2) / (2 * self.L1 * self.L2)
        cos_theta4 = max(-1.0, min(1.0, cos_theta4))
        theta4 = -1.0 * (math.pi - math.acos(cos_theta4))
        
        val_acos = (self.L1**2 + d**2 - self.L2**2) / (2 * self.L1 * d)
        val_acos = max(-1.0, min(1.0, val_acos))
        theta2 = math.atan2(z_rel, r) + math.acos(val_acos)
        
        # --- FIX DEFINITIVO MANO ---
        # Formula geometrica: per puntare in basso (-90 gradi),
        # il polso (J6) deve compensare l'inclinazione di spalla e gomito.
        # J6 = (theta2 + theta4) + 90 gradi
        theta6 = (theta2 + theta4) + 1.57
        
        # Controllo di sicurezza per il giunto Panda (Range -0.01 a 3.75)
        if theta6 < 0.0: theta6 = 0.0
        if theta6 > 3.75: theta6 = 3.75
            
        return [theta1, theta2, 0.0, theta4, 0.0, theta6, 0.79]

def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()