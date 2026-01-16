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
        
        # =========================================================================
        # --- PARAMETRI DI CALIBRAZIONE (MODIFICA QUI SE SBAGLIA POSIZIONE) ---
        # =========================================================================
        self.MOLTIPLICATORE_X = 1.0   # 1.0 o -1.0 (Inverti asse X)
        self.MOLTIPLICATORE_Y = -1.0  # 1.0 o -1.0 (Inverti asse Y)
        
        # CORREZIONI FINI (in metri):
        # Se il robot va troppo "oltre" il tassello, metti un valore negativo (es. -0.02)
        # Se si ferma "prima", metti positivo (es. 0.02)
        self.OFFSET_X = 0.00  
        
        # Se il robot è spostato a sinistra/destra rispetto al centro del tassello
        self.OFFSET_Y = 0.00  
        
        # ALTEZZE
        self.ALTEZZA_BASE = 1.305      
        self.Z_TASELLO = 1.32          
        self.ALTEZZA_PRESA = self.Z_TASELLO + 0.135  # Ho abbassato di 5mm per presa più sicura
        self.ALTEZZA_SICUREZZA = self.Z_TASELLO + 0.35 
        # =========================================================================

        self.L1 = 0.316
        self.L2 = 0.384
        self.HOME_POS = [0.0, -0.78, 0.0, -2.35, 0.0, 1.57, 0.78]
        self.last_target_x = 0.0
        self.last_target_y = 0.0

        self.subscription = self.create_subscription(Point, '/domino_position', self.listener_callback, 10)
        self.arm_publisher = self.create_publisher(JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        
        # Action Clients per la pinza
        self.gripper_client_main = ActionClient(self, GripperCommand, '/panda_hand_controller/gripper_cmd')
        self.gripper_client_left = ActionClient(self, GripperCommand, '/panda_handleft_controller/gripper_cmd')
        self.gripper_client_right = ActionClient(self, GripperCommand, '/panda_handright_controller/gripper_cmd')
            
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
        self.get_logger().info(f'NUOVO TARGET GREZZO: X={msg.x:.2f} Y={msg.y:.2f}')
        self.last_target_x = msg.x
        self.last_target_y = msg.y
        mission = threading.Thread(target=self.esegui_missione, args=(msg.x, msg.y))
        mission.start()

    def esegui_missione(self, raw_x, raw_y):
        # APPLICAZIONE OFFSET E MOLTIPLICATORI
        tx = (raw_x * self.MOLTIPLICATORE_X) + self.OFFSET_X
        ty = (raw_y * self.MOLTIPLICATORE_Y) + self.OFFSET_Y
        
        drop_x = 0.5 * self.MOLTIPLICATORE_X
        drop_y = 0.0 
        
        self.get_logger().info(f'--> 1. AVVICINAMENTO a X_corr={tx:.3f}, Y_corr={ty:.3f}')
        q_appr = self.calcola_ik_stable(tx, ty, self.ALTEZZA_SICUREZZA)
        if q_appr: self.muovi_braccio(q_appr, 4.0)
        time.sleep(4.5)
        
        self.get_logger().info('--> 2. APERTURA PINZA')
        self.muovi_pinza(0.04) 
        time.sleep(1.0)
        
        self.get_logger().info('--> 3. DISCESA')
        q_down = self.calcola_ik_stable(tx, ty, self.ALTEZZA_PRESA)
        if q_down: self.muovi_braccio(q_down, 3.0)
        time.sleep(3.5)
        
        self.get_logger().info('--> 4. PRESA (CHIUSURA)')
        self.muovi_pinza(0.0) 
        time.sleep(2.0) # Tempo extra per afferrare
        
        self.get_logger().info('--> 5. RISALITA')
        if q_appr: self.muovi_braccio(q_appr, 2.5)
        time.sleep(3.0)
        
        self.get_logger().info(f'--> 6. SPOSTAMENTO')
        q_drop = self.calcola_ik_stable(drop_x, drop_y, self.ALTEZZA_SICUREZZA)
        if q_drop:
            q_drop[6] += 1.57 
            self.muovi_braccio(q_drop, 5.0)
        time.sleep(5.5)
        
        self.get_logger().info('--> 7. DEPOSITO')
        q_drop_low = self.calcola_ik_stable(drop_x, drop_y, self.ALTEZZA_PRESA)
        if q_drop_low:
            q_drop_low[6] += 1.57
            self.muovi_braccio(q_drop_low, 3.0)
        time.sleep(3.5)
        
        self.muovi_pinza(0.04) 
        time.sleep(1.0)
        
        self.get_logger().info('--> 8. RITORNO HOME')
        self.muovi_braccio(self.HOME_POS, 4.0)
        time.sleep(4.0)
        
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
        goal = GripperCommand.Goal()
        goal.command.position = apertura
        goal.command.max_effort = 200.0 # Forza aumentata
        
        sent = False
        # Logica di tentativo a cascata
        clients = [
            (self.gripper_client_left, "LEFT"), 
            (self.gripper_client_right, "RIGHT"), 
            (self.gripper_client_main, "MAIN")
        ]
        
        for client, name in clients:
            if client.server_is_ready():
                client.send_goal_async(goal)
                sent = True
        
        if not sent:
            # Riprova veloce attendendo il server (ultima chance)
            if self.gripper_client_main.wait_for_server(timeout_sec=0.5):
                self.gripper_client_main.send_goal_async(goal)
            else:
                self.get_logger().warn('!!! ATTENZIONE: CONTROLLERS PINZA NON ATTIVI !!! Esegui "ros2 control list_controllers"')

    def calcola_ik_stable(self, x, y, z_desiderata):
        z_rel = z_desiderata - 0.33 - self.ALTEZZA_BASE
        theta1 = math.atan2(y, x)
        r = math.sqrt(x**2 + y**2) - 0.05 
        d = math.sqrt(r**2 + z_rel**2)
        if d > (self.L1 + self.L2): d = self.L1 + self.L2 - 0.001
        cos_theta4 = (self.L1**2 + self.L2**2 - d**2) / (2 * self.L1 * self.L2)
        cos_theta4 = max(-1.0, min(1.0, cos_theta4))
        theta4 = -1.0 * (math.pi - math.acos(cos_theta4))
        theta2 = math.atan2(z_rel, r) + math.acos((self.L1**2 + d**2 - self.L2**2) / (2 * self.L1 * d))
        return [theta1, theta2, 0.0, theta4, 0.0, 2.2, 0.79]

def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()