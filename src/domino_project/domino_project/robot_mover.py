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
        
        # --- TUNING ---
        self.HOME_POS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        
        self.OFFSET_LATERALE_BRACCIO = 0.02 
        self.OFFSET_J7_POLSO = 0.735
        self.OFFSET_DISTANZA_REACH = 0.0
        self.TARGET_WORLD_ANGLE = 1.57
        
        self.L1 = 0.316
        self.L2 = 0.384
        self.OFFSET_SPALLA = 0.33
        self.LUNGHEZZA_MANO = 0.22 
        self.Z_SICUREZZA = 0.235
        self.Z_ALTA = 0.35        
        
        self.CENTER_X = 0.50
        self.CENTER_Y = 0.00
        
        # Mappa per decodificare il colore
        self.COLOR_MAP = {1.0: "ROSSO", 2.0: "VERDE", 3.0: "BLU"}
        
        self.is_busy = False 
        self.muovi_braccio(self.HOME_POS, durata=5.0)
        self.get_logger().info('ROBOT PRONTO. Attendo target e colore...')

    def vision_callback(self, msg):
        if self.is_busy: return 
        if msg.x == 0.0 and msg.y == 0.0: return
        if math.sqrt(msg.x**2 + msg.y**2) > 0.85: return 

        self.is_busy = True
        # Passiamo anche msg.z (codice colore) al thread
        threading.Thread(target=self.esegui_missione, args=(msg.x, msg.y, msg.z)).start()

    def esegui_missione(self, target_x, target_y, color_code):
        # Decodifica Colore
        nome_colore = self.COLOR_MAP.get(float(color_code), "SCONOSCIUTO")
        
        # 1. Calcoli Angolari
        distanza_totale = math.sqrt(target_x**2 + target_y**2)
        theta_base_atan = math.atan2(target_y, target_x)
        
        try:
            delta_correction = math.asin(self.OFFSET_LATERALE_BRACCIO / distanza_totale)
        except ValueError:
            delta_correction = 0.0 
            
        theta1_scelto = theta_base_atan - delta_correction
        theta7_scelto = (self.TARGET_WORLD_ANGLE - theta1_scelto) + self.OFFSET_J7_POLSO

        # --- FUNZIONE LOG PERSISTENTE ---
        def log_status(fase):
            print("\n" + "="*40)
            print(f"  MISSIONE ATTIVA: {nome_colore}")   # <--- QUI MOSTRA IL COLORE
            print("="*40)
            print(f"  Target X : {target_x:.3f}")
            print(f"  Target Y : {target_y:.3f}")
            print("-" * 40)
            print(f"  ANGOLO Base (Calc) : {math.degrees(theta_base_atan):.2f}°")
            print(f"  CORREZIONE Offset  : {math.degrees(-delta_correction):.2f}°")
            print(f"  -> ANGOLO SCELTO   : {math.degrees(theta1_scelto):.2f}°")
            print("-" * 40)
            print(f"  STATUS: {fase}")
            print("="*40 + "\n")

        # --- ESECUZIONE ---
        log_status("INIZIO - Calcolo Completato")
        
        self.muovi_cinematica(target_x, target_y, self.Z_ALTA, theta1_scelto, theta7_scelto)
        log_status("Approccio Alto")
        time.sleep(4.0)
        
        self.muovi_pinza(0.04)
        time.sleep(1.0)
        self.muovi_cinematica(target_x, target_y, self.Z_SICUREZZA, theta1_scelto, theta7_scelto)
        log_status("Discesa sul pezzo")
        time.sleep(3.0)
        
        self.muovi_pinza(0.0265) # Chiusura pinza
        time.sleep(1.5)
        log_status("PRESA EFFETTUATA")

        self.muovi_cinematica(target_x, target_y, self.Z_ALTA, theta1_scelto, theta7_scelto)
        log_status("Sollevamento")
        time.sleep(3.0)
        
        # Piazzamento
        dest_x = self.CENTER_X
        dest_y = self.CENTER_Y + 0.06
        theta1_dest = math.atan2(dest_y, dest_x)
        theta7_dest = (self.TARGET_WORLD_ANGLE - theta1_dest)
        
        self.muovi_cinematica(dest_x, dest_y, self.Z_ALTA, theta1_dest, theta7_dest)
        log_status("Spostamento verso Centro")
        time.sleep(4.0)
        
        self.muovi_cinematica(dest_x, dest_y, 0.25, theta1_dest, theta7_dest)
        time.sleep(3.0)
        
        self.muovi_pinza(0.04)
        time.sleep(1.5)
        
        self.muovi_braccio(self.HOME_POS, durata=4.0)
        log_status("Ritorno in Home")
        time.sleep(4.0)
        
        self.is_busy = False 
        print(f"\n[FINE MISSIONE {nome_colore}] Torno in attesa.\n")

    def muovi_cinematica(self, x, y, z, theta1_cmd, theta7_cmd):
        if z < self.Z_SICUREZZA: z = self.Z_SICUREZZA
        z_rel = z - self.OFFSET_SPALLA
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