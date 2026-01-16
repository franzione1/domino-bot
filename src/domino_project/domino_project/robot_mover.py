import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        
        self.subscription = self.create_subscription(
            Point, '/domino_position', self.listener_callback, 10)
            
        self.publisher_ = self.create_publisher(
            JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
            
        self.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        
        self.target_locked = False
        
        # --- PARAMETRI ---
        self.ALTEZZA_PRESA = 0.05
        self.ALTEZZA_SICUREZZA = 0.30
        self.L1 = 0.316
        self.L2 = 0.384
        
        # --- DEFINIZIONE POSIZIONE "HOME" (Alta e Arretrata) ---
        # J2=0.0 (Dritto verticale), J4=-1.5 (Gomito a 90°), J6=1.57 (Polso dritto)
        # Questa posa libera sicuramente la visuale della telecamera
        self.HOME_POS = [0.0, 0.0, 0.0, -1.5, 0.0, 1.57, 0.78]

        # --- AUTO-RESET ALL'AVVIO ---
        # Creiamo un timer che scatta UNA volta sola dopo 1 secondo
        # per spostare il robot via dalla telecamera appena accendi il nodo.
        self.timer_init = self.create_timer(1.0, self.mossa_di_risveglio)
        
        self.get_logger().info('Robot Mover: In attesa... (Tra 1s eseguo reset posizione)')

    def mossa_di_risveglio(self):
        """Questa funzione viene chiamata SOLO all'avvio per liberare la visuale."""
        self.get_logger().info('>>> ESEGUO RESET POSIZIONE (Sposto il robot in HOME) <<<')
        
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        # Creiamo un punto per andare a casa in 4 secondi
        p_home = JointTrajectoryPoint()
        p_home.positions = self.HOME_POS
        p_home.time_from_start.sec = 4
        traj_msg.points.append(p_home)
        
        self.publisher_.publish(traj_msg)
        
        # Spegniamo il timer (non deve farlo più)
        self.timer_init.cancel()

    def calcola_ik(self, x, y, z_desiderata):
        theta1 = math.atan2(y, x)
        r = math.sqrt(x**2 + y**2) - 0.05 
        z_rel = z_desiderata - 0.33
        d = math.sqrt(r**2 + z_rel**2)
        
        if d > (self.L1 + self.L2):
            d = self.L1 + self.L2 - 0.01 
            
        cos_theta4 = (self.L1**2 + self.L2**2 - d**2) / (2 * self.L1 * self.L2)
        theta4 = -1.0 * (math.pi - math.acos(max(-1.0, min(1.0, cos_theta4)))) 
        
        alpha = math.atan2(z_rel, r)
        beta = math.acos((self.L1**2 + d**2 - self.L2**2) / (2 * self.L1 * d))
        theta2 = alpha + beta 
        
        # Calcolo Polso perpendicolare (Offset regolabile se punta avanti/indietro)
        theta6 = 3.14 - (theta2 + theta4) + 0.5 
        
        return theta1, theta2, theta4, theta6

    def listener_callback(self, msg):
        if self.target_locked:
            return

        tx = msg.x
        ty = msg.y
        drop_x = tx
        drop_y = ty - 0.15 
        
        self.target_locked = True
        self.get_logger().info(f'Target trovato! Eseguo sequenza completa con ritorno HOME.')
        
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        # --- SEQUENZA CON RITORNO A HOME ---
        
        # PUNTO 0: TORNA A HOME (Sicurezza prima di scendere)
        # Anche se è già lì, glielo ridiciamo per sicurezza
        p0 = JointTrajectoryPoint()
        p0.positions = self.HOME_POS
        p0.time_from_start.sec = 3 # Si prende 3 secondi per resettarsi
        traj_msg.points.append(p0)
        
        # PUNTO 1: AVVICINAMENTO (Sopra il tassello)
        j1, j2, j4, j6 = self.calcola_ik(tx, ty, self.ALTEZZA_SICUREZZA)
        p1 = JointTrajectoryPoint()
        p1.positions = [j1, j2, 0.0, j4, 0.0, j6, 0.79]
        p1.time_from_start.sec = 6 # Tempo aumentato perché parte da Home
        traj_msg.points.append(p1)
        
        # PUNTO 2: PRESA (Giù)
        j1, j2, j4, j6 = self.calcola_ik(tx, ty, self.ALTEZZA_PRESA)
        p2 = JointTrajectoryPoint()
        p2.positions = [j1, j2, 0.0, j4, 0.0, j6, 0.79]
        p2.time_from_start.sec = 9
        traj_msg.points.append(p2)
        
        # PUNTO 3: SU (Sicurezza)
        j1_up, j2_up, j4_up, j6_up = self.calcola_ik(tx, ty, self.ALTEZZA_SICUREZZA)
        p3 = JointTrajectoryPoint()
        p3.positions = [j1_up, j2_up, 0.0, j4_up, 0.0, j6_up, 0.79]
        p3.time_from_start.sec = 11
        traj_msg.points.append(p3)
        
        # PUNTO 4: SPOSTA (Ruota 90°)
        j1_drop, j2_drop, j4_drop, j6_drop = self.calcola_ik(drop_x, drop_y, self.ALTEZZA_SICUREZZA)
        p4 = JointTrajectoryPoint()
        p4.positions = [j1_drop, j2_drop, 0.0, j4_drop, 0.0, j6_drop, 2.36] 
        p4.time_from_start.sec = 14
        traj_msg.points.append(p4)

        # PUNTO 5: GIÙ (Deposita)
        j1_drop_low, j2_drop_low, j4_drop_low, j6_drop_low = self.calcola_ik(drop_x, drop_y, self.ALTEZZA_PRESA)
        p5 = JointTrajectoryPoint()
        p5.positions = [j1_drop_low, j2_drop_low, 0.0, j4_drop_low, 0.0, j6_drop_low, 2.36]
        p5.time_from_start.sec = 17
        traj_msg.points.append(p5)
        
        # PUNTO 6: FINALE (Torna a Home trionfante)
        p6 = JointTrajectoryPoint()
        p6.positions = self.HOME_POS
        p6.time_from_start.sec = 20
        traj_msg.points.append(p6)
        
        self.publisher_.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()