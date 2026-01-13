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
        
        # --- RESET CORREZIONI ---
        # Togliamo la correzione laterale per fidarci della visione (che vede -0.07)
        self.CORREZIONE_LATERALE = 0.0 
        
        self.get_logger().info('Robot Mover: Configurazione ESTESA (Reach 0.55m)')

    def listener_callback(self, msg):
        if self.target_locked:
            return

        target_x = msg.x
        target_y = msg.y
        
        # Applica correzione (ora è 0, quindi usiamo il dato puro)
        target_y_corretto = target_y + self.CORREZIONE_LATERALE
        
        base_angle = math.atan2(target_y_corretto, target_x)
        
        self.target_locked = True
        self.get_logger().info(f'Target X={target_x:.2f} Y={target_y:.2f}. Angolo Base: {base_angle:.2f}. Eseguo...')
        
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        # PUNTO 1: PRONTO (Alto)
        p1 = JointTrajectoryPoint()
        p1.positions = [base_angle, 0.5, 0.0, -1.0, 0.0, 1.5, 0.8]
        p1.time_from_start.sec = 2
        traj_msg.points.append(p1)
        
        # PUNTO 2: AVVICINAMENTO (Esteso)
        p2 = JointTrajectoryPoint()
        # J4 a -1.2 prepara l'estensione
        p2.positions = [base_angle, 1.0, 0.0, -1.2, 0.0, 1.8, 0.8]
        p2.time_from_start.sec = 4
        traj_msg.points.append(p2)

        # PUNTO 3: POSIZIONE GRU (Corretta)
        p3 = JointTrajectoryPoint()
        
        # J2 = 1.3  -> Spalla in avanti (un po' più bassa per aiutare l'estensione)
        # J4 = -1.0 -> Gomito aperto (allunga il braccio orizzontalmente)
        # J6 = 1.6  -> Polso a 90 GRADI ESATTI (Punta il tavolo, non la pancia!)
        # J7 = 0.79 -> Rotazione mano 
        p3.positions = [base_angle, 1.4, 0.0, -1.0, 0.0, 1.6, 0.79]
        
        p3.time_from_start.sec = 8 
        traj_msg.points.append(p3)
        
        self.publisher_.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()