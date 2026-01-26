#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>
#include <cmath>

// Costanti
const double Z_ALTA = 0.35;
const double Z_PRESA = 0.235;
const double DROP_X = 0.50;
const double DROP_Y = 0.00;

class RobotMover : public rclcpp::Node
{
public:
  RobotMover() : Node("robot_mover_cpp")
  {
    // Subscriber per la visione
    subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/domino_position", 10, std::bind(&RobotMover::vision_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "NODO C++ MOVEIT AVVIATO. In attesa di target...");
  }

  void setup_moveit()
  {
    // Inizializzazione MoveIt (deve girare su un thread separato per non bloccare ROS)
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "panda_arm");
    hand_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "hand");
    
    move_group_->setMaxVelocityScalingFactor(0.4);
    move_group_->setMaxAccelerationScalingFactor(0.4);
    
    // Posizione Home Iniziale
    go_to_home();
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_group_;
  bool is_busy_ = false;

  void vision_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    if (is_busy_) return;
    if (msg->x == 0.0 && msg->y == 0.0) return;

    // Filtro distanza semplice
    if (std::sqrt(std::pow(msg->x, 2) + std::pow(msg->y, 2)) > 0.8) return;

    RCLCPP_INFO(this->get_logger(), "Target Trovato: X=%.2f Y=%.2f", msg->x, msg->y);
    is_busy_ = true;

    // Eseguiamo in un thread separato per non bloccare la callback
    std::thread([this, msg]() {
      esegui_missione(msg->x, msg->y);
      is_busy_ = false;
    }).detach();
  }

  void esegui_missione(double x, double y)
  {
    // 1. Approccio Alto
    muovi_a_pos(x, y, Z_ALTA);
    
    // 2. Apri Pinza (MoveIt usa stringhe per stati predefiniti o valori joint)
    muovi_pinza("open"); 
    
    // 3. Discesa
    muovi_a_pos(x, y, Z_PRESA);
    
    // 4. Chiudi Pinza
    muovi_pinza("close");
    rclcpp::sleep_for(std::chrono::seconds(1));

    // 5. Risalita
    muovi_a_pos(x, y, Z_ALTA);

    // 6. Deposito Alto
    muovi_a_pos(DROP_X, DROP_Y, Z_ALTA);

    // 7. Deposito Basso
    muovi_a_pos(DROP_X, DROP_Y, Z_PRESA + 0.05);

    // 8. Rilascio
    muovi_pinza("open");
    rclcpp::sleep_for(std::chrono::seconds(1));

    // 9. Home
    go_to_home();
  }

  void muovi_a_pos(double x, double y, double z)
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    // Orientamento: Pinza verso il basso (Ruotata di 180 su X)
    tf2::Quaternion q;
    q.setRPY(3.14159, 0, 0); // Roll, Pitch, Yaw
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    move_group_->setPoseTarget(target_pose);
    move_group_->move(); // Pianifica ed esegui
  }

  void muovi_pinza(std::string command)
  {
    // "hand" group ha spesso stati predefiniti "open" e "close" nel SRDF standard del Panda
    // Se fallisce, usiamo valori joint manuali
    if (command == "open") {
        // Valori joint per pinza aperta (panda_finger_joint1, panda_finger_joint2)
        std::vector<double> joints = {0.04, 0.04}; 
        hand_group_->setJointValueTarget(joints);
    } else {
        // Chiusa
        std::vector<double> joints = {0.00, 0.00}; 
        hand_group_->setJointValueTarget(joints);
    }
    hand_group_->move();
  }

  void go_to_home()
  {
    // Posizione joint sicura ("ready" standard del panda)
    std::vector<double> joint_group_positions = {0, -0.785, 0, -2.356, 0, 1.571, 0.785};
    move_group_->setJointValueTarget(joint_group_positions);
    move_group_->move();
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotMover>();
  
  // Usiamo un executor multi-thread perché MoveIt ne ha bisogno
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  
  // Setup MoveIt dopo aver aggiunto il nodo all'executor
  std::thread([node]() {
    rclcpp::sleep_for(std::chrono::seconds(2)); // Aspetta che ROS sia su
    node->setup_moveit();
  }).detach();

  executor.spin();
  rclcpp::shutdown();
  return 0;
}