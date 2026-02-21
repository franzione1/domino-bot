#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h> 
#include <thread>
#include <cmath>
#include <visualization_msgs/msg/marker_array.hpp>
#include <fstream>

const double Z_ALTA_DEFAULT = 0.35;
const double Z_PRESA_DEFAULT = 0.235;
const double DROP_X_DEFAULT = 0.50;
const double DROP_Y_DEFAULT = 0.00;

class RobotMover : public rclcpp::Node
{
public:
  RobotMover() : Node("robot_mover_cpp")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/domino_pose", 10, std::bind(&RobotMover::vision_callback, this, std::placeholders::_1));
    detections_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/domino_detections", 10, std::bind(&RobotMover::detections_callback, this, std::placeholders::_1));
    
    this->declare_parameter<bool>("simulate_gripper", true);
    simulate_gripper_ = this->get_parameter("simulate_gripper").as_bool();

    this->declare_parameter<double>("approach.z_high", Z_ALTA_DEFAULT);
    this->declare_parameter<double>("approach.z_pick", Z_PRESA_DEFAULT);
    z_alta_ = this->get_parameter("approach.z_high").as_double();
    z_presa_ = this->get_parameter("approach.z_pick").as_double();

    this->declare_parameter<double>("planner.max_velocity_scaling", 0.4);
    this->declare_parameter<double>("planner.max_acceleration_scaling", 0.4);
    planner_max_velocity_ = this->get_parameter("planner.max_velocity_scaling").as_double();
    planner_max_acceleration_ = this->get_parameter("planner.max_acceleration_scaling").as_double();
    
    this->declare_parameter<int>("runtime.max_retries", 3);
    max_retries_ = this->get_parameter("runtime.max_retries").as_int();

    metrics_file_.open("/tmp/domino_metrics.csv", std::ios::app);
    if (metrics_file_.tellp() == 0) {
      metrics_file_ << "timestamp,event,success,info\n";
    }
  }

  void setup_moveit()
  {
    try {
      // Usa lo shared_ptr nativo del nodo ROS 2
      auto node_ptr = this->shared_from_this();
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "panda_arm");
      hand_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "hand");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Errore MoveGroup: %s", e.what());
      return;
    }
    
    move_group_->setMaxVelocityScalingFactor(planner_max_velocity_);
    move_group_->setMaxAccelerationScalingFactor(planner_max_acceleration_);

    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    add_scene_collision_objects();
    go_to_home();
    
    moveit_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "NODO C++ PRONTO: Logica di Gioco Attiva.");
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr detections_sub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  
  double z_alta_, z_presa_;
  double planner_max_velocity_, planner_max_acceleration_;
  int failure_count_ = 0, max_retries_ = 3, last_target_color_code_ = 0;
  bool simulate_gripper_ = true, is_busy_ = false;
  std::atomic_bool moveit_ready_{false};
  std::vector<visualization_msgs::msg::Marker> latest_markers_;
  std::ofstream metrics_file_;
  std::string attached_object_id_ = "";

  void detections_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) { latest_markers_ = msg->markers; }

  void add_scene_collision_objects()
  {
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    moveit_msgs::msg::CollisionObject table;
    table.id = "work_table";
    table.header.frame_id = "world";
    shape_msgs::msg::SolidPrimitive table_prim;
    table_prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    table_prim.dimensions = {1.2, 1.2, 0.05};
    geometry_msgs::msg::Pose table_pose;
    table_pose.position.y = 0.0;
    table_pose.position.z = 0.8 - 0.025; 
    table_pose.orientation.w = 1.0;
    table.primitives.push_back(table_prim);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;
    collision_objects.push_back(table);

    std::vector<std::pair<std::string, std::pair<double,double>>> dominos = {
      {"domino_rg", {0.5, 0.0}}, {"domino_gb", {0.5, 0.2}}, {"domino_br", {0.5, -0.2}}
    };
    for (auto &d : dominos) {
      moveit_msgs::msg::CollisionObject obj;
      obj.id = d.first;
      obj.header.frame_id = "world";
      shape_msgs::msg::SolidPrimitive prim;
      prim.type = shape_msgs::msg::SolidPrimitive::BOX;
      prim.dimensions = {0.06, 0.03, 0.02};
      geometry_msgs::msg::Pose p;
      p.position.x = d.second.first; p.position.y = d.second.second; p.position.z = 1.32; 
      p.orientation.w = 1.0;
      obj.primitives.push_back(prim);
      obj.primitive_poses.push_back(p);
      obj.operation = obj.ADD;
      collision_objects.push_back(obj);
    }
    planning_scene_interface_->applyCollisionObjects(collision_objects);
  }

  std::string find_closest_domino(double x, double y)
  {
      std::vector<std::string> ids = {"domino_rg", "domino_gb", "domino_br"};
      auto objects = planning_scene_interface_->getObjects(ids);
      std::string best_id = "";
      double min_dist = 0.15; 

      for (const auto& kv : objects) {
          if (!kv.second.primitive_poses.empty()) {
              double ox = kv.second.primitive_poses[0].position.x;
              double oy = kv.second.primitive_poses[0].position.y;
              double dist = std::sqrt(std::pow(ox - x, 2) + std::pow(oy - y, 2));
              if (dist < min_dist) {
                  min_dist = dist;
                  best_id = kv.first;
              }
          }
      }
      return best_id;
  }

  void vision_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    try { last_target_color_code_ = static_cast<int>(msg->position.z); } catch(...) { last_target_color_code_ = 0; }
    if (!moveit_ready_ || is_busy_ || (msg->position.x == 0.0 && msg->position.y == 0.0)) return;
    if (std::sqrt(std::pow(msg->position.x, 2) + std::pow(msg->position.y, 2)) > 0.8) return;

    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    RCLCPP_INFO(this->get_logger(), "Target Ricevuto: X=%.2f Y=%.2f Yaw=%.2f", msg->position.x, msg->position.y, yaw);
    is_busy_ = true;
    
    std::thread([this, msg, yaw]() {
      esegui_missione(msg->position.x, msg->position.y, yaw);
      is_busy_ = false;
    }).detach();
  }

  // NUOVA FUNZIONE: LOGICA DEL GIOCO DEL DOMINO
  void calcola_posa_rilascio(double pick_y, double &drop_x, double &drop_y, double &drop_yaw)
  {
      // Sappiamo che il domino centrale (Rosso-Verde) è a X=0.50, Y=0.0
      // La metà Rossa è a sinistra (X minore), la metà Verde è a destra (X maggiore).

      if (pick_y > 0.1) {
          // Abbiamo preso il pezzo a Y=0.2, ovvero il domino_gb (Verde-Blu).
          // Deve collegarsi alla metà Verde del pezzo centrale (a destra).
          drop_x = 0.565;  // 0.50 (centro) + 0.06 (lunghezza pezzo) + 0.005 gap di sicurezza
          drop_y = 0.0;
          drop_yaw = 0.0;  // Lo allineiamo orizzontalmente
          RCLCPP_INFO(this->get_logger(), "LOGICA GIOCO: Collegamento VERDE-VERDE a destra.");
      } else if (pick_y < -0.1) {
          // Abbiamo preso il pezzo a Y=-0.2, ovvero il domino_br (Blu-Rosso).
          // Deve collegarsi alla metà Rossa del pezzo centrale (a sinistra).
          drop_x = 0.435;  // 0.50 (centro) - 0.06 (lunghezza pezzo) - 0.005 gap di sicurezza
          drop_y = 0.0;
          drop_yaw = 0.0;  // Lo allineiamo orizzontalmente
          RCLCPP_INFO(this->get_logger(), "LOGICA GIOCO: Collegamento ROSSO-ROSSO a sinistra.");
      } else {
          // Fallback di sicurezza se non rientra nei parametri attesi
          drop_x = 0.50;
          drop_y = -0.15;
          drop_yaw = 0.0;
      }
  }

  void esegui_missione(double x, double y, double target_yaw)
  {
    bool grasped = false;
    double chosen_x = x, chosen_y = y, chosen_yaw = 0.0;
    
    muovi_pinza("open");
    
    if (try_grasp_candidates(x, y, target_yaw, chosen_x, chosen_y, chosen_yaw)) {
      log_metric("grasp_attempt", true, "Grasp Success");
      grasped = true;
    } else {
      log_metric("grasp_attempt", false, "All candidates failed");
      return;
    }

    if (grasped) {
      // 1. Solleva il pezzo
      if (!compute_cartesian_and_execute(chosen_x, chosen_y, z_alta_, chosen_yaw)) 
          plan_and_execute_pose(chosen_x, chosen_y, z_alta_, chosen_yaw);
      
      // 2. Calcola dove metterlo usando le regole del Domino!
      double target_drop_x, target_drop_y, target_drop_yaw;
      calcola_posa_rilascio(chosen_y, target_drop_x, target_drop_y, target_drop_yaw);
      
      // 3. Muovi sopra il punto di rilascio con il nuovo orientamento
      if (!plan_and_execute_pose(target_drop_x, target_drop_y, z_alta_, target_drop_yaw)) 
          emergency_stop_and_recover();

      // 4. Abbassa per posizionare il pezzo vicino all'altro
      compute_cartesian_and_execute(target_drop_x, target_drop_y, z_presa_ + 0.05, target_drop_yaw);
      
      muovi_pinza("open");
      
      if (!attached_object_id_.empty()) {
          try { 
              move_group_->detachObject(attached_object_id_); 
              attached_object_id_ = "";
          } catch (...) {}
      }
      
      go_to_home();
    }
  }

  bool try_grasp_candidates(double x, double y, double target_yaw, double &out_x, double &out_y, double &out_yaw)
  {
    std::vector<double> lateral = {0.0, -0.01, 0.01};
    std::vector<double> yaws = {target_yaw, target_yaw - 0.1, target_yaw + 0.1};

    for (double dx : lateral) {
        for (double yaw : yaws) {
          double tx = x + dx;
          double ty = y;
          
          if (!compute_cartesian_and_execute(tx, ty, z_alta_, yaw) && !plan_and_execute_pose(tx, ty, z_alta_, yaw)) continue;
          if (!compute_cartesian_and_execute(tx, ty, z_presa_, yaw) && !plan_and_execute_pose(tx, ty, z_presa_, yaw)) continue;
          
          muovi_pinza("close");
          rclcpp::sleep_for(std::chrono::milliseconds(500));

          std::string target_obj = find_closest_domino(tx, ty);
          if (!target_obj.empty()) {
              move_group_->attachObject(target_obj, "panda_hand");
              attached_object_id_ = target_obj;
              out_x = tx; out_y = ty; out_yaw = yaw;
              return true;
          } else if (simulate_gripper_) {
              out_x = tx; out_y = ty; out_yaw = yaw;
              return true; 
          }
          
          muovi_pinza("open");
          rclcpp::sleep_for(std::chrono::milliseconds(200));
        }
    }
    return false;
  }

  bool compute_cartesian_and_execute(double x, double y, double z, double yaw)
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x; target_pose.position.y = y; target_pose.position.z = z;
    
    tf2::Quaternion q; 
    q.setRPY(3.14159, 0, yaw); 
    target_pose.orientation.x = q.x(); target_pose.orientation.y = q.y(); 
    target_pose.orientation.z = q.z(); target_pose.orientation.w = q.w();
    
    std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};
    moveit_msgs::msg::RobotTrajectory trajectory;
    
    double fraction = move_group_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    
    if (fraction > 0.9) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      // Valutazione booleana sicura per ROS 2 Foxy
      if (move_group_->execute(plan)) return true;
    }
    
    failure_count_++;
    if (failure_count_ >= max_retries_) emergency_stop_and_recover();
    return false;
  }

  bool plan_and_execute_pose(double x, double y, double z, double yaw)
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x; target_pose.position.y = y; target_pose.position.z = z;
    
    tf2::Quaternion q; 
    q.setRPY(3.14159, 0, yaw); 
    target_pose.orientation.x = q.x(); target_pose.orientation.y = q.y(); 
    target_pose.orientation.z = q.z(); target_pose.orientation.w = q.w();
    
    move_group_->setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    // Valutazione booleana sicura per ROS 2 Foxy
    if (move_group_->plan(plan)) {
      if (move_group_->execute(plan)) {
        failure_count_ = 0; 
        return true; 
      }
    }
    
    failure_count_++;
    if (failure_count_ >= max_retries_) emergency_stop_and_recover();
    return false;
  }

  void emergency_stop_and_recover()
  {
    RCLCPP_ERROR(this->get_logger(), "Errori multipli. EMERGENCY STOP e reset.");
    try { move_group_->stop(); } catch (...) {}
    
    if (!attached_object_id_.empty()) {
        try { move_group_->detachObject(attached_object_id_); attached_object_id_ = ""; } catch (...) {}
        muovi_pinza("open");
    }
    
    go_to_home();
    failure_count_ = 0;
    rclcpp::sleep_for(std::chrono::seconds(2));
  }

  void log_metric(const std::string &event, bool success, const std::string &info)
  {
    if (!metrics_file_.is_open()) return;
    double now = static_cast<double>(this->get_clock()->now().nanoseconds()) / 1e9;
    metrics_file_ << now << "," << event << "," << (success?"1":"0") << "," << info << "\n";
    metrics_file_.flush();
  }

  void muovi_pinza(std::string command)
  {
    std::vector<double> joints = (command == "open") ? std::vector<double>{0.04, 0.04} : std::vector<double>{0.00, 0.00};
    
    if (simulate_gripper_) {
      rclcpp::sleep_for(std::chrono::milliseconds(300));
    } else if (hand_group_) {
      try { hand_group_->setJointValueTarget(joints); hand_group_->move(); } catch (...) {}
    }
  }

  void go_to_home()
  {
    std::vector<double> joint_group_positions = {0, -0.785, 0, -2.356, 0, 1.571, 0.785};
    try {
      move_group_->setJointValueTarget(joint_group_positions);
      move_group_->move();
    } catch (...) {}
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotMover>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread([node]() {
    rclcpp::sleep_for(std::chrono::seconds(2)); 
    node->setup_moveit();
  }).detach();
  executor.spin();
  rclcpp::shutdown();
  return 0;
}