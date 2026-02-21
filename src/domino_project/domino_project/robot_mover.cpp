#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>
#include <cmath>
#include <visualization_msgs/msg/marker_array.hpp>
#include <fstream>

// Costanti
const double Z_ALTA_DEFAULT = 0.35;
const double Z_PRESA_DEFAULT = 0.235;
const double DROP_X_DEFAULT = 0.50;
const double DROP_Y_DEFAULT = 0.00;

class RobotMover : public rclcpp::Node, public std::enable_shared_from_this<RobotMover>
{
public:
  RobotMover() : Node("robot_mover_cpp")
  {
    // Subscriber per la visione
    subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/domino_position", 10, std::bind(&RobotMover::vision_callback, this, std::placeholders::_1));
    detections_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/domino_detections", 10, std::bind(&RobotMover::detections_callback, this, std::placeholders::_1));
    
    // parameter to simulate gripper if controller/action is not available
    this->declare_parameter<bool>("simulate_gripper", true);
    simulate_gripper_ = this->get_parameter("simulate_gripper").as_bool();

    RCLCPP_INFO(this->get_logger(), "NODO C++ MOVEIT AVVIATO. In attesa di target... simulate_gripper=%s", simulate_gripper_?"true":"false");

    // Load tunable parameters with sensible defaults
    this->declare_parameter<double>("approach.z_high", Z_ALTA_DEFAULT);
    this->declare_parameter<double>("approach.z_pick", Z_PRESA_DEFAULT);
    this->declare_parameter<double>("approach.drop_x", DROP_X_DEFAULT);
    this->declare_parameter<double>("approach.drop_y", DROP_Y_DEFAULT);
    z_alta_ = this->get_parameter("approach.z_high").as_double();
    z_presa_ = this->get_parameter("approach.z_pick").as_double();
    drop_x_ = this->get_parameter("approach.drop_x").as_double();
    drop_y_ = this->get_parameter("approach.drop_y").as_double();

    // Planner scaling
    this->declare_parameter<double>("planner.max_velocity_scaling", 0.4);
    this->declare_parameter<double>("planner.max_acceleration_scaling", 0.4);
    planner_max_velocity_ = this->get_parameter("planner.max_velocity_scaling").as_double();
    planner_max_acceleration_ = this->get_parameter("planner.max_acceleration_scaling").as_double();
    failure_count_ = 0;
    max_retries_ = 3;
    // Open metrics CSV
    metrics_file_.open("/tmp/domino_metrics.csv", std::ios::app);
    if (metrics_file_.tellp() == 0) {
      metrics_file_ << "timestamp,event,success,info\n";
    }
  }

  void detections_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    latest_markers_ = msg->markers;
  }

  void setup_moveit()
  {
    // Inizializzazione MoveIt (deve girare su un thread separato per non bloccare ROS)
    try {
      auto self = std::enable_shared_from_this<RobotMover>::shared_from_this();
      rclcpp::Node::SharedPtr node_ptr = std::static_pointer_cast<rclcpp::Node>(self);
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "panda_arm");
      hand_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "hand");
      RCLCPP_INFO(this->get_logger(), "MoveIt MoveGroup interfaces created");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception creating MoveGroupInterface: %s", e.what());
      return;
    }
    
    move_group_->setMaxVelocityScalingFactor(planner_max_velocity_);
    move_group_->setMaxAccelerationScalingFactor(planner_max_acceleration_);

    // instantiate planning scene interface and add static collision geometry
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    add_scene_collision_objects();
    
    // Posizione Home Iniziale
    go_to_home();
    // runtime parameters for retry behavior
    this->declare_parameter<int>("runtime.max_retries", 3);
    max_retries_ = this->get_parameter("runtime.max_retries").as_int();
    // Signal that MoveIt is ready to accept commands
    moveit_ready_ = true;
  }

  void add_scene_collision_objects()
  {
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    // Table
    moveit_msgs::msg::CollisionObject table;
    table.id = "work_table";
    table.header.frame_id = "world";
    shape_msgs::msg::SolidPrimitive table_prim;
    geometry_msgs::msg::Pose table_pose;
    table_prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    table_prim.dimensions = {1.2, 1.2, 0.05};
    table_pose.position.y = 0.0;
    table_pose.position.z = 0.8 - 0.025; // top surface
    table_pose.orientation.w = 1.0;
    table.primitives.push_back(table_prim);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;
    collision_objects.push_back(table);

    // Domino placeholders (approximate boxes)
    std::vector<std::pair<std::string, std::pair<double,double>>> dominos = {{"domino_rg", {0.5, 0.0}}, {"domino_gb", {0.5, 0.2}}, {"domino_br", {0.5, -0.2}}};
    for (auto &d : dominos) {
      moveit_msgs::msg::CollisionObject obj;
      obj.id = d.first;
      obj.header.frame_id = "world";
      shape_msgs::msg::SolidPrimitive prim;
      prim.dimensions = {0.06, 0.02, 0.02};
      geometry_msgs::msg::Pose p;
      p.position.x = d.second.first;
      p.position.y = d.second.second;
      p.position.z = 1.32; // spawn z height used by launch
      p.orientation.w = 1.0;
      obj.primitives.push_back(prim);
      obj.primitive_poses.push_back(p);
      obj.operation = obj.ADD;
      collision_objects.push_back(obj);
    }

    try {
      planning_scene_interface_->applyCollisionObjects(collision_objects);
      RCLCPP_INFO(this->get_logger(), "Added %zu collision objects to planning scene", collision_objects.size());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception adding collision objects: %s", e.what());
    }
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  double z_alta_ = Z_ALTA_DEFAULT;
  double z_presa_ = Z_PRESA_DEFAULT;
  double drop_x_ = DROP_X_DEFAULT;
  double drop_y_ = DROP_Y_DEFAULT;
  double planner_max_velocity_ = 0.4;
  double planner_max_acceleration_ = 0.4;
  int failure_count_ = 0;
  int max_retries_ = 3;
  bool is_busy_ = false;
  std::atomic_bool moveit_ready_{false};
  bool simulate_gripper_ = true;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr detections_sub_;
  std::vector<visualization_msgs::msg::Marker> latest_markers_;
  int last_target_color_code_ = 0;
  std::ofstream metrics_file_;

  void vision_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    // store semantic color code if provided by vision (z)
    try { last_target_color_code_ = static_cast<int>(msg->z); } catch(...) { last_target_color_code_ = 0; }
    if (!moveit_ready_) return; // ignore inputs until MoveIt is ready
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
    // Generate grasp candidates and attempt until success
    bool grasped = false;
    double chosen_x = x, chosen_y = y, chosen_yaw = 0.0;
    muovi_pinza("open");
    if (try_grasp_candidates(x, y, chosen_x, chosen_y, chosen_yaw)) {
      RCLCPP_INFO(this->get_logger(), "Grasp succeeded at offset (%.3f, %.3f) yaw %.3f", chosen_x - x, chosen_y - y, chosen_yaw);
      grasped = true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "All grasp candidates failed - aborting mission");
      return;
    }

    // If grasped, lift and move to drop
    if (grasped) {
      if (!compute_cartesian_and_execute(chosen_x, chosen_y, z_alta_)) { plan_and_execute_pose(chosen_x, chosen_y, z_alta_); }
        if (!plan_and_execute_pose(drop_x_, drop_y_, z_alta_)) { RCLCPP_WARN(this->get_logger(), "Move to drop high failed"); }
        if (!compute_cartesian_and_execute(drop_x_, drop_y_, z_presa_ + 0.05)) { RCLCPP_WARN(this->get_logger(), "Drop descent failed"); }
      muovi_pinza("open");
      rclcpp::sleep_for(std::chrono::seconds(1));
      try { move_group_->detachObject(std::string("domino_rg")); } catch (const std::exception &e) { RCLCPP_WARN(this->get_logger(), "detachObject failed: %s", e.what()); }
      // Post-place verification (T13)
      bool ok = verify_post_place(drop_x_, drop_y_);
      if (!ok) { RCLCPP_WARN(this->get_logger(), "Post-place verification failed: color misalignment or object not found"); }
      go_to_home();
    }
  }

  bool try_grasp_candidates(double x, double y, double &out_x, double &out_y, double &out_yaw)
  {
    // candidate offsets (meters) and small yaw variations
    std::vector<double> lateral = {-0.01, 0.0, 0.01};
    std::vector<double> forward = {0.0};
    std::vector<double> yaws = {-0.1, 0.0, 0.1};

    for (double dx : lateral) {
      for (double dy : forward) {
        for (double yaw : yaws) {
          double tx = x + dx;
          double ty = y + dy;
          // Approach high
          if (!compute_cartesian_and_execute(tx, ty, z_alta_) && !plan_and_execute_pose(tx, ty, z_alta_)) continue;
          // Descent
          if (!compute_cartesian_and_execute(tx, ty, z_presa_) && !plan_and_execute_pose(tx, ty, z_presa_)) continue;
          // Close gripper
          muovi_pinza("close");
          rclcpp::sleep_for(std::chrono::milliseconds(700));
          // Verify attachment in planning scene
          try {
            std::vector<std::string> ids = {"domino_rg"};
            auto attached = planning_scene_interface_->getAttachedObjects(ids);
            if (!attached.empty()) {
              out_x = tx; out_y = ty; out_yaw = yaw;
              return true;
            }
          } catch (const std::exception &e) {
            // If planning scene query fails, assume success when simulating gripper
            if (simulate_gripper_) { out_x = tx; out_y = ty; out_yaw = yaw; return true; }
          }
          // If failed, open and try next candidate
          muovi_pinza("open");
          rclcpp::sleep_for(std::chrono::milliseconds(200));
        }
      }
    }
    return false;
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

    if (!move_group_) { RCLCPP_ERROR(this->get_logger(), "move_group_ not initialized"); return; }

    // Attempt Cartesian path for small vertical moves
    double fraction = 0.0;
    try {
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(target_pose);
      moveit_msgs::msg::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;
      fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      if (fraction > 0.9) {
        // execute the computed trajectory
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        move_group_->execute(plan);
        return;
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "Cartesian compute failed: %s", e.what());
    }

    // Fallback to normal planning+execution
    try {
      move_group_->setPoseTarget(target_pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool ok = (move_group_->plan(plan) && !plan.trajectory_.joint_trajectory.points.empty());
      if (ok) {
        move_group_->execute(plan);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed for target pose");
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in move planning/execution: %s", e.what());
    }
  }

  bool compute_cartesian_and_execute(double x, double y, double z)
  {
    if (!move_group_) return false;
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x; target_pose.position.y = y; target_pose.position.z = z;
    tf2::Quaternion q; q.setRPY(3.14159, 0, 0);
    target_pose.orientation.x = q.x(); target_pose.orientation.y = q.y(); target_pose.orientation.z = q.z(); target_pose.orientation.w = q.w();
    double fraction = 0.0;
    try {
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(target_pose);
      moveit_msgs::msg::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;
      fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      if (fraction > 0.9) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        move_group_->execute(plan);
        return true;
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "compute_cartesian_and_execute exception: %s", e.what());
    }
    // failure handling
    log_metric("cartesian_plan", false, "fraction=" + std::to_string(fraction));
    failure_count_++;
    if (failure_count_ >= max_retries_) { emergency_stop_and_recover(); }
    return false;
  }

  bool plan_and_execute_pose(double x, double y, double z)
  {
    if (!move_group_) return false;
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x; target_pose.position.y = y; target_pose.position.z = z;
    tf2::Quaternion q; q.setRPY(3.14159, 0, 0);
    target_pose.orientation.x = q.x(); target_pose.orientation.y = q.y(); target_pose.orientation.z = q.z(); target_pose.orientation.w = q.w();
    try {
      move_group_->setPoseTarget(target_pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool ok = (move_group_->plan(plan) && !plan.trajectory_.joint_trajectory.points.empty());
      if (ok) { move_group_->execute(plan); return true; }
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "plan_and_execute_pose exception: %s", e.what());
    }
    failure_count_++;
    if (failure_count_ >= max_retries_) { emergency_stop_and_recover(); }
    return false;
  }

  bool verify_post_place(double x, double y)
  {
    // check latest markers for an object close to (x,y) and with matching color
    for (auto &m : latest_markers_) {
      double mx = m.pose.position.x;
      double my = m.pose.position.y;
      double dist = std::sqrt((mx - x)*(mx - x) + (my - y)*(my - y));
      if (dist < 0.06) {
        // check color similarity to last_target_color_code_
        int detected_code = 0;
        if (m.color.r > 0.9 && m.color.g < 0.5 && m.color.b < 0.5) detected_code = 1; // red
        else if (m.color.g > 0.9) detected_code = 2; // green
        else if (m.color.b > 0.9) detected_code = 3; // blue
        if (last_target_color_code_ == 0 || detected_code == last_target_color_code_) return true;
      }
    }
    return false;
  }

  void emergency_stop_and_recover()
  {
    RCLCPP_ERROR(this->get_logger(), "Exceeded max retries (%d). Performing emergency stop and recover." , max_retries_);
    try {
      if (move_group_) move_group_->stop();
    } catch (...) {}
    // go to a safe home and reset counters
    try { go_to_home(); } catch (...) {}
    failure_count_ = 0;
    // small pause to allow system to settle
    rclcpp::sleep_for(std::chrono::seconds(1));
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
    // "hand" group ha spesso stati predefiniti "open" e "close" nel SRDF standard del Panda
    // Se fallisce, usiamo valori joint manuali
    if (command == "open") {
        // Valori joint per pinza aperta (panda_finger_joint1, panda_finger_joint2)
        std::vector<double> joints = {0.04, 0.04}; 
        if (simulate_gripper_) {
          RCLCPP_INFO(this->get_logger(), "Simulating gripper open");
          rclcpp::sleep_for(std::chrono::milliseconds(300));
        } else if (hand_group_) {
          try { hand_group_->setJointValueTarget(joints); hand_group_->move(); } catch (const std::exception &e) { RCLCPP_ERROR(this->get_logger(), "Exception moving hand (open): %s", e.what()); }
        } else { RCLCPP_ERROR(this->get_logger(), "hand_group_ not initialized and simulation disabled"); }
    } else {
        // Chiusa
        std::vector<double> joints = {0.00, 0.00}; 
        if (simulate_gripper_) {
          RCLCPP_INFO(this->get_logger(), "Simulating gripper close");
          rclcpp::sleep_for(std::chrono::milliseconds(300));
        } else if (hand_group_) {
          try { hand_group_->setJointValueTarget(joints); hand_group_->move(); } catch (const std::exception &e) { RCLCPP_ERROR(this->get_logger(), "Exception moving hand (close): %s", e.what()); }
        } else { RCLCPP_ERROR(this->get_logger(), "hand_group_ not initialized and simulation disabled"); }
    }
  }

  void go_to_home()
  {
    // Posizione joint sicura ("ready" standard del panda)
    std::vector<double> joint_group_positions = {0, -0.785, 0, -2.356, 0, 1.571, 0.785};
    if (!move_group_) { RCLCPP_ERROR(this->get_logger(), "move_group_ not initialized (home)"); return; }
    try {
      move_group_->setJointValueTarget(joint_group_positions);
      move_group_->move();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception going home: %s", e.what());
    }
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