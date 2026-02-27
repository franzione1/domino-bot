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
#include <std_msgs/msg/bool.hpp>
#include <fstream>

// ─── Physical constants ───────────────────────────────────────────────────────
// Domino piece dimensions (metres) — must match the SDF models
// Sized to match the reference project (Robot-Domino-Artist):
//   width=0.0075, length=0.024, height=0.048
// In our SDF the piece lies FLAT on the table:
//   SDF x = 0.048 (long axis on table)
//   SDF y = 0.024 (short axis on table)
//   SDF z = 0.0075 (thickness, faces up when lying flat)
const double DOMINO_LEN     = 0.048;   // long axis  (SDF x)
const double DOMINO_WID     = 0.024;   // short axis (SDF y) — gripper straddles this
const double DOMINO_H       = 0.0075;  // thickness  (SDF z)

// Environment geometry — derived from the Gazebo spawn configuration.
// work_table/model.sdf: box 0.8×1.2×1.0 m, spawned at -z 0.8 (box centre)
//   → table TOP = 0.8 + 0.5 = 1.30 m
const double TABLE_SPAWN_Z  = 0.80;
const double TABLE_H        = 1.00;
const double TABLE_TOP_Z    = TABLE_SPAWN_Z + TABLE_H / 2.0;   // 1.30 m

// Pieces settle on the table: centre z = table top + half piece thickness
const double DOMINO_REST_Z  = TABLE_TOP_Z + DOMINO_H / 2.0;    // 1.30375 m

// ── Panda EEF chain (derived from the URDF, gripper pointing DOWN, roll=π) ──
//   panda_link8  → panda_hand       : xyz="0 0 0"      (no z offset)
//   panda_hand   → panda_finger_joint1 origin : xyz="0 0 0.0584"
//   panda_leftfinger mesh z extent  : 0.0001 → 0.0538 m
//   ∴ fingertip is 0.0584 + 0.0538 = 0.1122 m BELOW panda_link8
//
// Context: cell_layout_2=true mounts the robot on a 1.3 m pedestal.
//   panda_link0 world z = 0.65 (pedestal_joint) + 0.655 (panda_joint) = 1.305 m
//   The arm reaches DOWNWARD — table top (1.30 m) is just below the robot base.
const double PANDA_LINK8_TO_FINGERTIP = 0.0584 + 0.0538;   // 0.1122 m

// Grasp z: fingertips at the piece centre height.
//   The piece is only 7.5 mm thick.  Placing the fingertips at the piece
//   centre (DOMINO_REST_Z = TABLE_TOP_Z + DOMINO_H/2 = 1.30375 m) keeps
//   the finger collision meshes ABOVE the table collision object so MoveIt
//   can actually plan the descent.  The vacuum gripper (max_distance 50 mm)
//   handles the real attachment in Gazebo.
//   Previous value (TABLE_TOP_Z − 1 mm) put the finger meshes INTO the table
//   collision object → "Unable to sample any valid states for goal tree".
const double Z_PRESA_DEFAULT = DOMINO_REST_Z + PANDA_LINK8_TO_FINGERTIP;
//   = 1.30375 + 0.1122 = 1.41595 m  (fingertip at piece centre, ~4 mm above table)

// EEF z for safe transit: fingertips 20 cm above the table surface
// (reference uses 22 cm above object, we use a generous 20 cm above table)
const double Z_ALTA_DEFAULT  =
    TABLE_TOP_Z + 0.20 + PANDA_LINK8_TO_FINGERTIP;
//   = 1.30 + 0.20 + 0.1122 = 1.6122 m

// Distance between centres when two pieces are placed touching end-to-end.
// Both pieces have the same length, so the contact distance = LEN/2 + LEN/2 = LEN.
const double CONTACT_OFFSET = DOMINO_LEN;   // 0.048 m

// ─────────────────────────────────────────────────────────────────────────────

class RobotMover : public rclcpp::Node
{
public:
  RobotMover() : Node("robot_mover_cpp")
  {
    // Subscribe to the rich game-state topic produced by vision_processor.py
    // Marker id=0 ns='center' → piece that stays; id=1 ns='target' → piece to pick
    detections_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/domino_detections", 10,
      std::bind(&RobotMover::detections_callback, this, std::placeholders::_1));

    this->declare_parameter<bool>("simulate_gripper", false);
    simulate_gripper_ = this->get_parameter("simulate_gripper").as_bool();

    // Vacuum gripper plugin on panda_leftfinger: toggle via /panda_hand/grasping
    vacuum_gripper_pub_ = this->create_publisher<std_msgs::msg::Bool>("/panda_hand/grasping", 10);

    // z_alta_ and z_presa_ are derived from physical constants — not overridable
    z_alta_  = Z_ALTA_DEFAULT;
    z_presa_ = Z_PRESA_DEFAULT;

    this->declare_parameter<double>("planner.max_velocity_scaling",     0.4);
    this->declare_parameter<double>("planner.max_acceleration_scaling", 0.4);
    planner_max_velocity_     = this->get_parameter("planner.max_velocity_scaling").as_double();
    planner_max_acceleration_ = this->get_parameter("planner.max_acceleration_scaling").as_double();

    metrics_file_.open("/tmp/domino_metrics.csv", std::ios::app);
    if (metrics_file_.tellp() == 0) {
      metrics_file_ << "timestamp,event,success,info\n";
    }
  }

  void setup_moveit()
  {
    try {
      auto node_ptr = this->shared_from_this();
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "panda_arm");
      hand_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "panda_gripper");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroup error: %s", e.what());
      return;
    }

    // Slow gripper to avoid knocking pieces away (reference uses speed=0.05)
    hand_group_->setMaxVelocityScalingFactor(0.3);
    hand_group_->setMaxAccelerationScalingFactor(0.3);
    hand_group_->setPlanningTime(5.0);

    // Log gripper group info for diagnostics
    RCLCPP_INFO(this->get_logger(), "GRIPPER group joints: %zu",
                hand_group_->getJointNames().size());
    for (const auto &jn : hand_group_->getJointNames()) {
      RCLCPP_INFO(this->get_logger(), "  - %s", jn.c_str());
    }

    move_group_->setMaxVelocityScalingFactor(planner_max_velocity_);
    move_group_->setMaxAccelerationScalingFactor(planner_max_acceleration_);
    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(15);
    move_group_->setGoalPositionTolerance(0.01);
    move_group_->setGoalOrientationTolerance(0.1);

    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // Give move_group time to advertise /apply_planning_scene.
    // Without this, applyCollisionObjects() publishes into the void on the first call.
    RCLCPP_INFO(this->get_logger(), "Waiting for MoveIt planning scene services…");
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Retry until at least one object is acknowledged (handles slow startup)
    for (int attempt = 1; attempt <= 5; ++attempt) {
      add_scene_collision_objects();
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      auto known = planning_scene_interface_->getKnownObjectNames();
      if (!known.empty()) {
        RCLCPP_INFO(this->get_logger(), "Scene objects confirmed after %d attempt(s).", attempt);
        break;
      }
      RCLCPP_WARN(this->get_logger(), "Scene objects not confirmed yet (attempt %d/5).", attempt);
    }

    // Diagnostic: verify which link MoveIt is planning for
    RCLCPP_INFO(this->get_logger(), "MoveIt EEF link: '%s'", move_group_->getEndEffectorLink().c_str());

    go_to_home();

    moveit_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "C++ NODE READY – Domino game logic active.");
  }

private:
  // ── ROS handles ──────────────────────────────────────────────────────────
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vacuum_gripper_pub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  // ── Parameters ───────────────────────────────────────────────────────────
  double z_alta_, z_presa_;
  double planner_max_velocity_, planner_max_acceleration_;
  bool simulate_gripper_ = true;

  // ── State ────────────────────────────────────────────────────────────────
  std::atomic_bool moveit_ready_{false};
  std::atomic_bool is_busy_{false};
  std::string attached_object_id_;
  std::ofstream metrics_file_;
  std::thread worker_thread_;

public:
  ~RobotMover() {
    if (worker_thread_.joinable()) worker_thread_.join();
  }

private:

  // ── Struct to hold a detected domino piece ───────────────────────────────
  struct DominoPiece {
    double x, y, yaw;
    double match_angle = 0.0;  // world-frame angle (rad) from piece centre to its matching-colour side
    bool valid = false;
  };

  // ─────────────────────────────────────────────────────────────────────────
  // detections_callback
  //   The MarkerArray from vision_processor contains two markers:
  //     ns='center', id=0  → center piece (stays on the table)
  //     ns='target', id=1  → piece to pick and place
  // ─────────────────────────────────────────────────────────────────────────
  void detections_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    if (!moveit_ready_ || is_busy_) return;
    if (msg->markers.size() < 2) return;

    DominoPiece center, target;

    for (const auto &m : msg->markers) {
      DominoPiece piece;
      piece.x   = m.pose.position.x;
      piece.y   = m.pose.position.y;

      // Recover yaw from half-angle quaternion (z/w only)
      double sz = m.pose.orientation.z;
      double cw = m.pose.orientation.w;
      piece.yaw = 2.0 * std::atan2(sz, cw);
      // scale.z repurposed by vision_processor: angle to matching-colour side
      piece.match_angle = m.scale.z;
      piece.valid = true;

      if (m.ns == "center" && m.id == 0) {
        center = piece;
      } else if (m.ns == "target" && m.id == 1) {
        target = piece;
      }
    }

    if (!center.valid || !target.valid) return;

    // Sanity: target must not already be touching center
    double dist = std::sqrt(std::pow(target.x - center.x, 2) + std::pow(target.y - center.y, 2));
    if (dist < CONTACT_OFFSET + 0.01) {
      RCLCPP_INFO(this->get_logger(), "Pieces already in contact – nothing to do.");
      return;
    }

    // Reject unreachable targets
    if (std::sqrt(target.x * target.x + target.y * target.y) > 0.85) return;

    is_busy_ = true;
    // Run mission on a separate thread so the executor keeps spinning.
    // Store the thread so it can be joined on shutdown (no detach).
    if (worker_thread_.joinable()) worker_thread_.join();
    worker_thread_ = std::thread([this, center, target]() {
      execute_domino_play(center, target);
      is_busy_ = false;
    });
  }

  // ─────────────────────────────────────────────────────────────────────────
  // execute_domino_play — simplified linear pick-and-place sequence:
  //
  //   1. Hover above piece
  //   2. Open fingers
  //   3. Descend onto piece
  //   4. Close fingers (grab)
  //   5. Go directly to destination (above, then lower)
  //   6. Open fingers (release)
  //   7. Come back up (ready to search for next piece)
  //
  //   No intermediate poses.  Straight: source → destination → up.
  // ─────────────────────────────────────────────────────────────────────────
  void execute_domino_play(const DominoPiece &center, const DominoPiece &target)
  {
    RCLCPP_INFO(this->get_logger(),
      "MISSION START  center=(%.3f,%.3f)  target=(%.3f,%.3f) yaw=%.2f",
      center.x, center.y, target.x, target.y, target.yaw);

    // ── Grasp yaw: fingers ⊥ piece long axis ──────────────────────────────
    const double grasp_yaw = target.yaw + 3.0 * M_PI / 4.0;

    // ── Compute destination pose ──────────────────────────────────────────
    double drop_x = center.x + std::cos(center.match_angle) * DOMINO_LEN;
    double drop_y = center.y + std::sin(center.match_angle) * DOMINO_LEN;
    double delta_piece = (center.match_angle + M_PI) - target.match_angle;
    while (delta_piece >  M_PI) delta_piece -= 2.0 * M_PI;
    while (delta_piece < -M_PI) delta_piece += 2.0 * M_PI;
    double drop_yaw = grasp_yaw + delta_piece;

    RCLCPP_INFO(this->get_logger(),
      "  Target piece @ (%.3f,%.3f)  Destination @ (%.3f,%.3f)",
      target.x, target.y, drop_x, drop_y);

    // ═══════════════════════════════════════════════════════════════════════
    //  STEP 1 — HOVER above piece
    // ═══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(this->get_logger(), "  STEP 1 — hovering above piece...");
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);

    bool hover_ok = false;
    const std::vector<double> yaw_tweaks = {0.0, 0.05, -0.05, 0.1, -0.1, M_PI};
    double pick_yaw = grasp_yaw;
    for (double dy : yaw_tweaks) {
      if (plan_and_execute(target.x, target.y, z_alta_, grasp_yaw + dy)) {
        pick_yaw = grasp_yaw + dy;
        hover_ok = true;
        break;
      }
    }
    move_group_->setMaxVelocityScalingFactor(planner_max_velocity_);
    move_group_->setMaxAccelerationScalingFactor(planner_max_acceleration_);

    if (!hover_ok) {
      RCLCPP_ERROR(this->get_logger(), "  Cannot reach above piece — aborting.");
      emergency_stop_and_recover();
      return;
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  STEP 2 — OPEN fingers
    // ═══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(this->get_logger(), "  STEP 2 — opening fingers...");
    muovi_pinza("open");
    rclcpp::sleep_for(std::chrono::seconds(2));

    // Remove collision object so we can descend onto the piece
    std::string target_obj = find_closest_domino(target.x, target.y);
    if (!target_obj.empty()) {
      planning_scene_interface_->removeCollisionObjects({target_obj});
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  STEP 3 — DESCEND onto piece
    // ═══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(this->get_logger(), "  STEP 3 — descending to z=%.4f ...", z_presa_);
    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(4);
    bool descended = plan_and_execute(target.x, target.y, z_presa_, pick_yaw);
    if (!descended) {
      RCLCPP_WARN(this->get_logger(), "    IK descent failed, trying Cartesian...");
      descended = cartesian_descend(target.x, target.y, z_alta_, z_presa_, pick_yaw);
    }
    if (!descended) {
      RCLCPP_WARN(this->get_logger(), "    Cartesian failed, IK retry...");
      descended = plan_and_execute(target.x, target.y, z_presa_, pick_yaw);
    }
    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(15);

    if (!descended) {
      RCLCPP_ERROR(this->get_logger(), "  Cannot descend onto piece — aborting.");
      if (!target_obj.empty()) add_domino_collision_object(target_obj, target.x, target.y);
      emergency_stop_and_recover();
      return;
    }
    rclcpp::sleep_for(std::chrono::seconds(1));  // settle

    // ═══════════════════════════════════════════════════════════════════════
    //  STEP 4 — CLOSE fingers (grab piece)
    // ═══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(this->get_logger(), "  STEP 4 — closing fingers...");
    // Attach collision object BEFORE close so MoveIt knows the piece is held
    if (!target_obj.empty()) {
      add_domino_as_attached(target_obj);
      attached_object_id_ = target_obj;
    }
    muovi_pinza("close");
    rclcpp::sleep_for(std::chrono::seconds(2));
    log_event("pick", true, "x=" + std::to_string(target.x));

    // ═══════════════════════════════════════════════════════════════════════
    //  STEP 5 — MOVE to destination (above drop, then lower)
    // ═══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(this->get_logger(), "  STEP 5a — transit above destination...");
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);

    bool transit_ok = false;
    const std::vector<double> drop_yaw_offsets = {0.0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3};
    for (double dyo : drop_yaw_offsets) {
      if (plan_and_execute(drop_x, drop_y, z_alta_, drop_yaw + dyo)) {
        drop_yaw += dyo;
        transit_ok = true;
        break;
      }
    }
    move_group_->setMaxVelocityScalingFactor(planner_max_velocity_);
    move_group_->setMaxAccelerationScalingFactor(planner_max_acceleration_);

    if (!transit_ok) {
      RCLCPP_ERROR(this->get_logger(), "  Cannot reach above destination — aborting.");
      emergency_stop_and_recover();
      return;
    }

    // Lower to placement height
    RCLCPP_INFO(this->get_logger(), "  STEP 5b — lowering to place...");
    bool lowered = cartesian_descend(drop_x, drop_y, z_alta_, z_presa_, drop_yaw);
    if (!lowered) {
      RCLCPP_WARN(this->get_logger(), "    Cartesian lower failed, trying IK...");
      lowered = plan_and_execute(drop_x, drop_y, z_presa_, drop_yaw);
    }
    if (!lowered) {
      RCLCPP_ERROR(this->get_logger(), "  Cannot lower to place — aborting.");
      emergency_stop_and_recover();
      return;
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  STEP 6 — OPEN fingers (release piece)
    // ═══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(this->get_logger(), "  STEP 6 — releasing piece...");
    if (!attached_object_id_.empty()) {
      try { move_group_->detachObject(attached_object_id_); attached_object_id_ = ""; }
      catch (...) {}
    }
    muovi_pinza("open");
    rclcpp::sleep_for(std::chrono::seconds(2));

    // ═══════════════════════════════════════════════════════════════════════
    //  STEP 7 — COME BACK UP (ready to search for next piece)
    // ═══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(this->get_logger(), "  STEP 7 — coming back up...");
    if (!cartesian_move(drop_x, drop_y, z_alta_, drop_yaw)) {
      plan_and_execute(drop_x, drop_y, z_alta_, drop_yaw);
    }
    go_to_home();

    RCLCPP_INFO(this->get_logger(), "MISSION COMPLETE – domino placed.");
    log_event("place", true, "success");
  }

  // ─────────────────────────────────────────────────────────────────────────
  // cartesian_descend — 2-point Cartesian straight-line descent
  //   Adapted from reference: builds waypoints = [above_pose, grasp_pose]
  //   so the planner has an explicit start→end pair for the vertical move.
  // ─────────────────────────────────────────────────────────────────────────
  bool cartesian_descend(double x, double y, double z_start, double z_end, double yaw)
  {
    auto current_state = move_group_->getCurrentState(2.0);
    if (!current_state) {
      RCLCPP_WARN(this->get_logger(), "cartesian_descend: getCurrentState timed out.");
      move_group_->setStartStateToCurrentState();
    } else {
      move_group_->setStartState(*current_state);
    }

    // Two-point Cartesian path: current height → grasp height
    // (reference builds [p1=above, p2=grasp] as explicit waypoints)
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(make_pose(x, y, z_start, yaw));  // start = above
    waypoints.push_back(make_pose(x, y, z_end,   yaw));  // end   = grasp

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_->computeCartesianPath(waypoints, 0.005, 0.0, trajectory, false);
    RCLCPP_INFO(this->get_logger(),
      "Cartesian descend fraction: %.2f  (%.3f → %.3f)", fraction, z_start, z_end);

    if (fraction >= 0.95) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      if (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        return true;
      }
      RCLCPP_ERROR(this->get_logger(), "Cartesian descend execution failed.");
    }
    return false;
  }

  // ─────────────────────────────────────────────────────────────────────────
  // Collision object helpers
  // ─────────────────────────────────────────────────────────────────────────
  void add_scene_collision_objects()
  {
    std::vector<moveit_msgs::msg::CollisionObject> objs;

    // Virtual table — dimensions and pose must EXACTLY match models/work_table/model.sdf
    // SDF box size: 0.8 x 1.2 x 1.0, spawned at z=0.8 (box centre)
    // → table top = 0.8 + 0.5 = 1.30 m = TABLE_TOP_Z  ✓
    moveit_msgs::msg::CollisionObject table;
    table.id = "work_table";
    table.header.frame_id = "world";
    shape_msgs::msg::SolidPrimitive tp;
    tp.type = shape_msgs::msg::SolidPrimitive::BOX;
    tp.dimensions = {0.8, 1.2, 0.99};  // 10 mm shorter than real table → collision top
                                          //   at 1.295 m, giving ~9 mm clearance for fingers
    geometry_msgs::msg::Pose tpose;
    tpose.position.x = 0.7; tpose.position.y = 0.0; tpose.position.z = 0.80;
    tpose.orientation.w = 1.0;
    table.primitives.push_back(tp);
    table.primitive_poses.push_back(tpose);
    table.operation = table.ADD;
    objs.push_back(table);

    // Domino pieces
    const std::vector<std::pair<std::string, std::pair<double,double>>> dominos = {
      {"domino_rg", {0.5,  0.0}},
      {"domino_gb", {0.5,  0.2}},
      {"domino_br", {0.5, -0.2}}
    };
    for (auto &d : dominos) {
      objs.push_back(make_domino_collision_object(d.first, d.second.first, d.second.second));
    }
    planning_scene_interface_->applyCollisionObjects(objs);
  }

  moveit_msgs::msg::CollisionObject make_domino_collision_object(
      const std::string &id, double x, double y)
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = id;
    obj.header.frame_id = "world";
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    prim.dimensions = {DOMINO_LEN, DOMINO_WID, DOMINO_H};
    geometry_msgs::msg::Pose p;
    p.position.x = x; p.position.y = y; p.position.z = DOMINO_REST_Z;
    p.orientation.w = 1.0;
    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(p);
    obj.operation = obj.ADD;
    return obj;
  }

  void add_domino_collision_object(const std::string &id, double x, double y)
  {
    planning_scene_interface_->applyCollisionObjects(
        {make_domino_collision_object(id, x, y)});
  }

  // Attach a domino as a held object (after grasping)
  void add_domino_as_attached(const std::string &id)
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = id;
    obj.header.frame_id = "panda_hand";
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    prim.dimensions = {DOMINO_LEN, DOMINO_WID, DOMINO_H};
    geometry_msgs::msg::Pose p;
    p.position.z = 0.04;  // centred near fingertips (piece is only 7.5 mm thick)
    p.orientation.w = 1.0;
    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(p);
    obj.operation = obj.ADD;
    planning_scene_interface_->applyCollisionObjects({obj});

    std::vector<std::string> touch_links = {
        "panda_hand", "panda_leftfinger", "panda_rightfinger"};
    move_group_->attachObject(id, "panda_hand", touch_links);
  }

  std::string find_closest_domino(double x, double y)
  {
    const std::vector<std::string> ids = {"domino_rg", "domino_gb", "domino_br"};
    auto objects = planning_scene_interface_->getObjects(ids);
    std::string best_id;
    double min_dist = 0.15;
    for (const auto &kv : objects) {
      if (!kv.second.primitive_poses.empty()) {
        double ox = kv.second.primitive_poses[0].position.x;
        double oy = kv.second.primitive_poses[0].position.y;
        double dist = std::sqrt(std::pow(ox - x, 2) + std::pow(oy - y, 2));
        if (dist < min_dist) { min_dist = dist; best_id = kv.first; }
      }
    }
    return best_id;
  }

  // ─────────────────────────────────────────────────────────────────────────
  // Motion primitives
  // ─────────────────────────────────────────────────────────────────────────

  // Build a gripper-down pose: roll=π (flip down), pitch=0 (orthogonal), yaw=yaw
  // NOTE: this yaw is the panda_link8 yaw — see grasp_yaw below for the
  // piece-relative correction that accounts for the panda_hand_joint -π/4 offset.
  geometry_msgs::msg::Pose make_pose(double x, double y, double z, double yaw)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x; pose.position.y = y; pose.position.z = z;
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, yaw);
    pose.orientation.x = q.x(); pose.orientation.y = q.y();
    pose.orientation.z = q.z(); pose.orientation.w = q.w();
    return pose;
  }

  // OMPL plan-and-execute (for large moves)
  bool plan_and_execute(double x, double y, double z, double yaw)
  {
    auto current_state = move_group_->getCurrentState(2.0);
    if (!current_state) {
      RCLCPP_WARN(this->get_logger(), "plan_and_execute: getCurrentState timed out");
      move_group_->setStartStateToCurrentState();
    } else {
      move_group_->setStartState(*current_state);
    }
    move_group_->setPoseTarget(make_pose(x, y, z, yaw));
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto plan_result = move_group_->plan(plan);
    if (plan_result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      auto exec_result = move_group_->execute(plan);
      if (exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        return true;
      }
      RCLCPP_ERROR(this->get_logger(), "Execution failed: %d", exec_result.val);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed: %d to (%.2f, %.2f, %.2f)",
                   plan_result.val, x, y, z);
    }
    return false;
  }

  // Cartesian straight-line move (for descents and lifts)
  bool cartesian_move(double x, double y, double z, double yaw)
  {
    // Wait for a guaranteed-fresh joint state before planning.
    // setStartStateToCurrentState() uses getCurrentState(0) = instant, stale read.
    // getCurrentState(2.0) blocks up to 2 s until a new /joint_states arrives.
    auto current_state = move_group_->getCurrentState(2.0);
    if (!current_state) {
      RCLCPP_WARN(this->get_logger(), "cartesian_move: getCurrentState timed out, falling back.");
      move_group_->setStartStateToCurrentState();
    } else {
      move_group_->setStartState(*current_state);
    }

    std::vector<geometry_msgs::msg::Pose> waypoints = {make_pose(x, y, z, yaw)};
    moveit_msgs::msg::RobotTrajectory trajectory;
    // Use smaller max_step (0.005 vs 0.01) for smoother Cartesian interpolation
    // matching the reference implementation's approach
    double fraction = move_group_->computeCartesianPath(waypoints, 0.005, 0.0, trajectory, false);
    RCLCPP_INFO(this->get_logger(), "Cartesian path fraction: %.2f (goal: %.1f, %.1f, %.1f)",
                fraction, x, y, z);

    if (fraction >= 0.95) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      if (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        return true;
      }
      RCLCPP_ERROR(this->get_logger(), "Cartesian path execution failed");
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Cartesian path coverage insufficient: %.2f < 0.95", fraction);
    }
    return false;
  }

  void muovi_pinza(const std::string &command)
  {
    bool is_close = (command == "close");
    RCLCPP_INFO(this->get_logger(), "GRIPPER: %s  (simulate=%s)",
                command.c_str(), simulate_gripper_ ? "true" : "false");

    // ── 1. Move finger joints via MoveIt hand_group_ ────────────────────
    //   This mirrors the IFRA framework's moveG_action: get current state,
    //   copy joint positions, set both finger values, plan & execute.
    //   MoveIt routes through GripperCommand to the separate
    //   panda_handleft_controller / panda_handright_controller.
    if (!simulate_gripper_ && hand_group_) {
      // Finger joint values (per-finger):
      //   Close: 0.01 each → total gap 0.02 m (matches reference toggle_gripper)
      //          Reference MoveIt: fer_finger_joint1=0.01, fer_finger_joint2=0.01
      //          Reference hardware: gripper_width(0.01) → 0.005/finger
      //          We use the MoveIt values since we're in Gazebo simulation.
      //   Open:  0.03 each → total gap 0.06 m (matches reference domino_open_value)
      double target_pos = is_close ? 0.01 : 0.03;

      bool finger_ok = false;
      for (int attempt = 1; attempt <= 3 && !finger_ok; ++attempt) {
        try {
          // Get current gripper state (exactly as IFRA moveG_action does)
          const auto *joint_model_group =
              hand_group_->getCurrentState(10)->getJointModelGroup("panda_gripper");

          moveit::core::RobotStatePtr current_state = hand_group_->getCurrentState(10);
          std::vector<double> joint_positions;
          current_state->copyJointGroupPositions(joint_model_group, joint_positions);

          RCLCPP_INFO(this->get_logger(),
              "GRIPPER: current positions [%.4f, %.4f], target %.4f (attempt %d)",
              joint_positions.size() > 0 ? joint_positions[0] : -1.0,
              joint_positions.size() > 1 ? joint_positions[1] : -1.0,
              target_pos, attempt);

          // Set BOTH finger joints to the target value
          if (joint_positions.size() >= 2) {
            joint_positions[0] = target_pos;
            joint_positions[1] = target_pos;
          }
          hand_group_->setJointValueTarget(joint_positions);

          // Plan and execute
          moveit::planning_interface::MoveGroupInterface::Plan plan;
          bool planned = (hand_group_->plan(plan) ==
                          moveit::planning_interface::MoveItErrorCode::SUCCESS);

          if (planned) {
            RCLCPP_INFO(this->get_logger(), "GRIPPER: plan succeeded, executing...");
            auto exec_result = hand_group_->execute(plan);
            if (exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
              finger_ok = true;
              RCLCPP_INFO(this->get_logger(), "GRIPPER: move executed successfully (attempt %d)", attempt);
            } else {
              RCLCPP_WARN(this->get_logger(),
                  "GRIPPER: execution failed (code %d), attempt %d",
                  exec_result.val, attempt);
            }
          } else {
            RCLCPP_WARN(this->get_logger(), "GRIPPER: planning failed, attempt %d", attempt);
          }

          if (!finger_ok) {
            rclcpp::sleep_for(std::chrono::milliseconds(500));
          }
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "GRIPPER: exception on attempt %d: %s", attempt, e.what());
          rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
      }

      if (!finger_ok) {
        RCLCPP_ERROR(this->get_logger(), "GRIPPER: ALL finger move attempts failed!");
      }
      // Give the controller time to settle (reference waits 2s)
      rclcpp::sleep_for(std::chrono::milliseconds(1500));
    } else {
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    // ── 2. Activate / deactivate vacuum gripper in Gazebo ──────────────
    //   The Panda URDF loads libgazebo_ros_vacuum_gripper.so on
    //   panda_leftfinger.  Publishing true creates a fixed joint to the
    //   nearest object within max_distance (0.05 m).
    //   Publish multiple times to ensure delivery in a laggy sim.
    std_msgs::msg::Bool vacuum_msg;
    vacuum_msg.data = is_close;
    for (int i = 0; i < 3; ++i) {
      vacuum_gripper_pub_->publish(vacuum_msg);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void go_to_home()
  {
    const std::vector<double> home = {0, -0.785, 0, -2.356, 0, 1.571, 0.785};
    try {
      RCLCPP_INFO(this->get_logger(), "Moving to home position...");
      move_group_->setJointValueTarget(home);
      auto result = move_group_->move();
      if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to move to home: %d", result.val);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in go_to_home: %s", e.what());
    }
  }

  // Move to the SRDF-defined 'ready' named configuration.
  // Reference: try_plan(named_configuration='ready', vel, acc)
  void go_to_ready(double vel = 0.8, double acc = 0.8)
  {
    try {
      RCLCPP_INFO(this->get_logger(), "Moving to SRDF 'ready' (vel=%.1f, acc=%.1f)...", vel, acc);
      move_group_->setMaxVelocityScalingFactor(vel);
      move_group_->setMaxAccelerationScalingFactor(acc);
      move_group_->setNamedTarget("ready");
      auto result = move_group_->move();
      move_group_->setMaxVelocityScalingFactor(planner_max_velocity_);
      move_group_->setMaxAccelerationScalingFactor(planner_max_acceleration_);
      if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to move to ready: %d", result.val);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in go_to_ready: %s", e.what());
      move_group_->setMaxVelocityScalingFactor(planner_max_velocity_);
      move_group_->setMaxAccelerationScalingFactor(planner_max_acceleration_);
    }
  }

  void emergency_stop_and_recover()
  {
    RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP – recovering.");
    try { move_group_->stop(); } catch (...) {}
    muovi_pinza("open");
    if (!attached_object_id_.empty()) {
      try {
        move_group_->detachObject(attached_object_id_);
        attached_object_id_ = "";
      } catch (...) {}
    }
    go_to_home();
    rclcpp::sleep_for(std::chrono::seconds(2));
    log_event("emergency_stop", false, "recovered");
  }

  void log_event(const std::string &event, bool success, const std::string &info)
  {
    if (!metrics_file_.is_open()) return;
    auto now = this->get_clock()->now();
    metrics_file_ << now.seconds() << "," << event << ","
                  << (success ? "1" : "0") << "," << info << "\n";
    metrics_file_.flush();
  }
};

// ─── main ────────────────────────────────────────────────────────────────────
int main(int argc, char **argv)
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
