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
const double DOMINO_LEN     = 0.06;   // long axis  (SDF x = 0.06)
const double DOMINO_WID     = 0.03;   // short axis  (SDF y = 0.03)
const double DOMINO_H       = 0.03;   // height      (SDF z = 0.03)

// Environment geometry — derived from the Gazebo spawn configuration.
// work_table/model.sdf: box 0.8×1.2×1.0 m, spawned at -z 0.8 (box centre)
//   → table TOP = 0.8 + 0.5 = 1.30 m
const double TABLE_SPAWN_Z  = 0.80;
const double TABLE_H        = 1.00;
const double TABLE_TOP_Z    = TABLE_SPAWN_Z + TABLE_H / 2.0;   // 1.30 m

// Pieces settle on the table: centre z = table top + half piece height
const double DOMINO_REST_Z  = TABLE_TOP_Z + DOMINO_H / 2.0;    // 1.315 m

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

// ── Why the previous value was wrong ─────────────────────────────────────────
// With panda_link8 at DOMINO_REST_Z + FINGERTIP_REACH = 1.4272 m:
//   finger joint (top of grip zone) = 1.4322 - 0.0584 = 1.3738 m
//   fingertip (bottom of grip zone) = 1.4322 - 0.1122 = 1.32 m  (piece CENTRE)
//   → finger spans 1.32 m – 1.3738 m, piece spans 1.31 m – 1.33 m
//   → only 1 cm overlap at the very tip   ← NEVER GRASPABLE
//
// Fix: lower until fingertips reach 5 mm above the table surface.
//   → fingertip z = TABLE_TOP_Z + 0.005 = 1.305 m
//   → full piece height (1.31–1.33 m) is inside the finger grip zone ✓
//   → 5 mm clearance from table   (cartesian path uses avoid_collisions=false)
const double Z_PRESA_DEFAULT = TABLE_TOP_Z + 0.005 + PANDA_LINK8_TO_FINGERTIP;
//   = 1.30 + 0.005 + 0.1122 = 1.4172 m  (1.5 cm lower than before)

// EEF z for safe transit: fingertips 15 cm above the piece top surface
const double Z_ALTA_DEFAULT  =
    DOMINO_REST_Z + DOMINO_H / 2.0 + 0.15 + PANDA_LINK8_TO_FINGERTIP;
//   = 1.315 + 0.015 + 0.15 + 0.1122 = 1.5922 m

// Distance between centres when two pieces are placed touching end-to-end.
// Both pieces have the same length, so the contact distance = LEN/2 + LEN/2 = LEN.
const double CONTACT_OFFSET = DOMINO_LEN;   // 0.06 m

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
  // execute_domino_play
  //   Full pick-and-place mission: pick `target`, place it adjacent to
  //   `center` so that their matching colour sides touch.
  // ─────────────────────────────────────────────────────────────────────────
  void execute_domino_play(const DominoPiece &center, const DominoPiece &target)
  {
    RCLCPP_INFO(this->get_logger(),
      "MISSION START  center=(%.3f,%.3f)  target=(%.3f,%.3f) yaw=%.2f",
      center.x, center.y, target.x, target.y, target.yaw);

    // ── 1. Open gripper ───────────────────────────────────────────────────
    muovi_pinza("open");

    // ── 2. Pick the target piece ──────────────────────────────────────────
    // Compute the panda_link8 yaw that puts the fingers PERPENDICULAR to the
    // piece long axis (so the fingers straddle the 0.03 m width, not the 0.06 m
    // length).  Derivation:
    //   panda_hand_joint applies Rz(-π/4) from panda_link8 → panda_hand.
    //   Fingers open along panda_hand Y.  With EEF roll=π (pointing down) and
    //   panda_link8 yaw=θ, the finger direction in world frame = angle (θ − π/4).
    //   For fingers ⊥ piece long axis (piece yaw φ):
    //     θ − π/4 = φ + π/2  →  θ = φ + 3π/4
    const double grasp_yaw = target.yaw + 3.0 * M_PI / 4.0;
    double picked_x = target.x, picked_y = target.y, picked_yaw = grasp_yaw;

    if (!pick_piece(picked_x, picked_y, picked_yaw)) {
      RCLCPP_ERROR(this->get_logger(), "PICK FAILED – aborting.");
      emergency_stop_and_recover();
      return;
    }

    // ── 3. Lift to safe travel height ─────────────────────────────────────
    RCLCPP_INFO(this->get_logger(), "Lifting piece to travel height…");
    if (!cartesian_move(picked_x, picked_y, z_alta_, picked_yaw)) {
      // Try OMPL as fallback
      if (!plan_and_execute(picked_x, picked_y, z_alta_, picked_yaw)) {
        RCLCPP_ERROR(this->get_logger(), "LIFT FAILED – aborting.");
        emergency_stop_and_recover();
        return;
      }
    }

    // ── 4. Compute drop pose ──────────────────────────────────────────────
    //
    //  Correct domino placement: place target piece END-TO-END with center,
    //  matching-colour sides physically touching.
    //
    //  center.match_angle  = world angle from center pos → center's matching-
    //                        colour half (encoded by vision_processor in scale.z).
    //
    //  drop_pos = center_pos + unit(center.match_angle) × DOMINO_LEN
    //           (= centre of target piece when its matching half abuts center's)
    //
    //  drop_yaw: rotate EEF so target's matching half faces BACK toward center.
    //    Required target.match_angle at drop = center.match_angle + π
    //    → delta_piece_yaw = (center.match_angle + π) − target.match_angle
    //    → drop EEF yaw    = grasp_yaw + delta_piece_yaw
    //
    double drop_x = center.x + std::cos(center.match_angle) * DOMINO_LEN;
    double drop_y = center.y + std::sin(center.match_angle) * DOMINO_LEN;

    double delta_piece = (center.match_angle + M_PI) - target.match_angle;
    // Normalise to (−π, π]
    while (delta_piece >  M_PI) delta_piece -= 2.0 * M_PI;
    while (delta_piece < -M_PI) delta_piece += 2.0 * M_PI;
    double drop_yaw = grasp_yaw + delta_piece;

    RCLCPP_INFO(this->get_logger(),
      "Drop pose: (%.3f, %.3f)  center_match_angle=%.2f  delta_piece=%.2f",
      drop_x, drop_y, center.match_angle, delta_piece);

    // ── 5. Transit above drop pose with IK yaw sweep ──────────────────────
    // drop_yaw after delta_piece rotation may land in a bad IK region.
    // Sweep small offsets (no ±π — that would flip the matching-colour side).
    const std::vector<double> drop_yaw_offsets = {0.0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3};
    bool transit_ok = false;
    int transit_attempt = 0;
    for (double dyo : drop_yaw_offsets) {
      transit_attempt++;
      if (plan_and_execute(drop_x, drop_y, z_alta_, drop_yaw + dyo)) {
        drop_yaw += dyo;   // keep consistent with descent and retract
        transit_ok = true;
        break;
      }
      // After 3 failures, try ready position recovery
      if (transit_attempt == 3 && !transit_ok) {
        RCLCPP_WARN(this->get_logger(), "Multiple transit failures, trying ready position");
        go_to_ready();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
      }
    }
    if (!transit_ok) {
      RCLCPP_ERROR(this->get_logger(), "TRANSIT TO DROP FAILED – aborting.");
      emergency_stop_and_recover();
      return;
    }

    // ── 6. Lower to place height ──────────────────────────────────────────
    // Release at the piece centre height so it settles flat on the table
    // Use Cartesian first (smoother descent), fallback to OMPL if needed
    double place_z = z_presa_;  // fingertips just above the table surface
    bool lowered = cartesian_move(drop_x, drop_y, place_z, drop_yaw);
    if (!lowered) {
      RCLCPP_WARN(this->get_logger(), "Cartesian lower failed, trying OMPL...");
      lowered = plan_and_execute(drop_x, drop_y, place_z, drop_yaw);
      if (!lowered) {
        // One more retry via ready position
        RCLCPP_WARN(this->get_logger(), "OMPL failed, recovery via ready...");
        go_to_ready();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        lowered = plan_and_execute(drop_x, drop_y, place_z, drop_yaw);
      }
    }
    if (!lowered) {
      RCLCPP_ERROR(this->get_logger(), "LOWER TO PLACE FAILED – aborting.");
      emergency_stop_and_recover();
      return;
    }

    // ── 7. Release piece ──────────────────────────────────────────────────
    muovi_pinza("open");
    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    if (!attached_object_id_.empty()) {
      try { move_group_->detachObject(attached_object_id_); attached_object_id_ = ""; }
      catch (...) {}
    }

    // ── 8. Retract and home ───────────────────────────────────────────────
    cartesian_move(drop_x, drop_y, z_alta_, drop_yaw);
    go_to_home();

    RCLCPP_INFO(this->get_logger(), "MISSION COMPLETE – domino placed.");
    log_event("place", true, "success");
  }

  // ─────────────────────────────────────────────────────────────────────────
  // pick_piece
  //   Approach-descend-grasp sequence with collision-object management.
  // ─────────────────────────────────────────────────────────────────────────
  bool pick_piece(double x, double y, double yaw)
  {
    // Try a small lateral sweep to handle IK degeneracies
    const std::vector<double> lateral_offsets = {0.0, 0.01, -0.01, 0.02, -0.02};
    // M_PI = grasp from opposite side (valid symmetric pose); -M_PI removed (duplicate of M_PI)
    const std::vector<double> yaw_offsets      = {0.0, 0.05, -0.05, 0.1, M_PI};

    // Recover piece yaw (long axis direction) from the grasp yaw.
    // grasp_yaw = piece_yaw + 3π/4  →  piece_yaw = yaw − 3π/4
    const double piece_yaw_est = yaw - 3.0 * M_PI / 4.0;
    const double long_x = std::cos(piece_yaw_est);  // unit vec along piece long axis
    const double long_y = std::sin(piece_yaw_est);

    int attempt_count = 0;
    const int max_attempts = lateral_offsets.size() * yaw_offsets.size();
    
    for (double dl : lateral_offsets) {
      for (double dy_yaw : yaw_offsets) {
        attempt_count++;
        // Sweep along the piece LONG axis so the gripper stays centred over the piece
        double tx   = x + dl * long_x;
        double ty   = y + dl * long_y;
        double tyaw = yaw + dy_yaw;

        RCLCPP_INFO(this->get_logger(),
          "  PICK attempt %d/%d  X=%.3f Y=%.3f yaw=%.2f", 
          attempt_count, max_attempts, tx, ty, tyaw);

        // Approach above piece - if fails after 3 tries, go to ready and retry
        bool reached_above = plan_and_execute(tx, ty, z_alta_, tyaw);
        if (!reached_above && attempt_count > 3) {
          RCLCPP_WARN(this->get_logger(), "Multiple failures, trying ready position recovery");
          go_to_ready();
          rclcpp::sleep_for(std::chrono::milliseconds(500));
          reached_above = plan_and_execute(tx, ty, z_alta_, tyaw);
        }
        if (!reached_above) continue;

        // Remove collision object so we can descend onto the piece
        std::string target_obj = find_closest_domino(tx, ty);
        if (!target_obj.empty()) {
          planning_scene_interface_->removeCollisionObjects({target_obj});
        }

        // Descend
        bool descended = cartesian_move(tx, ty, z_presa_, tyaw);
        if (!descended) {
          descended = plan_and_execute(tx, ty, z_presa_, tyaw);
        }
        if (!descended) {
          // Restore collision object and try next candidate
          if (!target_obj.empty()) {
            add_domino_collision_object(target_obj, tx, ty);
          }
          continue;
        }

        // Close gripper
        muovi_pinza("close");
        rclcpp::sleep_for(std::chrono::milliseconds(2000));

        // Attach the collision object to the end-effector so MoveIt tracks it
        if (!target_obj.empty()) {
          add_domino_as_attached(target_obj);
          attached_object_id_ = target_obj;
        }

        log_event("pick", true, "x=" + std::to_string(tx));
        return true;
      }
    }
    log_event("pick", false, "all_candidates_failed");
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
    tp.dimensions = {0.8, 1.2, 1.0};   // height=1.0 matches SDF (was wrongly 0.5)
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
    p.position.z = 0.05;  // centred in the palm
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

    // ── 1. Activate / deactivate vacuum gripper in Gazebo ──────────────
    //   The Panda URDF loads libgazebo_ros_vacuum_gripper.so on
    //   panda_leftfinger.  Publishing true creates a fixed joint to the
    //   nearest object within max_distance (0.05 m).
    std_msgs::msg::Bool vacuum_msg;
    vacuum_msg.data = is_close;
    vacuum_gripper_pub_->publish(vacuum_msg);
    // Publish a few times to ensure it gets through in a laggy sim
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    vacuum_gripper_pub_->publish(vacuum_msg);

    // ── 2. Move MoveIt finger joints (visual + additional clamping) ───
    if (simulate_gripper_) {
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      return;
    }
    if (!hand_group_) {
      RCLCPP_WARN(this->get_logger(), "GRIPPER: hand_group_ is null, skipping joint move.");
      return;
    }
    std::vector<double> joints =
        is_close ? std::vector<double>{0.01, 0.01}
                 : std::vector<double>{0.04, 0.04};
    try {
      hand_group_->setJointValueTarget(joints);
      auto result = hand_group_->move();
      if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "GRIPPER: move() returned error code %d", result.val);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "GRIPPER: exception: %s", e.what());
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

  // Move to ready position (safer intermediate pose for retries)
  void go_to_ready()
  {
    // Ready pose: slightly raised and centered for better reachability
    const std::vector<double> ready = {0, -0.3, 0, -2.2, 0, 1.9, 0.785};
    try {
      RCLCPP_INFO(this->get_logger(), "Moving to ready position...");
      move_group_->setJointValueTarget(ready);
      auto result = move_group_->move();
      if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to move to ready: %d", result.val);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in go_to_ready: %s", e.what());
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
    // Go to ready first, then home for safer recovery
    go_to_ready();
    rclcpp::sleep_for(std::chrono::seconds(1));
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
