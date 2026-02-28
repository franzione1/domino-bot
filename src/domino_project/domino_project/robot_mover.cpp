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
#include <std_srvs/srv/set_bool.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
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
const double DOMINO_WID     = 0.024;   // short axis (SDF y) — gripper closes across this
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
//   panda_leftfinger collision box: origin z=0.027, size z=0.052
//     pad TOP    = 0.0584 + 0.027 − 0.026 = 0.0594 m below link8
//     pad CENTRE = 0.0584 + 0.027          = 0.0854 m below link8
//     pad BOTTOM = 0.0584 + 0.027 + 0.026 = 0.1114 m below link8
//
// Context: cell_layout_2=true mounts the robot on a 1.3 m pedestal.
//   panda_link0 world z = 0.65 (pedestal_joint) + 0.655 (panda_joint) = 1.305 m
//   The arm reaches DOWNWARD — table top (1.30 m) is just below the robot base.
const double PANDA_LINK8_TO_FINGERTIP = 0.0584 + 0.0538;   // 0.1122 m (mesh end)
const double PANDA_LINK8_TO_PAD_BOTTOM = 0.0584 + 0.027 + 0.026;  // 0.1114 m (collision box bottom)

// Grasp z: computed from URDF finger geometry — NOT from the reference's TCP offset.
//
//   The reference uses fer_hand_tcp (0.1034 m below link8 on the real Franka).
//   Our URDF finger pad bottom is at 0.1114 m below link8 — different from 0.1034.
//   Using the reference's PANDA_TCP_OFFSET=0.1034 in the formula made the fingers
//   descend 0.008 m (8 mm) LESS than intended (0.1114 − 0.1034 = 0.008).
//
//   Correct approach: derive z_presa directly from the collision geometry.
//
//   Goal: finger pad bottom should be TABLE_CLEARANCE mm above the table,
//   maximising overlap with the 7.5 mm piece.
//
//   finger_pad_bottom = link8_z − 0.1114
//   finger_pad_bottom = TABLE_TOP_Z + TABLE_CLEARANCE
//   ∴ link8_z = TABLE_TOP_Z + TABLE_CLEARANCE + 0.1114
//
//   TABLE_CLEARANCE = 0.0005 m (0.5 mm) — enough to avoid Gazebo collision
//   with the table surface, verified safe (min_depth=0.0001 in ODE).
//
//   Result:
//     link8_z = 1.30 + 0.0005 + 0.1114 = 1.4119 m
//     finger_pad_bottom = 1.3005 m  (0.5 mm above table)
//     piece_top = 1.3075 m  →  overlap = 7.0 mm (93% of 7.5 mm piece)  ✓
//
//   Previous value was 1.41315 (finger bottom at 1.30175, overlap=5.75mm=77%).
//   Going 1.25 mm lower gains 17% more overlap.
const double TABLE_CLEARANCE = 0.0005;  // 0.5 mm above table surface
const double Z_PRESA_DEFAULT = TABLE_TOP_Z + TABLE_CLEARANCE + PANDA_LINK8_TO_PAD_BOTTOM;
//   = 1.30 + 0.0005 + 0.1114 = 1.4119 m

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

    // Vacuum gripper plugin: 'switch' is a SetBool SERVICE, 'grasping' is a Bool TOPIC.
    // The URDF remaps switch→vacuum_switch so it doesn't collide with the status topic.
    vacuum_gripper_client_ = this->create_client<std_srvs::srv::SetBool>(
        "/panda_hand/vacuum_switch");

    // GripperCommand action clients for individual finger control.
    // These bypass MoveIt and talk directly to the GripperActionControllers,
    // which have built-in stall detection (stall_timeout: 1.0 s).
    // This is equivalent to the reference project's adaptive_stop mechanism.
    left_gripper_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
        this, "/panda_handleft_controller/gripper_cmd");
    right_gripper_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
        this, "/panda_handright_controller/gripper_cmd");

    // Subscribe to vacuum status feedback from the Gazebo plugin
    vacuum_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/panda_hand/grasping", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          bool prev = vacuum_attached_.load();
          vacuum_attached_ = msg->data;
          if (msg->data != prev) {
            RCLCPP_INFO(this->get_logger(), "VACUUM STATUS: %s",
                        msg->data ? "*** ATTACHED ***" : "detached");
          }
        });

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

    // Slow gripper to match reference hardware (gripper_width speed=0.05 m/s).
    // URDF velocity limit = 0.2 m/s × scaling 0.1 = 0.02 m/s effective.
    // Reference hardware closes 0.02 m range in ~0.4 s; we take ~1.0 s (safer in sim).
    hand_group_->setMaxVelocityScalingFactor(0.1);
    hand_group_->setMaxAccelerationScalingFactor(0.1);
    hand_group_->setPlanningTime(5.0);
    hand_group_->setGoalJointTolerance(0.001);  // reference uses JointConstraint tolerance=0.001

    // Log gripper group info for diagnostics
    RCLCPP_INFO(this->get_logger(), "GRIPPER group joints: %zu",
                hand_group_->getJointNames().size());
    for (const auto &jn : hand_group_->getJointNames()) {
      RCLCPP_INFO(this->get_logger(), "  - %s", jn.c_str());
    }

    move_group_->setMaxVelocityScalingFactor(planner_max_velocity_);
    move_group_->setMaxAccelerationScalingFactor(planner_max_acceleration_);
    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(20);       // ref: num_planning_attempts=20
    move_group_->setGoalPositionTolerance(0.002);  // ref: 1e-3; 2 mm is safe for OMPL
    move_group_->setGoalOrientationTolerance(0.01); // ref: 1e-3; ~0.6° for OMPL

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

    // Wait for vacuum gripper service (Gazebo plugin must be loaded)
    RCLCPP_INFO(this->get_logger(), "Waiting for vacuum gripper service /panda_hand/vacuum_switch…");
    if (vacuum_gripper_client_->wait_for_service(std::chrono::seconds(30))) {
      RCLCPP_INFO(this->get_logger(), "Vacuum gripper service AVAILABLE.");
    } else {
      RCLCPP_WARN(this->get_logger(),
          "Vacuum gripper service NOT available after 30 s — vacuum grasping unavailable.");
    }

    // Wait for GripperCommand action servers (finger controllers)
    RCLCPP_INFO(this->get_logger(), "Waiting for gripper action servers…");
    bool left_ok = left_gripper_client_->wait_for_action_server(std::chrono::seconds(10));
    bool right_ok = right_gripper_client_->wait_for_action_server(std::chrono::seconds(10));
    RCLCPP_INFO(this->get_logger(), "Gripper action servers: left=%s, right=%s",
                left_ok ? "OK" : "MISSING", right_ok ? "OK" : "MISSING");

    go_to_home();

    moveit_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "C++ NODE READY – Domino game logic active.");
  }

private:
  // ── ROS handles ──────────────────────────────────────────────────────────
  using GripperAction = control_msgs::action::GripperCommand;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr detections_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr vacuum_gripper_client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vacuum_status_sub_;
  rclcpp_action::Client<GripperAction>::SharedPtr left_gripper_client_;
  rclcpp_action::Client<GripperAction>::SharedPtr right_gripper_client_;
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
  std::atomic_bool vacuum_attached_{false};
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

    // ── Grasp yaw: fingers close ACROSS piece short axis (24 mm) ────────
    //   panda_hand_joint rotates −π/4 from link8 around Z.
    //   Finger prismatic axis = hand Y.  Hand Y in world = θ_link8 − π/4.
    //   For hand Y ∥ short axis (target.yaw + π/2):
    //     θ_link8 = target.yaw + π/2 + π/4 = target.yaw + 3π/4
    //   Short axis chosen for simulation:
    //     • 18 mm descent clearance per side (vs 6 mm on long axis)
    //     • 24 mm across → stronger clamping than 48 mm
    //     • Reference uses long axis with force feedback we don't have
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
    //  STEP 2 — OPEN fingers (ref: gripper_width(0.06, 0.05))
    // ═══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(this->get_logger(), "  STEP 2 — opening fingers...");
    muovi_pinza("open");
    rclcpp::sleep_for(std::chrono::seconds(2));  // ref: time.sleep(2)

    // ═══════════════════════════════════════════════════════════════════════
    //  STEP 2b — Remove table + domino from MoveIt collision scene
    //   Reference: lower_until_force_limit() calls remove_table before descent.
    //   This allows the arm to descend low enough for finger pads to fully
    //   wrap the piece.  Table is re-added after gripping (STEP 4).
    // ═══════════════════════════════════════════════════════════════════════
    std::string target_obj = find_closest_domino(target.x, target.y);
    {
      std::vector<std::string> remove_ids = {"work_table"};
      if (!target_obj.empty()) remove_ids.push_back(target_obj);
      planning_scene_interface_->removeCollisionObjects(remove_ids);
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  STEP 3 — DESCEND onto piece
    //   Reference uses plan_pose_path (IK goal) for descent (MOVE 4),
    //   with vel=0.1, acc=0.1.  Then retries up to 100 times.
    //   We try IK with slow speed up to 5 times, then Cartesian fallback.
    //   Table collision is already removed (STEP 2b).
    // ═══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(this->get_logger(), "  STEP 3 — descending to z=%.4f (slow, IK first)...", z_presa_);
    move_group_->setMaxVelocityScalingFactor(0.1);     // ref: 0.1 for descent
    move_group_->setMaxAccelerationScalingFactor(0.1);  // ref: 0.1 for descent

    bool descended = false;
    for (int ik_try = 1; ik_try <= 5 && !descended; ++ik_try) {
      RCLCPP_INFO(this->get_logger(), "    IK descent attempt %d/5...", ik_try);
      descended = plan_and_execute(target.x, target.y, z_presa_, pick_yaw);
    }
    if (!descended) {
      RCLCPP_WARN(this->get_logger(), "    IK descent failed after 5 tries, trying Cartesian...");
      descended = cartesian_descend(target.x, target.y, z_alta_, z_presa_, pick_yaw);
    }

    move_group_->setMaxVelocityScalingFactor(planner_max_velocity_);
    move_group_->setMaxAccelerationScalingFactor(planner_max_acceleration_);

    if (!descended) {
      RCLCPP_ERROR(this->get_logger(), "  Cannot descend onto piece — aborting.");
      add_table_collision();
      if (!target_obj.empty()) add_domino_collision_object(target_obj, target.x, target.y);
      emergency_stop_and_recover();
      return;
    }
    rclcpp::sleep_for(std::chrono::seconds(1));  // settle

    // ═══════════════════════════════════════════════════════════════════════
    //  STEP 4 — GRASP piece
    //   Reference sequence (Robot-Domino-Artist):
    //     1. Attach collision object (for MoveIt planning)
    //     2. Close fingers (physical grip via GripperCommand with stall detection)
    //     3. Vacuum ON (backup ODE fixed joint)
    //   Fingers close FIRST so the piece is gripped by friction BEFORE
    //   the vacuum fixed joint is created.  If we did vacuum first, the
    //   leftfinger would drag the piece sideways when closing.
    // ═══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(this->get_logger(), "  STEP 4 — grasping piece...");

    // 4a. Attach collision object for MoveIt planning (ref: attach BEFORE close)
    if (!target_obj.empty()) {
      add_domino_as_attached(target_obj);
      attached_object_id_ = target_obj;
    }

    // 4b. Close fingers via GripperCommand action (with stall detection)
    //     Target: 0.005/finger (ref: domino_length_closing = 0.01 total).
    //     The controller has stall_timeout=1.0s — it stops when fingers
    //     contact the piece (stalls), just like the reference adaptive_stop.
    //     max_effort=10N prevents pushing the piece away.
    muovi_pinza("close");
    rclcpp::sleep_for(std::chrono::seconds(2));  // ref: time.sleep(2) after close

    // 4c. Vacuum ON for backup grip (ODE fixed joint)
    RCLCPP_INFO(this->get_logger(), "  STEP 4c — vacuum ON (backup)...");
    call_vacuum_service(true);
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(this->get_logger(), "  Vacuum status: %s",
                vacuum_attached_.load() ? "ATTACHED" : "not attached (finger grip only)");

    log_event("pick", true, "x=" + std::to_string(target.x));

    // ═══════════════════════════════════════════════════════════════════════
    //  STEP 4b — LIFT 10 cm (Cartesian, no collision check)
    //   CRITICAL: at z_presa, fingertips are only 1.75 mm above the table.
    //   MoveIt collision padding (~10 mm) would consider this IN-COLLISION
    //   with the table, causing OMPL to fail on the transit plan.
    //   Reference also lifts 15 cm (MOVEMENT 8) before transit.
    //   We lift BEFORE re-adding the table to avoid the collision issue.
    // ═══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(this->get_logger(), "  STEP 4b — Cartesian lift 10 cm...");
    double lift_z = z_presa_ + 0.10;
    if (!cartesian_move(target.x, target.y, lift_z, pick_yaw)) {
      RCLCPP_WARN(this->get_logger(), "    Cartesian lift failed, trying IK...");
      plan_and_execute(target.x, target.y, lift_z, pick_yaw);
    }
    // Check vacuum status after lift
    RCLCPP_INFO(this->get_logger(), "  Vacuum status after lift: %s",
                vacuum_attached_.load() ? "ATTACHED" : "finger grip only");

    // NOW re-add table collision — arm is 10 cm higher, well clear of padding
    add_table_collision();

    // ═══════════════════════════════════════════════════════════════════════
    //  STEP 5 — MOVE to destination (above drop, then lower)
    //   Reference: MOVE 8 (lift) → MOVE 9 (go home) → MOVE 10 (above goal)
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

    // Remove table collision before lowering (same as pick descent)
    remove_table_collision();

    // Lower to placement height — slow, IK first (like reference), Cartesian fallback
    RCLCPP_INFO(this->get_logger(), "  STEP 5b — lowering to place (slow, IK first)...");
    move_group_->setMaxVelocityScalingFactor(0.1);     // ref: 0.1 for descent
    move_group_->setMaxAccelerationScalingFactor(0.1);

    bool lowered = false;
    for (int ik_try = 1; ik_try <= 5 && !lowered; ++ik_try) {
      RCLCPP_INFO(this->get_logger(), "    IK lower attempt %d/5...", ik_try);
      lowered = plan_and_execute(drop_x, drop_y, z_presa_, drop_yaw);
    }
    if (!lowered) {
      RCLCPP_WARN(this->get_logger(), "    IK lower failed, trying Cartesian...");
      lowered = cartesian_descend(drop_x, drop_y, z_alta_, z_presa_, drop_yaw);
    }

    move_group_->setMaxVelocityScalingFactor(planner_max_velocity_);
    move_group_->setMaxAccelerationScalingFactor(planner_max_acceleration_);
    if (!lowered) {
      RCLCPP_ERROR(this->get_logger(), "  Cannot lower to place — aborting.");
      add_table_collision();
      emergency_stop_and_recover();
      return;
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  STEP 6 — OPEN fingers (release piece)
    //   Reference: detach, open gripper, sleep(2)
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
    //   Cartesian lift BEFORE re-adding table (same collision padding issue
    //   as STEP 4b — fingertips only 1.75 mm above table at z_presa).
    // ═══════════════════════════════════════════════════════════════════════
    RCLCPP_INFO(this->get_logger(), "  STEP 7 — lifting from place position...");
    double place_lift_z = z_presa_ + 0.10;
    if (!cartesian_move(drop_x, drop_y, place_lift_z, drop_yaw)) {
      plan_and_execute(drop_x, drop_y, place_lift_z, drop_yaw);
    }

    // NOW re-add table collision — arm is clear
    add_table_collision();

    RCLCPP_INFO(this->get_logger(), "  STEP 7b — returning to home...");
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

    if (fraction >= 0.98) {
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

    // Virtual table — EXACTLY matches models/work_table/model.sdf
    // SDF box size: 0.8 x 1.2 x 1.0, spawned at z=0.8 (box centre)
    // → table top = 0.8 + 0.5 = 1.30 m = TABLE_TOP_Z  ✓
    // Reference approach: full-height table, removed/re-added for descent
    // (reference: table.size=[0.908,0.605,0.838], top at z=0 exactly)
    moveit_msgs::msg::CollisionObject table;
    table.id = "work_table";
    table.header.frame_id = "world";
    shape_msgs::msg::SolidPrimitive tp;
    tp.type = shape_msgs::msg::SolidPrimitive::BOX;
    tp.dimensions = {0.8, 1.2, 1.0};  // full height — top at 1.30 = real table
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

  // ── Table collision management (matches reference remove_table / add_table) ──
  //   Reference removes the table before descent so MoveIt allows the gripper
  //   to reach below the table surface, then re-adds it after gripping.
  void remove_table_collision()
  {
    RCLCPP_INFO(this->get_logger(), "Removing table collision for descent...");
    planning_scene_interface_->removeCollisionObjects({"work_table"});
    rclcpp::sleep_for(std::chrono::milliseconds(300));
  }

  void add_table_collision()
  {
    RCLCPP_INFO(this->get_logger(), "Re-adding table collision...");
    moveit_msgs::msg::CollisionObject table;
    table.id = "work_table";
    table.header.frame_id = "world";
    shape_msgs::msg::SolidPrimitive tp;
    tp.type = shape_msgs::msg::SolidPrimitive::BOX;
    tp.dimensions = {0.8, 1.2, 1.0};
    geometry_msgs::msg::Pose tpose;
    tpose.position.x = 0.7; tpose.position.y = 0.0; tpose.position.z = 0.80;
    tpose.orientation.w = 1.0;
    table.primitives.push_back(tp);
    table.primitive_poses.push_back(tpose);
    table.operation = table.ADD;
    planning_scene_interface_->applyCollisionObjects({table});
    rclcpp::sleep_for(std::chrono::milliseconds(300));
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

    if (fraction >= 0.98) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      if (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        return true;
      }
      RCLCPP_ERROR(this->get_logger(), "Cartesian path execution failed");
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Cartesian path coverage insufficient: %.2f < 0.98", fraction);
    }
    return false;
  }

  // ── Helper: plan and execute a single finger position target ─────────
  //   Used by muovi_pinza() for the two-stage close pattern that matches
  //   the reference project's adaptive_stop behaviour.
  bool move_fingers_to(double target_pos)
  {
    if (!hand_group_) return false;
    for (int attempt = 1; attempt <= 3; ++attempt) {
      try {
        const auto *jmg =
            hand_group_->getCurrentState(10)->getJointModelGroup("panda_gripper");
        auto cur_state = hand_group_->getCurrentState(10);
        std::vector<double> positions;
        cur_state->copyJointGroupPositions(jmg, positions);

        RCLCPP_INFO(this->get_logger(),
            "GRIPPER: current [%.4f, %.4f] \u2192 target %.4f (attempt %d)",
            positions.size() > 0 ? positions[0] : -1.0,
            positions.size() > 1 ? positions[1] : -1.0,
            target_pos, attempt);

        if (positions.size() >= 2) {
          positions[0] = target_pos;
          positions[1] = target_pos;
        }
        hand_group_->setJointValueTarget(positions);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (hand_group_->plan(plan) ==
            moveit::planning_interface::MoveItErrorCode::SUCCESS) {
          auto res = hand_group_->execute(plan);
          if (res == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(),
                "GRIPPER: finger move OK (attempt %d)", attempt);
            return true;
          }
          RCLCPP_WARN(this->get_logger(),
              "GRIPPER: exec failed (%d), attempt %d", res.val, attempt);
        } else {
          RCLCPP_WARN(this->get_logger(),
              "GRIPPER: plan failed, attempt %d", attempt);
        }
        rclcpp::sleep_for(std::chrono::milliseconds(500));
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(),
            "GRIPPER: exception attempt %d: %s", attempt, e.what());
        rclcpp::sleep_for(std::chrono::milliseconds(500));
      }
    }
    RCLCPP_ERROR(this->get_logger(),
        "GRIPPER: ALL attempts failed for target %.4f!", target_pos);
    return false;
  }

  void muovi_pinza(const std::string &command)
  {
    bool is_close = (command == "close");
    RCLCPP_INFO(this->get_logger(), "GRIPPER: %s  (simulate=%s)",
                command.c_str(), simulate_gripper_ ? "true" : "false");

    // ── Grasp strategy (matching reference Robot-Domino-Artist) ──────────
    //   CLOSE: fingers close FIRST (physical grip), then vacuum ON for backup.
    //   OPEN:  vacuum OFF, then fingers open.
    //
    //   Uses MoveIt panda_gripper group so both joints move in a SINGLE
    //   synchronised trajectory.  Separate GripperCommand action clients
    //   caused asymmetric timing (one finger arrived before the other),
    //   tipping the piece sideways.

    if (is_close) {
      // ── CLOSE: both fingers simultaneously via MoveIt hand_group ───
      //   Piece short axis = DOMINO_WID (24 mm).  Fingers close across it.
      //   Contact when gap = DOMINO_WID → q_contact = DOMINO_WID/2 = 0.012.
      //
      //   We command q = DOMINO_WID/2 − 0.002 = 0.010/finger (20 mm gap).
      //   This is only 2 mm past contact per side — gentle clamping.
      const double close_pos = DOMINO_WID / 2.0 - 0.002;  // 0.010/finger
      RCLCPP_INFO(this->get_logger(),
          "GRIPPER: closing to %.4f/finger (%.1fmm gap, piece=%.0fmm)...",
          close_pos, close_pos * 2000.0, DOMINO_WID * 1000.0);
      move_fingers_to(close_pos);
      rclcpp::sleep_for(std::chrono::seconds(2));
      RCLCPP_INFO(this->get_logger(), "GRIPPER: fingers closed.");
    } else {
      // ── OPEN: vacuum OFF first, then open fingers ──────────────────
      RCLCPP_INFO(this->get_logger(), "GRIPPER: vacuum OFF, opening fingers...");
      call_vacuum_service(false);
      rclcpp::sleep_for(std::chrono::milliseconds(500));

      move_fingers_to(0.035);
      rclcpp::sleep_for(std::chrono::seconds(1));
      rclcpp::sleep_for(std::chrono::seconds(2));
      RCLCPP_INFO(this->get_logger(), "GRIPPER: fingers open.");
    }
  }

  // ── Helper: send GripperCommand to one finger controller ────────────────
  //   Uses rclcpp_action to send a goal with position + max_effort.
  //   The GripperActionController will stop the finger at the piece surface
  //   thanks to stall_timeout (1.0s) and stall_velocity_threshold (0.001),
  //   mimicking the reference project's adaptive_stop mechanism.
  void send_gripper_command(
      rclcpp_action::Client<GripperAction>::SharedPtr &client,
      double position, double max_effort)
  {
    if (!client->action_server_is_ready()) {
      RCLCPP_WARN(this->get_logger(),
          "Gripper action server not ready, falling back to MoveIt...");
      // Fallback: use MoveIt hand_group
      if (hand_group_) move_fingers_to(position);
      return;
    }

    auto goal = GripperAction::Goal();
    goal.command.position = position;
    goal.command.max_effort = max_effort;

    auto send_opts = rclcpp_action::Client<GripperAction>::SendGoalOptions();
    // Fire-and-forget: the controller will handle stall detection internally.
    // We wait with a sleep in the caller.
    auto future = client->async_send_goal(goal, send_opts);
    // Wait briefly for goal acceptance
    if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
      auto gh = future.get();
      if (!gh) {
        RCLCPP_WARN(this->get_logger(), "Gripper goal rejected.");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Gripper goal acceptance timed out.");
    }
  }

  // ── Helper: call the vacuum gripper SetBool service ─────────────────────
  bool call_vacuum_service(bool on)
  {
    if (!vacuum_gripper_client_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "Vacuum service not ready, waiting 5 s...");
      if (!vacuum_gripper_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Vacuum service still not available!");
        return false;
      }
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = on;

    auto future = vacuum_gripper_client_->async_send_request(request);

    // Block until the response arrives (safe — we're in the worker thread,
    // the MultiThreadedExecutor processes the response on another thread)
    auto status = future.wait_for(std::chrono::seconds(10));
    if (status == std::future_status::ready) {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(),
          "VACUUM %s → success=%s, msg='%s'",
          on ? "ON" : "OFF",
          response->success ? "true" : "false",
          response->message.c_str());
      return response->success;
    }

    RCLCPP_ERROR(this->get_logger(), "Vacuum service call timed out (10 s)!");
    return false;
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
