#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <thread>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

class DominoPicker : public rclcpp::Node
{
public:
  DominoPicker() : Node("domino_picker_cpp")
  {
    subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/domino_detections", 10,
      std::bind(&DominoPicker::detection_callback, this, std::placeholders::_1));

    // Declare HSV threshold parameters for colors (degrees, 0-360 for H; 0-1 for S,V)
    this->declare_parameter<double>("red_h_min", 350.0);
    this->declare_parameter<double>("red_h_max", 10.0);
    this->declare_parameter<double>("red_s_min", 0.4);
    this->declare_parameter<double>("red_v_min", 0.2);

    this->declare_parameter<double>("green_h_min", 80.0);
    this->declare_parameter<double>("green_h_max", 160.0);
    this->declare_parameter<double>("green_s_min", 0.3);
    this->declare_parameter<double>("green_v_min", 0.2);

    this->declare_parameter<double>("blue_h_min", 200.0);
    this->declare_parameter<double>("blue_h_max", 260.0);
    this->declare_parameter<double>("blue_s_min", 0.3);
    this->declare_parameter<double>("blue_v_min", 0.2);

    // Read params
    red_h_min_ = this->get_parameter("red_h_min").as_double();
    red_h_max_ = this->get_parameter("red_h_max").as_double();
    red_s_min_ = this->get_parameter("red_s_min").as_double();
    red_v_min_ = this->get_parameter("red_v_min").as_double();

    green_h_min_ = this->get_parameter("green_h_min").as_double();
    green_h_max_ = this->get_parameter("green_h_max").as_double();
    green_s_min_ = this->get_parameter("green_s_min").as_double();
    green_v_min_ = this->get_parameter("green_v_min").as_double();

    blue_h_min_ = this->get_parameter("blue_h_min").as_double();
    blue_h_max_ = this->get_parameter("blue_h_max").as_double();
    blue_s_min_ = this->get_parameter("blue_s_min").as_double();
    blue_v_min_ = this->get_parameter("blue_v_min").as_double();

    RCLCPP_INFO(this->get_logger(), "Domino picker node started (HSV thresholds loaded)");
  }

  void setup_moveit()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "panda_arm");
    hand_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "hand");
    move_group_->setMaxVelocityScalingFactor(0.4);
    move_group_->setMaxAccelerationScalingFactor(0.4);
    go_to_home();
  }

private:
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_group_;
  std::atomic_bool is_busy_{false};

  // HSV thresholds (degrees 0-360 for H; S,V 0-1)
  double red_h_min_, red_h_max_, red_s_min_, red_v_min_;
  double green_h_min_, green_h_max_, green_s_min_, green_v_min_;
  double blue_h_min_, blue_h_max_, blue_s_min_, blue_v_min_;

  void detection_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    if (is_busy_) return;
    if (msg->markers.empty()) return;

    // Process first marker only for simplicity
    auto marker = msg->markers.front();
    is_busy_ = true;

    std::thread([this, marker]() {
      process_marker(marker);
      is_busy_ = false;
    }).detach();
  }

  void process_marker(const visualization_msgs::msg::Marker &marker)
  {
    // Extract detection pose
    double x = marker.pose.position.x;
    double y = marker.pose.position.y;
    double z = marker.pose.position.z;

    RCLCPP_INFO(this->get_logger(), "Processing domino at X=%.3f Y=%.3f Z=%.3f", x, y, z);

    // Pre-grasp high
    move_to_pose(x, y, z + 0.20);

    // Open gripper
    move_gripper("open");

    // Approach grasp
    move_to_pose(x, y, z + 0.02);

    // Close gripper
    move_gripper("close");
    rclcpp::sleep_for(700ms);

    // Lift
    move_to_pose(x, y, z + 0.20);

    // Compute place pose based on color
    double place_x = 0.50;
    static int index = 0;
    double place_y = -0.10 + 0.06 * (index % 8);
    double place_z = 0.235;

    double yaw = color_to_yaw(marker.color);

    move_to_pose(place_x, place_y, place_z + 0.20);

    // Place orientation
    move_to_pose_with_yaw(place_x, place_y, place_z + 0.02, yaw);

    // Release
    move_gripper("open");
    rclcpp::sleep_for(500ms);

    // Back to home
    go_to_home();
    index++;
  }

  double color_to_yaw(const std_msgs::msg::ColorRGBA &c)
  {
    // Convert RGB to HSV and match against thresholds
    double h, s, v;
    rgb_to_hsv(c.r, c.g, c.b, h, s, v);

    std::string label = detect_color_from_hsv(h, s, v);
    if (label == "red") return 0.0;
    if (label == "green") return 1.5708;
    if (label == "blue") return 3.14159;
    return 0.0;
  }

  void rgb_to_hsv(double r, double g, double b, double &h, double &s, double &v)
  {
    // r,g,b expected 0..1
    double mx = std::max({r, g, b});
    double mn = std::min({r, g, b});
    v = mx;
    double delta = mx - mn;
    if (mx == 0) {
      s = 0;
      h = 0;
      return;
    }
    s = delta / mx;
    if (delta == 0) {
      h = 0;
    } else if (mx == r) {
      h = 60.0 * (fmod(((g - b) / delta), 6.0));
    } else if (mx == g) {
      h = 60.0 * (((b - r) / delta) + 2.0);
    } else {
      h = 60.0 * (((r - g) / delta) + 4.0);
    }
    if (h < 0) h += 360.0;
  }

  std::string detect_color_from_hsv(double h, double s, double v)
  {
    // Helper to check if hue in range (handles wrap-around)
    auto in_range = [&](double hmin, double hmax, double hv) {
      if (hmin <= hmax) return (hv >= hmin && hv <= hmax);
      return (hv >= hmin || hv <= hmax);
    };

    if (s >= red_s_min_ && v >= red_v_min_ && in_range(red_h_min_, red_h_max_, h)) return "red";
    if (s >= green_s_min_ && v >= green_v_min_ && in_range(green_h_min_, green_h_max_, h)) return "green";
    if (s >= blue_s_min_ && v >= blue_v_min_ && in_range(blue_h_min_, blue_h_max_, h)) return "blue";
    return "unknown";
  }

  void move_to_pose(double x, double y, double z)
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(3.14159, 0, 0);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    move_group_->setPoseTarget(target_pose);
    move_group_->move();
  }

  void move_to_pose_with_yaw(double x, double y, double z, double yaw)
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(3.14159, 0, yaw);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    move_group_->setPoseTarget(target_pose);
    move_group_->move();
  }

  void move_gripper(const std::string &cmd)
  {
    if (!hand_group_) return;
    if (cmd == "open") {
      std::vector<double> joints = {0.04, 0.04};
      hand_group_->setJointValueTarget(joints);
    } else {
      std::vector<double> joints = {0.00, 0.00};
      hand_group_->setJointValueTarget(joints);
    }
    hand_group_->move();
  }

  void go_to_home()
  {
    if (!move_group_) return;
    std::vector<double> joint_group_positions = {0, -0.785, 0, -2.356, 0, 1.571, 0.785};
    move_group_->setJointValueTarget(joint_group_positions);
    move_group_->move();
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DominoPicker>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Setup MoveIt after node is running
  std::thread([node]() {
    rclcpp::sleep_for(std::chrono::seconds(2));
    node->setup_moveit();
  }).detach();

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
