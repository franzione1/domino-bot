/*
 * link_attacher_plugin.cpp
 *
 * Gazebo World Plugin that creates/destroys a FIXED ODE JOINT between the
 * panda_leftfinger link and the nearest domino model.
 *
 * This is the standard "cheat" for Gazebo grasping simulation:
 *   - Real robots grip with friction + force feedback (Robot-Domino-Artist)
 *   - Gazebo can't simulate that reliably, so we create a rigid joint instead
 *
 * Services:
 *   /link_attacher/attach  (std_srvs/SetBool, data ignored)
 *     → Finds the nearest model within 10 cm of panda_leftfinger
 *     → Creates a fixed joint locking it to panda_hand
 *
 *   /link_attacher/detach  (std_srvs/SetBool, data ignored)
 *     → Removes the fixed joint, freeing the object
 *
 * Approach adapted from pal-robotics/gazebo_ros_link_attacher (ROS 1),
 * ported to ROS 2 Foxy + Gazebo 11.
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <memory>
#include <string>
#include <mutex>

namespace gazebo
{

class LinkAttacherPlugin : public WorldPlugin
{
public:
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = world;
    ros_node_ = gazebo_ros::Node::Get(sdf);

    attach_service_ = ros_node_->create_service<std_srvs::srv::SetBool>(
      "/link_attacher/attach",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr /*req*/,
             std_srvs::srv::SetBool::Response::SharedPtr res) {
        std::lock_guard<std::mutex> lock(mutex_);
        this->Attach(res);
      });

    detach_service_ = ros_node_->create_service<std_srvs::srv::SetBool>(
      "/link_attacher/detach",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr /*req*/,
             std_srvs::srv::SetBool::Response::SharedPtr res) {
        std::lock_guard<std::mutex> lock(mutex_);
        this->Detach(res);
      });

    RCLCPP_INFO(ros_node_->get_logger(),
      "LinkAttacherPlugin LOADED — services: /link_attacher/attach, /link_attacher/detach");
  }

private:
  void Attach(std_srvs::srv::SetBool::Response::SharedPtr res)
  {
    if (fixed_joint_) {
      res->success = true;
      res->message = "Already attached to " + attached_model_name_;
      RCLCPP_WARN(ros_node_->get_logger(), "Already attached to %s", attached_model_name_.c_str());
      return;
    }

    // Find robot model and gripper link
    auto robot_model = world_->ModelByName("panda");
    if (!robot_model) {
      res->success = false;
      res->message = "Robot model 'panda' not found in world";
      RCLCPP_ERROR(ros_node_->get_logger(), "%s", res->message.c_str());
      return;
    }

    // Use panda_hand (not finger) as parent — more stable joint
    auto parent_link = robot_model->GetLink("panda_hand");
    if (!parent_link) {
      // Fallback: try panda_leftfinger
      parent_link = robot_model->GetLink("panda_leftfinger");
    }
    if (!parent_link) {
      res->success = false;
      res->message = "Neither panda_hand nor panda_leftfinger found";
      RCLCPP_ERROR(ros_node_->get_logger(), "%s", res->message.c_str());
      return;
    }

    // Find the nearest non-robot, non-table model within search radius
    auto parent_pos = parent_link->WorldPose().Pos();
    const double search_radius = 0.15;  // 15 cm
    double min_dist = search_radius;
    physics::ModelPtr nearest_model;

    for (unsigned int i = 0; i < world_->ModelCount(); ++i) {
      auto model = world_->ModelByIndex(i);
      const std::string &name = model->GetName();

      // Skip robot, ground, table, camera
      if (name == "panda" || name == "ground_plane" ||
          name == "work_table" || name.find("camera") != std::string::npos) {
        continue;
      }

      // Only attach to domino models (safety check)
      if (name.find("domino") == std::string::npos) {
        continue;
      }

      auto model_pos = model->WorldPose().Pos();
      double dist = (model_pos - parent_pos).Length();
      if (dist < min_dist) {
        min_dist = dist;
        nearest_model = model;
      }
    }

    if (!nearest_model) {
      res->success = false;
      res->message = "No domino model found within " +
                     std::to_string(search_radius) + "m of gripper";
      RCLCPP_WARN(ros_node_->get_logger(), "%s", res->message.c_str());
      return;
    }

    // Get the main link of the target model
    auto child_link = nearest_model->GetLinks().front();
    if (!child_link) {
      res->success = false;
      res->message = "Target model has no links";
      RCLCPP_ERROR(ros_node_->get_logger(), "%s", res->message.c_str());
      return;
    }

    // Create fixed joint using the approach from gazebo_ros_link_attacher:
    //   - Create a revolute joint with zero upper/lower limits = effectively fixed
    //   - This is more reliable than CreateJoint("fixed") in some Gazebo versions
    fixed_joint_ = world_->Physics()->CreateJoint("revolute", robot_model);
    fixed_joint_->Attach(parent_link, child_link);
    fixed_joint_->Load(parent_link, child_link, ignition::math::Pose3d());
    fixed_joint_->SetModel(robot_model);
    fixed_joint_->SetUpperLimit(0, 0.0);
    fixed_joint_->SetLowerLimit(0, 0.0);
    fixed_joint_->Init();

    attached_model_name_ = nearest_model->GetName();

    res->success = true;
    res->message = "FIXED JOINT: panda_hand <-> " + attached_model_name_ +
                   " (dist=" + std::to_string(min_dist) + "m)";
    RCLCPP_INFO(ros_node_->get_logger(),
      "*** FIXED JOINT CREATED: panda_hand <-> %s (dist=%.4fm) ***",
      attached_model_name_.c_str(), min_dist);
  }

  void Detach(std_srvs::srv::SetBool::Response::SharedPtr res)
  {
    if (!fixed_joint_) {
      res->success = true;
      res->message = "Not attached — nothing to detach";
      return;
    }

    fixed_joint_->Detach();
    fixed_joint_.reset();

    RCLCPP_INFO(ros_node_->get_logger(),
      "*** FIXED JOINT REMOVED: %s ***", attached_model_name_.c_str());

    res->success = true;
    res->message = "Detached " + attached_model_name_;
    attached_model_name_.clear();
  }

  physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr attach_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr detach_service_;
  physics::JointPtr fixed_joint_;
  std::string attached_model_name_;
  std::mutex mutex_;
};

GZ_REGISTER_WORLD_PLUGIN(LinkAttacherPlugin)

}  // namespace gazebo
