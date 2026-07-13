#include "franka_ros2_control/local_safety_observation.hpp"

#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace franka_ros2_control
{

class LocalSafetyObservationAdapter
{
public:
  explicit LocalSafetyObservationAdapter(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    group_name_ = node_->declare_parameter<std::string>("move_group_name", "panda_arm");
    tip_link_name_ = node_->declare_parameter<std::string>("tip_link_name", "panda_hand");
    const double publish_rate_hz = node_->declare_parameter<double>("publish_rate_hz", 20.0);
    publisher_ = node_->create_publisher<std_msgs::msg::String>(
      "/tomato_harvest/local_safety_status", rclcpp::SystemDefaultsQoS());

    planning_scene_monitor_ =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description");
    if (!planning_scene_monitor_->getPlanningScene()) {
      throw std::runtime_error("PlanningSceneMonitor failed to load robot model");
    }
    planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");
    planning_scene_monitor_->startStateMonitor("/joint_states");

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz));
    timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {publish_observation();});
    RCLCPP_INFO(node_->get_logger(), "Safety observation adapter started at %.1f Hz", publish_rate_hz);
  }

private:
  void publish_observation()
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    moveit::core::RobotState state(scene->getCurrentState());
    const moveit::core::JointModelGroup * group = state.getJointModelGroup(group_name_);
    const moveit::core::LinkModel * tip_link = state.getLinkModel(tip_link_name_);
    if (group == nullptr || tip_link == nullptr) {
      RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000, "Unknown group '%s' or tip link '%s'",
        group_name_.c_str(), tip_link_name_.c_str());
      return;
    }

    Eigen::MatrixXd jacobian;
    if (!state.getJacobian(group, tip_link, Eigen::Vector3d::Zero(), jacobian)) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000, "Jacobian calculation failed");
      return;
    }
    // Harvest correction is position-dominant.  The translational block avoids
    // classifying a harmless wrist-orientation singularity as a Cartesian stop.
    const auto singularity = normalized_singularity_margin(jacobian.topRows(3));
    if (!singularity.has_value()) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000, "Invalid Jacobian singular values");
      return;
    }

    state.updateCollisionBodyTransforms();
    double clearance = scene->distanceToCollision(state);
    if (scene->isStateColliding(state, group_name_)) {
      clearance = 0.0;
    }
    if (!std::isfinite(clearance) || clearance > 1.0) {
      // No world object is close enough for the collision backend to return a finite distance.
      clearance = 1.0;
    }

    std_msgs::msg::String message;
    message.data = local_safety_observation_json({clearance, *singularity});
    publisher_->publish(message);
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "LOCAL_SAFETY_OBSERVATION %s", message.data.c_str());
  }

  std::string group_name_;
  std::string tip_link_name_;
  rclcpp::Node::SharedPtr node_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace franka_ros2_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("local_safety_observation_node");
  auto adapter = std::make_shared<franka_ros2_control::LocalSafetyObservationAdapter>(node);
  rclcpp::spin(node);
  adapter.reset();
  rclcpp::shutdown();
  return 0;
}
