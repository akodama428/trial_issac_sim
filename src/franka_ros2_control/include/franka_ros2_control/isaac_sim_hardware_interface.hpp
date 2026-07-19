#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace franka_ros2_control
{

class IsaacSimHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);
  void on_gripper_command(const std_msgs::msg::String::SharedPtr msg);
  void apply_gripper_command_to_fingers();

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_closed_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_effort_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr finger_position_cmd_pub_;

  std::mutex state_mutex_;
  std::vector<double> position_state_;
  std::vector<double> velocity_state_;
  std::vector<double> position_command_;
  std::vector<double> velocity_command_;
  std::vector<double> effort_command_;

  std::string isaac_joint_states_topic_;
  std::string isaac_joint_commands_topic_;
  std::string isaac_arm_effort_commands_topic_;
  std::string isaac_finger_position_commands_topic_;
  std::string gripper_closed_topic_;
  std::string arm_command_mode_;
  std::optional<bool> gripper_closed_command_;
  bool state_received_{false};
};

}  // namespace franka_ros2_control
