
#include "franka_ros2_control/isaac_sim_hardware_interface.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace franka_ros2_control
{

hardware_interface::CallbackReturn IsaacSimHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  isaac_joint_states_topic_ =
    info_.hardware_parameters.count("isaac_joint_states_topic")
    ? info_.hardware_parameters.at("isaac_joint_states_topic")
    : "/isaac_joint_states";

  isaac_joint_commands_topic_ =
    info_.hardware_parameters.count("isaac_joint_commands_topic")
    ? info_.hardware_parameters.at("isaac_joint_commands_topic")
    : "/isaac_joint_commands";

  const std::size_t n_joints = info_.joints.size();
  position_state_.resize(n_joints, 0.0);
  velocity_state_.resize(n_joints, 0.0);
  position_command_.resize(n_joints, std::numeric_limits<double>::quiet_NaN());
  velocity_command_.resize(n_joints, 0.0);

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 2) {
      RCLCPP_ERROR(rclcpp::get_logger("IsaacSimHardwareInterface"),
        "Joint '%s' must have exactly 2 command interfaces (position, velocity).",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_ERROR(rclcpp::get_logger("IsaacSimHardwareInterface"),
        "Joint '%s' must have exactly 2 state interfaces (position, velocity).",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IsaacSimHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  node_ = std::make_shared<rclcpp::Node>(
    "isaac_sim_hardware_interface",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    isaac_joint_states_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      on_joint_state(msg);
    });

  joint_cmd_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    isaac_joint_commands_topic_,
    rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    rclcpp::get_logger("IsaacSimHardwareInterface"),
    "Configured: subscribing to '%s', publishing to '%s'",
    isaac_joint_states_topic_.c_str(), isaac_joint_commands_topic_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IsaacSimHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::size_t n_joints = info_.joints.size();

  // Wait up to 5 seconds for initial joint state from Isaac Sim.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!state_received_ && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  if (!state_received_) {
    RCLCPP_WARN(rclcpp::get_logger("IsaacSimHardwareInterface"),
      "No joint state received from Isaac Sim within 5s — using zeros.");
  }

  // Seed command positions with current state so we don't jump on activate.
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    position_command_ = position_state_;
  }
  velocity_command_.assign(n_joints, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IsaacSimHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
IsaacSimHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &position_state_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &velocity_state_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
IsaacSimHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &position_command_[i]);
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &velocity_command_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type IsaacSimHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  rclcpp::spin_some(node_);
  std::lock_guard<std::mutex> lock(state_mutex_);
  // position_state_ / velocity_state_ are already updated by the subscription callback.
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IsaacSimHardwareInterface::write(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Isaac Sim が接続して最初の joint_state を送るまでは命令を送らない。
  // 接続前に [0,0,...,0] を送ると JTC が "ゼロでホールド" し、
  // 接続後に Isaac Sim の実位置との不一致で起動時の意図しない動きが生じるため。
  if (!state_received_) {
    return hardware_interface::return_type::OK;
  }

  // Franka Panda URDF 関節位置上下限 (rad)。URDF の <limit> と一致させること。
  // panda_joint4 の upper は URDF 変更に合わせて -0.069 にしている。
  static constexpr std::array<double, 7> LOWER = {
    -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973
  };
  static constexpr std::array<double, 7> UPPER = {
     2.8973,  1.7628,  2.8973, -0.069,   2.8973,  3.7525,  2.8973
  };

  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = time;
  msg.name.reserve(info_.joints.size());
  msg.position.reserve(info_.joints.size());
  msg.velocity.reserve(info_.joints.size());

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    msg.name.push_back(info_.joints[i].name);
    double pos = std::isnan(position_command_[i])
      ? position_state_[i]
      : position_command_[i];
    // URDF 境界外の命令を Isaac Sim へ送らない。
    // JTC が起動時に「ゼロでホールド」を命令しても panda_joint4=0 は
    // 可動域外 [-3.0718, -0.069] であるため、ここでクランプして防ぐ。
    if (i < LOWER.size()) {
      pos = std::clamp(pos, LOWER[i], UPPER[i]);
    }
    msg.position.push_back(pos);
    msg.velocity.push_back(velocity_command_[i]);
  }

  joint_cmd_pub_->publish(msg);
  return hardware_interface::return_type::OK;
}

void IsaacSimHardwareInterface::on_joint_state(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  const std::size_t n_joints = info_.joints.size();
  std::lock_guard<std::mutex> lock(state_mutex_);

  // Map by joint name so order in the message doesn't need to match URDF.
  for (std::size_t i = 0; i < n_joints; ++i) {
    const auto & joint_name = info_.joints[i].name;
    for (std::size_t j = 0; j < msg->name.size(); ++j) {
      if (msg->name[j] == joint_name) {
        if (j < msg->position.size()) {
          position_state_[i] = msg->position[j];
        }
        if (j < msg->velocity.size()) {
          velocity_state_[i] = msg->velocity[j];
        }
        break;
      }
    }
  }

  if (!state_received_) {
    // Isaac Sim との初回接続: 実際の関節位置で position_command_ をリシード。
    // JTC が 0 を "ホールド位置" として保持していた場合でも、
    // 次の write() は実際の位置を送るため起動時の意図しない動きを防ぐ。
    position_command_ = position_state_;
    velocity_command_.assign(n_joints, 0.0);
    RCLCPP_INFO(
      rclcpp::get_logger("IsaacSimHardwareInterface"),
      "Initial joint state received from Isaac Sim. Reseeding command positions.");
  }
  state_received_ = true;
}

}  // namespace franka_ros2_control

PLUGINLIB_EXPORT_CLASS(
  franka_ros2_control::IsaacSimHardwareInterface,
  hardware_interface::SystemInterface)
