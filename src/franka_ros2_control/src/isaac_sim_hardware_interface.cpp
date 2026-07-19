
#include "franka_ros2_control/isaac_sim_hardware_interface.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace franka_ros2_control
{

namespace
{

constexpr char kFingerJoint1[] = "panda_finger_joint1";
constexpr char kFingerJoint2[] = "panda_finger_joint2";
constexpr double kGripperOpenPosition = 0.04;
constexpr double kGripperClosedPosition = 0.0;
constexpr std::size_t kArmJointCount = 7;
constexpr std::array<double, kArmJointCount> kMaxEffort = {
  87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0
};

}  // namespace

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
  isaac_arm_effort_commands_topic_ =
    info_.hardware_parameters.count("isaac_arm_effort_commands_topic")
    ? info_.hardware_parameters.at("isaac_arm_effort_commands_topic")
    : "/isaac_arm_effort_commands";
  isaac_finger_position_commands_topic_ =
    info_.hardware_parameters.count("isaac_finger_position_commands_topic")
    ? info_.hardware_parameters.at("isaac_finger_position_commands_topic")
    : "/isaac_finger_position_commands";
  arm_command_mode_ =
    info_.hardware_parameters.count("arm_command_mode")
    ? info_.hardware_parameters.at("arm_command_mode")
    : "position_velocity";
  if (arm_command_mode_ != "position_velocity" && arm_command_mode_ != "effort") {
    RCLCPP_ERROR(
      rclcpp::get_logger("IsaacSimHardwareInterface"),
      "Unsupported arm_command_mode '%s'.", arm_command_mode_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  gripper_closed_topic_ =
    info_.hardware_parameters.count("gripper_closed_topic")
    ? info_.hardware_parameters.at("gripper_closed_topic")
    : "/tomato_harvest/gripper_closed";

  const std::size_t n_joints = info_.joints.size();
  position_state_.resize(n_joints, 0.0);
  velocity_state_.resize(n_joints, 0.0);
  position_command_.resize(n_joints, std::numeric_limits<double>::quiet_NaN());
  velocity_command_.resize(n_joints, 0.0);
  effort_command_.resize(n_joints, 0.0);

  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    const auto & joint = info_.joints[index];
    const std::size_t expected_command_interfaces = index < kArmJointCount ? 3 : 2;
    if (joint.command_interfaces.size() != expected_command_interfaces) {
      RCLCPP_ERROR(rclcpp::get_logger("IsaacSimHardwareInterface"),
        "Joint '%s' must have %zu command interfaces.",
        joint.name.c_str(), expected_command_interfaces);
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
  arm_effort_cmd_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    isaac_arm_effort_commands_topic_,
    rclcpp::SystemDefaultsQoS());
  finger_position_cmd_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    isaac_finger_position_commands_topic_,
    rclcpp::SystemDefaultsQoS());

  gripper_closed_sub_ = node_->create_subscription<std_msgs::msg::String>(
    gripper_closed_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      on_gripper_command(msg);
    });

  RCLCPP_INFO(
    rclcpp::get_logger("IsaacSimHardwareInterface"),
    "Configured: mode='%s', subscribing to '%s' and '%s', publishing commands",
    arm_command_mode_.c_str(),
    isaac_joint_states_topic_.c_str(),
    gripper_closed_topic_.c_str());

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
    apply_gripper_command_to_fingers();
  }
  velocity_command_.assign(n_joints, 0.0);
  effort_command_.assign(n_joints, 0.0);

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
    if (i < kArmJointCount) {
      command_interfaces.emplace_back(
        info_.joints[i].name,
        hardware_interface::HW_IF_EFFORT,
        &effort_command_[i]);
    }
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
  static constexpr std::array<double, 2> FINGER_LOWER = {0.0, 0.0};
  static constexpr std::array<double, 2> FINGER_UPPER = {0.04, 0.04};

  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = time;
  msg.name.reserve(info_.joints.size());
  msg.position.reserve(info_.joints.size());
  msg.velocity.reserve(info_.joints.size());

  std::lock_guard<std::mutex> lock(state_mutex_);
  apply_gripper_command_to_fingers();

  if (arm_command_mode_ == "effort") {
    auto arm_msg = sensor_msgs::msg::JointState();
    arm_msg.header.stamp = time;
    arm_msg.name.reserve(kArmJointCount);
    arm_msg.effort.reserve(kArmJointCount);
    for (std::size_t i = 0; i < kArmJointCount; ++i) {
      const double requested = std::isfinite(effort_command_[i]) ? effort_command_[i] : 0.0;
      arm_msg.name.push_back(info_.joints[i].name);
      arm_msg.effort.push_back(std::clamp(requested, -kMaxEffort[i], kMaxEffort[i]));
    }

    auto finger_msg = sensor_msgs::msg::JointState();
    finger_msg.header.stamp = time;
    finger_msg.name.reserve(2);
    finger_msg.position.reserve(2);
    for (std::size_t i = kArmJointCount; i < info_.joints.size(); ++i) {
      const auto finger_index = i - kArmJointCount;
      const double requested = std::isfinite(position_command_[i])
        ? position_command_[i] : position_state_[i];
      finger_msg.name.push_back(info_.joints[i].name);
      finger_msg.position.push_back(
        std::clamp(requested, FINGER_LOWER[finger_index], FINGER_UPPER[finger_index]));
    }
    arm_effort_cmd_pub_->publish(arm_msg);
    finger_position_cmd_pub_->publish(finger_msg);
    return hardware_interface::return_type::OK;
  }

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
    } else if (i - LOWER.size() < FINGER_LOWER.size()) {
      const auto finger_index = i - LOWER.size();
      pos = std::clamp(pos, FINGER_LOWER[finger_index], FINGER_UPPER[finger_index]);
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
    apply_gripper_command_to_fingers();
    RCLCPP_INFO(
      rclcpp::get_logger("IsaacSimHardwareInterface"),
      "Initial joint state received from Isaac Sim. Reseeding command positions.");
  }
  state_received_ = true;
}

void IsaacSimHardwareInterface::on_gripper_command(
  const std_msgs::msg::String::SharedPtr msg)
{
  std::string data = msg->data;
  std::transform(data.begin(), data.end(), data.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  if (data != "true" && data != "false") {
    RCLCPP_WARN(
      rclcpp::get_logger("IsaacSimHardwareInterface"),
      "Ignoring unsupported gripper command: '%s'",
      msg->data.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  gripper_closed_command_ = data == "true";
  apply_gripper_command_to_fingers();
}

void IsaacSimHardwareInterface::apply_gripper_command_to_fingers()
{
  if (!gripper_closed_command_.has_value()) {
    return;
  }

  const double finger_target =
    *gripper_closed_command_ ? kGripperClosedPosition : kGripperOpenPosition;

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint_name = info_.joints[i].name;
    if (joint_name == kFingerJoint1 || joint_name == kFingerJoint2) {
      position_command_[i] = finger_target;
      velocity_command_[i] = 0.0;
    }
  }
}

}  // namespace franka_ros2_control

PLUGINLIB_EXPORT_CLASS(
  franka_ros2_control::IsaacSimHardwareInterface,
  hardware_interface::SystemInterface)
