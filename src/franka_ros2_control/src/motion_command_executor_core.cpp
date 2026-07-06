#include "franka_ros2_control/motion_command_executor_core.hpp"

#include <stdexcept>
#include <utility>

#include "yaml-cpp/yaml.h"

namespace franka_ros2_control
{

namespace
{

std::string require_string(const YAML::Node & node, const char * key)
{
  const YAML::Node value = node[key];
  if (!value || value.IsNull() || !value.IsScalar()) {
    throw std::runtime_error(std::string("missing or invalid string field: ") + key);
  }
  return value.as<std::string>();
}

std::optional<bool> read_optional_bool(const YAML::Node & node, const char * key)
{
  const YAML::Node value = node[key];
  if (!value || value.IsNull()) {
    return std::nullopt;
  }
  if (!value.IsScalar()) {
    throw std::runtime_error(std::string("invalid bool field: ") + key);
  }
  return value.as<bool>();
}

ParsedTrajectory parse_trajectory(const YAML::Node & trajectory_node)
{
  if (!trajectory_node.IsMap()) {
    throw std::runtime_error("joint_trajectory must be a map");
  }

  ParsedTrajectory trajectory;

  const YAML::Node joint_names = trajectory_node["joint_names"];
  if (!joint_names || joint_names.IsNull() || !joint_names.IsSequence()) {
    throw std::runtime_error("joint_trajectory.joint_names must be a sequence");
  }
  for (const YAML::Node & joint_name : joint_names) {
    trajectory.joint_names.push_back(joint_name.as<std::string>());
  }

  const YAML::Node points = trajectory_node["points"];
  if (!points || points.IsNull() || !points.IsSequence()) {
    throw std::runtime_error("joint_trajectory.points must be a sequence");
  }
  for (const YAML::Node & point_node : points) {
    if (!point_node.IsMap()) {
      throw std::runtime_error("joint_trajectory.points[*] must be a map");
    }

    ParsedTrajectoryPoint point;
    const YAML::Node positions = point_node["positions_rad"];
    if (!positions || positions.IsNull() || !positions.IsSequence()) {
      throw std::runtime_error("joint_trajectory.points[*].positions_rad must be a sequence");
    }
    for (const YAML::Node & position : positions) {
      point.positions_rad.push_back(position.as<double>());
    }

    const YAML::Node time_from_start = point_node["time_from_start_sec"];
    if (!time_from_start || time_from_start.IsNull() || !time_from_start.IsScalar()) {
      throw std::runtime_error(
              "joint_trajectory.points[*].time_from_start_sec must be a scalar");
    }
    point.time_from_start_sec = time_from_start.as<double>();
    trajectory.points.push_back(std::move(point));
  }

  return trajectory;
}

}  // namespace

ParsedMotionCommand parse_motion_command_json(const std::string & json)
{
  const YAML::Node root = YAML::Load(json);
  if (!root.IsMap()) {
    throw std::runtime_error("motion_command root must be a map");
  }

  ParsedMotionCommand command;
  command.command_name = require_string(root, "command_name");
  command.gripper_closed = read_optional_bool(root, "gripper_closed");

  const YAML::Node phase_motion_plan = root["phase_motion_plan"];
  if (!phase_motion_plan || phase_motion_plan.IsNull()) {
    return command;
  }
  if (!phase_motion_plan.IsMap()) {
    throw std::runtime_error("phase_motion_plan must be a map");
  }

  command.has_phase_motion_plan = true;

  const YAML::Node joint_trajectory = phase_motion_plan["joint_trajectory"];
  if (!joint_trajectory || joint_trajectory.IsNull()) {
    return command;
  }

  command.joint_trajectory = parse_trajectory(joint_trajectory);
  return command;
}

bool should_abort_on_missing_trajectory(const std::string & command_name)
{
  return command_name.rfind("hold_", 0) != 0;
}

}  // namespace franka_ros2_control
