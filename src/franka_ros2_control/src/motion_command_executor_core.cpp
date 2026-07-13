#include "franka_ros2_control/motion_command_executor_core.hpp"

#include <cmath>
#include <sstream>
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
  command.phase_id = require_string(phase_motion_plan, "phase_id");

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

TrackingErrorPeak update_tracking_error_peak(
  TrackingErrorPeak peak,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & error_positions_rad,
  const std::vector<double> & desired_positions_rad,
  const std::vector<double> & actual_positions_rad)
{
  if (joint_names.size() != error_positions_rad.size()) {
    return peak;
  }
  const bool positions_available =
    desired_positions_rad.size() == joint_names.size() &&
    actual_positions_rad.size() == joint_names.size();
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    const double error = std::abs(error_positions_rad[i]);
    if (!peak.has_value || error > peak.max_error_rad) {
      peak.max_error_rad = error;
      peak.limiting_joint = joint_names[i];
      peak.has_value = true;
      peak.has_positions = positions_available;
      if (positions_available) {
        peak.limiting_joint_desired_rad = desired_positions_rad[i];
        peak.limiting_joint_actual_rad = actual_positions_rad[i];
      }
    }
  }
  return peak;
}

bool should_publish_tracking_error(
  const TrackingErrorPeak & recent_peak,
  const double last_publish_at_sec,
  const double now_sec,
  const double publish_interval_sec)
{
  return recent_peak.has_value && publish_interval_sec > 0.0 &&
         now_sec >= last_publish_at_sec &&
         now_sec - last_publish_at_sec >= publish_interval_sec;
}

std::string abort_reason_from_jtc(int error_code, const std::string & error_string)
{
  // control_msgs/FollowJointTrajectory Result のerror_code (0=SUCCESSFUL)。
  switch (error_code) {
    case -1: return "invalid_goal";
    case -2: return "invalid_joints";
    case -3: return "old_header_timestamp";
    case -4: return "path_tolerance_violated";
    case -5: return "goal_tolerance_violated";
    default:
      (void)error_string;
      return "jtc_error_" + std::to_string(error_code);
  }
}

namespace
{

std::string json_escape(const std::string & value)
{
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    if (character == '"' || character == '\\') {
      escaped.push_back('\\');
    }
    if (static_cast<unsigned char>(character) >= 0x20) {
      escaped.push_back(character);
    }
  }
  return escaped;
}

}  // namespace

std::string execution_status_json(
  const std::string & status,
  const TrackingErrorPeak & peak,
  const std::optional<std::string> & abort_reason)
{
  std::ostringstream stream;
  stream << "{\"status\":\"" << json_escape(status) << "\"";
  if (peak.has_value) {
    if (status == "running") {
      stream << ",\"tracking_error_rad\":" << peak.max_error_rad;
    }
    stream << ",\"max_joint_error_rad\":" << peak.max_error_rad
           << ",\"limiting_joint\":\"" << json_escape(peak.limiting_joint) << "\"";
    if (peak.has_positions) {
      stream << ",\"limiting_joint_desired_rad\":" << peak.limiting_joint_desired_rad
             << ",\"limiting_joint_actual_rad\":" << peak.limiting_joint_actual_rad;
    }
  }
  if (abort_reason.has_value()) {
    stream << ",\"abort_reason\":\"" << json_escape(*abort_reason) << "\"";
  }
  stream << "}";
  return stream.str();
}

}  // namespace franka_ros2_control
