#pragma once

#include <optional>
#include <string>
#include <vector>

namespace franka_ros2_control
{

struct ParsedTrajectoryPoint
{
  std::vector<double> positions_rad;
  double time_from_start_sec{0.0};
};

struct ParsedTrajectory
{
  std::vector<std::string> joint_names;
  std::vector<ParsedTrajectoryPoint> points;
};

struct ParsedMotionCommand
{
  std::string command_name;
  std::optional<bool> gripper_closed;
  bool has_phase_motion_plan{false};
  std::optional<ParsedTrajectory> joint_trajectory;
};

ParsedMotionCommand parse_motion_command_json(const std::string & json);

bool should_abort_on_missing_trajectory(const std::string & command_name);

}  // namespace franka_ros2_control
