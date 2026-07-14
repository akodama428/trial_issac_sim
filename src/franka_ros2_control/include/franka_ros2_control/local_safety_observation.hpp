#pragma once

#include <Eigen/Core>

#include <optional>
#include <string>

namespace franka_ros2_control
{

struct LocalSafetyObservation
{
  double collision_clearance_m;
  double singularity_measure;
};

std::optional<double> normalized_singularity_margin(
  const Eigen::MatrixXd & jacobian, double slowdown_condition = 17.0,
  double hard_stop_condition = 30.0);

std::string local_safety_observation_json(const LocalSafetyObservation & observation);

}  // namespace franka_ros2_control
