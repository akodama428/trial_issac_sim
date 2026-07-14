#include "franka_ros2_control/local_safety_observation.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace franka_ros2_control
{

std::optional<double> normalized_singularity_margin(
  const Eigen::MatrixXd & jacobian, const double slowdown_condition,
  const double hard_stop_condition)
{
  if (jacobian.rows() == 0 || jacobian.cols() == 0 || !jacobian.allFinite() ||
    slowdown_condition <= 1.0 || hard_stop_condition <= slowdown_condition)
  {
    return std::nullopt;
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto singular_values = svd.singularValues();
  if (singular_values.size() == 0 || singular_values(0) <= 0.0) {
    return std::nullopt;
  }
  const double smallest = singular_values(singular_values.size() - 1);
  const double condition = smallest <= 1e-12 ? INFINITY : singular_values(0) / smallest;
  const double inverse_condition = std::isfinite(condition) ? 1.0 / condition : 0.0;
  const double hard_inverse = 1.0 / hard_stop_condition;
  const double slow_inverse = 1.0 / slowdown_condition;
  return std::clamp((inverse_condition - hard_inverse) / (slow_inverse - hard_inverse), 0.0, 1.0);
}

std::string local_safety_observation_json(const LocalSafetyObservation & observation)
{
  std::ostringstream output;
  output << std::setprecision(10) << "{\"collision_clearance_m\":"
         << std::max(0.0, observation.collision_clearance_m)
         << ",\"singularity_measure\":"
         << std::clamp(observation.singularity_measure, 0.0, 1.0) << "}";
  return output.str();
}

}  // namespace franka_ros2_control
