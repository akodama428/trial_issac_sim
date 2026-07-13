#include "franka_ros2_control/local_safety_observation.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>

namespace franka_ros2_control
{

TEST(LocalSafetyObservation, WellConditionedJacobianHasFullMargin)
{
  const Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, 6);
  const auto margin = normalized_singularity_margin(jacobian);
  ASSERT_TRUE(margin.has_value());
  EXPECT_DOUBLE_EQ(*margin, 1.0);
}

TEST(LocalSafetyObservation, ServoHardStopConditionHasZeroMargin)
{
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, 6);
  jacobian(5, 5) = 1.0 / 30.0;
  const auto margin = normalized_singularity_margin(jacobian);
  ASSERT_TRUE(margin.has_value());
  EXPECT_NEAR(*margin, 0.0, 1e-12);
}

TEST(LocalSafetyObservation, ServoSlowdownConditionHasFullMargin)
{
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, 6);
  jacobian(5, 5) = 1.0 / 17.0;
  const auto margin = normalized_singularity_margin(jacobian);
  ASSERT_TRUE(margin.has_value());
  EXPECT_NEAR(*margin, 1.0, 1e-12);
}

TEST(LocalSafetyObservation, InvalidJacobianIsRejected)
{
  EXPECT_FALSE(normalized_singularity_margin(Eigen::MatrixXd()).has_value());
}

TEST(LocalSafetyObservation, JsonUsesExistingLocalPlannerContract)
{
  EXPECT_EQ(
    local_safety_observation_json({0.043, 0.75}),
    "{\"collision_clearance_m\":0.043,\"singularity_measure\":0.75}");
}

}  // namespace franka_ros2_control
