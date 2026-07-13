#include "franka_ros2_control/motion_command_executor_core.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace
{

TEST(MotionCommandExecutorCoreTest, ParsesTrajectoryAndGripperState)
{
  const auto command = franka_ros2_control::parse_motion_command_json(R"json(
    {
      "command_name": "move_to_pregrasp",
      "planner_name": "moveit2",
      "gripper_closed": true,
      "phase_motion_plan": {
        "phase_id": "moving_to_pregrasp",
        "joint_trajectory": {
          "joint_names": ["panda_joint1", "panda_joint2"],
          "points": [
            {
              "positions_rad": [0.0, -0.4],
              "time_from_start_sec": 0.0
            },
            {
              "positions_rad": [0.1, -0.3],
              "time_from_start_sec": 1.2
            }
          ]
        }
      }
    }
  )json");

  ASSERT_EQ(command.command_name, "move_to_pregrasp");
  ASSERT_TRUE(command.phase_id.has_value());
  EXPECT_EQ(*command.phase_id, "moving_to_pregrasp");
  ASSERT_TRUE(command.gripper_closed.has_value());
  EXPECT_TRUE(*command.gripper_closed);
  EXPECT_TRUE(command.has_phase_motion_plan);
  ASSERT_TRUE(command.joint_trajectory.has_value());
  ASSERT_EQ(command.joint_trajectory->joint_names.size(), 2U);
  EXPECT_EQ(command.joint_trajectory->joint_names[0], "panda_joint1");
  ASSERT_EQ(command.joint_trajectory->points.size(), 2U);
  EXPECT_DOUBLE_EQ(command.joint_trajectory->points[1].positions_rad[1], -0.3);
  EXPECT_DOUBLE_EQ(command.joint_trajectory->points[1].time_from_start_sec, 1.2);
}

TEST(MotionCommandExecutorCoreTest, NullPhaseMotionPlanDoesNotRequireTrajectory)
{
  const auto command = franka_ros2_control::parse_motion_command_json(R"json(
    {
      "command_name": "move_home",
      "planner_name": "direct",
      "gripper_closed": false,
      "phase_motion_plan": null
    }
  )json");

  EXPECT_FALSE(command.has_phase_motion_plan);
  EXPECT_FALSE(command.phase_id.has_value());
  EXPECT_FALSE(command.joint_trajectory.has_value());
  ASSERT_TRUE(command.gripper_closed.has_value());
  EXPECT_FALSE(*command.gripper_closed);
}

TEST(MotionCommandExecutorCoreTest, HoldCommandsDoNotAbortWithoutTrajectory)
{
  EXPECT_FALSE(franka_ros2_control::should_abort_on_missing_trajectory("hold_at_grasp"));
  EXPECT_FALSE(franka_ros2_control::should_abort_on_missing_trajectory("hold_placed"));
}

TEST(MotionCommandExecutorCoreTest, MoveAndPullCommandsAbortWithoutTrajectory)
{
  EXPECT_TRUE(franka_ros2_control::should_abort_on_missing_trajectory("move_to_place"));
  EXPECT_TRUE(franka_ros2_control::should_abort_on_missing_trajectory("pull_to_detach"));
}

TEST(MotionCommandExecutorCoreTest, InvalidPayloadThrows)
{
  EXPECT_THROW(
    franka_ros2_control::parse_motion_command_json(R"json({"planner_name":"moveit2"})json"),
    std::runtime_error);
}

// ---------------------------------------------------------------------------
// Issue #32 abort診断: 追従誤差のピーク追跡・abort分類・status JSON
// ---------------------------------------------------------------------------

TEST(TrackingErrorPeakTest, KeepsMaximumErrorAndLimitingJoint)
{
  franka_ros2_control::TrackingErrorPeak peak;
  peak = franka_ros2_control::update_tracking_error_peak(
    peak, {"panda_joint1", "panda_joint2"}, {0.02, -0.05});
  peak = franka_ros2_control::update_tracking_error_peak(
    peak, {"panda_joint1", "panda_joint2"}, {-0.11, 0.04});
  peak = franka_ros2_control::update_tracking_error_peak(
    peak, {"panda_joint1", "panda_joint2"}, {0.03, 0.01});

  ASSERT_TRUE(peak.has_value);
  EXPECT_DOUBLE_EQ(peak.max_error_rad, 0.11);
  EXPECT_EQ(peak.limiting_joint, "panda_joint1");
}

TEST(TrackingErrorPeakTest, MismatchedLengthsAreIgnored)
{
  franka_ros2_control::TrackingErrorPeak peak;
  peak = franka_ros2_control::update_tracking_error_peak(
    peak, {"panda_joint1", "panda_joint2"}, {0.02});

  EXPECT_FALSE(peak.has_value);
}

// Issue #37: 固着調査のため、ピーク時点の律速jointの目標値・実位置も記録する。
// 関節限界近傍での固着かどうかを実位置から直接判定できるようにする。
TEST(TrackingErrorPeakTest, RecordsDesiredAndActualAtPeak)
{
  franka_ros2_control::TrackingErrorPeak peak;
  peak = franka_ros2_control::update_tracking_error_peak(
    peak, {"panda_joint1", "panda_joint3"}, {0.02, -0.05},
    {0.10, 0.00}, {0.08, 0.05});
  peak = franka_ros2_control::update_tracking_error_peak(
    peak, {"panda_joint1", "panda_joint3"}, {0.03, -2.40},
    {0.20, 0.00}, {0.17, 2.40});

  ASSERT_TRUE(peak.has_value);
  EXPECT_DOUBLE_EQ(peak.max_error_rad, 2.40);
  EXPECT_EQ(peak.limiting_joint, "panda_joint3");
  ASSERT_TRUE(peak.has_positions);
  EXPECT_DOUBLE_EQ(peak.limiting_joint_desired_rad, 0.00);
  EXPECT_DOUBLE_EQ(peak.limiting_joint_actual_rad, 2.40);
}

TEST(TrackingErrorPeakTest, MissingPositionsKeepErrorTrackingOnly)
{
  franka_ros2_control::TrackingErrorPeak peak;
  peak = franka_ros2_control::update_tracking_error_peak(
    peak, {"panda_joint1"}, {0.15});

  ASSERT_TRUE(peak.has_value);
  EXPECT_FALSE(peak.has_positions);
}

TEST(TrackingErrorPeakTest, CompletesMissingAbortPositionsFromControllerState)
{
  auto peak = franka_ros2_control::update_tracking_error_peak(
    {}, {"panda_joint1", "panda_joint3"}, {0.02, -0.40});

  peak = franka_ros2_control::complete_tracking_error_diagnostics(
    peak,
    {"panda_joint1", "panda_joint3"},
    {0.10, 2.40},
    {0.08, 2.80});

  ASSERT_TRUE(peak.has_positions);
  EXPECT_EQ(peak.limiting_joint, "panda_joint3");
  EXPECT_DOUBLE_EQ(peak.limiting_joint_desired_rad, 2.40);
  EXPECT_DOUBLE_EQ(peak.limiting_joint_actual_rad, 2.80);
}

TEST(TrackingErrorPeakTest, BuildsAbortPeakFromControllerStateWithoutActionFeedback)
{
  const auto peak = franka_ros2_control::complete_tracking_error_diagnostics(
    {},
    {"panda_joint1", "panda_joint3"},
    {0.10, 2.40},
    {0.08, 2.80});

  ASSERT_TRUE(peak.has_value);
  ASSERT_TRUE(peak.has_positions);
  EXPECT_DOUBLE_EQ(peak.max_error_rad, 0.40);
  EXPECT_EQ(peak.limiting_joint, "panda_joint3");
}

TEST(AbortReasonTest, MapsJtcErrorCodesToStableNames)
{
  EXPECT_EQ(franka_ros2_control::abort_reason_from_jtc(-4, ""), "path_tolerance_violated");
  EXPECT_EQ(franka_ros2_control::abort_reason_from_jtc(-5, ""), "goal_tolerance_violated");
  EXPECT_EQ(franka_ros2_control::abort_reason_from_jtc(-1, ""), "invalid_goal");
  EXPECT_EQ(
    franka_ros2_control::abort_reason_from_jtc(-99, "custom failure"),
    "jtc_error_-99");
}

TEST(ExecutionStatusJsonTest, AbortPayloadCarriesDiagnostics)
{
  franka_ros2_control::TrackingErrorPeak peak;
  peak = franka_ros2_control::update_tracking_error_peak(
    peak, {"panda_joint4"}, {0.184});

  const std::string payload = franka_ros2_control::execution_status_json(
    "aborted", peak, "goal_tolerance_violated");

  EXPECT_NE(payload.find("\"status\":\"aborted\""), std::string::npos);
  EXPECT_NE(payload.find("\"max_joint_error_rad\":0.184"), std::string::npos);
  EXPECT_NE(payload.find("\"limiting_joint\":\"panda_joint4\""), std::string::npos);
  EXPECT_NE(payload.find("\"abort_reason\":\"goal_tolerance_violated\""), std::string::npos);
}

TEST(ExecutionStatusJsonTest, AbortPayloadCarriesPeakPositionsWhenAvailable)
{
  franka_ros2_control::TrackingErrorPeak peak;
  peak = franka_ros2_control::update_tracking_error_peak(
    peak, {"panda_joint3"}, {-2.4}, {0.0}, {2.4});

  const std::string payload = franka_ros2_control::execution_status_json(
    "aborted", peak, "goal_tolerance_violated");

  EXPECT_NE(payload.find("\"limiting_joint_desired_rad\":0"), std::string::npos);
  EXPECT_NE(payload.find("\"limiting_joint_actual_rad\":2.4"), std::string::npos);
}

TEST(ExecutionStatusJsonTest, NonAbortPayloadHasOnlyStatus)
{
  const std::string payload = franka_ros2_control::execution_status_json(
    "running", franka_ros2_control::TrackingErrorPeak{}, std::nullopt);

  EXPECT_EQ(payload, "{\"status\":\"running\"}");
}

TEST(ExecutionStatusJsonTest, RunningPayloadCarriesInstantaneousTrackingError)
{
  franka_ros2_control::TrackingErrorPeak peak;
  peak = franka_ros2_control::update_tracking_error_peak(
    peak, {"panda_joint2"}, {-0.125}, {-0.50}, {-0.375});

  const std::string payload = franka_ros2_control::execution_status_json(
    "running", peak, std::nullopt);

  EXPECT_NE(payload.find("\"status\":\"running\""), std::string::npos);
  EXPECT_NE(payload.find("\"tracking_error_rad\":0.125"), std::string::npos);
  EXPECT_NE(payload.find("\"limiting_joint\":\"panda_joint2\""), std::string::npos);
  EXPECT_NE(payload.find("\"limiting_joint_desired_rad\":-0.5"), std::string::npos);
  EXPECT_NE(payload.find("\"limiting_joint_actual_rad\":-0.375"), std::string::npos);
}

TEST(TrackingErrorPublicationTest, BuildsOneInstantaneousSamplePerFeedback)
{
  const auto first = franka_ros2_control::tracking_error_sample(
    {"panda_joint1", "panda_joint2"}, {0.11, -0.04});
  const auto second = franka_ros2_control::tracking_error_sample(
    {"panda_joint1", "panda_joint2"}, {0.02, -0.03});

  ASSERT_TRUE(first.has_value);
  EXPECT_DOUBLE_EQ(first.max_error_rad, 0.11);
  EXPECT_EQ(first.limiting_joint, "panda_joint1");
  ASSERT_TRUE(second.has_value);
  EXPECT_DOUBLE_EQ(second.max_error_rad, 0.03);
}

TEST(ExecutionStatusJsonTest, AbortWithoutFeedbackOmitsErrorFields)
{
  const std::string payload = franka_ros2_control::execution_status_json(
    "aborted", franka_ros2_control::TrackingErrorPeak{}, "missing_trajectory");

  EXPECT_NE(payload.find("\"status\":\"aborted\""), std::string::npos);
  EXPECT_NE(payload.find("\"abort_reason\":\"missing_trajectory\""), std::string::npos);
  EXPECT_EQ(payload.find("max_joint_error_rad"), std::string::npos);
  EXPECT_EQ(payload.find("limiting_joint"), std::string::npos);
}

}  // namespace
