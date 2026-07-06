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

}  // namespace
