#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "franka_ros2_control/motion_command_executor_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace
{

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory =
  rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

constexpr char kMotionCommandTopic[] = "/tomato_harvest/motion_command";
constexpr char kExecutionStatusTopic[] = "/tomato_harvest/execution_status";
constexpr char kGripperClosedTopic[] = "/tomato_harvest/gripper_closed";
// JTC の action 名は controller 名に依存するため parameter で受け取る。
// 既定値は franka_controllers.yaml の joint_trajectory_controller に対応する。
constexpr char kDefaultJointTrajectoryAction[] =
  "/joint_trajectory_controller/follow_joint_trajectory";

class MotionCommandExecutorNode : public rclcpp::Node
{
public:
  MotionCommandExecutorNode()
  : Node("motion_command_executor_node")
  {
    status_pub_ = create_publisher<std_msgs::msg::String>(kExecutionStatusTopic, 10);
    gripper_pub_ = create_publisher<std_msgs::msg::String>(kGripperClosedTopic, 10);
    const std::string joint_trajectory_action = declare_parameter<std::string>(
      "follow_joint_trajectory_action", kDefaultJointTrajectoryAction);
    action_client_ =
      rclcpp_action::create_client<FollowJointTrajectory>(this, joint_trajectory_action);

    motion_command_sub_ = create_subscription<std_msgs::msg::String>(
      kMotionCommandTopic,
      10,
      std::bind(&MotionCommandExecutorNode::on_motion_command, this, std::placeholders::_1));

    publish_status("idle");
  }

private:
  void on_motion_command(const std_msgs::msg::String::SharedPtr msg)
  {
    franka_ros2_control::ParsedMotionCommand command;
    try {
      command = franka_ros2_control::parse_motion_command_json(msg->data);
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(get_logger(), "Failed to parse motion_command: %s", exc.what());
      return;
    }

    publish_gripper_if_needed(command.gripper_closed);

    if (!command.has_phase_motion_plan) {
      return;
    }
    if (!command.joint_trajectory.has_value()) {
      if (franka_ros2_control::should_abort_on_missing_trajectory(command.command_name)) {
        publish_status("aborted");
      }
      return;
    }

    send_trajectory(*command.joint_trajectory);
  }

  void publish_gripper_if_needed(const std::optional<bool> & gripper_closed)
  {
    if (!gripper_closed.has_value() || gripper_closed_ == gripper_closed) {
      return;
    }

    gripper_closed_ = gripper_closed;
    std_msgs::msg::String message;
    message.data = *gripper_closed ? "true" : "false";
    gripper_pub_->publish(message);
  }

  void send_trajectory(const franka_ros2_control::ParsedTrajectory & trajectory)
  {
    if (goal_handle_) {
      action_client_->async_cancel_goal(goal_handle_);
      goal_handle_.reset();
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(get_logger(), "JTC action server not available");
      publish_status("aborted");
      return;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = trajectory.joint_names;
    for (const auto & point : trajectory.points) {
      trajectory_msgs::msg::JointTrajectoryPoint ros_point;
      ros_point.positions = point.positions_rad;
      ros_point.time_from_start = rclcpp::Duration::from_seconds(
        point.time_from_start_sec);
      goal.trajectory.points.push_back(std::move(ros_point));
    }

    publish_status("running");

    rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;
    options.goal_response_callback =
      [this](std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
        if (!goal_handle) {
          RCLCPP_WARN(get_logger(), "JTC goal rejected");
          publish_status("aborted");
          return;
        }
        goal_handle_ = std::move(goal_handle);
      };
    options.result_callback =
      [this](const GoalHandleFollowJointTrajectory::WrappedResult & result) {
        goal_handle_.reset();
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            publish_status("succeeded");
            return;
          case rclcpp_action::ResultCode::CANCELED:
            return;
          default:
            RCLCPP_WARN(get_logger(), "JTC goal ended with code=%d", static_cast<int>(result.code));
            publish_status("aborted");
            return;
        }
      };

    action_client_->async_send_goal(goal, options);
  }

  void publish_status(const std::string & status)
  {
    std_msgs::msg::String message;
    message.data = status;
    status_pub_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_command_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle_;
  std::optional<bool> gripper_closed_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionCommandExecutorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
