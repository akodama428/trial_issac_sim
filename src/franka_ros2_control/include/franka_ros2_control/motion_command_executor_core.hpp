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
  std::optional<std::string> phase_id;
  std::optional<bool> gripper_closed;
  bool has_phase_motion_plan{false};
  std::optional<ParsedTrajectory> joint_trajectory;
};

ParsedMotionCommand parse_motion_command_json(const std::string & json);

bool should_abort_on_missing_trajectory(const std::string & command_name);

// ---------------------------------------------------------------------------
// Issue #32 abort診断: JTC feedbackの追従誤差ピークとabort分類をstatusへ載せる
// ---------------------------------------------------------------------------

struct TrackingErrorPeak
{
  double max_error_rad{0.0};
  std::string limiting_joint;
  bool has_value{false};
  // Issue #37: ピーク時点の律速jointの目標値・実位置。関節限界近傍での
  // 固着かどうかを実位置から直接判定するために記録する。
  double limiting_joint_desired_rad{0.0};
  double limiting_joint_actual_rad{0.0};
  bool has_positions{false};
};

// JTC feedback 1回分の関節誤差でピークを更新する。配列長不一致のsampleは無視する。
// desired/actual は省略可能で、誤差配列と同じ長さのときだけピークへ記録される。
TrackingErrorPeak update_tracking_error_peak(
  TrackingErrorPeak peak,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & error_positions_rad,
  const std::vector<double> & desired_positions_rad = {},
  const std::vector<double> & actual_positions_rad = {});

// JTC feedback 1回分を、window状態を持たない瞬時tracking error sampleへ変換する。
TrackingErrorPeak tracking_error_sample(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & error_positions_rad,
  const std::vector<double> & desired_positions_rad = {},
  const std::vector<double> & actual_positions_rad = {});

// action feedbackで位置が欠けたabort診断を、JTC controller_stateの
// reference/feedbackで補完する。peak自体が無い場合はcontroller stateから生成する。
TrackingErrorPeak complete_tracking_error_diagnostics(
  TrackingErrorPeak peak,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & desired_positions_rad,
  const std::vector<double> & actual_positions_rad);

// FollowJointTrajectory result の error_code を安定した分類名へ変換する。
std::string abort_reason_from_jtc(int error_code, const std::string & error_string);

// execution_status topic のJSON payloadを組み立てる。診断が無いstatusは
// {"status": "..."} のみになる。
std::string execution_status_json(
  const std::string & status,
  const TrackingErrorPeak & peak,
  const std::optional<std::string> & abort_reason);

}  // namespace franka_ros2_control
