from __future__ import annotations

import unittest

from tomato_harvest_sim.api.contracts import (
    HarvestMotionPlan,
    JointTrajectory,
    JointTrajectoryPoint,
    MotionCommand,
    PhaseId,
    PhaseMotionPlan,
    Pose3D,
    ScenePhase,
    SceneSnapshot,
    TargetEstimate,
    TomatoStatus,
)
from tomato_harvest_sim.simulator.debug_visualization import build_scene_runtime_debug_state


def _pose(x: float, y: float, z: float) -> Pose3D:
    return Pose3D(x, y, z, 180.0, 0.0, 0.0)


def _snapshot() -> SceneSnapshot:
    # motion_waypoints are now in active_phase_motion_plan
    waypoints = (_pose(0.50, 0.0, 0.62), _pose(0.52, 0.0, 0.60))
    plan = PhaseMotionPlan(
        phase_id=PhaseId.MOVING_TO_PREGRASP,
        phase_goal_pose=_pose(0.52, 0.0, 0.60),
        active_waypoints=waypoints,
        joint_trajectory=None,
    )
    return SceneSnapshot(
        phase=ScenePhase.RUNNING,
        active_camera="fixed_camera",
        tomato_attached=True,
        tomato_status=TomatoStatus.ATTACHED,
        gripper_closed=False,
        robot_home=False,
        cycle_id=1,
        robot_model="Franka Panda",
        robot_base_pose=_pose(0.0, 0.0, 0.0),
        fixed_camera_pose=_pose(0.8, 0.0, 1.35),
        hand_camera_pose=_pose(0.4, 0.0, 0.7),
        branch_pose=_pose(0.6, 0.0, 0.6),
        stem_pose=_pose(0.6, 0.0, 0.55),
        tomato_pose=_pose(0.6, 0.0, 0.5),
        tray_pose=_pose(0.35, -0.35, 0.45),
        robot_tool_pose=_pose(0.25, 0.0, 0.65),
        target_tool_pose=_pose(0.52, 0.0, 0.60),
        grasp_result_reason=None,
        active_phase_motion_plan=plan,
    )


class DebugVisualizationStateTest(unittest.TestCase):
    def test_uses_joint_trajectory_preview_for_planner_and_tracking_paths(self) -> None:
        trajectory = JointTrajectory(
            joint_names=("panda_joint1",),
            points=(
                JointTrajectoryPoint((0.0,), 0.0),
                JointTrajectoryPoint((0.2,), 1.0),
            ),
        )
        plan = HarvestMotionPlan(
            planner_name="moveit2_service_bridge",
            target_pose=_pose(0.6, 0.0, 0.5),
            pregrasp_pose=_pose(0.48, 0.0, 0.61),
            grasp_pose=_pose(0.60, 0.0, 0.55),
            pull_pose=_pose(0.53, 0.0, 0.59),
            place_pose=_pose(0.35, -0.35, 0.57),
            pregrasp_waypoints=(_pose(0.48, 0.0, 0.61),),
            grasp_waypoints=(_pose(0.56, 0.0, 0.60), _pose(0.60, 0.0, 0.55)),
            pull_waypoints=(_pose(0.55, 0.0, 0.60), _pose(0.53, 0.0, 0.59)),
            place_waypoints=(_pose(0.40, -0.20, 0.62), _pose(0.35, -0.35, 0.57)),
            pregrasp_joint_trajectory=trajectory,
        )
        active_command = MotionCommand(
            command_name="move_to_pregrasp",
            planner_name="moveit2_service_bridge",
            target_pose=plan.pregrasp_pose,
            phase_motion_plan=PhaseMotionPlan(
                phase_id=PhaseId.MOVING_TO_PREGRASP,
                phase_goal_pose=plan.pregrasp_pose,
                active_waypoints=plan.pregrasp_waypoints,
                joint_trajectory=trajectory,
            ),
        )
        preview_path = (_pose(0.26, 0.0, 0.65), _pose(0.40, 0.0, 0.63), _pose(0.48, 0.0, 0.61))
        provider_calls: list[JointTrajectory] = []

        state = build_scene_runtime_debug_state(
            snapshot=_snapshot(),
            target_estimate=None,
            plan=plan,
            active_motion_command=active_command,
            trajectory_path_provider=lambda current: provider_calls.append(current) or preview_path,
        )

        self.assertEqual(state.pregrasp_path_points, preview_path)
        self.assertEqual(state.tracking_path_points, preview_path)
        # active_waypoint_pose は active_phase_motion_plan の waypoints[1] から取得
        snapshot = _snapshot()
        plan_waypoints = snapshot.active_phase_motion_plan.active_waypoints
        self.assertEqual(state.active_waypoint_pose, plan_waypoints[1])
        self.assertEqual(provider_calls, [trajectory, trajectory])

    def test_falls_back_to_waypoints_and_builds_perception_ray(self) -> None:
        plan = HarvestMotionPlan(
            planner_name="moveit2_pregrasp_demo",
            target_pose=_pose(0.6, 0.0, 0.5),
            pregrasp_pose=_pose(0.48, 0.0, 0.61),
            grasp_pose=_pose(0.60, 0.0, 0.55),
            pull_pose=_pose(0.53, 0.0, 0.59),
            place_pose=_pose(0.35, -0.35, 0.57),
            pregrasp_waypoints=(_pose(0.48, 0.0, 0.61),),
            grasp_waypoints=(_pose(0.56, 0.0, 0.60), _pose(0.60, 0.0, 0.55)),
            pull_waypoints=(_pose(0.55, 0.0, 0.60), _pose(0.53, 0.0, 0.59)),
            place_waypoints=(_pose(0.40, -0.20, 0.62), _pose(0.35, -0.35, 0.57)),
        )
        estimate = TargetEstimate(
            camera_name="fixed_camera",
            target_world_pose=_pose(0.6, 0.0, 0.5),
            target_camera_pose=_pose(0.1, 0.0, 0.3),
            confidence=0.99,
        )

        state = build_scene_runtime_debug_state(
            snapshot=_snapshot(),
            target_estimate=estimate,
            plan=plan,
            active_motion_command=None,
        )

        self.assertEqual(state.target_estimate_pose, estimate.target_world_pose)
        self.assertEqual(state.perception_ray_points, (_snapshot().fixed_camera_pose, estimate.target_world_pose))
        self.assertEqual(state.grasp_path_points, plan.grasp_waypoints)
        self.assertEqual(state.place_path_points, plan.place_waypoints)
        # tracking_path_points comes from active_phase_motion_plan waypoints
        snapshot = _snapshot()
        self.assertEqual(state.tracking_path_points, snapshot.active_phase_motion_plan.active_waypoints)


if __name__ == "__main__":
    unittest.main()
