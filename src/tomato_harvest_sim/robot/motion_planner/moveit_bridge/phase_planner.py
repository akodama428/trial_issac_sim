from __future__ import annotations

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    HarvestTaskPhase,
    JointStateSnapshot,
    SceneSnapshot,
)
from tomato_harvest_sim.robot.msg.planner import MoveIt2PlanningResult
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.client import (
    Ros2MoveIt2Clients,
)
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.config import (
    MoveItPlannerConfig,
)
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.goal_planner import (
    MoveItGoalPlanner,
)
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.phase_policy import (
    PhasePlanningSpec,
    phase_planning_specs,
)
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.trajectory import (
    clamp_joint_state_to_bounds,
)
from tomato_harvest_sim.robot.motion_planner.phase_suffix_replan import (
    terminal_joint_state_of_phase,
)
from tomato_harvest_sim.robot.motion_planner.ros_python import (
    ensure_ros_python_modules_available,
)


def moveit2_python_available() -> bool:
    return ensure_ros_python_modules_available("rclpy", "moveit_msgs")


class Ros2MoveIt2PlannerBridge(MoveItGoalPlanner):
    """Coordinate one phase plan from current state through MoveIt services."""

    def __init__(
        self,
        *,
        service_name: str | None = None,
        scene_service_name: str | None = None,
        group_name: str | None = None,
        end_effector_link: str | None = None,
        planning_timeout_sec: float | None = None,
        allowed_planning_time_sec: float | None = None,
        position_tolerance_m: float = 0.01,
        orientation_tolerance_rad: float = 0.10,
        config: MoveItPlannerConfig | None = None,
    ) -> None:
        resolved_config = config or MoveItPlannerConfig.from_env(
            service_name=service_name,
            scene_service_name=scene_service_name,
            group_name=group_name,
            end_effector_link=end_effector_link,
            planning_timeout_sec=planning_timeout_sec,
            allowed_planning_time_sec=allowed_planning_time_sec,
            position_tolerance_m=position_tolerance_m,
            orientation_tolerance_rad=orientation_tolerance_rad,
        )
        super().__init__(resolved_config)
        self._clients: Ros2MoveIt2Clients | None = None

    def plan_phase_trajectory(
        self,
        *,
        phase: HarvestTaskPhase,
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        scene_snapshot: SceneSnapshot,
        plan: HarvestMotionPlan,
    ) -> MoveIt2PlanningResult:
        """Plan a phase immediately before execution from the latest state."""
        if not moveit2_python_available():
            return self._fallback_result("moveit2_python_unavailable")
        clients = self._require_clients()
        if clients is None:
            return self._fallback_result("service_client_unavailable")
        if not clients.wait_for_services(
            timeout_sec=self._config.planning_timeout_sec
        ):
            return self._fallback_result("service_unavailable")

        current = clamp_joint_state_to_bounds(joint_state)
        fallback_joint_goal = terminal_joint_state_of_phase(plan, phase)
        for spec in phase_planning_specs(
            plan=plan,
            joint_state=current,
            home_via_threshold_rad=self._config.home_via_threshold_rad,
        ):
            if phase is spec.phase:
                return self._plan_configured_phase(
                    clients=clients,
                    joint_state=current,
                    base_frame_id=base_frame_id,
                    scene_snapshot=scene_snapshot,
                    spec=spec,
                    fallback_joint_goal=fallback_joint_goal,
                )
        return self._fallback_result("unsupported_phase")

    def _plan_configured_phase(
        self,
        *,
        clients: object,
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        scene_snapshot: SceneSnapshot,
        spec: PhasePlanningSpec,
        fallback_joint_goal: JointStateSnapshot | None,
    ) -> MoveIt2PlanningResult:
        attempts = tuple(
            (targets, "service_ok") for targets in spec.target_sequences
        )
        if (
            fallback_joint_goal is not None
            and spec.joint_fallback_success_reason is not None
        ):
            attempts += (
                (
                    (fallback_joint_goal,),
                    spec.joint_fallback_success_reason,
                ),
            )
        final_primary_attempt = len(spec.target_sequences) - 1
        for index, (targets, success_reason) in enumerate(attempts):
            trajectory = self._plan_phase(
                clients=clients,
                joint_state=joint_state,
                base_frame_id=base_frame_id,
                scene_snapshot=scene_snapshot,
                planning_targets=targets,
                attach_tomato=spec.attach_tomato,
                phase_label=spec.phase.value,
                fallback_joint_goal=(
                    fallback_joint_goal
                    if (
                        spec.joint_fallback_success_reason is None
                        and index == final_primary_attempt
                    )
                    else None
                ),
            )
            if trajectory is not None:
                return MoveIt2PlanningResult(
                    success=True,
                    backend_name="moveit2_service_bridge",
                    reason=success_reason,
                    joint_trajectory=trajectory,
                )
        return self._fallback_result(spec.failure_reason)

    @staticmethod
    def _fallback_result(reason: str) -> MoveIt2PlanningResult:
        return MoveIt2PlanningResult(
            success=False,
            backend_name="moveit2_service_bridge_fallback",
            reason=reason,
        )

    def _require_clients(self) -> Ros2MoveIt2Clients | None:
        if self._clients is not None:
            return self._clients
        try:
            self._clients = Ros2MoveIt2Clients(
                motion_plan_service_name=self._config.service_name,
                planning_scene_service_name=self._config.scene_service_name,
                state_validity_service_name=(
                    self._config.state_validity_service_name
                ),
                ik_service_name=self._config.ik_service_name,
            )
        except Exception:
            self._clients = None
        return self._clients
