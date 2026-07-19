from __future__ import annotations

from dataclasses import dataclass

from tomato_harvest_sim.msg.contracts import (
    JointStateSnapshot,
    JointTrajectory,
)
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.trajectory import (
    joint_trajectory_from_msg,
)
from tomato_harvest_sim.robot.motion_planner.planning_diagnostics import (
    StateValidityReport,
)


@dataclass(frozen=True)
class MotionPlanOutcome:
    """Result and diagnostic reason from one GetMotionPlan call."""

    trajectory: JointTrajectory | None
    error_code: int | None
    failure_reason: str | None


class Ros2MoveIt2Clients:
    """Lazy ROS2 service boundary used by the planning application logic."""

    def __init__(
        self,
        *,
        motion_plan_service_name: str,
        planning_scene_service_name: str,
        state_validity_service_name: str = "/check_state_validity",
        ik_service_name: str = "/compute_ik",
    ) -> None:
        import rclpy
        from moveit_msgs.srv import (
            ApplyPlanningScene,
            GetMotionPlan,
            GetPlanningScene,
            GetPositionIK,
            GetStateValidity,
        )
        from rclpy.executors import SingleThreadedExecutor

        self._rclpy = rclpy
        if not self._rclpy.ok():
            self._rclpy.init(args=None)
        self._node = self._rclpy.create_node("tomato_harvest_moveit_bridge")
        self._motion_plan_client = self._node.create_client(
            GetMotionPlan, motion_plan_service_name
        )
        self._planning_scene_client = self._node.create_client(
            ApplyPlanningScene, planning_scene_service_name
        )
        self._get_planning_scene_client = self._node.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self._state_validity_client = self._node.create_client(
            GetStateValidity, state_validity_service_name
        )
        self._ik_client = self._node.create_client(
            GetPositionIK, ik_service_name
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

    def _spin_until_done(self, future: object, timeout_sec: float) -> None:
        self._executor.spin_until_future_complete(
            future, timeout_sec=timeout_sec
        )

    def wait_for_services(self, *, timeout_sec: float) -> bool:
        if not self._motion_plan_client.wait_for_service(
            timeout_sec=timeout_sec
        ):
            return False
        if not self._planning_scene_client.wait_for_service(
            timeout_sec=timeout_sec
        ):
            return False
        return bool(
            self._get_planning_scene_client.wait_for_service(
                timeout_sec=timeout_sec
            )
        )

    def get_allowed_collision_matrix(
        self, *, timeout_sec: float
    ) -> object | None:
        """現在のPlanningSceneから既存ACMを取得する。

        Args:
            timeout_sec: service応答待ちの上限秒。

        Returns:
            現在のAllowedCollisionMatrix。timeoutまたは空応答時はNone。
        """
        from moveit_msgs.msg import PlanningSceneComponents
        from moveit_msgs.srv import GetPlanningScene

        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        future = self._get_planning_scene_client.call_async(request)
        self._spin_until_done(future, timeout_sec)
        response = future.result() if future.done() else None
        if response is None:
            return None
        return response.scene.allowed_collision_matrix

    def apply_planning_scene(
        self, request: object, *, timeout_sec: float
    ) -> bool:
        future = self._planning_scene_client.call_async(request)
        self._spin_until_done(future, timeout_sec)
        response = future.result() if future.done() else None
        return bool(response is not None and getattr(response, "success", False))

    def plan_motion(
        self, request: object, *, timeout_sec: float
    ) -> MotionPlanOutcome:
        future = self._motion_plan_client.call_async(request)
        self._spin_until_done(future, timeout_sec)
        if not future.done():
            return MotionPlanOutcome(None, None, "service_timeout")
        response = future.result()
        if response is None:
            return MotionPlanOutcome(None, None, "empty_response")
        error_code = int(response.motion_plan_response.error_code.val)
        if error_code != 1:
            print(
                "[MoveItBridge] motion plan service returned "
                f"error_code={error_code}.",
                flush=True,
            )
            return MotionPlanOutcome(None, error_code, "motion_plan_error")
        robot_trajectory = response.motion_plan_response.trajectory
        message = getattr(robot_trajectory, "joint_trajectory", None)
        if message is None:
            return MotionPlanOutcome(None, error_code, "empty_trajectory")
        trajectory = joint_trajectory_from_msg(message)
        if trajectory is None:
            return MotionPlanOutcome(None, error_code, "empty_trajectory")
        has_velocities = any(
            point.velocities_rad_s is not None for point in trajectory.points
        )
        first_velocity = (
            trajectory.points[0].velocities_rad_s
            if trajectory.points
            else None
        )
        last_velocity = (
            trajectory.points[-1].velocities_rad_s
            if trajectory.points
            else None
        )
        print(
            "[MoveItBridge] motion plan response "
            f"points={len(trajectory.points)} "
            f"has_velocities={has_velocities} "
            f"first_vel={first_velocity} last_vel={last_velocity} "
            f"joint_names={trajectory.joint_names}",
            flush=True,
        )
        return MotionPlanOutcome(trajectory, error_code, None)

    def compute_nearest_ik(
        self,
        *,
        seed_joint_state: JointStateSnapshot,
        base_frame_id: str,
        target_pose_xyz: tuple[float, float, float],
        target_orientation_xyzw: tuple[float, float, float, float],
        group_name: str,
        timeout_sec: float,
    ) -> JointStateSnapshot | None:
        from moveit_msgs.srv import GetPositionIK
        from sensor_msgs.msg import JointState

        if not self._ik_client.wait_for_service(timeout_sec=timeout_sec):
            return None
        request = GetPositionIK.Request()
        request.ik_request.group_name = group_name
        request.ik_request.avoid_collisions = False
        request.ik_request.robot_state.joint_state = JointState()
        request.ik_request.robot_state.joint_state.name = list(
            seed_joint_state.joint_names
        )
        request.ik_request.robot_state.joint_state.position = [
            float(value) for value in seed_joint_state.positions_rad
        ]
        pose = request.ik_request.pose_stamped
        pose.header.frame_id = base_frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            target_pose_xyz
        )
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = target_orientation_xyzw
        future = self._ik_client.call_async(request)
        self._spin_until_done(future, timeout_sec)
        if not future.done():
            return None
        response = future.result()
        if response is None or int(response.error_code.val) != 1:
            return None
        solution = response.solution.joint_state
        return JointStateSnapshot(
            joint_names=tuple(str(name) for name in solution.name),
            positions_rad=tuple(float(value) for value in solution.position),
        )

    def check_state_validity(
        self,
        *,
        joint_state: JointStateSnapshot,
        group_name: str,
        timeout_sec: float,
    ) -> StateValidityReport:
        from moveit_msgs.srv import GetStateValidity
        from sensor_msgs.msg import JointState

        if not self._state_validity_client.wait_for_service(
            timeout_sec=timeout_sec
        ):
            return StateValidityReport(checked=False)
        request = GetStateValidity.Request()
        request.group_name = group_name
        request.robot_state.joint_state = JointState()
        request.robot_state.joint_state.name = list(joint_state.joint_names)
        request.robot_state.joint_state.position = [
            float(position) for position in joint_state.positions_rad
        ]
        future = self._state_validity_client.call_async(request)
        self._spin_until_done(future, timeout_sec)
        if not future.done() or future.result() is None:
            return StateValidityReport(checked=False)
        response = future.result()
        contacts = tuple(
            f"{contact.contact_body_1}|{contact.contact_body_2}"
            for contact in getattr(response, "contacts", ())
        )
        return StateValidityReport(
            checked=True,
            valid=bool(response.valid),
            contacts=contacts,
        )
