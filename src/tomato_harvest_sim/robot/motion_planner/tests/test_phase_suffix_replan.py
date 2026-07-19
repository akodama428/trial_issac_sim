from __future__ import annotations

import unittest
from dataclasses import replace

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan, HarvestTaskPhase, JointStateSnapshot, JointTrajectory,
    JointTrajectoryPoint, Pose3D, ScenePhase, SceneSnapshot,
    TomatoStatus,
)
from tomato_harvest_sim.robot.msg.planner import MoveIt2PlanningResult
from tomato_harvest_sim.robot.motion_planner.moveit_service_bridge import (
    MoveIt2ServiceBridgePlanner,
)
from tomato_harvest_sim.robot.motion_planner.phase_suffix_replan import (
    PHASE_ENTRY_PLANNING_PHASES,
    PHASE_TRAJECTORY_FIELD_BY_PHASE,
    SUFFIX_REPLAN_PHASES,
    PhasePlanRetryMemory,
    PhasePlanningGate,
    evaluate_phase_plan_update,
    memory_after_phase_plan_attempt,
    phase_trajectory,
    should_retry_missing_phase_plan,
    should_plan_phase_on_entry,
    terminal_joint_state_of_phase,
)

_SUFFIX_FIELD_BY_PHASE = {
    HarvestTaskPhase.MOVING_TO_PREGRASP: "pregrasp_joint_trajectory",
    HarvestTaskPhase.MOVING_TO_GRASP: "grasp_joint_trajectory",
    HarvestTaskPhase.DETACHING: "pull_joint_trajectory",
    HarvestTaskPhase.MOVING_TO_PLACE: "place_joint_trajectory",
    HarvestTaskPhase.RETURNING_HOME: "home_joint_trajectory",
}


def _trajectory(endpoint: float) -> JointTrajectory:
    return JointTrajectory(
        joint_names=("joint1", "joint2"),
        points=(JointTrajectoryPoint((0.0, 0.0), 0.0),
                JointTrajectoryPoint((endpoint, endpoint), 1.0)),
    )


def _plan(*, phase: HarvestTaskPhase, endpoint: float, revision: int = 1) -> HarvestMotionPlan:
    pose = Pose3D(0, 0, 0, 0, 0, 0)
    return HarvestMotionPlan(
        planner_name="test", target_pose=pose, pregrasp_pose=pose,
        grasp_pose=pose, pull_pose=pose, place_pose=pose,
        plan_revision=revision,
        **{_SUFFIX_FIELD_BY_PHASE[phase]: _trajectory(endpoint)},
    )


class SuffixReplanPhaseSetTest(unittest.TestCase):
    def test_every_trajectory_motion_phase_is_planned_on_entry(self) -> None:
        self.assertEqual(
            PHASE_ENTRY_PLANNING_PHASES,
            frozenset(_SUFFIX_FIELD_BY_PHASE),
        )

    def test_free_space_motion_phases_are_suffix_replan_targets(self) -> None:
        self.assertEqual(SUFFIX_REPLAN_PHASES, frozenset({
            HarvestTaskPhase.MOVING_TO_PREGRASP,
            HarvestTaskPhase.MOVING_TO_GRASP,
            HarvestTaskPhase.MOVING_TO_PLACE,
            HarvestTaskPhase.RETURNING_HOME,
        }))

    def test_returning_home_is_a_suffix_replan_target(self) -> None:
        """home復帰も自由空間移動であり、abort復旧をfull replanに依存しない (Issue #32)。"""
        self.assertIn(HarvestTaskPhase.RETURNING_HOME, SUFFIX_REPLAN_PHASES)

    def test_contact_dominant_detaching_is_excluded_from_suffix_replan(self) -> None:
        self.assertNotIn(HarvestTaskPhase.DETACHING, SUFFIX_REPLAN_PHASES)


class SuffixTrajectorySelectionTest(unittest.TestCase):
    def test_each_phase_selects_its_own_remaining_trajectory(self) -> None:
        for phase, field in _SUFFIX_FIELD_BY_PHASE.items():
            with self.subTest(phase=phase):
                plan = _plan(phase=phase, endpoint=1.0)
                self.assertEqual(phase_trajectory(plan, phase), getattr(plan, field))
                self.assertEqual(PHASE_TRAJECTORY_FIELD_BY_PHASE[phase], field)

    def test_unsupported_phase_has_no_phase_trajectory(self) -> None:
        plan = _plan(phase=HarvestTaskPhase.MOVING_TO_PLACE, endpoint=1.0)
        self.assertIsNone(phase_trajectory(plan, HarvestTaskPhase.AT_GRASP))


class SuffixUpdateTest(unittest.TestCase):
    def test_small_endpoint_difference_keeps_current_plan_in_each_phase(self) -> None:
        for phase in SUFFIX_REPLAN_PHASES:
            with self.subTest(phase=phase):
                decision = evaluate_phase_plan_update(
                    phase=phase,
                    current_plan=_plan(phase=phase, endpoint=1.0),
                    candidate_plan=_plan(phase=phase, endpoint=1.005, revision=2),
                    minimum_endpoint_delta_rad=0.02,
                )
                self.assertFalse(decision.adopted)
                self.assertEqual(decision.reason, "rejected_small_trajectory_delta")

    def test_significant_endpoint_difference_adopts_suffix_in_each_phase(self) -> None:
        for phase in SUFFIX_REPLAN_PHASES:
            with self.subTest(phase=phase):
                decision = evaluate_phase_plan_update(
                    phase=phase,
                    current_plan=_plan(phase=phase, endpoint=1.0),
                    candidate_plan=_plan(phase=phase, endpoint=1.05, revision=2),
                    minimum_endpoint_delta_rad=0.02,
                )
                self.assertTrue(decision.adopted)
                self.assertEqual(decision.reason, "adopted_significant_trajectory_delta")

    def test_missing_candidate_phase_trajectory_is_rejected(self) -> None:
        phase = HarvestTaskPhase.MOVING_TO_PREGRASP
        candidate = replace(
            _plan(phase=phase, endpoint=1.2), pregrasp_joint_trajectory=None
        )
        decision = evaluate_phase_plan_update(
            phase=phase,
            current_plan=_plan(phase=phase, endpoint=1.0),
            candidate_plan=candidate,
        )
        self.assertFalse(decision.adopted)
        self.assertEqual(decision.reason, "rejected_missing_phase_trajectory")

    def test_unsupported_phase_is_rejected(self) -> None:
        place_plan = _plan(phase=HarvestTaskPhase.MOVING_TO_PLACE, endpoint=1.0)
        decision = evaluate_phase_plan_update(
            phase=HarvestTaskPhase.AT_GRASP,
            current_plan=place_plan,
            candidate_plan=replace(place_plan, plan_revision=2),
        )
        self.assertFalse(decision.adopted)
        self.assertEqual(decision.reason, "rejected_unsupported_phase")


class PhaseEntryPlanningTest(unittest.TestCase):
    def test_entering_each_motion_phase_triggers_trajectory_planning(self) -> None:
        for phase in PHASE_ENTRY_PLANNING_PHASES:
            with self.subTest(phase=phase):
                self.assertTrue(should_plan_phase_on_entry(None, phase))

    def test_staying_in_same_motion_phase_does_not_plan_again(self) -> None:
        for phase in PHASE_ENTRY_PLANNING_PHASES:
            with self.subTest(phase=phase):
                self.assertFalse(should_plan_phase_on_entry(phase, phase))

    def test_non_motion_phase_does_not_trigger_trajectory_planning(self) -> None:
        self.assertFalse(should_plan_phase_on_entry(
            HarvestTaskPhase.MOVING_TO_GRASP,
            HarvestTaskPhase.AT_GRASP,
        ))


class PhasePlanRetryTest(unittest.TestCase):
    def test_missing_phase_trajectory_is_retried_after_interval(self) -> None:
        phase = HarvestTaskPhase.MOVING_TO_GRASP
        plan = replace(
            _plan(phase=phase, endpoint=1.0),
            grasp_joint_trajectory=None,
            planned_from_phase=HarvestTaskPhase.MOVING_TO_PREGRASP,
        )
        memory = memory_after_phase_plan_attempt(
            phase=phase,
            now_sec=10.0,
            retry_interval_sec=1.0,
        )

        self.assertFalse(should_retry_missing_phase_plan(
            phase=phase,
            plan=plan,
            memory=memory,
            now_sec=10.9,
        ))
        self.assertTrue(should_retry_missing_phase_plan(
            phase=phase,
            plan=plan,
            memory=memory,
            now_sec=11.0,
        ))

    def test_adopted_phase_trajectory_stops_retry(self) -> None:
        phase = HarvestTaskPhase.MOVING_TO_GRASP
        plan = replace(_plan(phase=phase, endpoint=1.0), planned_from_phase=phase)

        self.assertFalse(should_retry_missing_phase_plan(
            phase=phase,
            plan=plan,
            memory=PhasePlanRetryMemory(),
            now_sec=20.0,
        ))

    def test_phase_change_does_not_wait_for_previous_phase_interval(self) -> None:
        previous = HarvestTaskPhase.MOVING_TO_PREGRASP
        current = HarvestTaskPhase.MOVING_TO_GRASP
        plan = replace(
            _plan(phase=current, endpoint=1.0),
            grasp_joint_trajectory=None,
            planned_from_phase=previous,
        )
        memory = memory_after_phase_plan_attempt(
            phase=previous,
            now_sec=10.0,
            retry_interval_sec=30.0,
        )

        self.assertTrue(should_retry_missing_phase_plan(
            phase=current,
            plan=plan,
            memory=memory,
            now_sec=10.1,
        ))


class TerminalJointStateTest(unittest.TestCase):
    """abort後の関節空間goal fallbackが使う既知の有効goal構成の抽出 (Issue #28 改善2)。"""

    def test_terminal_configuration_of_adopted_trajectory_is_extracted(self) -> None:
        for phase in SUFFIX_REPLAN_PHASES:
            with self.subTest(phase=phase):
                plan = _plan(phase=phase, endpoint=1.0)

                terminal = terminal_joint_state_of_phase(plan, phase)

                self.assertIsNotNone(terminal)
                assert terminal is not None
                self.assertEqual(terminal.joint_names, ("joint1", "joint2"))
                self.assertEqual(terminal.positions_rad, (1.0, 1.0))

    def test_unsupported_phase_has_no_terminal_configuration(self) -> None:
        plan = _plan(phase=HarvestTaskPhase.MOVING_TO_PLACE, endpoint=1.0)
        self.assertIsNone(
            terminal_joint_state_of_phase(plan, HarvestTaskPhase.DETACHING)
        )

    def test_missing_phase_trajectory_has_no_terminal_configuration(self) -> None:
        plan = replace(
            _plan(phase=HarvestTaskPhase.MOVING_TO_GRASP, endpoint=1.0),
            grasp_joint_trajectory=None,
        )
        self.assertIsNone(
            terminal_joint_state_of_phase(plan, HarvestTaskPhase.MOVING_TO_GRASP)
        )


class PhasePlanningGateTest(unittest.TestCase):
    def test_second_planner_start_is_suppressed_while_in_flight(self) -> None:
        gate = PhasePlanningGate()
        self.assertTrue(gate.try_begin())
        self.assertFalse(gate.try_begin())
        gate.finish()
        self.assertTrue(gate.try_begin())


def _scene() -> SceneSnapshot:
    pose = Pose3D(0, 0, 0, 0, 0, 0)
    return SceneSnapshot(
        phase=ScenePhase.RUNNING, active_camera="fixed", tomato_attached=True,
        tomato_status=TomatoStatus.HELD, gripper_closed=True, robot_home=False,
        cycle_id=1, robot_model="panda", robot_base_pose=pose,
        fixed_camera_pose=pose, hand_camera_pose=pose, branch_pose=pose,
        stem_pose=pose, tomato_pose=pose, tray_pose=pose, robot_tool_pose=pose,
        target_tool_pose=None, grasp_result_reason=None,
    )


_BASE_FRAME_ID = "panda_link0"


class _SuffixFakeBridge:
    """phase-aware suffix要求を記録し、決められた1区間だけ返すfake。"""

    def __init__(self, suffix: JointTrajectory) -> None:
        self._suffix = suffix
        self.received_phase: HarvestTaskPhase | None = None
        self.received_joint_state: JointStateSnapshot | None = None

    def plan_phase_trajectory(self, **kwargs: object) -> MoveIt2PlanningResult:
        phase = kwargs["phase"]
        assert isinstance(phase, HarvestTaskPhase)
        self.received_phase = phase
        self.received_joint_state = kwargs["joint_state"]  # type: ignore[assignment]
        return MoveIt2PlanningResult(
            success=True, backend_name="suffix_bridge", reason="service_ok",
            joint_trajectory=self._suffix,
        )


class _FlakySuffixFakeBridge(_SuffixFakeBridge):
    def __init__(self, suffix: JointTrajectory) -> None:
        super().__init__(suffix)
        self.calls = 0

    def plan_phase_trajectory(self, **kwargs: object) -> MoveIt2PlanningResult:
        self.calls += 1
        if self.calls < 3:
            return MoveIt2PlanningResult(
                success=False, backend_name="suffix_bridge", reason="temporary_failure"
            )
        return super().plan_phase_trajectory(**kwargs)


class PhaseSuffixIntegrationTest(unittest.TestCase):
    def test_temporary_suffix_failure_is_retried(self) -> None:
        phase = HarvestTaskPhase.MOVING_TO_PLACE
        suffix = _trajectory(1.0)
        bridge = _FlakySuffixFakeBridge(suffix)
        planner = MoveIt2ServiceBridgePlanner(bridge=bridge)  # type: ignore[arg-type]

        candidate = planner.plan_phase_trajectory(
            phase,
            _plan(phase=phase, endpoint=1.0),
            JointStateSnapshot(("joint1", "joint2"), (0.25, 0.25)),
            _BASE_FRAME_ID,
            _scene(),
        )

        self.assertIsNotNone(candidate)
        self.assertEqual(bridge.calls, 3)

    def test_deviation_replans_only_current_phase_suffix_from_latest_joints(self) -> None:
        for phase, field in _SUFFIX_FIELD_BY_PHASE.items():
            with self.subTest(phase=phase):
                current_joints = JointStateSnapshot(("joint1", "joint2"), (0.25, 0.25))
                suffix = JointTrajectory(
                    ("joint1", "joint2"),
                    (JointTrajectoryPoint(current_joints.positions_rad, 0.0),
                     JointTrajectoryPoint((1.0, 1.0), 1.0)),
                )
                bridge = _SuffixFakeBridge(suffix)
                planner = MoveIt2ServiceBridgePlanner(bridge=bridge)  # type: ignore[arg-type]
                prior = _plan(phase=phase, endpoint=1.0)
                untouched_fields = [
                    other for other in _SUFFIX_FIELD_BY_PHASE.values() if other != field
                ]

                candidate = planner.plan_phase_trajectory(
                    phase, prior, current_joints, _BASE_FRAME_ID, _scene()
                )

                self.assertIsNotNone(candidate)
                self.assertEqual(bridge.received_phase, phase)
                self.assertEqual(bridge.received_joint_state, current_joints)
                self.assertEqual(getattr(candidate, field), suffix)
                for other in untouched_fields:
                    self.assertEqual(getattr(candidate, other), getattr(prior, other))
                decision = evaluate_phase_plan_update(
                    phase=phase, current_plan=prior, candidate_plan=candidate,  # type: ignore[arg-type]
                )
                self.assertTrue(decision.adopted)

    def test_unsupported_phase_returns_none_without_calling_bridge(self) -> None:
        bridge = _SuffixFakeBridge(_trajectory(1.0))
        planner = MoveIt2ServiceBridgePlanner(bridge=bridge)  # type: ignore[arg-type]
        prior = _plan(phase=HarvestTaskPhase.MOVING_TO_PLACE, endpoint=1.0)

        candidate = planner.plan_phase_trajectory(
            HarvestTaskPhase.AT_GRASP, prior,
            JointStateSnapshot(("joint1", "joint2"), (0.25, 0.25)),
            _BASE_FRAME_ID, _scene(),
        )

        self.assertIsNone(candidate)
        self.assertIsNone(bridge.received_phase)
