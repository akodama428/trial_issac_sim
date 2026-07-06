from __future__ import annotations

from pathlib import Path

import yaml

from tomato_harvest_sim.msg.contracts import (
    AbortPolicy,
    PhaseExecutionIntent,
    PhaseId,
    PoseSemantics,
    SuccessJudge,
    SuccessPolicy,
    TomatoStatus,
)


def _default_config_path() -> Path:
    return Path(__file__).resolve().parent / "config" / "phase_execution.yaml"


class PhaseExecutionIntentBuilder:
    def __init__(self, *, config_path: Path | None = None) -> None:
        self._config_path = config_path or _default_config_path()
        self._phase_policies = self._load_phase_policies()

    def build(self, phase_id: PhaseId) -> PhaseExecutionIntent:
        policy = self._phase_policies[phase_id]
        return PhaseExecutionIntent(
            phase_id=phase_id,
            phase_goal_pose=None,
            pose_semantics=policy["pose_semantics"],
            success=policy["success"],
            abort=policy["abort"],
        )

    def _load_phase_policies(self) -> dict[PhaseId, dict[str, object]]:
        payload = yaml.safe_load(self._config_path.read_text(encoding="utf-8")) or {}
        phase_payload = payload.get("phases", {})
        if not isinstance(phase_payload, dict):
            raise ValueError(f"Invalid phase policy payload: {self._config_path}")

        policies: dict[PhaseId, dict[str, object]] = {}
        for phase_id in PhaseId:
            raw_policy = phase_payload.get(phase_id.value)
            if not isinstance(raw_policy, dict):
                raise ValueError(f"Missing policy for phase {phase_id.value}: {self._config_path}")
            policies[phase_id] = self._parse_phase_policy(raw_policy)
        return policies

    def _parse_phase_policy(self, raw_policy: dict[str, object]) -> dict[str, object]:
        success_raw = raw_policy.get("success", {})
        abort_raw = raw_policy.get("abort", {})
        if not isinstance(success_raw, dict) or not isinstance(abort_raw, dict):
            raise ValueError(f"Invalid policy entry in {self._config_path}")

        required_tomato_status = success_raw.get("required_tomato_status")
        return {
            "pose_semantics": PoseSemantics(str(raw_policy.get("pose_semantics", PoseSemantics.TOOL_CENTER.value))),
            "success": SuccessPolicy(
                judge=SuccessJudge(str(success_raw.get("judge", SuccessJudge.END_EFFECTOR_POSE.value))),
                position_tolerance_m=_optional_float(success_raw.get("position_tolerance_m")),
                stable_steps=max(1, int(success_raw.get("stable_steps", 1))),
                required_tomato_status=(
                    TomatoStatus(str(required_tomato_status)) if required_tomato_status is not None else None
                ),
            ),
            "abort": AbortPolicy(
                nominal_timeout_sec=_optional_float(abort_raw.get("nominal_timeout_sec")),
                stall_timeout_sec=_optional_float(abort_raw.get("stall_timeout_sec")),
                min_progress_delta_m=_optional_float(abort_raw.get("min_progress_delta_m")),
                joint_path_tolerance_rad=_optional_float(abort_raw.get("joint_path_tolerance_rad")),
                allow_replan=bool(abort_raw.get("allow_replan", True)),
            ),
        }


def _optional_float(value: object) -> float | None:
    if value is None:
        return None
    return float(value)
