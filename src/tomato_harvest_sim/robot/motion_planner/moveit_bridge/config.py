from __future__ import annotations

import os
from dataclasses import dataclass


def _env_enabled(name: str, default: str) -> bool:
    return os.environ.get(name, default).strip() not in {"", "0", "false", "False"}


@dataclass(frozen=True)
class MoveItPlannerConfig:
    """MoveIt bridge configuration resolved at the composition boundary."""

    service_name: str
    scene_service_name: str
    state_validity_service_name: str
    ik_service_name: str
    group_name: str
    end_effector_link: str
    planning_timeout_sec: float
    allowed_planning_time_sec: float
    position_tolerance_m: float
    orientation_tolerance_rad: float
    goal_joint_window_rad: float
    seeded_ik_goal_enabled: bool
    home_via_threshold_rad: float
    enforce_orientation_constraint: bool
    debug_enabled: bool

    moveit_link_to_runtime_tool_offset_m: tuple[float, float, float] = (
        0.0,
        0.0,
        0.0584,
    )
    tray_inner_size_m: tuple[float, float, float] = (0.22, 0.16, 0.05)
    tray_wall_thickness_m: float = 0.012
    tray_collision_margin_m: float = 0.015
    branch_size_m: tuple[float, float, float] = (0.18, 0.02, 0.02)
    stem_size_m: tuple[float, float, float] = (0.008, 0.008, 0.06)
    attached_tomato_radius_m: float = 0.01
    attached_tomato_offset_m: tuple[float, float, float] = (0.0, 0.0, 0.1034)
    noop_trajectory_tolerance_rad: float = 1e-3
    joint_goal_tolerance_rad: float = 0.01

    @classmethod
    def from_env(
        cls,
        *,
        service_name: str | None = None,
        scene_service_name: str | None = None,
        group_name: str | None = None,
        end_effector_link: str | None = None,
        planning_timeout_sec: float | None = None,
        allowed_planning_time_sec: float | None = None,
        position_tolerance_m: float = 0.01,
        orientation_tolerance_rad: float = 0.10,
    ) -> "MoveItPlannerConfig":
        return cls(
            service_name=service_name
            or os.environ.get(
                "TOMATO_HARVEST_MOVEIT_SERVICE", "/plan_kinematic_path"
            ),
            scene_service_name=scene_service_name
            or os.environ.get(
                "TOMATO_HARVEST_MOVEIT_SCENE_SERVICE",
                "/apply_planning_scene",
            ),
            state_validity_service_name=os.environ.get(
                "TOMATO_HARVEST_MOVEIT_STATE_VALIDITY_SERVICE",
                "/check_state_validity",
            ),
            ik_service_name=os.environ.get(
                "TOMATO_HARVEST_MOVEIT_IK_SERVICE", "/compute_ik"
            ),
            group_name=group_name
            or os.environ.get("TOMATO_HARVEST_MOVEIT_GROUP", "panda_arm"),
            end_effector_link=end_effector_link
            or os.environ.get("TOMATO_HARVEST_MOVEIT_EE_LINK", "panda_hand"),
            planning_timeout_sec=planning_timeout_sec
            or float(
                os.environ.get(
                    "TOMATO_HARVEST_MOVEIT_SERVICE_TIMEOUT_SEC", "1.50"
                )
            ),
            allowed_planning_time_sec=allowed_planning_time_sec
            or float(
                os.environ.get(
                    "TOMATO_HARVEST_MOVEIT_ALLOWED_PLANNING_TIME_SEC", "1.00"
                )
            ),
            position_tolerance_m=float(
                os.environ.get(
                    "TOMATO_HARVEST_MOVEIT_POSITION_TOLERANCE_M",
                    position_tolerance_m,
                )
            ),
            orientation_tolerance_rad=float(
                os.environ.get(
                    "TOMATO_HARVEST_MOVEIT_ORIENTATION_TOLERANCE_RAD",
                    orientation_tolerance_rad,
                )
            ),
            goal_joint_window_rad=float(
                os.environ.get(
                    "TOMATO_HARVEST_MOVEIT_GOAL_JOINT1_WINDOW_RAD", "2.2"
                )
            ),
            seeded_ik_goal_enabled=_env_enabled(
                "TOMATO_HARVEST_MOVEIT_SEEDED_IK_GOAL", "1"
            ),
            home_via_threshold_rad=float(
                os.environ.get("TOMATO_HARVEST_HOME_VIA_THRESHOLD_RAD", "1.2")
            ),
            enforce_orientation_constraint=_env_enabled(
                "TOMATO_HARVEST_MOVEIT_ENFORCE_ORIENTATION", "1"
            ),
            debug_enabled=_env_enabled("TOMATO_HARVEST_DEBUG_MOVEIT", ""),
        )
