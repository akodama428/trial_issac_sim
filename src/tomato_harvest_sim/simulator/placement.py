"""Tomato配置の幾何判定と静定状態機械。ROS/PhysXへ依存しない。"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum

from tomato_harvest_sim.msg.contracts import Pose3D
from tomato_harvest_sim.simulator.scene_config import PlacementConfig, SettlingConfig


class PlacementDecision(StrEnum):
    PENDING = "pending"
    PLACED = "placed"
    FAILED = "failed"


@dataclass(frozen=True)
class PlacementObservation:
    tomato_pose: Pose3D
    linear_speed_m_s: float
    angular_speed_rad_s: float
    tomato_tray_contact: bool
    dt_sec: float


@dataclass(frozen=True)
class ContainmentResult:
    contained: bool
    escaped: bool
    local_x_m: float
    local_y_m: float
    local_z_m: float
    margin_x_m: float
    margin_y_m: float


@dataclass(frozen=True)
class PlacementResult:
    decision: PlacementDecision
    reason: str
    settle_steps: int
    elapsed_sec: float
    containment: ContainmentResult | None = None


class PlacementGeometry:
    def __init__(self, *, tray_pose: Pose3D, config: PlacementConfig) -> None:
        self._tray_pose = tray_pose
        self._geometry = config.scene_geometry
        self._containment = config.containment

    def evaluate(self, tomato_pose: Pose3D) -> ContainmentResult:
        dx = tomato_pose.x - self._tray_pose.x
        dy = tomato_pose.y - self._tray_pose.y
        yaw = math.radians(self._tray_pose.yaw)
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        local_z = tomato_pose.z - self._tray_pose.z
        radius = self._geometry.tomato_radius_m
        half_x = self._geometry.tray_inner_size_m[0] * 0.5 - radius
        half_y = self._geometry.tray_inner_size_m[1] * 0.5 - radius
        margin_x = half_x - abs(local_x)
        margin_y = half_y - abs(local_y)
        contained = (
            margin_x >= self._containment.boundary_margin_m
            and margin_y >= self._containment.boundary_margin_m
            and local_z >= self._geometry.tray_wall_thickness_m * 0.5
        )
        escaped = (
            margin_x < -self._containment.escape_margin_m
            or margin_y < -self._containment.escape_margin_m
        )
        return ContainmentResult(
            contained=contained,
            escaped=escaped,
            local_x_m=local_x,
            local_y_m=local_y,
            local_z_m=local_z,
            margin_x_m=margin_x,
            margin_y_m=margin_y,
        )


class PlacementEvaluator:
    def __init__(self, geometry: PlacementGeometry, config: SettlingConfig) -> None:
        self._geometry = geometry
        self._config = config
        self.reset()

    @property
    def result(self) -> PlacementResult:
        return self._result

    def reset(self) -> None:
        self._active = False
        self._elapsed_sec = 0.0
        self._settle_steps = 0
        self._contact_seen = False
        self._result = PlacementResult(
            PlacementDecision.PENDING, "inactive", 0, 0.0
        )

    def release_started(self) -> None:
        self.reset()
        self._active = True
        self._result = PlacementResult(
            PlacementDecision.PENDING, "release_started", 0, 0.0
        )

    def observe(self, observation: PlacementObservation) -> PlacementResult:
        if not self._active or self._result.decision is not PlacementDecision.PENDING:
            return self._result
        self._elapsed_sec += max(0.0, observation.dt_sec)
        containment = self._geometry.evaluate(observation.tomato_pose)
        if containment.escaped:
            return self._terminal(PlacementDecision.FAILED, "escaped_tray", containment)

        if observation.tomato_tray_contact:
            self._contact_seen = True

        stable = (
            self._contact_seen
            and containment.contained
            and observation.linear_speed_m_s <= self._config.max_linear_speed_m_s
            and observation.angular_speed_rad_s <= self._config.max_angular_speed_rad_s
        )
        self._settle_steps = self._settle_steps + 1 if stable else 0
        if self._settle_steps >= self._config.required_consecutive_steps:
            return self._terminal(PlacementDecision.PLACED, "settled_in_tray", containment)
        if not self._contact_seen and self._elapsed_sec >= self._config.release_timeout_sec:
            return self._terminal(
                PlacementDecision.FAILED, "release_contact_timeout", containment
            )
        if self._contact_seen and self._elapsed_sec >= self._config.settle_timeout_sec:
            return self._terminal(
                PlacementDecision.FAILED, "settling_timeout", containment
            )
        self._result = PlacementResult(
            PlacementDecision.PENDING,
            "settling" if self._contact_seen else "awaiting_tray_contact",
            self._settle_steps,
            self._elapsed_sec,
            containment,
        )
        return self._result

    def _terminal(
        self,
        decision: PlacementDecision,
        reason: str,
        containment: ContainmentResult,
    ) -> PlacementResult:
        self._result = PlacementResult(
            decision, reason, self._settle_steps, self._elapsed_sec, containment
        )
        return self._result
