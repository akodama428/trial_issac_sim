from __future__ import annotations

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    Pose3D,
    SceneSnapshot,
    TargetEstimate,
)
from tomato_harvest_sim.simulator.scene_config import (
    ReleasePoseConfig,
    load_placement_config,
)


class HarvestPoseWaypointPlanner:
    """収穫対象とtrayのposeから、各phaseの目標pose・waypointを生成する。"""

    def __init__(
        self,
        *,
        approach_offset_m: float = 0.12,
        vertical_offset_m: float = 0.09,
        grasp_hover_offset_z_m: float = 0.11,
        grasp_entry_offset_z_m: float = 0.085,
        # Step 3-7実測: 0.060ではfinger pad中点がtomato中心より約12 mm上になる。
        grasp_target_offset_z_m: float = 0.048,
        pull_offset_x_m: float = 0.08,
        pull_offset_z_m: float = 0.08,
        pull_lift_offset_x_m: float = 0.02,
        pull_lift_offset_z_m: float = 0.06,
        place_vertical_segment_count: int = 20,
        release_pose_config: ReleasePoseConfig | None = None,
    ) -> None:
        if place_vertical_segment_count < 1:
            raise ValueError("place_vertical_segment_count must be at least 1")
        self._approach_offset_m = approach_offset_m
        self._vertical_offset_m = vertical_offset_m
        self._grasp_hover_offset_z_m = grasp_hover_offset_z_m
        self._grasp_entry_offset_z_m = grasp_entry_offset_z_m
        self._grasp_target_offset_z_m = grasp_target_offset_z_m
        self._pull_offset_x_m = pull_offset_x_m
        self._pull_offset_z_m = pull_offset_z_m
        self._pull_lift_offset_x_m = pull_lift_offset_x_m
        self._pull_lift_offset_z_m = pull_lift_offset_z_m
        self._place_vertical_segment_count = place_vertical_segment_count
        placement = release_pose_config or load_placement_config().release_pose
        self._place_vertical_offset_m = placement.vertical_offset_m
        self._place_hover_offset_m = placement.hover_offset_m

    def plan(
        self,
        target_estimate: TargetEstimate,
        scene_snapshot: SceneSnapshot,
    ) -> HarvestMotionPlan:
        target_pose = target_estimate.target_world_pose
        tray_pose = scene_snapshot.tray_pose
        pregrasp_pose = Pose3D(
            round(target_pose.x - self._approach_offset_m, 6),
            round(target_pose.y, 6),
            round(target_pose.z + self._vertical_offset_m, 6),
            180.0,
            0.0,
            0.0,
        )
        grasp_hover_pose = Pose3D(
            round(target_pose.x, 6),
            round(target_pose.y, 6),
            round(target_pose.z + self._grasp_hover_offset_z_m, 6),
            180.0,
            0.0,
            0.0,
        )
        grasp_entry_pose = Pose3D(
            round(target_pose.x, 6),
            round(target_pose.y, 6),
            round(target_pose.z + self._grasp_entry_offset_z_m, 6),
            180.0,
            0.0,
            0.0,
        )
        grasp_pose = Pose3D(
            round(target_pose.x, 6),
            round(target_pose.y, 6),
            round(target_pose.z + self._grasp_target_offset_z_m, 6),
            180.0,
            0.0,
            0.0,
        )
        pull_lift_pose = Pose3D(
            round(target_pose.x - self._pull_lift_offset_x_m, 6),
            round(target_pose.y, 6),
            round(target_pose.z + self._pull_lift_offset_z_m, 6),
            180.0,
            0.0,
            0.0,
        )
        pull_pose = Pose3D(
            round(target_pose.x - self._pull_offset_x_m, 6),
            round(target_pose.y, 6),
            round(target_pose.z + self._pull_offset_z_m, 6),
            180.0,
            0.0,
            0.0,
        )
        place_pose = Pose3D(
            round(tray_pose.x, 6),
            round(tray_pose.y, 6),
            round(tray_pose.z + self._place_vertical_offset_m, 6),
            180.0,
            0.0,
            0.0,
        )
        # 1本の長いjoint-space計画では、終端poseが鉛直上方でも途中の手先が
        # trayリブ側へ膨らむ場合がある。25 mm区間の物理E2Eでは手先が最大
        # 約13 mm下降して接触が残ったため、既定の100 mm昇降を5 mm区間へ
        # 分割し、MOVING_TO_PLACEとRETURNING_HOMEの往復で同じ点列を共有する。
        place_waypoints = tuple(
            Pose3D(
                place_pose.x,
                place_pose.y,
                round(
                    place_pose.z
                    + self._place_hover_offset_m
                    * segment
                    / self._place_vertical_segment_count,
                    6,
                ),
                place_pose.roll,
                place_pose.pitch,
                place_pose.yaw,
            )
            for segment in range(
                self._place_vertical_segment_count,
                -1,
                -1,
            )
        )
        return HarvestMotionPlan(
            planner_name="harvest_pose_waypoint_planner",
            target_pose=target_pose,
            pregrasp_pose=pregrasp_pose,
            grasp_pose=grasp_pose,
            pull_pose=pull_pose,
            place_pose=place_pose,
            pregrasp_waypoints=(pregrasp_pose,),
            grasp_waypoints=(grasp_hover_pose, grasp_entry_pose, grasp_pose),
            pull_waypoints=(pull_lift_pose, pull_pose),
            place_waypoints=place_waypoints,
        )
