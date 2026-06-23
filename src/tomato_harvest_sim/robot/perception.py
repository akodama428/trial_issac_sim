from __future__ import annotations

from tomato_harvest_sim.api.contracts import CameraFrame, TargetEstimate, TfTreeSnapshot
from tomato_harvest_sim.robot.geometry import world_point_to_local


class TomatoTargetEstimator:
    def estimate(self, camera_frame: CameraFrame, tf_tree: TfTreeSnapshot) -> TargetEstimate:
        del tf_tree
        target_camera_pose = world_point_to_local(camera_frame.target_world_pose, camera_frame.camera_pose)
        return TargetEstimate(
            camera_name=camera_frame.camera_name,
            target_world_pose=camera_frame.target_world_pose,
            target_camera_pose=target_camera_pose,
            confidence=0.99,
        )
