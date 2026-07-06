from __future__ import annotations

from typing import Protocol

from tomato_harvest_sim.msg.contracts import CameraFrame, TargetEstimate, TfTreeSnapshot


class TargetEstimator(Protocol):
    def estimate(self, camera_frame: CameraFrame, tf_tree: TfTreeSnapshot) -> TargetEstimate: ...
