from __future__ import annotations

from tomato_harvest_sim.api.contracts import Pose3D
from tomato_harvest_sim.simulator.scene_runtime import IsaacSceneRuntime


def main() -> None:
    runtime = IsaacSceneRuntime()
    snapshot = runtime.boot()
    print("[boot]", runtime.describe_scene(), f"phase={snapshot.phase.value}")

    hand_snapshot = runtime.set_active_camera("hand_camera")
    print("[camera]", f"active={hand_snapshot.active_camera}")

    runtime.move_robot_home(False)
    runtime.set_tomato_pose(Pose3D(0.70, 0.10, 0.40, 0.0, 0.0, 0.0))
    runtime.detach_tomato()
    print("[mutated]", runtime.describe_scene())

    reset_snapshot = runtime.reset_scene()
    print("[reset]", runtime.describe_scene(), f"phase={reset_snapshot.phase.value}")


if __name__ == "__main__":
    main()
