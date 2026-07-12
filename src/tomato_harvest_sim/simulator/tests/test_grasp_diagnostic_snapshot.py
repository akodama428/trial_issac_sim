from tomato_harvest_sim.simulator.scene_runtime import IsaacSceneRuntime


def test_snapshot_exposes_target_pose_and_finger_diagnostics() -> None:
    runtime = IsaacSceneRuntime(physics_grasp_enabled=True)
    runtime.boot()

    runtime.sync_grasp_diagnostics(
        left_contact=True,
        right_contact=False,
        left_force_n=1.5,
        right_force_n=0.0,
    )
    snapshot = runtime.snapshot()

    assert snapshot.target_tool_pose is not None
    assert snapshot.target_tool_pose.z == snapshot.tomato_pose.z + runtime.GRASP_TOMATO_OFFSET_Z_M
    assert snapshot.left_finger_contact is True
    assert snapshot.right_finger_contact is False
    assert snapshot.left_finger_force_n == 1.5
