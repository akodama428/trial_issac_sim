from tomato_harvest_sim.msg.contracts import Pose3D
from tomato_harvest_sim.robot.execute_manager.pose_tracking_observability import (
    pose_tracking_metric_fields,
    tf_lookup_failure_metric_fields,
)


def test_pose_tracking_sample_keeps_command_tf_error_and_servo_status_together() -> None:
    fields = pose_tracking_metric_fields(
        sequence_id=7,
        published_count=7,
        planning_frame="panda_link0",
        end_effector_frame="panda_link8",
        target=Pose3D(0.4, 0.1, 0.55, 180.0, 0.0, 90.0),
        current=Pose3D(0.39, 0.1, 0.54, 179.0, 0.0, 90.0),
        position_error_m=0.014142,
        orientation_error_rad=0.017453,
        reached=False,
        stable_samples=0,
        servo_status=3,
        tf_success_count=6,
        tf_failure_count=1,
    )

    assert fields["sequence_id"] == 7
    assert fields["published_count"] == 7
    assert fields["target_xyz_m"] == [0.4, 0.1, 0.55]
    assert fields["current_xyz_m"] == [0.39, 0.1, 0.54]
    assert fields["position_error_m"] == 0.014142
    assert fields["servo_status"] == 3
    assert fields["tf_success_count"] == 6
    assert fields["tf_failure_count"] == 1


def test_tf_failure_sample_exposes_exception_and_last_success_age() -> None:
    fields = tf_lookup_failure_metric_fields(
        sequence_id=8,
        published_count=8,
        planning_frame="panda_link0",
        end_effector_frame="panda_link8",
        error="frame does not exist",
        servo_status=None,
        tf_success_count=0,
        tf_failure_count=8,
        last_success_age_sec=None,
    )

    assert fields["sequence_id"] == 8
    assert fields["tf_lookup_succeeded"] is False
    assert fields["tf_error"] == "frame does not exist"
    assert fields["last_tf_success_age_sec"] is None
