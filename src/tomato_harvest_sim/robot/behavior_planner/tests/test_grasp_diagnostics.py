import math
import pytest

from tomato_harvest_sim.msg.contracts import Pose3D
from tomato_harvest_sim.robot.behavior_planner.grasp_diagnostics import calculate_pose_error


def test_calculate_pose_error_reports_axis_and_norm() -> None:
    actual = Pose3D(0.1, 0.3, 0.2, 0.1, -0.2, 0.3)
    target = Pose3D(0.2, 0.1, 0.5, 0.2, 0.1, -0.1)

    error = calculate_pose_error(actual, target)

    assert error.position_xyz_m == pytest.approx((0.1, -0.2, 0.3))
    assert math.isclose(error.position_norm_m, math.sqrt(0.14))
    assert error.orientation_rpy_rad == pytest.approx((0.1, 0.3, -0.4))


def test_calculate_pose_error_wraps_orientation_at_pi() -> None:
    actual = Pose3D(0, 0, 0, 0, 0, math.pi - 0.1)
    target = Pose3D(0, 0, 0, 0, 0, -math.pi + 0.1)

    error = calculate_pose_error(actual, target)

    assert math.isclose(error.orientation_rpy_rad[2], 0.2)
