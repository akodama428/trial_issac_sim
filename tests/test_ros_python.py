from __future__ import annotations

import unittest
from unittest.mock import patch

from tomato_harvest_sim.robot.ros_python import ensure_ros_python_modules_available


class RosPythonTest(unittest.TestCase):
    def test_module_availability_retries_after_ros_path_restore(self) -> None:
        with patch(
            "tomato_harvest_sim.robot.ros_python.find_spec",
            side_effect=[None, object(), object()],
        ) as find_spec_mock, patch(
            "tomato_harvest_sim.robot.ros_python.ensure_ros_python_path"
        ) as ensure_path_mock:
            available = ensure_ros_python_modules_available("rclpy", "moveit_msgs")

        self.assertTrue(available)
        ensure_path_mock.assert_called_once_with()
        self.assertEqual(find_spec_mock.call_count, 3)

    def test_module_availability_returns_false_when_modules_still_missing(self) -> None:
        with patch(
            "tomato_harvest_sim.robot.ros_python.find_spec",
            side_effect=[None, None, object(), None],
        ), patch("tomato_harvest_sim.robot.ros_python.ensure_ros_python_path") as ensure_path_mock:
            available = ensure_ros_python_modules_available("rclpy", "moveit_msgs")

        self.assertFalse(available)
        ensure_path_mock.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
