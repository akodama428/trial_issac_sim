from __future__ import annotations

import unittest

from tomato_harvest_poc.isaac_runtime import build_launch_plan


class IsaacRuntimePlanTest(unittest.TestCase):
    def test_launch_plan_points_to_native_runtime(self) -> None:
        plan = build_launch_plan()

        self.assertEqual(plan.container_entrypoint, "./python.sh scripts/run_poc.py --mode isaac")
        self.assertIn("native Isaac Sim 3DView", plan.notes[1])


if __name__ == "__main__":
    unittest.main()
