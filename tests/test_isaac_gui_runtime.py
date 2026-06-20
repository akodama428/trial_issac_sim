from __future__ import annotations

import unittest

from tomato_harvest_poc.isaac_gui_runtime import (
    build_appframework_argv,
    compute_scene_visuals,
)
from tomato_harvest_poc.model import SimulationStatus, Snapshot, VisualState


def make_snapshot(
    *,
    status: SimulationStatus = SimulationStatus.READY,
    visual: VisualState | None = None,
) -> Snapshot:
    return Snapshot(
        status=status,
        result_message=status.value,
        target_label="Target Tomato",
        stage_items=("/World", "/World/TomatoPlant/Fruit"),
        visual=visual or VisualState(),
    )


class IsaacGuiRuntimeTest(unittest.TestCase):
    def test_appframework_argv_enables_gui_runtime_extensions(self) -> None:
        argv = build_appframework_argv()

        self.assertIn("/isaac-sim/exts", argv)
        self.assertIn("/isaac-sim/extscache", argv)
        self.assertIn("/isaac-sim/apps", argv)
        self.assertIn("--/renderer/asyncInit=true", argv)
        self.assertIn("--/persistent/renderer/startupMessageDisplayed=true", argv)
        self.assertIn("omni.kit.viewport.window", argv)

    def test_scene_visuals_move_tool_toward_target_during_harvest(self) -> None:
        ready = compute_scene_visuals(make_snapshot())
        harvesting = compute_scene_visuals(
            make_snapshot(
                status=SimulationStatus.APPROACHING,
                visual=VisualState(arm_progress=0.7),
            )
        )

        self.assertLess(ready.tool_position[0], harvesting.tool_position[0])
        self.assertFalse(harvesting.gripper_closed)

    def test_scene_visuals_detach_tomato_on_success(self) -> None:
        detached = compute_scene_visuals(
            make_snapshot(
                status=SimulationStatus.DETACHED,
                visual=VisualState(
                    arm_progress=1.0,
                    gripper_closed=True,
                    tomato_detached=True,
                ),
            )
        )

        self.assertTrue(detached.gripper_closed)
        self.assertGreater(detached.fruit_position[0], detached.stem_anchor_position[0])
        self.assertEqual(detached.fruit_color, (0.86, 0.31, 0.22))


if __name__ == "__main__":
    unittest.main()
