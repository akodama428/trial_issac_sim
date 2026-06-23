from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest


class IsaacFrankaSmokeArgsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        script_path = Path(__file__).resolve().parents[1] / "scripts" / "isaac_franka_smoke.py"
        spec = importlib.util.spec_from_file_location("isaac_franka_smoke", script_path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"Failed to load Franka smoke script from {script_path}")
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        cls.script = module

    def test_defaults_do_not_enable_hand_camera_or_centering(self) -> None:
        args = self.script.parse_args([])

        self.assertFalse(args.headless)
        self.assertFalse(args.test)
        self.assertFalse(args.use_hand_camera)
        self.assertFalse(args.center_tomato)

    def test_accepts_center_tomato_flag(self) -> None:
        args = self.script.parse_args(["--headless", "--use-hand-camera", "--center-tomato"])

        self.assertTrue(args.headless)
        self.assertTrue(args.use_hand_camera)
        self.assertTrue(args.center_tomato)


if __name__ == "__main__":
    unittest.main()
