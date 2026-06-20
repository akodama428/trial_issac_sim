from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest


class RunPocArgsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        script_path = Path(__file__).resolve().parents[1] / "scripts" / "run_poc.py"
        spec = importlib.util.spec_from_file_location("run_poc", script_path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"Failed to load run_poc.py from {script_path}")
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        cls.script = module

    def test_defaults_keep_native_runtime_window_enabled(self) -> None:
        args = self.script.parse_args([])

        self.assertEqual(args.mode, "mock")
        self.assertFalse(args.headless)
        self.assertFalse(args.test)
        self.assertEqual(args.camera_view, "fixed")

    def test_isaac_runtime_accepts_headless_test_and_hand_camera(self) -> None:
        args = self.script.parse_args(
            ["--mode", "isaac", "--headless", "--test", "--camera-view", "hand"]
        )

        self.assertEqual(args.mode, "isaac")
        self.assertTrue(args.headless)
        self.assertTrue(args.test)
        self.assertEqual(args.camera_view, "hand")


if __name__ == "__main__":
    unittest.main()
