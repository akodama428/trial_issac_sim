#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from tomato_harvest_poc.config import RuntimeConfig  # noqa: E402
from tomato_harvest_poc.isaac_native_runtime import IsaacNativeRuntime  # noqa: E402
from tomato_harvest_poc.native_harvest import CameraViewMode  # noqa: E402
from tomato_harvest_poc.isaac_runtime import IsaacRuntimeUnavailable, require_isaac_modules  # noqa: E402
from tomato_harvest_poc.model import ScenarioMode  # noqa: E402
from tomato_harvest_poc.server import create_server  # noqa: E402


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the tomato harvest PoC runtime.")
    parser.add_argument("--mode", choices=("mock", "isaac"), default=os.environ.get("POC_RUNTIME", "mock"))
    parser.add_argument("--host", default=os.environ.get("POC_HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("POC_PORT", "8080")))
    parser.add_argument("--headless", action="store_true", default=os.environ.get("POC_HEADLESS", "0") == "1")
    parser.add_argument("--test", action="store_true", default=os.environ.get("POC_TEST_MODE", "0") == "1")
    parser.add_argument(
        "--camera-view",
        choices=tuple(mode.value for mode in CameraViewMode),
        default=os.environ.get("POC_CAMERA_VIEW", CameraViewMode.FIXED.value),
    )
    parser.add_argument(
        "--scenario",
        choices=tuple(mode.value for mode in ScenarioMode),
        default=os.environ.get("POC_SCENARIO", ScenarioMode.SUCCESS.value),
    )
    return parser.parse_args(argv)


def main() -> int:
    args = parse_args()
    if args.mode == "isaac":
        try:
            require_isaac_modules()
        except IsaacRuntimeUnavailable as exc:
            print(str(exc), file=sys.stderr)
            return 2

    config = RuntimeConfig(ui_host=args.host, ui_port=args.port)
    scenario = ScenarioMode(args.scenario)
    if args.mode == "isaac":
        return _run_isaac_mode(headless=args.headless, test_mode=args.test, camera_view=CameraViewMode(args.camera_view))
    return _run_mock_mode(config=config, scenario=scenario, host=args.host, port=args.port)


def _run_mock_mode(
    *,
    config: RuntimeConfig,
    scenario: ScenarioMode,
    host: str,
    port: int,
) -> int:
    server = create_server(config=config, scenario=scenario)
    print(f"Tomato harvest PoC running on http://{host}:{port} in mock mode")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


def _run_isaac_mode(
    *,
    headless: bool,
    test_mode: bool,
    camera_view: CameraViewMode,
) -> int:
    runtime = IsaacNativeRuntime(
        headless=headless,
        test_mode=test_mode,
        initial_camera_view=camera_view,
    )
    print("Tomato harvest PoC running in native Isaac Sim mode")
    print("Controls: Start / Stop / Reset in the Isaac Sim window")
    print(f"Initial camera: {camera_view.value}")
    try:
        return runtime.run()
    except KeyboardInterrupt:
        return 0
    finally:
        runtime.close()


if __name__ == "__main__":
    raise SystemExit(main())
