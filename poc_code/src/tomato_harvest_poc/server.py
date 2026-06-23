from __future__ import annotations

import json
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

from .config import RuntimeConfig
from .model import ScenarioMode
from .render import render_camera_svg, render_stage_items, render_viewport_svg
from .service import HarvestSimulationService


def build_public_state(service: HarvestSimulationService, config: RuntimeConfig) -> dict[str, object]:
    snapshot = service.get_snapshot()
    return {
        "status": snapshot.status.value,
        "resultMessage": snapshot.result_message,
        "failureReason": snapshot.failure_reason.value if snapshot.failure_reason else None,
        "targetLabel": snapshot.target_label,
        "logs": [{"step": item.step, "message": item.message} for item in snapshot.logs],
        "instructions": list(snapshot.instructions),
        "stageHtml": render_stage_items(snapshot),
        "viewportSvg": render_viewport_svg(snapshot, config),
        "cameraSvg": render_camera_svg(snapshot),
        "attemptsCompleted": snapshot.visual.attempts_completed,
        "helpText": "1. Confirm target  2. Press Harvest Start  3. Check result",
    }


class TomatoHarvestRequestHandler(BaseHTTPRequestHandler):
    server_version = "TomatoHarvestPOC/0.1"

    @property
    def config(self) -> RuntimeConfig:
        return self.server.runtime_config  # type: ignore[attr-defined]

    @property
    def service(self) -> HarvestSimulationService:
        return self.server.runtime_service  # type: ignore[attr-defined]

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path == "/api/state":
            self._send_json(HTTPStatus.OK, build_public_state(self.service, self.config))
            return
        if parsed.path == "/":
            self._send_file(self.config.static_dir / "index.html", "text/html; charset=utf-8")
            return
        if parsed.path == "/static/app.js":
            self._send_file(self.config.static_dir / "app.js", "application/javascript; charset=utf-8")
            return
        if parsed.path == "/static/styles.css":
            self._send_file(self.config.static_dir / "styles.css", "text/css; charset=utf-8")
            return
        self._send_json(HTTPStatus.NOT_FOUND, {"error": "not_found"})

    def do_POST(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path == "/api/harvest":
            started = self.service.start_harvest()
            status = HTTPStatus.ACCEPTED if started else HTTPStatus.CONFLICT
            self._send_json(status, build_public_state(self.service, self.config))
            return
        if parsed.path == "/api/reset":
            reset = self.service.reset_scene()
            status = HTTPStatus.OK if reset else HTTPStatus.CONFLICT
            self._send_json(status, build_public_state(self.service, self.config))
            return
        self._send_json(HTTPStatus.NOT_FOUND, {"error": "not_found"})

    def log_message(self, format: str, *args: object) -> None:  # noqa: A003
        return

    def _send_json(self, status: HTTPStatus, payload: dict[str, object]) -> None:
        data = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_file(self, path: Path, content_type: str) -> None:
        if not path.exists():
            self._send_json(HTTPStatus.NOT_FOUND, {"error": "not_found"})
            return
        data = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def create_server(
    config: RuntimeConfig,
    scenario: ScenarioMode,
    service: HarvestSimulationService | None = None,
) -> ThreadingHTTPServer:
    runtime_service = service or HarvestSimulationService(config=config, scenario=scenario)
    if service is None:
        runtime_service.boot()
    server = ThreadingHTTPServer((config.ui_host, config.ui_port), TomatoHarvestRequestHandler)
    server.runtime_config = config  # type: ignore[attr-defined]
    server.runtime_service = runtime_service  # type: ignore[attr-defined]
    return server
