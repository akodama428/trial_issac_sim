from __future__ import annotations

import sys
from importlib.util import find_spec
from pathlib import Path


def _candidate_ros_python_paths() -> tuple[str, ...]:
    version = f"python{sys.version_info.major}.{sys.version_info.minor}"
    candidates: list[str] = []
    for distro in ("jazzy", "humble"):
        path = Path("/opt/ros") / distro / "lib" / version / "site-packages"
        if path.exists():
            candidates.append(str(path))
    return tuple(candidates)


def ensure_ros_python_path() -> None:
    for candidate in _candidate_ros_python_paths():
        if candidate not in sys.path:
            sys.path.append(candidate)


def ensure_ros_python_modules_available(*module_names: str) -> bool:
    if all(find_spec(module_name) is not None for module_name in module_names):
        return True
    ensure_ros_python_path()
    return all(find_spec(module_name) is not None for module_name in module_names)
