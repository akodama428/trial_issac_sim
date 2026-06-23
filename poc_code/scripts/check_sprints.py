#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from tomato_harvest_poc.sprint_checks import run_sprint_checks  # noqa: E402


def main() -> int:
    results = run_sprint_checks(REPO_ROOT)
    payload = [
        {
            "sprint": result.sprint,
            "passed": result.passed,
            "details": list(result.details),
        }
        for result in results
    ]
    print(json.dumps(payload, ensure_ascii=True, indent=2))
    return 0 if all(item["passed"] for item in payload) else 1


if __name__ == "__main__":
    raise SystemExit(main())
