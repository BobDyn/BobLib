from __future__ import annotations

import os
import sys
from pathlib import Path


TESTS_DIR = Path(__file__).resolve().parent
if str(TESTS_DIR) not in sys.path:
    sys.path.insert(0, str(TESTS_DIR))


def pytest_addoption(parser) -> None:
    group = parser.getgroup("boblib")
    group.addoption(
        "--boblib-package-root",
        action="store",
        default=os.environ.get("BOBLIB_PACKAGE_ROOT"),
        help="Path to the BobLib package directory containing package.mo.",
    )
    group.addoption(
        "--boblib-initialization-baseline",
        action="store",
        default=os.environ.get("BOBLIB_INITIALIZATION_BASELINE"),
        help="CSV baseline for Modelica initialization metrics.",
    )
    group.addoption(
        "--boblib-initialization-timeout-s",
        action="store",
        default=os.environ.get("BOBLIB_INITIALIZATION_TIMEOUT_S"),
        type=int,
        help="Per-model OpenModelica initialization timeout in seconds.",
    )
    group.addoption(
        "--boblib-runtime-scale",
        action="store",
        default=os.environ.get("BOBLIB_RUNTIME_SCALE", "4.0"),
        type=float,
        help="Multiplier applied to runtime baseline budgets before failing.",
    )
    group.addoption(
        "--boblib-runtime-strict",
        action="store_true",
        default=os.environ.get("BOBLIB_RUNTIME_STRICT", "").lower()
        in {"1", "true", "yes", "on"},
        help="Fail when runtime exceeds the unscaled baseline budget.",
    )
