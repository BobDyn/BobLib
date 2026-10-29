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
        "--boblib-shard",
        action="store",
        default=os.environ.get("BOBLIB_SHARD"),
        help="Run one shard of the test files, as INDEX/COUNT with INDEX from 1.",
    )
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
        "--boblib-fmu-timeout-s",
        action="store",
        default=os.environ.get("BOBLIB_FMU_TIMEOUT_S"),
        type=int,
        help="Timeout in seconds for each OpenModelica and CMake step of an FMU export.",
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


def pytest_collection_modifyitems(config, items) -> None:
    shard = config.getoption("--boblib-shard")
    if not shard:
        return
    index, count = (int(part) for part in shard.split("/"))
    if not 1 <= index <= count:
        raise ValueError(f"--boblib-shard {shard} must be INDEX/COUNT with 1 <= INDEX <= COUNT.")
    # A file stays whole so its module fixtures run once, unless it sets BOBLIB_SHARD_BY_TEST.
    per_test = sorted(item.nodeid for item in items if getattr(item.module, "BOBLIB_SHARD_BY_TEST", False))
    files = sorted({item.path for item in items if not getattr(item.module, "BOBLIB_SHARD_BY_TEST", False)})
    keep = {node for position, node in enumerate(per_test) if position % count == index - 1}
    keep |= {path for position, path in enumerate(files) if position % count == index - 1}
    selected = [item for item in items if item.nodeid in keep or item.path in keep]
    if len(selected) != len(items):
        config.hook.pytest_deselected(items=[item for item in items if item not in selected])
        items[:] = selected
