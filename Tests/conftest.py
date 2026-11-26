from __future__ import annotations

import os
import sys
from pathlib import Path

TESTS_DIR = Path(__file__).resolve().parent
if str(TESTS_DIR) not in sys.path:
    sys.path.insert(0, str(TESTS_DIR))

import shard_plan  # noqa: E402


def pytest_addoption(parser) -> None:
    group = parser.getgroup("boblib")
    group.addoption(
        "--boblib-shard",
        action="store",
        default=os.environ.get("BOBLIB_SHARD"),
        help="Run one shard of the tests, as INDEX/COUNT with INDEX from 1.",
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
    # Units are planned by recorded duration (shard_plan.py). A file stays whole so its
    # module fixtures run once, unless it sets BOBLIB_SHARD_BY_TEST; a file with an
    # xdist_group stays whole regardless, since that group shares one module fixture.
    # Units are keyed from the repo root so they match shard_durations.json whatever
    # rootdir pytest picked.
    def unit(item) -> str:
        path = Path(str(item.path)).resolve().relative_to(shard_plan.ROOT).as_posix()
        by_test = getattr(item.module, "BOBLIB_SHARD_BY_TEST", False)
        if by_test and item.get_closest_marker("xdist_group") is None:
            return f"{path}::{item.nodeid.split('::', 1)[1]}"
        return path

    keep = shard_plan.plan({unit(item) for item in items}, count, shard_plan.load_durations())[index - 1]
    selected = [item for item in items if unit(item) in keep]
    if len(selected) != len(items):
        config.hook.pytest_deselected(items=[item for item in items if unit(item) not in keep])
        items[:] = selected
