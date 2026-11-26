from __future__ import annotations

import os
import sys
from pathlib import Path

TESTS_DIR = Path(__file__).resolve().parent
if str(TESTS_DIR) not in sys.path:
    sys.path.insert(0, str(TESTS_DIR))


def pytest_addoption(parser) -> None:
    group = parser.getgroup("boblib")
    group.addoption("--boblib-impact-plan", default=os.environ.get("BOBLIB_IMPACT_PLAN"),
                    help="JSON dependency selection plan from Tests/ci_changes.py")
    group.addoption("--boblib-impact-report", help="Write selection decisions as JSON")
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


def _shard_key(item) -> str:
    group = item.get_closest_marker("xdist_group")
    if group is None:
        return item.nodeid
    return "group:" + str(group.args[0] if group.args else group.kwargs.get("name"))


def pytest_collection_modifyitems(config, items) -> None:
    import impact_selection

    impact_selection.apply(config, items, TESTS_DIR.parent)
    shard = config.getoption("--boblib-shard")
    if not shard:
        return
    index, count = (int(part) for part in shard.split("/"))
    if not 1 <= index <= count:
        raise ValueError(f"--boblib-shard {shard} must be INDEX/COUNT with 1 <= INDEX <= COUNT.")
    # Deal tests out one at a time so a slow parametrized family spreads across
    # shards. An xdist_group stays whole so its module fixture still builds once.
    keys = sorted({_shard_key(item) for item in items})
    keep = {key for position, key in enumerate(keys) if position % count == index - 1}
    selected = [item for item in items if _shard_key(item) in keep]
    if len(selected) != len(items):
        config.hook.pytest_deselected(items=[item for item in items if _shard_key(item) not in keep])
        items[:] = selected


def pytest_configure(config) -> None:
    import impact_selection

    impact_selection.configure(config, TESTS_DIR.parent)
    config.addinivalue_line(
        "markers", "modelica_models(*names): complete local Modelica dependencies; empty means none"
    )


def pytest_sessionfinish(session, exitstatus) -> None:
    # A valid impact plan can leave a job or shard with no affected tests.
    # Collection errors and test failures must retain their nonzero status.
    if exitstatus == 5 and hasattr(session.config, "_boblib_impact_plan"):
        session.exitstatus = 0
