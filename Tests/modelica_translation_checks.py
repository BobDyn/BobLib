#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ModelCheck:
    name: str
    expected_equations: int | None = None


DEFAULT_CHECKS = (
    ModelCheck("BobLib.Standards.VehicleSim", expected_equations=18130),
    ModelCheck("BobLib.Standards.FourPostSim", expected_equations=18622),
    ModelCheck(
        "BobLib.Tests.Regression.VehicleSimAnimationOn",
        expected_equations=24772,
    ),
    ModelCheck(
        "BobLib.Tests.Regression.MF52PureSlipSmoke",
        expected_equations=8,
    ),
)


def test_fixture_checks(package_root: Path) -> tuple[ModelCheck, ...]:
    tests_root = package_root / "Tests"
    tests = sorted(
        {
            _class_path(package_root, path)
            for path in tests_root.rglob("*.mo")
            if path.name != "package.mo"
        }
    )
    return tuple(ModelCheck(test) for test in tests)


def _class_path(package_root: Path, modelica_file: Path) -> str:
    rel = modelica_file.relative_to(package_root).with_suffix("")
    return ".".join((package_root.name, *rel.parts))


def default_package_root() -> Path:
    cwd = Path.cwd()
    candidates = [
        cwd / "BobLib",
        cwd / "_0_Utils/external/BobLib/BobLib",
    ]
    for candidate in candidates:
        if (candidate / "package.mo").is_file():
            return candidate
    raise FileNotFoundError(
        "Could not locate BobLib/package.mo. Pass --package-root explicitly."
    )


def render_mos(package_root: Path, check: ModelCheck) -> str:
    lines = [
        "clear();",
        'loadModel(Modelica, {"4.1.0"});',
        f'loadFile("{package_root.as_posix()}/package.mo");',
    ]
    lines.extend(
        [
            f"checkModel({check.name});",
            "getErrorString();",
        ]
    )
    return "\n".join(lines) + "\n"


def run_check(package_root: Path, check: ModelCheck, *, enforce_counts: bool) -> None:
    with tempfile.NamedTemporaryFile("w", suffix=".mos", delete=False) as mos:
        mos.write(render_mos(package_root, check))
        mos_path = Path(mos.name)

    try:
        completed = subprocess.run(
            ["omc", str(mos_path)],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
    finally:
        mos_path.unlink(missing_ok=True)

    output = completed.stdout
    if completed.returncode != 0:
        raise RuntimeError(f"omc failed for {check.name}:\n{output}")

    success = f"Check of {check.name} completed successfully."
    if success not in output:
        raise RuntimeError(f"checkModel did not report success for {check.name}:\n{output}")

    match = re.search(
        rf"Class {re.escape(check.name)} has ([0-9]+) equation\(s\)",
        output,
    )
    if match is None:
        raise RuntimeError(f"Could not parse equation count for {check.name}:\n{output}")

    equation_count = int(match.group(1))
    print(f"{check.name}: {equation_count} equations")

    if (
        enforce_counts
        and check.expected_equations is not None
        and equation_count != check.expected_equations
    ):
        raise RuntimeError(
            f"{check.name} equation count changed: "
            f"expected {check.expected_equations}, got {equation_count}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run BobLib Modelica translation regression checks."
    )
    parser.add_argument(
        "--package-root",
        type=Path,
        default=None,
        help="Path to the BobLib package directory containing package.mo.",
    )
    parser.add_argument(
        "--no-equation-counts",
        action="store_true",
        help="Only require successful checkModel translation, not exact equation counts.",
    )
    parser.add_argument(
        "--no-vehicle-test-fixtures",
        action="store_true",
        dest="no_test_fixtures",
        help="Only translate core regression models, not every Vehicle test fixture.",
    )
    parser.add_argument(
        "--no-test-fixtures",
        action="store_true",
        dest="no_test_fixtures",
        help="Only translate core regression models, not every BobLib.Tests fixture.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    package_root = args.package_root or default_package_root()
    package_root = package_root.resolve()

    if not (package_root / "package.mo").is_file():
        raise FileNotFoundError(f"package.mo not found under {package_root}")

    checks = list(DEFAULT_CHECKS)
    if not args.no_test_fixtures:
        checks.extend(test_fixture_checks(package_root))

    unique_checks = {}
    for check in checks:
        unique_checks[check.name] = check

    for check in unique_checks.values():
        run_check(package_root, check, enforce_counts=not args.no_equation_counts)


if __name__ == "__main__":
    main()
