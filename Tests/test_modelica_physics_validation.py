from __future__ import annotations

import csv
import math
import os
import re
import shutil
import subprocess
import tempfile
import time
import warnings
from dataclasses import dataclass
from pathlib import Path

import pytest


MODELICA_VERSION = "4.1.0"
OMC_COMMAND_LINE_OPTIONS = (
    "--matchingAlgorithm=PFPlusExt "
    "--indexReductionMethod=dynamicStateSelection "
    "-d=initialization,NLSanalyticJacobian,disableStartCalc "
    "--maxSizeLinearTearing=5000 "
    "--generateDynamicJacobian=none"
)
DEFAULT_TIMEOUT_S = 180


@dataclass(frozen=True)
class SignalExpectation:
    signal: str
    expected: float
    abs_tol: float
    rel_tol: float
    description: str


@dataclass(frozen=True)
class PhysicalBaseline:
    model: str
    stop_time: float
    expectations: tuple[SignalExpectation, ...]

    @property
    def signals(self) -> tuple[str, ...]:
        return tuple(expectation.signal for expectation in self.expectations)


def _configured_package_root(config: pytest.Config) -> Path:
    configured = config.getoption("--boblib-package-root")
    if configured:
        package_root = Path(configured)
        if not package_root.is_absolute():
            package_root = Path.cwd() / package_root
        return package_root.resolve()

    candidate = Path.cwd() / "BobLib"
    if (candidate / "package.mo").is_file():
        return candidate.resolve()
    raise FileNotFoundError("Could not locate BobLib/package.mo")


def _read_physics_baselines() -> tuple[PhysicalBaseline, ...]:
    path = Path(__file__).with_name("modelica_physics_baseline.csv")
    with path.open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))

    grouped: dict[tuple[str, float], list[SignalExpectation]] = {}
    for row in rows:
        key = (row["model"], float(row["stop_time"]))
        grouped.setdefault(key, []).append(
            SignalExpectation(
                signal=row["signal"],
                expected=float(row["expected"]),
                abs_tol=float(row["abs_tol"]),
                rel_tol=float(row["rel_tol"]),
                description=row["description"],
            )
        )

    return tuple(
        PhysicalBaseline(model, stop_time, tuple(expectations))
        for (model, stop_time), expectations in grouped.items()
    )


def _runtime_budgets() -> dict[str, float]:
    path = Path(__file__).with_name("modelica_runtime_baseline.csv")
    with path.open("r", encoding="utf-8", newline="") as handle:
        return {
            row["model"]: float(row["budget_s"])
            for row in csv.DictReader(handle)
        }


def pytest_generate_tests(metafunc: pytest.Metafunc) -> None:
    if "baseline" not in metafunc.fixturenames:
        return

    baselines = _read_physics_baselines()
    metafunc.parametrize("baseline", baselines, ids=[item.model for item in baselines])


def _result_prefix(model: str) -> str:
    return "physics_" + re.sub(r"[^A-Za-z0-9_]", "_", model.rsplit(".", 1)[-1])


def _render_mos(
    package_root: Path,
    work_dir: Path,
    baseline: PhysicalBaseline,
) -> str:
    signal_filter = "time|" + "|".join(re.escape(signal) for signal in baseline.signals)
    return f"""
clear();
setCommandLineOptions("{OMC_COMMAND_LINE_OPTIONS}");
loadModel(Modelica, {{"{MODELICA_VERSION}"}});
loadFile("{package_root.as_posix()}/package.mo");
cd("{work_dir.as_posix()}");
simulate(
  {baseline.model},
  startTime=0,
  stopTime={baseline.stop_time},
  numberOfIntervals=1,
  tolerance=1e-6,
  method="dassl",
  fileNamePrefix="{_result_prefix(baseline.model)}",
  outputFormat="csv",
  variableFilter="{signal_filter}",
  simflags="-jacobian=internalNumerical -noEventEmit");
getErrorString();
""".strip() + "\n"


def _run_omc(mos_text: str, model: str, *, timeout_s: int) -> str:
    omc = shutil.which("omc")
    if omc is None:
        pytest.skip("OpenModelica omc is not installed")

    with tempfile.NamedTemporaryFile("w", suffix=".mos", delete=False) as mos:
        mos.write(mos_text)
        mos_path = Path(mos.name)

    try:
        try:
            completed = subprocess.run(
                [omc, str(mos_path)],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout_s,
            )
        except subprocess.TimeoutExpired as exc:
            output = exc.stdout or ""
            pytest.fail(f"omc timed out after {timeout_s}s for {model}:\n{output}")
    finally:
        mos_path.unlink(missing_ok=True)

    if completed.returncode != 0:
        pytest.fail(f"omc failed for {model}:\n{completed.stdout}")
    return completed.stdout


def _read_result(path: Path, signals: tuple[str, ...]) -> dict[str, float]:
    text = path.read_text(encoding="utf-8")
    dialect = csv.excel
    try:
        dialect = csv.Sniffer().sniff(text[:2048], delimiters=",;")
    except csv.Error:
        pass

    rows = list(csv.DictReader(text.splitlines(), dialect=dialect))
    assert rows, f"No result rows found in {path}"

    final_row = rows[-1]
    return {signal: float(final_row[signal]) for signal in signals}


def _simulate_baseline(
    package_root: Path,
    baseline: PhysicalBaseline,
    *,
    timeout_s: int,
) -> tuple[dict[str, float], float]:
    with tempfile.TemporaryDirectory(prefix="boblib_physics_validation_") as raw_dir:
        work_dir = Path(raw_dir)
        start = time.perf_counter()
        output = _run_omc(
            _render_mos(package_root, work_dir, baseline),
            baseline.model,
            timeout_s=timeout_s,
        )
        elapsed_s = time.perf_counter() - start

        result_path = work_dir / f"{_result_prefix(baseline.model)}_res.csv"
        assert "Simulation execution failed" not in output
        assert result_path.is_file(), output
        return _read_result(result_path, baseline.signals), elapsed_s


def _runtime_scale(config: pytest.Config) -> float:
    option = config.getoption("--boblib-runtime-scale")
    return float(option)


def _runtime_strict(config: pytest.Config) -> bool:
    return bool(config.getoption("--boblib-runtime-strict"))


def _check_runtime_budget(
    model: str,
    elapsed_s: float,
    *,
    budget_s: float,
    config: pytest.Config,
) -> None:
    scaled_budget_s = budget_s * _runtime_scale(config)
    limit_s = budget_s if _runtime_strict(config) else scaled_budget_s
    message = (
        f"{model} runtime {elapsed_s:.2f}s exceeded baseline budget "
        f"{budget_s:.2f}s (active limit {limit_s:.2f}s)"
    )

    if elapsed_s > limit_s:
        assert elapsed_s <= limit_s, message
    elif elapsed_s > budget_s:
        warnings.warn(message, RuntimeWarning, stacklevel=2)


def test_modelica_physical_baseline_matches(
    baseline: PhysicalBaseline,
    pytestconfig: pytest.Config,
) -> None:
    package_root = _configured_package_root(pytestconfig)
    actual, elapsed_s = _simulate_baseline(
        package_root,
        baseline,
        timeout_s=int(os.environ.get("BOBLIB_PHYSICS_TIMEOUT_S", DEFAULT_TIMEOUT_S)),
    )

    for expectation in baseline.expectations:
        actual_value = actual[expectation.signal]
        assert math.isclose(
            actual_value,
            expectation.expected,
            rel_tol=expectation.rel_tol,
            abs_tol=expectation.abs_tol,
        ), (
            f"{baseline.model}.{expectation.signal} changed: "
            f"expected {expectation.expected:.17g}, got {actual_value:.17g}; "
            f"{expectation.description}"
        )

    budget = _runtime_budgets()[baseline.model]
    _check_runtime_budget(
        baseline.model,
        elapsed_s,
        budget_s=budget,
        config=pytestconfig,
    )
