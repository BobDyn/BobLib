#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import math
import os
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path


BASELINE_COLUMNS = (
    "model",
    "numeric_count",
    "finite_count",
    "nonfinite_count",
    "names_sha256",
    "sum",
    "sum_abs",
    "max_abs",
    "values_sha256",
)

MODELICA_VERSION = "4.1.0"
OMC_COMMAND_LINE_OPTIONS = (
    "--matchingAlgorithm=PFPlusExt "
    "--indexReductionMethod=dynamicStateSelection "
    "-d=initialization,NLSanalyticJacobian,disableStartCalc "
    "--maxSizeLinearTearing=5000 "
    "--generateDynamicJacobian=none"
)
DEFAULT_TIMEOUT_S = 600


@dataclass(frozen=True)
class InitializationMetrics:
    model: str
    numeric_count: int
    finite_count: int
    nonfinite_count: int
    names_sha256: str
    sum: float
    sum_abs: float
    max_abs: float
    values_sha256: str

    def as_row(self) -> dict[str, str]:
        return {
            "model": self.model,
            "numeric_count": str(self.numeric_count),
            "finite_count": str(self.finite_count),
            "nonfinite_count": str(self.nonfinite_count),
            "names_sha256": self.names_sha256,
            "sum": f"{self.sum:.17g}",
            "sum_abs": f"{self.sum_abs:.17g}",
            "max_abs": f"{self.max_abs:.17g}",
            "values_sha256": self.values_sha256,
        }

    @classmethod
    def from_row(cls, row: dict[str, str]) -> InitializationMetrics:
        return cls(
            model=row["model"],
            numeric_count=int(row["numeric_count"]),
            finite_count=int(row["finite_count"]),
            nonfinite_count=int(row["nonfinite_count"]),
            names_sha256=row["names_sha256"],
            sum=float(row["sum"]),
            sum_abs=float(row["sum_abs"]),
            max_abs=float(row["max_abs"]),
            values_sha256=row["values_sha256"],
        )


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


def test_fixture_models(package_root: Path) -> tuple[str, ...]:
    tests_root = package_root / "Tests"
    return tuple(
        sorted(
            {
                _class_path(package_root, path)
                for path in tests_root.rglob("*.mo")
                if path.name != "package.mo"
            }
        )
    )


def render_mos(package_root: Path, work_dir: Path, model: str, prefix: str) -> str:
    return f"""
clear();
setCommandLineOptions("{OMC_COMMAND_LINE_OPTIONS}");
loadModel(Modelica, {{"{MODELICA_VERSION}"}});
loadFile("{package_root.as_posix()}/package.mo");
cd("{work_dir.as_posix()}");
simulate(
  {model},
  startTime=0,
  stopTime=0,
  numberOfIntervals=1,
  tolerance=1e-6,
  method="dassl",
  fileNamePrefix="{prefix}",
  outputFormat="csv",
  variableFilter=".*",
  simflags="-jacobian=internalNumerical -noEventEmit");
getErrorString();
""".strip() + "\n"


def run_omc(mos_text: str, model: str, *, timeout_s: int) -> str:
    omc = shutil.which("omc")
    if omc is None:
        raise RuntimeError("OpenModelica omc is not installed")

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
            raise RuntimeError(
                f"omc timed out after {timeout_s} seconds while initializing {model}:\n"
                f"{output}"
            ) from exc
    finally:
        mos_path.unlink(missing_ok=True)

    if completed.returncode != 0:
        raise RuntimeError(f"omc failed for {model}:\n{completed.stdout}")
    if "The initialization finished successfully" not in completed.stdout:
        raise RuntimeError(
            f"initialization did not report success for {model}:\n{completed.stdout}"
        )
    return completed.stdout


def _hash_text(parts: list[str]) -> str:
    digest = hashlib.sha256()
    for part in parts:
        digest.update(part.encode("utf-8"))
        digest.update(b"\0")
    return digest.hexdigest()


def metrics_from_result(model: str, result_path: Path) -> InitializationMetrics:
    text = result_path.read_text(encoding="utf-8")
    dialect = csv.Sniffer().sniff(text[:4096], delimiters=",;")
    rows = list(csv.DictReader(text.splitlines(), dialect=dialect))
    if len(rows) != 1:
        raise RuntimeError(f"Expected one initialization row in {result_path}, got {len(rows)}")

    values_by_name: dict[str, float] = {}
    for name, value_text in rows[0].items():
        if name == "time":
            continue
        try:
            values_by_name[name] = float(value_text)
        except (TypeError, ValueError):
            continue

    names = sorted(values_by_name)
    values = [values_by_name[name] for name in names]
    finite_values = [value for value in values if math.isfinite(value)]
    normalized_values = [
        "nan" if math.isnan(value) else f"{value:.17g}" for value in values
    ]

    return InitializationMetrics(
        model=model,
        numeric_count=len(values),
        finite_count=len(finite_values),
        nonfinite_count=len(values) - len(finite_values),
        names_sha256=_hash_text(names),
        sum=math.fsum(finite_values),
        sum_abs=math.fsum(abs(value) for value in finite_values),
        max_abs=max((abs(value) for value in finite_values), default=0.0),
        values_sha256=_hash_text(normalized_values),
    )


def initialize_model(
    package_root: Path,
    model: str,
    *,
    timeout_s: int,
) -> InitializationMetrics:
    prefix = "init_" + hashlib.sha1(model.encode("utf-8")).hexdigest()[:12]
    with tempfile.TemporaryDirectory(prefix="boblib_initialization_") as raw_dir:
        work_dir = Path(raw_dir)
        run_omc(
            render_mos(package_root, work_dir, model, prefix),
            model,
            timeout_s=timeout_s,
        )
        result_path = work_dir / f"{prefix}_res.csv"
        if not result_path.is_file():
            raise RuntimeError(f"OpenModelica did not write {result_path} for {model}")
        return metrics_from_result(model, result_path)


def read_baseline(path: Path) -> dict[str, InitializationMetrics]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    return {
        row["model"]: InitializationMetrics.from_row(row)
        for row in rows
    }


def write_baseline(path: Path, metrics: list[InitializationMetrics]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=BASELINE_COLUMNS, lineterminator="\n")
        writer.writeheader()
        for metric in sorted(metrics, key=lambda item: item.model):
            writer.writerow(metric.as_row())


def assert_matches_baseline(
    actual: InitializationMetrics,
    expected: InitializationMetrics,
    *,
    rtol: float,
    atol: float,
) -> None:
    if actual.numeric_count != expected.numeric_count:
        raise AssertionError(
            f"{actual.model}: numeric variable count changed: "
            f"expected {expected.numeric_count}, got {actual.numeric_count}"
        )
    if actual.finite_count != expected.finite_count:
        raise AssertionError(
            f"{actual.model}: finite variable count changed: "
            f"expected {expected.finite_count}, got {actual.finite_count}"
        )
    if actual.nonfinite_count != expected.nonfinite_count:
        raise AssertionError(
            f"{actual.model}: non-finite variable count changed: "
            f"expected {expected.nonfinite_count}, got {actual.nonfinite_count}"
        )
    if actual.names_sha256 != expected.names_sha256:
        raise AssertionError(f"{actual.model}: initialized numeric variable set changed")

    for field_name in ("sum", "sum_abs", "max_abs"):
        actual_value = getattr(actual, field_name)
        expected_value = getattr(expected, field_name)
        if not math.isclose(actual_value, expected_value, rel_tol=rtol, abs_tol=atol):
            raise AssertionError(
                f"{actual.model}: initialization {field_name} changed: "
                f"expected {expected_value:.17g}, got {actual_value:.17g}"
            )


def parse_args() -> argparse.Namespace:
    default_timeout_s = int(
        os.environ.get("BOBLIB_INITIALIZATION_TIMEOUT_S", str(DEFAULT_TIMEOUT_S))
    )
    parser = argparse.ArgumentParser(
        description="Run BobLib Modelica initialization regression checks."
    )
    parser.add_argument(
        "--package-root",
        type=Path,
        default=None,
        help="Path to the BobLib package directory containing package.mo.",
    )
    parser.add_argument(
        "--baseline",
        type=Path,
        default=Path("Tests/modelica_initialization_baseline.csv"),
        help="CSV baseline for initialization metrics.",
    )
    parser.add_argument(
        "--update-baseline",
        action="store_true",
        help="Regenerate the initialization baseline instead of checking it.",
    )
    parser.add_argument(
        "--model",
        action="append",
        default=None,
        help="Specific Modelica class to initialize. May be passed multiple times.",
    )
    parser.add_argument(
        "--timeout-s",
        type=int,
        default=default_timeout_s,
        help="Per-model OpenModelica timeout in seconds.",
    )
    parser.add_argument(
        "--rtol",
        type=float,
        default=1e-9,
        help="Relative tolerance for aggregate floating-point metrics.",
    )
    parser.add_argument(
        "--atol",
        type=float,
        default=1e-8,
        help="Absolute tolerance for aggregate floating-point metrics.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    package_root = args.package_root or default_package_root()
    package_root = package_root.resolve()
    if not (package_root / "package.mo").is_file():
        raise FileNotFoundError(f"package.mo not found under {package_root}")

    models = tuple(args.model) if args.model else test_fixture_models(package_root)
    actual_metrics: list[InitializationMetrics] = []
    for model in models:
        print(f"Initializing {model} with timeout {args.timeout_s}s", flush=True)
        metrics = initialize_model(package_root, model, timeout_s=args.timeout_s)
        actual_metrics.append(metrics)
        print(
            f"{metrics.model}: initialized "
            f"{metrics.finite_count}/{metrics.numeric_count} numeric values"
        )

    baseline_path = args.baseline
    if args.update_baseline:
        write_baseline(baseline_path, actual_metrics)
        print(f"Wrote initialization baseline: {baseline_path}")
        return

    expected_by_model = read_baseline(baseline_path)
    missing = sorted(set(models) - set(expected_by_model))
    extra = sorted(set(expected_by_model) - set(models))
    if missing:
        raise AssertionError(f"Missing initialization baseline rows: {missing}")
    if extra and not args.model:
        raise AssertionError(f"Stale initialization baseline rows: {extra}")

    for actual in actual_metrics:
        assert_matches_baseline(
            actual,
            expected_by_model[actual.model],
            rtol=args.rtol,
            atol=args.atol,
        )


if __name__ == "__main__":
    main()
