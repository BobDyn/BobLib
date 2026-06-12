from __future__ import annotations

import csv
import math
import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest


MF52_REGRESSION_MODEL = "BobLib.Tests.Regression.MF52PureSlipSmoke"
MF52_RESULT_SIGNALS = (
    "Fx",
    "Fy",
    "Mx",
    "My",
    "Mz",
    "pneumaticTrail",
    "residualScrub",
    "MzReconstructed",
)

LOW_LEVEL_SIGNAL_CASES = (
    (
        "BobLib.Tests.TestVehicle.TestAero.TestBilinear2D",
        ("zInterior", "zClamped"),
        0.01,
    ),
    (
        "BobLib.Tests.TestVehicle.TestAero.TestCFDAeroMap",
        ("aero.drag", "aero.downforce"),
        0.01,
    ),
    (
        "BobLib.Tests.TestVehicle.TestPowertrain.TestVCU",
        ("vcu.P_req", "vcu.tau_cmd_limited"),
        0.01,
    ),
)

MODELICA_VERSION = "4.1.0"
OMC_COMMAND_LINE_OPTIONS = (
    "--matchingAlgorithm=PFPlusExt "
    "--indexReductionMethod=dynamicStateSelection "
    "-d=initialization,NLSanalyticJacobian "
    "--maxSizeLinearTearing=5000 "
    "--generateDynamicJacobian=none"
)


def _package_root() -> Path:
    cwd = Path.cwd()
    candidates = (
        cwd / "BobLib",
        cwd / "_0_Utils/external/BobLib/BobLib",
    )
    for candidate in candidates:
        if (candidate / "package.mo").is_file():
            return candidate.resolve()
    return Path(__file__).resolve().parents[1] / "BobLib"


def _render_mos(
    package_root: Path,
    work_dir: Path,
    model: str,
    signals: tuple[str, ...],
    stop_time: float,
) -> str:
    result_prefix = model.rsplit(".", maxsplit=1)[-1]
    signal_filter = "time|" + "|".join(signals)
    return f"""
clear();
setCommandLineOptions("{OMC_COMMAND_LINE_OPTIONS}");
loadModel(Modelica, {{"{MODELICA_VERSION}"}});
loadFile("{package_root.as_posix()}/package.mo");
cd("{work_dir.as_posix()}");
simulate(
  {model},
  startTime=0,
  stopTime={stop_time},
  numberOfIntervals=1,
  tolerance=1e-6,
  method="dassl",
  fileNamePrefix="{result_prefix}",
  outputFormat="csv",
  variableFilter="{signal_filter}",
  simflags="-jacobian=internalNumerical -noEventEmit");
getErrorString();
""".strip() + "\n"


def _run_omc(mos_text: str, model: str) -> str:
    omc = shutil.which("omc")
    if omc is None:
        pytest.skip("OpenModelica omc is not installed")

    with tempfile.NamedTemporaryFile("w", suffix=".mos", delete=False) as mos:
        mos.write(mos_text)
        mos_path = Path(mos.name)

    try:
        completed = subprocess.run(
            [omc, str(mos_path)],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
    finally:
        mos_path.unlink(missing_ok=True)

    if completed.returncode != 0:
        pytest.fail(f"omc failed for {model}:\n{completed.stdout}")
    return completed.stdout


def _read_csv(path: Path, signals: tuple[str, ...]) -> dict[str, list[float]]:
    text = path.read_text(encoding="utf-8")
    dialect = csv.Sniffer().sniff(text[:2048], delimiters=",;")
    rows = list(csv.DictReader(text.splitlines(), dialect=dialect))
    if not rows:
        raise AssertionError(f"No result rows found in {path}")

    data: dict[str, list[float]] = {}
    for signal in ("time", *signals):
        data[signal] = [float(row[signal]) for row in rows]
    return data


def _simulate_model(
    model: str,
    signals: tuple[str, ...],
    stop_time: float,
) -> dict[str, list[float]]:
    package_root = _package_root()
    assert (package_root / "package.mo").is_file()

    with tempfile.TemporaryDirectory(prefix="boblib_modelica_regression_") as raw_dir:
        work_dir = Path(raw_dir)
        output = _run_omc(
            _render_mos(package_root, work_dir, model, signals, stop_time),
            model,
        )
        result_prefix = model.rsplit(".", maxsplit=1)[-1]
        result_path = work_dir / f"{result_prefix}_res.csv"

        assert "Simulation execution failed" not in output
        assert result_path.is_file(), output

        return _read_csv(result_path, signals)


def _finite(values: list[float]) -> list[float]:
    return [value for value in values if math.isfinite(value)]


def test_mf52_pure_slip_smoke_runs_and_returns_sane_signals() -> None:
    data = _simulate_model(MF52_REGRESSION_MODEL, MF52_RESULT_SIGNALS, 0.01)

    required = {"time", *MF52_RESULT_SIGNALS}
    assert required.issubset(data)
    assert len(data["time"]) >= 2

    for signal in MF52_RESULT_SIGNALS:
        values = _finite(data[signal])
        assert len(values) == len(data[signal]), f"{signal} contains non-finite values"

    final_fx = data["Fx"][-1]
    final_fy = data["Fy"][-1]
    final_mz = data["Mz"][-1]
    final_mz_reconstructed = data["MzReconstructed"][-1]

    assert abs(final_fx) < 100.0
    assert abs(final_fy) > 10.0
    assert abs(final_mz - final_mz_reconstructed) < 1e-6
    assert math.isfinite(data["pneumaticTrail"][-1])
    assert math.isfinite(data["residualScrub"][-1])


def test_low_level_signal_models_return_expected_values() -> None:
    for model, signals, stop_time in LOW_LEVEL_SIGNAL_CASES:
        data = _simulate_model(model, signals, stop_time)

        for signal in signals:
            values = _finite(data[signal])
            assert len(values) == len(data[signal]), f"{model}.{signal} is non-finite"

        final_values = {signal: data[signal][-1] for signal in signals}

        if model.endswith("TestBilinear2D"):
            assert abs(final_values["zInterior"] - 25.0) < 1e-9
            assert abs(final_values["zClamped"] - 30.0) < 1e-9
        elif model.endswith("TestCFDAeroMap"):
            assert abs(final_values["aero.drag"] - 52.0) < 1e-9
            assert abs(final_values["aero.downforce"] - 520.0) < 1e-9
        elif model.endswith("TestVCU"):
            assert abs(final_values["vcu.tau_cmd_limited"] - 200.0) < 1e-9
            assert abs(final_values["vcu.P_req"] - 20000.0) < 1e-6
        else:
            raise AssertionError(f"Unhandled low-level signal model: {model}")
