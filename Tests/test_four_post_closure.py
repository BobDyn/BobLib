"""Check FourPostSim force closure on every output row, not only the final one."""
import csv
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
MODEL = "BobLib.Experiments.Standards.FourPostSim"
AXLES = ("fr", "rr")
COMMANDS = ("heave", "roll", "fx", "fy")
# About 1e-6 of the 1 kN static axle load.
FORCE_TOL = 1e-3
MOMENT_TOL = 1e-3
FILTER = (r"time|(fr|rr)KnC\.(jackingForce|heave|roll|fx|fy)"
          r"|(fr|rr)AxleDW\.(left|right)CP\.(f\[[1-3]\]|R\.T\[[1-3],3\])"
          r"|(fr|rr)ChassisActuator\.sprungLoads\.frame_a\.(f\[[12]\]|t\[1\])")
MOS_FILTER = FILTER.replace("\\", "\\\\")


def contact_fz(row, axle):
    """World-vertical contact force; KnC Fz is clamped at zero when a tire lifts off."""
    return sum(row[f"{axle}AxleDW.{side}CP.R.T[{i},3]"] * row[f"{axle}AxleDW.{side}CP.f[{i}]"]
               for side in ("left", "right") for i in (1, 2, 3))


def assert_rows(rows, label, residual, tol):
    worst = max(rows, key=lambda row: abs(residual(row)))
    assert abs(residual(worst)) < tol, (
        f"{label} = {residual(worst):.3e} at t = {worst['time']} s (tolerance {tol})")


def test_four_post_closure_at_every_row(tmp_path):
    if shutil.which("omc") is None:
        pytest.skip("OpenModelica is not installed")
    script = tmp_path / "run.mos"
    script.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
simulate({MODEL}, outputFormat="csv", simflags="-nls=kinsol",
  variableFilter="{MOS_FILTER}");
getErrorString();
''')
    result = subprocess.run(["omc", str(script)], cwd=tmp_path, capture_output=True,
                            text=True, timeout=900, check=True)
    assert "The simulation finished successfully" in result.stdout, result.stdout
    assert "Error:" not in result.stdout, result.stdout
    with (tmp_path / f"{MODEL}_res.csv").open() as stream:
        rows = [{key: float(value) for key, value in row.items()} for row in csv.DictReader(stream)]
    for command in ("heave", "roll"):
        assert any(abs(row[f"frKnC.{command}"]) > 1e-3 for row in rows), f"No {command} sequence"
    zero = [row for row in rows
            if all(abs(row[f"{axle}KnC.{name}"]) <= 1e-9 for axle in AXLES for name in COMMANDS)]
    assert zero, "No zero-command rows"
    for axle in AXLES:
        actuator = f"{axle}ChassisActuator.sprungLoads.frame_a"
        assert_rows(rows, f"{axle} vertical closure",
                    lambda row, a=axle: contact_fz(row, a) + row[f"{a}KnC.jackingForce"], FORCE_TOL)
        for signal, tol in (("f[1]", FORCE_TOL), ("f[2]", FORCE_TOL), ("t[1]", MOMENT_TOL)):
            assert_rows(zero, f"{actuator}.{signal} at zero command",
                        lambda row, s=f"{actuator}.{signal}": row[s], tol)
