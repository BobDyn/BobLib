"""Compare reused artifacts with a separately translated calibrated vehicle."""

import importlib.util
import json
import os
import re
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_calibration_families_match_rebuilt_reference(tmp_path):
    archive_name = os.environ.get("BOBLIB_CALIBRATION_FMU")
    native_name = os.environ.get("BOBLIB_CALIBRATION_NATIVE")
    if not archive_name or not native_name:
        pytest.skip(
            "Set BOBLIB_CALIBRATION_FMU and BOBLIB_CALIBRATION_NATIVE to matching artifacts"
        )
    np = pytest.importorskip("numpy")
    fmpy = pytest.importorskip("fmpy")
    spec = importlib.util.spec_from_file_location(
        "verify_calibration", ROOT / "Tools/verify_fmu.py"
    )
    verifier = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(verifier)
    archive = Path(archive_name).resolve()
    native = Path(native_name).resolve()
    manifest = json.loads((archive.parent / "manifest.json").read_text())
    metadata = manifest["parameters"]
    convert = {
        "Real": float,
        "Integer": int,
        "Enumeration": int,
        "Boolean": lambda s: s == "true",
        "String": str,
    }
    defaults = {
        k: convert[v["type"]](v["start"]) for k, v in metadata.items() if k.startswith("pVehicle.")
    }
    defaults["initialVel"] = 10.0

    def scaled(name, factor):
        return defaults[name] * factor

    families = {
        "geometry": {
            "pVehicle.chassis.frontAxle.geometry.upperFore_i[1]": defaults[
                "pVehicle.chassis.frontAxle.geometry.upperFore_i[1]"
            ]
            + 0.001
        },
        "mass_inertia": {
            "pVehicle.chassis.frame.driverMass.m": defaults["pVehicle.chassis.frame.driverMass.m"]
            + 3,
            "pVehicle.chassis.frame.baseMass.inertia[1,1]": scaled(
                "pVehicle.chassis.frame.baseMass.inertia[1,1]", 1.03
            ),
            "pVehicle.chassis.frontAxle.masses.unsprungMass.m": scaled(
                "pVehicle.chassis.frontAxle.masses.unsprungMass.m", 1.02
            ),
        },
        "steering": {
            "pVehicle.chassis.frontAxle.steering.cFactor": scaled(
                "pVehicle.chassis.frontAxle.steering.cFactor", 1.03
            )
        },
        "spring_damper": {
            "pVehicle.chassis.frontAxle.actuation.springFreeLength": scaled(
                "pVehicle.chassis.frontAxle.actuation.springFreeLength", 1.005
            ),
            "pVehicle.chassis.frontAxle.actuation.damperTable[1,2]": scaled(
                "pVehicle.chassis.frontAxle.actuation.damperTable[1,2]", 1.05
            ),
            "pVehicle.chassis.frontAxle.actuation.damperTable[3,2]": scaled(
                "pVehicle.chassis.frontAxle.actuation.damperTable[3,2]", 1.05
            ),
        },
        "tires": {
            "pVehicle.chassis.frontAxle.tire.force.fyPure.LMUY": 0.97,
            "pVehicle.chassis.frontAxle.tire.wheel.geometry.R0": defaults[
                "pVehicle.chassis.frontAxle.tire.wheel.geometry.R0"
            ]
            + 0.0005,
        },
        "drivetrain": {
            "pVehicle.transmission.gearRatio": scaled("pVehicle.transmission.gearRatio", 1.03)
        },
        "aero": {"pVehicle.aero.referenceDensity": scaled("pVehicle.aero.referenceDensity", 1.01)},
        "electrical": {
            "pVehicle.battery.SOC_start": 0.95,
            "pVehicle.motor.eta_mot": 0.94,
            "pVehicle.inverter.eta_mot": 0.95,
        },
    }
    combined = {k: v for changes in families.values() for k, v in changes.items()}
    outputs = (
        ["accX", "accY", "velX", "roll", "Fz_FL", "Fz_FR", "Fz_RL", "Fz_RR"]
        + list(combined)
        + [
            "pVehicle.chassis.frame.mass.m",
            "pVehicle.chassis.frontAxle.tire.force.setup.UNLOADED_RADIUS",
            "pVehicle.chassis.frontAxle.tire.force.relaxation.UNLOADED_RADIUS",
        ]
    )
    inputs = np.array(
        [(0.0, 0.02, 0.1, 0.0), (0.2, 0.02, 0.1, 0.0)],
        dtype=[
            ("time", float),
            ("steeringAngleCommand", float),
            ("acceleratorPedalCommand", float),
            ("brakePedalCommand", float),
        ],
    )
    report = {}

    def compare(actual, expected):
        differences = {}
        for name in outputs:
            sampled = np.interp(expected["time"], actual["time"], actual[name])
            differences[name] = float(np.max(np.abs(sampled - expected[name])))
            tolerance = (
                1e-9
                if name.startswith("pVehicle.")
                else 0.5
                if name.startswith("Fz_")
                else 0.01
                if name in ("accX", "accY")
                else 1e-4
            )
            np.testing.assert_allclose(
                sampled, expected[name], rtol=0, atol=tolerance, err_msg=name
            )
        return differences

    for label, changes in [*families.items(), ("combined", combined)]:
        result = fmpy.simulate_fmu(
            str(archive),
            start_time=0,
            stop_time=0.2,
            output_interval=0.002,
            relative_tolerance=1e-6,
            start_values=defaults | changes,
            input=inputs,
            output=outputs,
            fmi_type="CoSimulation",
        )
        radius = (defaults | changes)["pVehicle.chassis.frontAxle.tire.wheel.geometry.R0"]
        for field in ("setup", "relaxation"):
            np.testing.assert_allclose(
                result[f"pVehicle.chassis.frontAxle.tire.force.{field}.UNLOADED_RADIUS"],
                radius,
                rtol=0,
                atol=1e-12,
            )
        actual, elapsed = verifier.native_trajectory(
            native, tmp_path / label, defaults | changes, inputs, outputs
        )
        report[label] = {
            "changed": changes,
            "native_seconds": elapsed,
            "differences": compare(actual, result),
        }
        (tmp_path / "calibration-summary.json").write_text(json.dumps(report, indent=2) + "\n")
        if label == "combined":
            combined_result = result
    # Materialize only changed arrays; untouched record fields retain their declared dependencies.
    tree = {}
    for name, value in combined.items():
        parts = name.removeprefix("pVehicle.").split(".")
        branch = tree
        for part in parts[:-1]:
            branch = branch.setdefault(part, {})
        field = parts[-1].split("[")[0]
        if "[" not in parts[-1]:
            branch[field] = value
            continue
        prefix = name[: name.index("[")]
        entries = {
            tuple(int(i) - 1 for i in re.findall(r"\d+", key[len(prefix) :])): val
            for key, val in defaults.items()
            if key.startswith(prefix + "[")
        }
        shape = tuple(
            max(index[d] for index in entries) + 1 for d in range(len(next(iter(entries))))
        )
        array = np.zeros(shape)
        for index, val in entries.items():
            array[index] = combined.get(
                prefix + "[" + ",".join(str(i + 1) for i in index) + "]", val
            )
        branch[field] = array.tolist()

    def literal(value):
        return (
            "{" + ",".join(literal(v) for v in value) + "}"
            if isinstance(value, list)
            else str(value)
        )

    def modifiers(branch):
        return ",".join(
            k + "(" + modifiers(v) + ")" if isinstance(v, dict) else k + "=" + literal(v)
            for k, v in branch.items()
        )

    reference = tmp_path / "CalibrationReference.mo"
    reference.write_text(
        f"model CalibrationReference\n extends {archive.stem}(initialVel=10,pVehicle({modifiers(tree)}));\nend CalibrationReference;\n"
    )
    script = tmp_path / "reference.mos"
    script.write_text(f'''loadModel(Modelica,{{"4.1.0"}});
loadModel(VehicleInterfaces,{{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{archive.with_suffix(".mo")}");
loadFile("{reference}");
buildModel(CalibrationReference,stopTime=0.2,outputFormat="csv",cflags="-Os");
getErrorString();
''')
    build = subprocess.run(
        ["omc", str(script)], cwd=tmp_path, capture_output=True, text=True, timeout=900
    )
    (tmp_path / "reference-build.log").write_text(build.stdout + build.stderr)
    assert build.returncode == 0 and (tmp_path / "CalibrationReference").exists(), build.stdout
    actual, elapsed = verifier.native_trajectory(
        tmp_path / "CalibrationReference", tmp_path / "rebuilt", {}, inputs, outputs
    )
    report["rebuilt_combined"] = {
        "native_seconds": elapsed,
        "differences": compare(actual, combined_result),
    }
    (tmp_path / "calibration-summary.json").write_text(json.dumps(report, indent=2) + "\n")
