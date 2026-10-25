"""Reproduce the PR-stack full-vehicle comparison; this is not a golden baseline."""
import argparse
import csv
import hashlib
import json
import math
import subprocess
from pathlib import Path

OPTIONS = ("--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection "
           "-d=initialization,NLSanalyticJacobian,disableStartCalc --maxSizeLinearTearing=5000 "
           "--generateDynamicJacobian=none --homotopyApproach=equidistantLocal")
SIGNALS = ["accX", "accY", "velX", "yawVel", "roll", "sideslip",
           "Fz_FL", "Fz_FR", "Fz_RL", "Fz_RR"]


def run(command, work, log):
    result = subprocess.run(command, cwd=work, capture_output=True, text=True, timeout=900)
    output = result.stdout + result.stderr
    (work / log).write_text(output)
    if result.returncode or "Error:" in output or "not possible to override" in output or "override variable name not found" in output:
        raise RuntimeError(f"Comparison failed; inspect {work / log}")
    return output


def build(root, work, controller):
    for prefix, model in [("vehicle", "BobLib.Experiments.Standards.VehicleSim"),
                          ("straight", "ReviewVehicle")]:
        (work / "ReviewVehicle.mo").write_text(
            "model ReviewVehicle\n  extends BobLib.Experiments.Standards.VehicleSim("
            f"{controller}(targetVel=15,steerStart=20));\nend ReviewVehicle;\n")
        script = f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{root}/BobLib/package.mo");
loadFile("ReviewVehicle.mo");
setCommandLineOptions("{OPTIONS}");
buildModel({model}, stopTime=10, numberOfIntervals=5000, tolerance=1e-6,
  outputFormat="csv", fileNamePrefix="{prefix}",
  variableFilter="time|terminationStatus|accX|accY|velX|velY|yawVel|roll|sideslip|handwheelAngle|Fz_.*");
getErrorString();
'''
        (work / f"{prefix}.mos").write_text(script)
        run(["omc", f"{prefix}.mos"], work, f"{prefix}-build.log")
    run(["./vehicle", "-r=default.csv"], work, "default.log")
    for name, target in [("braking", 10), ("acceleration", 20)]:
        run(["./straight", "-stopTime=2", "-stepSize=0.002",
             f"-override={controller}.targetVel={target}", f"-r={name}.csv"], work, f"{name}.log")


def summarize(work):
    results = {}
    for name in ["default", "braking", "acceleration"]:
        log = (work / f"{name}.log").read_text()
        assert "The simulation finished successfully" in log, log
        assert "not possible to override" not in log and "override variable name not found" not in log, log
        with (work / f"{name}.csv").open() as stream:
            rows = [{key: float(value) for key, value in row.items()} for row in csv.DictReader(stream)]
        assert rows and all(math.isfinite(row[key]) for row in rows for key in SIGNALS)
        if name != "default":
            assert abs(rows[-1]["time"] - 2) < 1e-6
        results[name] = {"stop_time_s": rows[-1]["time"],
                         "final": {key: rows[-1][key] for key in SIGNALS},
                         "minimum_tire_load_N": min(row[key] for row in rows for key in SIGNALS if key.startswith("Fz_"))}
        if "terminationStatus" in rows[-1]:
            results[name]["termination_status"] = int(rows[-1]["terminationStatus"])
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", type=Path, required=True)
    parser.add_argument("--after", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summarize-only", action="store_true")
    args = parser.parse_args()
    report = {"compiler": subprocess.check_output(["omc", "--version"], text=True).strip(),
              "libraries": {"Modelica": "4.1.0", "VehicleInterfaces": "2.0.2"},
              "scope": "Shipped default ramp plus 2 s straight acceleration/braking from 15 m/s; aggregate behavior changes, not isolated causal attribution or full-catalog acceptance."}
    for name, root, controller in [("before", args.before, "vcu"), ("after", args.after, "driver")]:
        root = root.resolve()
        work = args.output.resolve() / name
        work.mkdir(parents=True, exist_ok=True)
        if not args.summarize_only:
            build(root, work, controller)
        digest = hashlib.sha256()
        for path in sorted((root / "BobLib").rglob("*.mo")):
            digest.update(str(path.relative_to(root)).encode())
            digest.update(path.read_bytes())
        report[name] = {"revision": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
                        "modelica_source_sha256": digest.hexdigest(), "results": summarize(work)}
    (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
