"""Physical regressions for signed brake reactions, power, and static load paths."""
import csv
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
LOAD = f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{ROOT}/Tests/BobLibTest/package.mo");
'''


def run_model(work, script):
    if shutil.which("omc") is None:
        pytest.skip("OpenModelica is not installed")
    work.mkdir(parents=True, exist_ok=True)
    mos = work / "run.mos"
    mos.write_text(LOAD + script + '\ngetErrorString();\n')
    result = subprocess.run(["omc", str(mos)], cwd=work, capture_output=True,
                            text=True, timeout=600, check=True)
    (work / "omc.log").write_text(result.stdout)
    assert "The simulation finished successfully" in result.stdout, result.stdout
    assert "Error:" not in result.stdout, result.stdout


def test_brake_reaction_power_and_gearing(tmp_path):
    run_model(tmp_path, 'simulate(BobLibTest.Brakes.Internal.BrakeReactionSmoke, stopTime=0.01);')


def load_force_step(path):
    with path.open() as stream:
        row = min(csv.DictReader(stream), key=lambda item: abs(float(item["time"]) - 3))
    return {key: float(value) for key, value in row.items()}


def assert_static_load_paths(rows):
    outboard, inboard, differential = (rows[key] for key in ("Outboard", "Inboard", "DifferentialInput"))
    for axle, brakes in (("frAxleDW", "frontBrakes"), ("rrAxleDW", "rearBrakes")):
        arm = f"{axle}.leftWishboneUprightLoop.upperFrame_i.f[1]"
        assert abs(outboard[arm] - inboard[arm]) > 100, "Brake placement must change suspension loads"
        for axis in range(1, 4):
            moment = f"{axle}.axleFrame.t[{axis}]"
            # Include the complete brake support boundary, including explicit calipers.
            total_inboard = inboard[moment] + inboard[f"{brakes}.sprungFrame.t[{axis}]"]
            assert abs(outboard[moment] - total_inboard) < 0.02, "Reaction transfer changed net axle moment"
        actuator = "frChassisActuator" if axle == "frAxleDW" else "rrChassisActuator"
        jacking = f"{actuator}.jackingOutput"
        assert abs(inboard[jacking] - outboard[jacking]) > 1, "Brake location must change jacking at the same prescribed pose and load"
        assert abs(inboard[jacking] - differential[jacking]) < 0.02, "Symmetric inboard and differential brakes must have the same static jacking"
        for row in (inboard, differential):
            for side in ("left", "right"):
                transmitted = row[f"{brakes}.{side}Shaft.transmittedTorque"]
                assert abs(transmitted - row[f"{brakes}.{side}Brake.tau"]) < 1e-6
        assert abs(differential[f"{brakes}.leftBrake.tau"] - differential[f"{brakes}.rightBrake.tau"]) < 1e-8
        assert abs(3 * differential[f"{brakes}.inputBrakeTorque"] - 2 * differential[f"{brakes}.leftBrake.tau"]) < 1e-6


@pytest.mark.parametrize("architecture", ["DWDirect", "DWBC", "DWBCStabar"])
def test_static_brake_load_paths(tmp_path, architecture):
    rows = {}
    for mode in ("Outboard", "Inboard", "DifferentialInput"):
        work = tmp_path / mode
        work.mkdir()
        fixture = work / "Rig.mo"
        fixture.write_text(f'''model Rig
  extends BobLibTest.Experiments.Standards.Configurations.FourPostSim_{architecture}_{architecture}(
    headless=true,
    frontBrakePlacement=BobLib.Brakes.Types.Placement.{mode},
    rearBrakePlacement=BobLib.Brakes.Types.Placement.{mode},
    frontBrakeInputRatio=3, rearBrakeInputRatio=3);
end Rig;
''')
        run_model(work, f'''loadFile("{fixture}");
setCommandLineOptions("--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --maxSizeLinearTearing=5000 --generateDynamicJacobian=none");
simulate(Rig, stopTime=3.1, numberOfIntervals=31, tolerance=1e-6,
  outputFormat="csv", simflags="-jacobian=internalNumerical -noEventEmit");''')
        rows[mode] = load_force_step(work / "Rig_res.csv")
    assert_static_load_paths(rows)
    # Fixed calibration/pose reference detects duplicate road-torque support in static rigs.
    if architecture == "DWDirect":
        assert rows["Outboard"]["frChassisActuator.jackingOutput"] == pytest.approx(-3190.563348682308, abs=0.02)
        assert rows["Outboard"]["rrChassisActuator.jackingOutput"] == pytest.approx(-2773.845760161949, abs=0.02)


@pytest.mark.parametrize("placement", ["Outboard", "Inboard", "DifferentialInput"])
def test_explicit_caliper_support_boundary(tmp_path, placement):
    fixture = tmp_path / "CaliperSupport.mo"
    fixture.write_text(f'''model CaliperSupport
  inner Modelica.Mechanics.MultiBody.World world(enableAnimation=false, driveTrainMechanics3D=true);
  BobLib.Brakes.Internal.RigAxleBrake brake(useWheelFrames=true,
    placement=BobLib.Brakes.Types.Placement.{placement}, inputRatio=3,
    leftContactTorque=100, rightContactTorque=140);
  Modelica.Mechanics.MultiBody.Parts.Fixed leftUpright;
  Modelica.Mechanics.MultiBody.Parts.Fixed rightUpright;
  Modelica.Mechanics.MultiBody.Parts.Fixed chassis;
  Modelica.Mechanics.Rotational.Components.Fixed leftRotor;
  Modelica.Mechanics.Rotational.Components.Fixed rightRotor;
equation
  connect(leftRotor.flange,brake.left);
  connect(rightRotor.flange,brake.right);
  connect(leftUpright.frame_b,brake.leftWheelFrame);
  connect(rightUpright.frame_b,brake.rightWheelFrame);
''' + ('''  assert(abs(brake.leftWheelFrame.t[2]+100)<1e-9 and abs(brake.rightWheelFrame.t[2]+140)<1e-9,
    "Outboard caliper reaction must enter the unsprung supports");
''' if placement == "Outboard" else '''  connect(chassis.frame_b,brake.sprungFrame);
  assert(abs(brake.sprungFrame.t[2]+240)<1e-9,
    "Inboard caliper reaction must enter the sprung support");
  assert(sum(brake.leftWheelFrame.t.^2)+sum(brake.rightWheelFrame.t.^2)<1e-12,
    "Aligned inboard shafts must not add an upright caliper reaction");
''') + '''end CaliperSupport;
''')
    run_model(tmp_path, f'loadFile("{fixture}"); simulate(CaliperSupport, stopTime=0.01);')


@pytest.mark.parametrize("spatial", [False, True])
@pytest.mark.parametrize("differential", [False, True])
def test_vehicle_brake_axle_assemblies(tmp_path, spatial, differential):
    """Public VI ports preserve bias, speed publication and caliper reactions."""
    spatial_mo, differential_mo = str(spatial).lower(), str(differential).lower()
    fixture = tmp_path / "BrakeAssemblies.mo"
    fixture.write_text(f'''model BrakeAssemblies
  inner Modelica.Mechanics.MultiBody.World world(driveTrainMechanics3D=true);
  BobLib.Brakes.BasicVCUBrakes brakes(usingMultiBodyChassis={spatial_mo},
    useDifferentialInputBrake={differential_mo},
    pBrakes(maxTorque=1000,frontBrakeBias=0.6,differentialBrakeRatio=3));
  Modelica.Mechanics.Rotational.Sources.Speed wheels[4](each exact=true,w_ref={{10,-10,10,-10}});
  Modelica.Mechanics.Rotational.Sources.Speed inputSpeed(exact=true,w_ref=30) if {differential_mo};
equation
  brakes.controlBus.brakesControlBus.mechanicalBrakeTorqueRequest=2000;
''' + '\n'.join(
        f'  connect(wheels[{i}].flange,brakes.wheelHub_{i}.flange);'
        + (f'\n  connect(world.frame_b,brakes.wheelHub_{i}.bearingFrame);' if spatial else '')
        for i in range(1, 5)) + ('''
  connect(inputSpeed.flange,brakes.differentialInput.flange);
  assert(abs(brakes.differentialInput.flange.tau-400/3)<1e-8,"Input brake reduction");
''' + ('''
  connect(world.frame_b,brakes.differentialInput.bearingFrame);
  assert(abs(brakes.differentialInput.bearingFrame.t[2]+400/3)<1e-8,"Input caliper reaction");
''' if spatial else '') if differential else '') + f'''
  assert(abs(brakes.wheelHub_1.flange.tau-300)<1e-8 and abs(brakes.wheelHub_2.flange.tau+300)<1e-8,"Front axle bias and direction");
  assert(abs(brakes.wheelHub_3.flange.tau-{0 if differential else 200})<1e-8 and abs(brakes.wheelHub_4.flange.tau+{0 if differential else 200})<1e-8,"Rear axle or differential selection");
  assert(abs(brakes.controlBus.brakesBus.wheelSpeed_1-10)<1e-8 and abs(brakes.controlBus.brakesBus.wheelSpeed_4+10)<1e-8,"Wheel speed publication");
''' + ('''
  assert(abs(brakes.wheelHub_1.bearingFrame.t[2]+300)<1e-8 and abs(brakes.wheelHub_2.bearingFrame.t[2]-300)<1e-8,"Independent front caliper reactions");
''' if spatial else '') + '\nend BrakeAssemblies;\n')
    run_model(tmp_path, f'loadFile("{fixture}");\nsimulate(BrakeAssemblies,stopTime=0.01);')
