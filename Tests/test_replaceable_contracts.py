"""Compiler regressions for replacement contracts and persistent calibration."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_tire_replacements_preserve_geometry(tmp_path):
    if shutil.which('omc') is None:
        pytest.skip('OpenModelica is not installed')
    fixture = tmp_path / 'ReplacementContracts.mo'
    fixture.write_text('''package ReplacementContracts
  model ExternalSlip
    extends BobLib.Chassis.Suspension.Tires.MF52.SlipModel.BaseSlipModel;
  equation
    alpha = 0;
    kappa = 0;
  end ExternalSlip;

  model RotatingWheel
    extends BobLibTest.Chassis.Suspension.Tires.TireForceFrameSmoke(
      pWheel(R0=0.37),
      tire(redeclare replaceable BobLib.Chassis.Suspension.Tires.Wheels.Wheel1DOF_Y wheelModel(pWheel(rotation(wheelJ=0.08)))
        constrainedby BobLib.Chassis.Suspension.Tires.Wheels.PartialWheel,
        redeclare ExternalSlip slipModel));
  initial equation
    assert(abs(tire.wheelModel.pWheel.geometry.R0-0.37)<1e-12,
      "Wheel replacement lost tire geometry");
  end RotatingWheel;

  model CompliantWheel
    extends RotatingWheel(tire(
      redeclare BobLib.Chassis.Suspension.Tires.Wheels.Wheel1DOF_Z wheelModel(pWheel(vertical(wheelC=30000, wheelD=800)))));
    // A massless free contact frame leaves the vertical DOF unsolvable at initialization.
    Modelica.Mechanics.MultiBody.Parts.Body cpLoad(m=0.1, r_CM={0,0,0},
      I_11=1e-4, I_22=1e-4, I_33=1e-4, animation=false);
  equation
    connect(tire.cpFrame, cpLoad.frame_a);
  end CompliantWheel;

  model RigidWheel
    extends RotatingWheel(tire(
      redeclare BobLib.Chassis.Suspension.Tires.Wheels.Wheel0DOF wheelModel));
  end RigidWheel;
end ReplacementContracts;
''')
    mos = tmp_path / 'run.mos'
    mos.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{ROOT}/Tests/BobLibTest/package.mo");
loadFile("{fixture}");
simulate(ReplacementContracts.CompliantWheel, stopTime=0.001);
getErrorString();
simulate(ReplacementContracts.RotatingWheel, stopTime=0.001);
getErrorString();
simulate(ReplacementContracts.RigidWheel, stopTime=0.001);
getErrorString();
''')
    result = subprocess.run(['omc', str(mos)], cwd=tmp_path, capture_output=True,
                            text=True, timeout=180, check=True)
    (tmp_path / 'omc.log').write_text(result.stdout + result.stderr)
    assert 'Error:' not in result.stdout, result.stdout
    assert result.stdout.count('The simulation finished successfully') == 3, result.stdout


def test_compliant_laws_are_replaceable(tmp_path):
    if shutil.which('omc') is None:
        pytest.skip('OpenModelica is not installed')
    fixture = tmp_path / 'ComplianceContracts.mo'
    fixture.write_text('''package ComplianceContracts
  model LinearShock
    extends BobLibTest.Chassis.Suspension.Components.TestShockLinkage(shock(
      redeclare Modelica.Mechanics.Translational.Components.Spring TabularSpring(
        c=1000, s_rel0=0.9),
      redeclare Modelica.Mechanics.Translational.Components.Damper TabularDamper(d=50)));
  equation
    assert(abs(shock.TabularSpring.f-100)<1e-6, "Replacement spring force is incorrect");
  end LinearShock;
  model CustomHalfshaft
    extends Modelica.Mechanics.Rotational.Interfaces.PartialCompliant;
    parameter Modelica.Units.SI.RotationalSpringConstant stiffness=15000;
  equation
    tau = stiffness*phi_rel;
  end CustomHalfshaft;
  model Driveline
    extends BobLibTest.Drivelines.TestRearFinalDriveDifferential(dut(
      redeclare model HalfshaftModel = CustomHalfshaft(stiffness=20000)));
  equation
    assert(dut.leftHalfshaft.compliance.stiffness==dut.rightHalfshaft.compliance.stiffness,
      "Halfshaft calibration differs between sides");
  end Driveline;
'''+''.join(f'''  model Axle{i}
    extends BobLibTest.Chassis.Suspension.Test{name}(frAxleDW(
      redeclare model SpringModel = Modelica.Mechanics.Translational.Components.Spring(
        c=30000, s_rel0=0.2),
      redeclare model DamperModel = Modelica.Mechanics.Translational.Components.Damper(d=800)));
  end Axle{i};
''' for i, name in enumerate(['DoubleWishboneDirect', 'DoubleWishboneBellcrank',
                             'DoubleWishboneBellcrankStabar']))+'''
end ComplianceContracts;
''')
    checks = ''.join(f'checkModel(ComplianceContracts.Axle{i});\ngetErrorString();\n'
                     for i in range(3))
    mos = tmp_path / 'run.mos'
    mos.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{ROOT}/Tests/BobLibTest/package.mo");
loadFile("{fixture}");
{checks}
simulate(ComplianceContracts.LinearShock, stopTime=0.01);
getErrorString();
simulate(ComplianceContracts.Driveline, stopTime=0.01);
getErrorString();
''')
    result = subprocess.run(['omc', str(mos)], cwd=tmp_path, capture_output=True,
                            text=True, timeout=240, check=True)
    (tmp_path / 'omc.log').write_text(result.stdout + result.stderr)
    assert 'Error:' not in result.stdout, result.stdout
    assert result.stdout.count('completed successfully') == 3, result.stdout
    assert result.stdout.count('The simulation finished successfully') == 2, result.stdout
