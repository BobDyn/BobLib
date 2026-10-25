"""Nested calibration modifications must survive whole-vehicle record defaults."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT=Path(__file__).resolve().parents[1]


def test_nested_record_calibration_in_all_architectures(tmp_path):
    if not shutil.which('omc'):
        pytest.skip('OpenModelica is required')
    records=sorted((ROOT/'BobLib/Records/VehicleDefn').glob('*Record.mo'))
    assert len(records)==9
    models=[]
    for index,record in enumerate(records):
        models.append(f'''model Case{index}
  parameter BobLib.Records.VehicleDefn.{record.stem} p(
    driveline(inputRatio=2),
    chassis(
      frontAxle(tire(force(fyPure(LMUY=0.97),fxPure(LMUX=0.96))),
        masses(unsprungMass(m=12),ucaMass(m=3))),
      rearAxle(tire(force(mzPure(LTR=0.95))),masses(lcaMass(m=4),tieMass(m=2))),
      frame(driverMass(m=70))));
initial equation
  assert(abs(p.controller.finalDriveRatio-2*p.transmission.gearRatio)<1e-12,"Controller reduction must include the differential input stage");
  assert(p.brakes.differentialBrakeRatio==2,"Brake conversion must follow the driveline input reduction");
  assert(p.chassis.frontAxle.tire.force.fyPure.LMUY==0.97,"Nested lateral tire calibration ignored");
  assert(p.chassis.frontAxle.tire.force.fxPure.LMUX==0.96,"Nested longitudinal tire calibration ignored");
  assert(p.chassis.rearAxle.tire.force.mzPure.LTR==0.95,"Nested aligning moment calibration ignored");
  assert(p.chassis.frontAxle.masses.unsprungMass.m==12 and p.chassis.frontAxle.masses.ucaMass.m==3,"Front component mass modifications ignored");
  assert(p.chassis.rearAxle.masses.lcaMass.m==4 and p.chassis.rearAxle.masses.tieMass.m==2,"Rear component mass modifications ignored");
  assert(abs(p.chassis.frame.mass.m-p.chassis.frame.baseMass.m-70)<1e-9,"Derived sprung mass did not recompute");
end Case{index};''')
    (tmp_path/'RecordCases.mo').write_text('package RecordCases\n'+'\n'.join(models)+'\nend RecordCases;\n')
    (tmp_path/'run.mos').write_text(f'''loadModel(Modelica,{{"4.1.0"}});
loadModel(VehicleInterfaces,{{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("RecordCases.mo");
'''+''.join(f'simulate(RecordCases.Case{i},stopTime=0);getErrorString();\n' for i in range(9)))
    result=subprocess.run(['omc','run.mos'],cwd=tmp_path,capture_output=True,text=True,timeout=180)
    (tmp_path/'omc.log').write_text(result.stdout+result.stderr)
    assert result.returncode==0 and 'Error:' not in result.stdout,result.stdout
    assert result.stdout.count('The simulation finished successfully')==9,result.stdout
