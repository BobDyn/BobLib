"""Aero clearances follow the road datum through the actual chassis adapter."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.skipif(shutil.which('omc') is None, reason='OpenModelica required')
def test_chassis_ride_height_above_offset_road(tmp_path):
    (tmp_path / 'HeightCases.mo').write_text('''package HeightCases
  model OffsetRoad
    extends VehicleInterfaces.Roads.FlatRoad(animation=false,
      redeclare function position=offsetPosition);
    function offsetPosition
      extends VehicleInterfaces.Roads.Interfaces.positionBase;
    algorithm
      r_0:={s,w,-0.02};
    end offsetPosition;
  end OffsetRoad;
  model Chassis
    extends BobLib.Chassis.DoubleWishbone;
  equation
    assert(abs(rideHeight_1-rideHeightPoint_1[3]-0.02)<1e-8,"Front-left road-relative clearance");
    assert(abs(rideHeight_2-rideHeightPoint_2[3]-0.02)<1e-8,"Front-right road-relative clearance");
    assert(abs(rideHeight_3-rideHeightPoint_3[3]-0.02)<1e-8,"Rear-left road-relative clearance");
    assert(abs(rideHeight_4-rideHeightPoint_4[3]-0.02)<1e-8,"Rear-right road-relative clearance");
  end Chassis;
  model Vehicle
    extends BobLib.Experiments.Standards.Examples.RoadAndAtmosphereVehicle(
      headless=true,redeclare OffsetRoad road(mu=0.35),
      redeclare Chassis chassis(
        redeclare record ChassisRecord=VehicleRecord.ChassisRecord,
        pChassis=pVehicle.chassis,flatRoadFastPath=false));
  end Vehicle;
end HeightCases;
''')
    (tmp_path / 'run.mos').write_text(f'''loadModel(Modelica,{{"4.1.0"}});
loadModel(VehicleInterfaces,{{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");loadFile("HeightCases.mo");
setCommandLineOptions("--generateDynamicJacobian=none");
setCFlags(getCFlags()+" -Os");
simulate(HeightCases.Vehicle,stopTime=0.01,tolerance=1e-6);getErrorString();
''')
    result = subprocess.run(['omc', 'run.mos'], cwd=tmp_path, text=True,
                            capture_output=True, timeout=600, check=True)
    output = result.stdout + result.stderr
    (tmp_path / 'omc.log').write_text(output)
    assert 'Error:' not in output, output
    assert 'LOG_ASSERT' not in output, output
    assert 'The simulation finished successfully' in output, output
