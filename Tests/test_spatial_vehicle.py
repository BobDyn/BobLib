"""Translate and simulate the complete spatial plant at each brake station."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('spatial', [False, True])
def test_spatial_vehicle_brake_stations(tmp_path, spatial):
    if shutil.which('omc') is None:
        pytest.skip('OpenModelica is not installed')
    fixture = tmp_path / 'SpatialVehicle.mo'
    fixture.write_text('''package SpatialVehicle
  model Outboard
    extends BobLib.Experiments.Standards.Examples.SpatialBrakingVehicle(
      vehicle(headless=true, includeRotorEffects=SPATIAL), brake(startTime=0),
      rearBrakePlacement=BobLib.Brakes.Types.Placement.Outboard);
  equation
    when terminal() then
      assert(abs(vehicle.brakePedalCommand - 0.3) < 1e-12,
        "The brake command must reach the plant");
      assert(vehicle.velX < 14.9 and vehicle.velX > 0,
        "Spatial vehicle must brake without reversing");
    end when;
  end Outboard;
  model Inboard
    extends Outboard(frontBrakePlacement=BobLib.Brakes.Types.Placement.Inboard,
      rearBrakePlacement=BobLib.Brakes.Types.Placement.Inboard);
  end Inboard;
  model Differential
    extends Outboard(rearBrakePlacement=BobLib.Brakes.Types.Placement.DifferentialInput,
      differentialInputRatio=2);
  end Differential;
end SpatialVehicle;
'''.replace('SPATIAL', str(spatial).lower()))
    script = tmp_path / 'check.mos'
    script.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{fixture}");
getErrorString();
''' + ''.join(f'''simulate(SpatialVehicle.{name},stopTime=0.25,tolerance=1e-6,
 variableFilter="time|vehicle.velX|vehicle.Fz_.*");
getErrorString();
''' for name in ['Outboard','Inboard','Differential']))
    result = subprocess.run(['omc', str(script)], cwd=tmp_path,
                            capture_output=True, text=True, timeout=1200, check=True)
    output = result.stdout + result.stderr
    (tmp_path / 'omc.log').write_text(output)
    assert 'Error:' not in output, output
    assert output.count('The simulation finished successfully') == 3, output
