"""Driven-axle ports are structural, rather than hidden unused connectors."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_driven_axle_ports_and_halfshaft(tmp_path):
    if shutil.which('omc') is None:
        pytest.skip('OpenModelica is not installed')
    fixture = tmp_path / 'Ports.mo'
    fixture.write_text('package Ports\n' + ''.join(
        f'model {layout}\n extends BobLib.Drivelines.Interfaces.Base(driveLayout=BobLib.Drivelines.Types.DriveLayout.{layout});\nend {layout};\n'
        for layout in ('Front', 'Rear', 'All')) + '''model FrontBrakeOnly
  extends BobLib.Drivelines.Interfaces.Base(driveLayout=BobLib.Drivelines.Types.DriveLayout.Rear, useFrontInboardBrakes=true);
end FrontBrakeOnly;
model RearInboard
  extends BobLib.Drivelines.Interfaces.Base(useRearInboardBrakes=true);
end RearInboard;
end Ports;
''')
    script = tmp_path / 'run.mos'
    script.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{ROOT}/Tests/BobLibTest/package.mo");
loadFile("{fixture}");
''' + ''.join(f'writeFile("{layout}.txt", instantiateModel(Ports.{layout}));\ngetErrorString();\n'
              for layout in ('Front', 'Rear', 'All', 'FrontBrakeOnly', 'RearInboard')) + '''
simulate(BobLibTest.Drivelines.Internal.TestHalfshaft, stopTime=0.1, tolerance=1e-8);
getErrorString();
checkModel(BobLib.Experiments.Standards.VehicleSim);
getErrorString();
''')
    result = subprocess.run(['omc', str(script)], cwd=tmp_path, capture_output=True,
                            text=True, timeout=240, check=True)
    output = result.stdout + result.stderr
    (tmp_path / 'omc.log').write_text(output)
    assert 'Error:' not in output, output
    assert 'The simulation finished successfully' in output, output
    assert 'Check of BobLib.Experiments.Standards.VehicleSim completed successfully' in output, output
    for layout, ports in [('Front', {1, 2}), ('Rear', {3, 4}), ('All', {1, 2, 3, 4}), ('FrontBrakeOnly', {1, 2, 3, 4}), ('RearInboard', {3, 4})]:
        flat = (tmp_path / f'{layout}.txt').read_text()
        assert flat
        for i in range(1, 5):
            assert (f'wheelHub_{i}.flange.phi' in flat) == (i in ports), (layout, i)

        brakes = {1, 2} if layout == 'FrontBrakeOnly' else {3, 4} if layout == 'RearInboard' else set()
        for i in range(1, 5):
            assert (f'inboardBrake_{i}.flange.phi' in flat) == (i in brakes), (layout, i)
