"""Graphical annotations must respect the same visibility rules as model code."""
import shutil
import subprocess
from pathlib import Path

import pytest


@pytest.mark.skipif(shutil.which('omc') is None, reason='OpenModelica required')
def test_vehicle_graphics_do_not_access_protected_vi_parameters(tmp_path):
    root = Path(__file__).resolve().parents[1]
    models = [
        'BobLib.Experiments.Standards.VehicleSim',
        'BobLib.Experiments.Standards.Examples.PhysicalPedalVehicle',
        'BobLibTest.Experiments.Standards.BaseClasses.TestVIInteroperability',
    ]
    script = tmp_path / 'graphics.mos'
    script.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{root}/BobLib/package.mo");
loadFile("{root}/Tests/BobLibTest/package.mo");
''' + '\n'.join(f'''writeFile("graphics_{i}.json", getModelInstance({model}));
getErrorString();
checkModel({model});
getErrorString();''' for i, model in enumerate(models)))
    result = subprocess.run(['omc', str(script)], cwd=tmp_path,
                            capture_output=True, text=True, timeout=180)
    assert result.returncode == 0, result.stderr
    assert 'Error:' not in result.stdout + result.stderr, result.stdout + result.stderr
    for i, model in enumerate(models):
        assert (tmp_path / f'graphics_{i}.json').stat().st_size > 100
        assert f'Check of {model} completed successfully' in result.stdout
