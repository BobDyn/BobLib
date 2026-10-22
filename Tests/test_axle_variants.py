"""Translate each attachment/steering branch; this does not calibrate new geometry."""
import re
import shutil
import subprocess
from pathlib import Path

import pytest


def test_axle_attachment_and_steering_variants(tmp_path):
    if shutil.which("omc") is None:
        pytest.skip("OpenModelica is not installed")
    root = Path(__file__).resolve().parents[1]
    models = []
    source = ["package AxleVariants"]
    for architecture in ("DWDirect", "DWBC", "DWBCStabar"):
        for lower in ("true", "false"):
            for steer in ("true", "false"):
                name = f"{architecture}_{lower}_{steer}"
                models.append(name)
                source.append(f'''model {name}
  extends BobLibTest.Experiments.Standards.Configurations.FourPostSim_{architecture}_{architecture}(
    frAxleDW(steerable={steer}),
    pVehicle(      chassis(      frontAxle(      actuation(rodToLower={lower})),
      rearAxle(      actuation(rodToLower={lower})))));
end {name};''')
    source.append("end AxleVariants;")
    fixture = tmp_path / "AxleVariants.mo"
    fixture.write_text("\n".join(source))
    script = tmp_path / "check.mos"
    commands = [
        'loadModel(Modelica, {"4.1.0"});',
        'loadModel(VehicleInterfaces, {"2.0.2"});',
        f'loadFile("{root}/BobLib/package.mo");',
        f'loadFile("{root}/Tests/BobLibTest/package.mo");',
        f'loadFile("{fixture}");',
    ]
    for name in models:
        commands.extend((f"checkModel(AxleVariants.{name});", "getErrorString();"))
    script.write_text("\n".join(commands))
    result = subprocess.run(["omc", str(script)], cwd=tmp_path,
                            capture_output=True, text=True, check=True, timeout=240)
    assert "Error:" not in result.stdout, result.stdout
    for name in models:
        assert f"Check of AxleVariants.{name} completed successfully" in result.stdout, result.stdout
    counts = re.findall(r"(\d+) equation\(s\) and (\d+) variable\(s\)", result.stdout)
    assert len(counts) == len(models), result.stdout
    assert all(equations == variables for equations, variables in counts), result.stdout
