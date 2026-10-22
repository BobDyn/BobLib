from __future__ import annotations

from pathlib import Path

import pytest


def test_wheel_alignment_uses_degree_type_and_explicit_radian_conversion() -> None:
    root = Path(__file__).resolve().parents[1]
    record = (
        root
        / "BobLib"
        / "Records"
        / "VehicleRecord"
        / "Chassis"
        / "Suspension"
        / "Templates"
        / "Tire"
        / "Templates"
        / "PartialWheelRecord.mo"
    ).read_text(encoding="utf-8")

    assert "parameter NonSI.Angle_deg staticAlpha" in record
    assert "parameter NonSI.Angle_deg staticGamma" in record
    assert "parameter SI.Angle staticAlpha" not in record
    assert "parameter SI.Angle staticGamma" not in record

    consumers = (
        root / "BobLib" / "Chassis" / "DoubleWishbone.mo",
        root
        / "BobLib"
        / "Experiments"
        / "Standards"
        / "BaseClasses"
        / "PartialDoubleWishboneFourPost.mo",
    )

    for path in consumers:
        source = path.read_text(encoding="utf-8")
        assert "staticAlpha*pi/180" not in source
        assert "staticGamma*pi/180" not in source
        assert "Conversions.from_deg" in source


@pytest.mark.parametrize("side", ["left", "right"])
def test_steer_measurement_is_invariant_to_vehicle_attitude(tmp_path, side):
    import re
    import shutil
    import subprocess

    if not shutil.which('omc'):
        pytest.skip('OpenModelica is required')
    root = Path(__file__).resolve().parents[1]
    source = (root / 'BobLib/Chassis/BaseClasses/PartialChassis.mo').read_text()
    equations = '\n'.join(re.search(r'^  ' + name + r'\s*=.*?;', source,
                                    re.MULTILINE | re.DOTALL)[0]
                          for name in (f'{side}WheelVector', f'{side}SteerAngle'))
    declarations, assertions, assignments = [], [], []
    for i, angle in enumerate((0.4, -0.7, 2.2)):
        declarations.append(f'''parameter Frames.Orientation axleR{i} =
  Frames.axesRotations({{1,2,3}}, {{0.23,-0.18,0.8}}, {{0,0,0}});
parameter Frames.Orientation wheelR{i} = Frames.absoluteRotation(
  axleR{i}, Frames.planarRotation({{0,0,1}}, {angle}, 0));
Real vector{i}[3];
Real angle{i};''')
        assignments.append(equations.replace('frAxleDW.axleFrame.R', f'axleR{i}')
                           .replace(f'frAxleDW.{side}CP.R', f'wheelR{i}')
                           .replace(f'{side}WheelVector', f'vector{i}')
                           .replace(f'{side}SteerAngle', f'angle{i}'))
        assertions.append(f'assert(abs(angle{i}-({angle})) < 1e-12, '
                          '"Road-wheel steer must be measured in the axle frame");')
    (tmp_path / 'SteerFrameProbe.mo').write_text(
        'model SteerFrameProbe\nimport Frames = Modelica.Mechanics.MultiBody.Frames;\n'
        + '\n'.join(declarations) + '\ninitial equation\n' + '\n'.join(assertions)
        + '\nequation\n' + '\n'.join(assignments) + '\nend SteerFrameProbe;\n')
    (tmp_path / 'run.mos').write_text('''loadModel(Modelica, {"4.1.0"});
loadFile("SteerFrameProbe.mo");
simulate(SteerFrameProbe, stopTime=0.001);getErrorString();
''')
    result = subprocess.run(['omc', 'run.mos'], cwd=tmp_path, text=True,
                            capture_output=True, timeout=120)
    output = result.stdout + result.stderr
    assert 'The simulation finished successfully' in output, output
    assert 'Error:' not in output, output
