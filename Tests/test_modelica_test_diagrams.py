"""Basic diagram guardrails; rendered OMEdit review is still required."""
import re
from pathlib import Path

import pytest
from modelica_routing_audit import _placed_boxes

from modelica_linter import _matching_paren_index

FIXTURES = tuple(sorted((Path(__file__).parent / 'BobLibTest').rglob('*.mo')))


@pytest.mark.parametrize(
    'path', FIXTURES + tuple(sorted((Path(__file__).parents[1] / 'BobLib').rglob('*.mo'))),
    ids=lambda path: str(path.relative_to(Path(__file__).parents[1])),
)
def test_connections_have_explicit_orthogonal_routes(path):
    text = path.read_text()
    for match in re.finditer(r'^\s*connect\s*\(', text, re.MULTILINE):
        end = _matching_paren_index(text, match.end() - 1)
        assert end is not None
        statement = text[match.start():text.index(';', end)]
        route = re.search(r'\bpoints\s*=\s*(\{\{.*?\}\})', statement, re.DOTALL)
        assert route, f'{path}: connection needs an explicit diagram route: {statement}'
        points = [tuple(float(value.strip()) for value in pair.split(','))
                  for pair in re.findall(r'\{([^{}]+)\}', route[1])]
        assert len(points) >= 2 and all(len(point) == 2 for point in points)
        assert all(a[0] == b[0] or a[1] == b[1] for a, b in zip(points, points[1:], strict=False)), statement
        assert all(
            (b[0]-a[0])*(c[0]-b[0]) + (b[1]-a[1])*(c[1]-b[1]) >= 0
            for a, b, c in zip(points, points[1:], points[2:], strict=False)
        ), f'Connection doubles back over its own segment: {statement}'


def test_custom_graphic_labels_are_instance_names():
    for path in FIXTURES:
        for label in re.findall(r'\btextString\s*=\s*"([^"]*)"', path.read_text()):
            assert label == '%name', f'{path}: use the instance name, not a diagram caption'



def test_local_fixture_symbols_do_not_overlap():
    for path in FIXTURES + (Path(__file__).parents[1] / 'BobLib/Controllers/VCU.mo',):
        boxes = list(_placed_boxes(path.read_text()))
        for index, (scope, line, a) in enumerate(boxes):
            for other_scope, other_line, b in boxes[index + 1:]:
                if scope != other_scope:
                    continue
                overlap = min(a[2], b[2]) - max(a[0], b[0]) > 1e-6 and min(a[3], b[3]) - max(a[1], b[1]) > 1e-6
                assert not overlap, f'{path}:{line} overlaps the symbol at line {other_line}'


def test_placed_boxes_handle_rotation_and_nested_classes():
    text = '''model Outer
      model Inner
        A a annotation(Placement(transformation(origin={10,20},extent={{-2,-1},{2,1}},rotation=90)));
      end Inner;
      A b annotation(Placement(transformation(extent={{0,0},{4,6}})));
    end Outer;'''
    inner, outer = list(_placed_boxes(text))
    assert inner[0] == 'Inner' and inner[2] == pytest.approx((9, 18, 11, 22))
    assert outer[0] == 'Outer' and outer[2] == (0, 0, 4, 6)


@pytest.mark.parametrize('relative_path,components', [
    ('Chassis/Suspension/BaseClasses/PartialDoubleWishboneAxle.mo',
     ('rackAndPinion', 'toRack')),
    ('Brakes/Internal/BrakeActuation.mo', ('pedalPosition', 'frontBrakes')),
    ('Drivelines/RearFinalDriveDifferential.mo', ('frontRightBrakeShaft', 'leftTorqueSignal')),
    ('Drivelines/RearFinalDriveDifferential.mo', ('frontRightBrakeShaft', 'drivelineBus')),
])
def test_simultaneously_visible_production_symbols_are_separated(relative_path, components):
    text = (Path(__file__).parents[1] / 'BobLib' / relative_path).read_text()
    boxes = []
    for name in components:
        declaration = re.search(r'\b' + re.escape(name) + r'\s*(?:\(|if\b|annotation\b)', text)
        assert declaration is not None
        statement = text[declaration.start():text.index(';', declaration.end()) + 1]
        placements = list(_placed_boxes(statement))
        assert len(placements) == 1
        boxes.append(placements[0][2])
    a, b = boxes
    assert min(a[2], b[2]) <= max(a[0], b[0]) or min(a[3], b[3]) <= max(a[1], b[1])


def test_bus_and_expression_placements_keep_native_aspect_ratio():
    # VI/MSL buses have square canvases; their artwork already supplies the wide shape.
    roots = (Path(__file__).parents[1] / 'BobLib', Path(__file__).parent / 'BobLibTest')
    for root in roots:
        for path in root.rglob('*.mo'):
            text = path.read_text()
            for declaration in re.finditer(
                    r'\b(?:[\w.]+Bus|Modelica\.Blocks\.Sources\.(?:(?:Real|Boolean|Integer)Expression|BooleanConstant|IntegerConstant|Constant|Step))\s+\w+[^;]*;', text):
                if 'Placement(' not in declaration[0]:
                    continue
                for extent in re.finditer(r'extent\s*=\s*\{\{([^}]+)\},\s*\{([^}]+)\}\}', declaration[0]):
                    x1, y1, x2, y2 = map(float, (extent[1] + ',' + extent[2]).split(','))
                    assert abs(x2 - x1) == pytest.approx(abs(y2 - y1)), (
                        f'{path}: stretched native icon: {declaration[0]}'
                    )


def test_world_occupies_bottom_left_diagram_cell():
    for path in FIXTURES + tuple(sorted((Path(__file__).parents[1] / 'BobLib').rglob('*.mo'))):
        text = path.read_text()
        for world in re.finditer(r'Modelica\.Mechanics\.MultiBody\.World\s+world\b[^;]*;', text):
            origin = re.search(r'origin = \{([^}]+)\}', world[0])
            if origin is None:  # Unplaced outer worlds have no diagram symbol.
                continue
            diagrams = list(re.finditer(r'Diagram\(coordinateSystem\(extent = \{\{([^}]+)\}, \{([^}]+)\}\}', text))
            assert diagrams, f'{path}: placed world needs explicit diagram bounds'
            corner = tuple(float(v) + 10 for v in diagrams[-1][1].split(','))
            actual = tuple(float(v) for v in origin[1].split(','))
            assert actual == corner, f'{path}: world must occupy the bottom-left-most 20-unit cell'
            assert 'extent = {{-10, -10}, {10, 10}}' in world[0], path
