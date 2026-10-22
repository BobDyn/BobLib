"""Rank local route geometry for review; this is not rendered visual acceptance."""
import argparse
import json
import re
from pathlib import Path

from modelica_linter import _matching_paren_index, _split_top_level_commas


def masked_source(text):
    """Keep offsets while excluding comments and strings from syntax searches."""
    return re.sub(
        r'"(?:\\.|[^"\\])*"|//[^\n]*|/\*[\s\S]*?\*/',
        lambda match: re.sub(r'[^\n]', ' ', match[0]), text,
    )


def visibility(text, kind):
    match = re.search(r'\b' + kind + r'\s*\(', text)
    if match:
        end = _matching_paren_index(text, match.end()-1)
        if end is not None:
            for part in _split_top_level_commas(text[match.end():end]):
                setting = re.match(r'\s*visible\s*=\s*(.*)', part, re.S)
                if setting:
                    return setting[1].strip()
    return 'true'


def complementary(a, b):
    def normalize(value):
        value = value.strip()
        while value.startswith('(') and _matching_paren_index(value, 0) == len(value)-1:
            value = value[1:-1].strip()
        return re.sub(r'\s+', '', value)
    for positive, negative in [(a, b), (b, a)]:
        match = re.match(r'not\b(.*)', negative.strip(), re.S)
        if match:
            operand = match[1].strip()
            whole_operand = re.fullmatch(r'[\w.]+', operand) or (
                operand.startswith('(') and _matching_paren_index(operand, 0) == len(operand)-1)
            if whole_operand and normalize(positive) == normalize(operand):
                return True
    return False


def _placed_boxes(text):
    """Local symbol rectangles, grouped by their containing Modelica class."""
    import math

    classes = []
    for match in re.finditer(r'^\s*(?:partial\s+)?(?:model|block)\s+(\w+)\b', text, re.MULTILINE):
        closing = re.search(r'\bend\s+' + re.escape(match[1]) + r'\s*;', text[match.end():])
        if closing:
            classes.append((match.start(), match.end() + closing.end(), match[1]))
    for match in re.finditer(r'\bPlacement\s*\(', text):
        end = _matching_paren_index(text, match.end() - 1)
        assert end is not None
        placement = text[match.end():end]
        transform = re.search(r'\btransformation\s*\(', placement)
        assert transform is not None
        finish = _matching_paren_index(placement, transform.end() - 1)
        assert finish is not None
        transform = placement[transform.end():finish]
        extent = re.search(r'\bextent\s*=\s*\{\s*\{([^}]+)\},\s*\{([^}]+)\}\s*\}', transform)
        assert extent is not None
        x1, y1, x2, y2 = (float(value) for value in (extent[1] + ',' + extent[2]).split(','))
        origin = re.search(r'\borigin\s*=\s*\{([^}]+)\}', transform)
        ox, oy = (float(value) for value in origin[1].split(',')) if origin else (0, 0)
        rotation = re.search(r'\brotation\s*=\s*([-\d.]+)', transform)
        angle = math.radians(float(rotation[1])) if rotation else 0
        corners = [(ox + x*math.cos(angle) - y*math.sin(angle),
                    oy + x*math.sin(angle) + y*math.cos(angle))
                   for x in (x1, x2) for y in (y1, y2)]
        scope = max((entry for entry in classes if entry[0] < match.start() < entry[1]),
                    default=(0, len(text), 'root'))[2]
        box = (min(x for x, y in corners), min(y for x, y in corners),
               max(x for x, y in corners), max(y for x, y in corners))
        yield scope, text.count('\n', 0, match.start()) + 1, box


def crossings(a, b):
    """Strict interior intersections; shared endpoints/trunk branches are allowed."""
    points = set()
    for p, q in zip(a, a[1:], strict=False):
        for r, s in zip(b, b[1:], strict=False):
            if p == q or r == s:
                continue
            h, v = ((p, q), (r, s)) if p[1] == q[1] else ((r, s), (p, q))
            if h[0][1] != h[1][1] or v[0][0] != v[1][0]:
                continue
            x, y = v[0][0], h[0][1]
            if min(h[0][0], h[1][0]) < x < max(h[0][0], h[1][0]) and \
                    min(v[0][1], v[1][1]) < y < max(v[0][1], v[1][1]):
                points.add((x, y))
    return sorted(points)


def crosses_box(points, box):
    x1, y1, x2, y2 = box
    for a, b in zip(points, points[1:], strict=False):
        if a[0] == b[0] and x1 < a[0] < x2 and max(min(a[1], b[1]), y1) < min(max(a[1], b[1]), y2):
            return True
        if a[1] == b[1] and y1 < a[1] < y2 and max(min(a[0], b[0]), x1) < min(max(a[0], b[0]), x2):
            return True
    return False


def audit(text):
    source = masked_source(text)
    classes = []
    for match in re.finditer(r'\b(?:model|block)\s+(\w+)', source):
        closing = re.search(r'\bend\s+' + re.escape(match[1]) + r'\s*;', source[match.end():])
        if closing:
            classes.append((match.start(), match.end() + closing.end(), match[1]))
    routes, unresolved = [], []
    for match in re.finditer(r'\bconnect\s*\(', source):
        line = text.count('\n', 0, match.start()) + 1
        end = _matching_paren_index(source, match.end() - 1)
        if end is None:
            unresolved.append({'line': line, 'reason': 'unparsed connect'})
            continue
        finish = source.find(';', end)
        statement = source[end:finish]
        args = _split_top_level_commas(source[match.end():end])
        points = re.search(r'\bpoints\s*=\s*\{\{(.*?)\}\}', statement, re.S)
        if len(args) != 2 or not points:
            unresolved.append({'line': line, 'reason': 'unparsed route'})
            continue
        try:
            vertices = [tuple(map(float, pair.split(',')))
                        for pair in re.split(r'\}\s*,\s*\{', points[1])]
            if len(vertices) < 2 or any(len(p) != 2 for p in vertices):
                raise ValueError
        except ValueError:
            unresolved.append({'line': line, 'reason': 'nonliteral route'})
            continue
        if re.search(r'\bvisible\s*=\s*false\b', statement):
            continue
        conditional = bool(re.search(r'\bvisible\s*=', statement)) and not bool(
            re.search(r'\bvisible\s*=\s*true\b', statement))
        if conditional:
            unresolved.append({'line': line, 'reason': 'evaluate visibility per variant'})
        scope = max((c for c in classes if c[0] < match.start() < c[1]),
                    default=(0, len(text), 'root'))
        routes.append({'line': line, 'scope': scope[0], 'class': scope[2],
                       'connectors': [a.strip() for a in args], 'points': vertices,
                       'conditional': conditional, 'visibility': visibility(statement, 'Line')})
    findings, duplicates = [], []
    for index, route in enumerate(routes):
        for other in routes[index + 1:]:
            if route['scope'] != other['scope']:
                continue
            if complementary(route['visibility'], other['visibility']):
                continue
            if sorted(route['connectors']) == sorted(other['connectors']):
                duplicates.append({'lines': [route['line'], other['line']],
                                   'class': route['class'], 'connectors': route['connectors']})
            for point in crossings(route['points'], other['points']):
                findings.append({'lines': [route['line'], other['line']],
                                 'class': route['class'], 'point': point,
                                 'conditional': route['conditional'] or other['conditional']})
    symbols = []
    for scope, line, box in _placed_boxes(source):
        offset = sum(len(part) for part in source.splitlines(keepends=True)[:line-1])
        offset = source.find('Placement', offset)
        header = source[source.rfind(';', 0, offset)+1:offset].split('annotation')[0]
        name = re.search(r'(?:^|\n)\s*(?:(?:redeclare|replaceable|inner|outer|final)\s+)*'
                         r'[\w.]+\s+(\w+)\s*(?=\(|if\b|$)', header)
        symbols.append((scope, line, box, name[1] if name else None,
                        visibility(source[offset:], 'Placement')))
    symbol_crossings, symbol_overlaps, exclusive_overlays = [], [], []
    for scope, line, box, name, visible in symbols:
        for route in routes:
            if scope != route['class'] or any(
                    endpoint.split('.')[0].split('[')[0] == name for endpoint in route['connectors']):
                continue
            if visible == 'false' or complementary(visible, route['visibility']):
                continue
            if crosses_box(route['points'], box):
                symbol_crossings.append({'route_line': route['line'], 'symbol_line': line,
                                         'symbol': name, 'class': scope})
    for i, (scope, line, box, name, visible) in enumerate(symbols):
        for other_scope, other_line, other, other_name, other_visible in symbols[i+1:]:
            if scope == other_scope and min(box[2], other[2])-max(box[0], other[0]) > 1e-6 and \
                    min(box[3], other[3])-max(box[1], other[1]) > 1e-6:
                destination = exclusive_overlays if complementary(visible, other_visible) else symbol_overlaps
                destination.append({'lines': [line, other_line], 'class': scope,
                                    'symbols': [name, other_name], 'visibility': [visible, other_visible]})
    bends = 0
    for route in routes:
        points = route['points']
        directions = [(q[0] - p[0], q[1] - p[1])
                      for p, q in zip(points, points[1:], strict=False) if p != q]
        turns = sum(a[0] * b[1] != a[1] * b[0]
                    for a, b in zip(directions, directions[1:], strict=False))
        minimum = int(points[0][0] != points[-1][0] and points[0][1] != points[-1][1])
        bends += max(0, turns - minimum)
    # shortcut: local literal routes only; resolve inheritance/variants before enforcing release acceptance.
    return {'routes': len(routes), 'crossings': findings, 'duplicate_connections': duplicates,
            'extra_bends': bends,
            'symbol_crossings': symbol_crossings, 'symbol_overlaps': symbol_overlaps,
            'exclusive_overlays': exclusive_overlays,
            'review_score': max(0, 100 - 10 * (len(findings) + len(symbol_crossings)) -
                                20 * (len(symbol_overlaps) + len(duplicates)) - bends) if routes or symbols else None,
            'unresolved': unresolved, 'visual_acceptance': 'not established'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('roots', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    files = sorted({p for root in args.roots for p in root.rglob('*.mo')})
    rows = [{'path': str(path), **audit(path.read_text())} for path in files]
    rows.sort(key=lambda row: (row['review_score'] if row['review_score'] is not None else 101,
                              row['path']))
    args.output.write_text(json.dumps({
        'scope': 'Local literal routes and symbol rectangles only. Inheritance, label collisions, '
                 'conditional equations and actual rendering require separate review. '
                 'Symbol findings include inactive symbols; evaluate each variant. '
                 'Score is triage, not a release threshold.',
        'scoring': 'max(0, 100 - 10*(crossings + symbol_crossings) '
                   '- 20*(symbol_overlaps + duplicate_connections) - extra_bends); '
                   'null when no local routes or symbols. '
                   'Extra bends may be necessary to avoid symbols. Crossing candidates '
                   'must be evaluated in their active variants.',
        'files': rows,
    }, indent=2) + '\n')


if __name__ == '__main__':
    main()
