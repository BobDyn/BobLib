import re
from pathlib import Path

from modelica_parameters import library_files, masked, parameters

ROOT = Path(__file__).resolve().parents[1]


def test_parameter_reader_handles_tables_comments_and_nested_strings():
    source = '''model Example
  // parameter Real ignored;
  parameter Real table[:,2]=[0,1;2,3] "Time and value" annotation(Dialog(group="Curve"));
  parameter String fileName="input;data";
  parameter Real x(unit="m")=1 "Position";
  final parameter Boolean flag=true "Derived flag";
  parameter Real[3] vector=zeros(3) "Vector parameter";
end Example;'''
    rows = list(parameters(source))
    assert [r['name'] for r in rows] == ['table', 'fileName', 'x', 'flag', 'vector']
    assert rows[0]['description'] == '"Time and value"'
    assert rows[1]['description'] is None
    assert rows[2]['description'] == '"Position"'
    assert rows[3]['final']
    assert source[rows[0]['end']-1] == ';'


def test_every_parameter_has_a_description():
    missing = []
    for path in library_files(ROOT):
        source = path.read_text()
        rows = list(parameters(source))
        assert len(rows) == len(re.findall(r'\bparameter\s+', masked(source))), path
        for row in rows:
            if not row['description'] or not row['description'].strip('" '):
                missing.append(f"{path.relative_to(ROOT)}:{row['name']}")
    assert not missing, '\n'.join(missing)
