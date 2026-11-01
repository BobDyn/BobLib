import json

import modelica_test_library as library
import pytest


def test_catalog_covers_every_model_and_block():
    rows = library.coverage()
    assert rows
    assert len({r['test'] for r in rows}) == len(rows)


def test_coverage_rejects_unlisted_model(tmp_path):
    (tmp_path / 'BobLib').mkdir()
    (tmp_path / 'Tests').mkdir()
    (tmp_path / 'BobLib/NewModel.mo').write_text('encapsulated model NewModel\nend NewModel;')
    (tmp_path / 'Tests/model_coverage.json').write_text(json.dumps({'models': []}))
    with pytest.raises(ValueError, match='Coverage mismatch'):
        library.coverage(tmp_path)


def test_parser_requires_completed_simulation_and_all_phase_timings():
    text = 'resultFile = "fixture_res.mat"\nThe simulation finished successfully.\n'
    assert library.parse_result(text, 'simulate', 'Test')['status'] == 'failed'
    text += '\n'.join(f'{p} = 0.25' for p in library.PHASES)
    result = library.parse_result(text, 'simulate', 'Test')
    assert result['status'] == 'passed'
    assert result['timings_s']['timeCompile'] == 0.25
    early = library.parse_result(text + '\nSimulation call terminate() at time 0.15', 'simulate', 'Test')
    assert early['termination_time'] == 0.15
    assert library.parse_result(text.replace('0.25', '+nan'), 'simulate', 'Test')['status'] == 'failed'
    assert library.parse_result(text.replace('0.25', '1e999'), 'simulate', 'Test')['status'] == 'failed'
    assert library.parse_result(text + '\nError: assertion', 'simulate', 'Test')['status'] == 'failed'
    assert library.parse_result('Check of Test completed successfully.', 'check', 'Test') == {'status': 'passed'}


def test_rollups_exclude_failed_attempts_and_structural_checks():
    base = {'model': 'BobLib.Chassis.Body.Frame', 'scope': 'component', 'mode': 'simulate'}
    rows = [{**base, 'status': 'passed', 'timings_s': {p: 1.5 for p in library.PHASES}},
            {**base, 'status': 'failed', 'timings_s': {p: 100 for p in library.PHASES}},
            {**base, 'status': 'passed', 'mode': 'check', 'scope': 'structural'}]
    summary = library.rollups(rows)
    assert set(summary) == {'BobLib', 'BobLib.Chassis', 'BobLib.Chassis.Body'}
    assert summary['BobLib']['timings_s']['timeCompile'] == 1.5
    assert summary['BobLib']['simulated'] == 1
    assert summary['BobLib']['checked'] == 1
    assert summary['BobLib']['failed'] == 1


def test_package_orders_are_complete_and_unique():
    root = library.ROOT / 'Tests/BobLibTest'
    for package in root.rglob('package.mo'):
        directory = package.parent
        actual = (directory / 'package.order').read_text().splitlines()
        expected = {p.stem for p in directory.glob('*.mo') if p.name != 'package.mo'}
        expected |= {p.name for p in directory.iterdir() if p.is_dir() and (p / 'package.mo').exists()}
        assert set(actual) == expected, directory
        assert len(actual) == len(set(actual)), directory
        if directory != root:
            def group(name, directory=directory):
                if name in ('UsersGuide', 'Tutorial'):
                    return 0
                if name == 'Interfaces':
                    return 1
                if name == 'Internal':
                    return 5
                if name == 'BaseClasses':
                    return 3
                return 2 if (directory / name / 'package.mo').exists() else 4
            assert actual == sorted(actual, key=group), directory


def test_initialization_catalog_preserves_baselines():
    import csv

    import modelica_initialization_checks as initialization
    baseline = library.ROOT / 'Tests/modelica_initialization_baseline.csv'
    with baseline.open() as stream:
        names = {row['model'] for row in csv.DictReader(stream)}
    assert set(initialization.test_fixture_models(library.ROOT / 'BobLib')) == names
    for name in names:
        assert (library.ROOT / 'Tests' / (name.replace('.', '/') + '.mo')).exists()


def test_documentation_examples_do_not_count_as_models(tmp_path):
    (tmp_path / 'BobLib').mkdir()
    (tmp_path / 'Tests').mkdir()
    (tmp_path / 'BobLib/Guide.mo').write_text('class Guide\n  annotation(Documentation(info="<html>\nmodel Example\nend Example;\n</html>"));\nend Guide;')
    (tmp_path / 'Tests/model_coverage.json').write_text(json.dumps({'models': []}))
    assert library.coverage(tmp_path) == []
