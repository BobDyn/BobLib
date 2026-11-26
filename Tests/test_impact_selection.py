import json
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from impact_selection import dependencies, select


def item(root, module="test_modelica_components.py", function="test_modelica_component_simulates", model="BobLibTest.TestA", markers=()):
    return SimpleNamespace(
        path=root / "Tests" / module, name=function, originalname=function,
        nodeid=f"Tests/{module}::{function}[{model}]", fixturenames=[],
        callspec=SimpleNamespace(params={"case": {"test": model}}),
        iter_markers=lambda name: iter(SimpleNamespace(args=args) for args in markers),
    )


def plan(**changes):
    return {"schema": 1, "mode": "impact", "reason": "test", "changed_tests": [],
            "affected_models": ["BobLibTest.TestA"],
            "known_models": ["BobLibTest.TestA", "BobLibTest.TestB"], **changes}


def test_filters_known_unaffected_models_but_keeps_unknown_tests_and_models(tmp_path):
    cases = [item(tmp_path), item(tmp_path, model="BobLibTest.TestB"),
             item(tmp_path, model="GeneratedModel"), item(tmp_path, function="test_unknown")]
    selected, decisions = select(cases, plan(), tmp_path)
    assert selected == [cases[0], cases[2], cases[3]]
    assert [d["selected"] for d in decisions] == [True, False, True, True]


def test_changed_test_module_always_runs_every_case(tmp_path):
    case = item(tmp_path, model="BobLibTest.TestB")
    for mode in ("impact", "tests"):
        selected, _ = select([case], plan(mode=mode, changed_tests=["Tests/test_modelica_components.py"]), tmp_path)
        assert selected == [case]


def test_full_fallback_keeps_everything(tmp_path):
    cases = [item(tmp_path, model="BobLibTest.TestB"), item(tmp_path, markers=[()])]
    assert select(cases, plan(mode="full"), tmp_path)[0] == cases


def test_markers_are_complete_contracts_and_combine_inherited_marks(tmp_path):
    case = item(tmp_path, markers=[("BobLibTest.TestB",), ("BobLibTest.TestA",)])
    assert dependencies(case) == {"BobLibTest.TestA", "BobLibTest.TestB"}
    assert select([case], plan(), tmp_path)[0] == [case]
    independent = item(tmp_path, markers=[()])
    assert select([independent], plan(), tmp_path)[0] == []


def test_fmu_fixture_dependencies_are_distinct(tmp_path):
    case = item(tmp_path, module="test_modelica_fmu_export.py", function="test_vehicle")
    case.fixturenames = ["vehicle_fmu"]
    assert dependencies(case) == {"BobLib.Experiments.Standards.VehicleFMI"}
    case.fixturenames = ["reset_probe_fmu"]
    assert dependencies(case) == set()
    case.fixturenames = []
    assert dependencies(case) is None


@pytest.mark.parametrize("module,function,key,value", [
    ("test_modelica_translation.py", "test_modelica_model_translates", "model_check", SimpleNamespace(name="BobLibTest.TestA")),
    ("test_modelica_initialization.py", "test_modelica_model_initializes_to_baseline", "model", "BobLibTest.TestA"),
    ("test_modelica_physics_validation.py", "test_modelica_physical_baseline_matches", "baseline", SimpleNamespace(model="BobLibTest.TestA")),
])
def test_parameter_contracts(module, function, key, value, tmp_path):
    case = item(tmp_path, module=module, function=function)
    case.callspec.params = {key: value}
    assert dependencies(case) == {"BobLibTest.TestA"}


def test_real_pytest_filter_precedes_shards_and_empty_selection_succeeds(tmp_path):
    import shutil

    tests = tmp_path / "Tests"
    tests.mkdir()
    source = Path(__file__).parent
    for name in ("conftest.py", "impact_selection.py", "ci_changes.py", "modelica_impact.py"):
        shutil.copy2(source / name, tests / name)
    (tests / "test_cases.py").write_text('''import pytest
@pytest.mark.modelica_models("BobLibTest.TestA")
@pytest.mark.parametrize("number", range(4))
def test_affected(number):
    assert number >= 0
@pytest.mark.modelica_models("BobLibTest.TestB")
def test_unaffected():
    assert False, "unaffected test executed"
''')
    (tmp_path / "BobLib").mkdir()
    (tmp_path / "BobLib/A.mo").write_text("model A end A;")
    from ci_changes import source_digest
    from modelica_impact import working_sources

    manifest = plan(source_digest=source_digest(working_sources(tmp_path)))
    path = tmp_path / "plan.json"
    path.write_text(json.dumps(manifest))
    env = {k: v for k, v in os.environ.items() if k not in {"BOBLIB_IMPACT_PLAN", "BOBLIB_SHARD"}}
    for shard, expected in (("1/2", "2 passed"), ("2/2", "2 passed"), ("5/5", "5 deselected")):
        result = subprocess.run(
            [sys.executable, "-m", "pytest", "-q", "Tests", "--boblib-impact-plan", str(path),
             "--boblib-shard", shard], cwd=tmp_path, env=env, text=True, capture_output=True,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        assert expected in result.stdout
    # The xdist controller does not run collection hooks itself. Empty shards
    # must still succeed, while real failures must not be converted to passes.
    pytest.importorskip("xdist")
    result = subprocess.run(
        [sys.executable, "-m", "pytest", "-q", "Tests", "--boblib-impact-plan", str(path),
         "--boblib-shard", "5/5", "-n", "2", "--dist", "loadgroup"],
        cwd=tmp_path, env=env, text=True, capture_output=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    # A selected test failure must retain pytest's failure exit code.
    cases_file = tests / "test_cases.py"
    cases_file.write_text(cases_file.read_text().replace("assert number >= 0", "assert False"))
    result = subprocess.run(
        [sys.executable, "-m", "pytest", "-q", "Tests", "--boblib-impact-plan", str(path)],
        cwd=tmp_path, env=env, text=True, capture_output=True,
    )
    assert result.returncode == 1, result.stdout + result.stderr
    # A stale plan must fail rather than silently omit newly affected models.
    (tmp_path / "BobLib/A.mo").write_text("model A Real x; end A;")
    result = subprocess.run([sys.executable, "-m", "pytest", "-q", "Tests", "--boblib-impact-plan", str(path)],
                            cwd=tmp_path, env=env, text=True, capture_output=True)
    assert result.returncode != 0
    assert "sources changed" in result.stderr
