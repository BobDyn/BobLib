import json

import pytest

import ci_changes


def test_modelica_change_runs_everything():
    out = ci_changes.classify(["BobLib/Chassis/Body/Frame.mo", "README.md"])
    assert out["full"] == "true"
    assert json.loads(out["shards"]) == [1, 2, 3, 4]
    assert out["component_tests"] == ""


@pytest.mark.parametrize(
    "path",
    [
        "msl_setup.mos",
        "makefile",
        ".github/workflows/ci.yml",
        "Tests/conftest.py",
        "Tests/modelica_parameters.py",
        "Tests/model_coverage.json",
        "Tests/modelica_initialization_baseline.csv",
        "Tests/requirements-dev.txt",
        "Tests/BobLibTest/package.mo",
        "Tools/export_fmu.py",
        "some/new/thing.txt",
    ],
)
def test_shared_inputs_and_unknown_paths_force_the_full_run(path):
    assert ci_changes.classify(["Tests/test_knc.py", path])["full"] == "true"


def test_empty_input_never_skips():
    assert ci_changes.classify([])["full"] == "true"
    assert ci_changes.classify(["", "  "])["full"] == "true"


def test_docs_only_skips_every_modelica_job():
    out = ci_changes.classify(["README.md", "BobLib/UsersGuide/notes.md", "LICENSE"])
    assert out == {
        "full": "false",
        "smoke": "false",
        "fmu": "false",
        "components": "false",
        "shards": "[]",
        "shard_count": "1",
        "component_tests": "",
        "extra_tests": "",
    }


def test_test_only_change_runs_just_those_tests_in_one_shard():
    out = ci_changes.classify(["Tests/test_knc.py", "Tests/test_rod_mass.py", "CHANGELOG.md"])
    assert out["full"] == "false"
    assert out["components"] == "true"
    assert json.loads(out["shards"]) == [1]
    assert out["component_tests"] == "Tests/test_knc.py Tests/test_rod_mass.py"
    assert out["smoke"] == "false"
    assert out["fmu"] == "false"


def test_fmu_and_initialization_tests_keep_their_jobs():
    out = ci_changes.classify(["Tests/test_fmu_events.py", "Tests/test_modelica_initialization.py"])
    assert out["fmu"] == "true"
    assert out["smoke"] == "false"
    assert out["component_tests"] == "Tests/test_fmu_events.py Tests/test_modelica_initialization.py"


def test_nested_test_paths_are_not_test_modules():
    assert ci_changes.classify(["Tests/BobLibTest/test_fake.py"])["full"] == "true"


def test_deleted_test_modules_are_dropped(tmp_path):
    (tmp_path / "Tests").mkdir()
    (tmp_path / "Tests/test_kept.py").write_text("")
    out = ci_changes.classify(["Tests/test_kept.py", "Tests/test_gone.py"], tmp_path)
    assert out["component_tests"] == "Tests/test_kept.py"


@pytest.mark.parametrize("name", [
    "test_modelica_translation.py", "test_modelica_physics_validation.py",
    "test_modelica_regression.py", "test_boblib_modelica.py", "test_modelica_initialization.py",
])
def test_formerly_excluded_suites_always_run_when_changed(name):
    out = ci_changes.classify(["Tests/" + name])
    assert out["components"] == "true"
    assert out["component_tests"] == "Tests/" + name



def test_main_prints_github_outputs(capsys, monkeypatch, tmp_path):
    monkeypatch.setattr("sys.stdin", __import__("io").StringIO("README.md\n"))
    assert ci_changes.main(["--root", str(tmp_path)]) == 0
    assert "components=false" in capsys.readouterr().out
    assert ci_changes.main(["--full"]) == 0
    assert "full=true" in capsys.readouterr().out


def test_graph_failure_falls_back_to_full_gate(tmp_path, monkeypatch):
    def unavailable(*args):
        raise OSError("history unavailable")
    monkeypatch.setattr(ci_changes, "revision_sources", unavailable)
    plan = ci_changes.make_plan(["BobLib/Thing.mo"], tmp_path, "base")
    assert plan["mode"] == "full"
    assert "history unavailable" in plan["reason"]


def test_mixed_model_and_test_changes_preserve_changed_suite(monkeypatch, tmp_path):
    source = {"BobLib/A.mo": "model A end A;"}
    monkeypatch.setattr(ci_changes, "working_sources", lambda root: source)
    monkeypatch.setattr(ci_changes, "revision_sources", lambda root, base: source)
    plan = ci_changes.make_plan(["BobLib/A.mo", "Tests/test_modelica_regression.py"], tmp_path, "base")
    assert plan["mode"] == "impact"
    assert plan["affected_models"] == ["BobLib.A"]
    assert ci_changes.job_outputs(plan)["extra_tests"] == "Tests/test_modelica_regression.py"


def test_full_fallback_also_runs_changed_suites_outside_components():
    out = ci_changes.classify(["makefile", "Tests/test_modelica_regression.py"])
    assert out["full"] == "true"
    assert out["extra_tests"] == "Tests/test_modelica_regression.py"


def test_workflow_distributes_one_plan_and_filters_every_pytest_job():
    # Keep this stdlib-only: the changes job deliberately needs no pip install.
    text = (ci_changes.Path(__file__).resolve().parents[1] / ".github/workflows/ci.yml").read_text()
    assert 'git merge-base "$BASE" "$HEAD"' in text
    assert "--plan impact-plan.json" in text
    assert text.count("BOBLIB_IMPACT_PLAN: ${{ github.workspace }}/impact-plan.json") == 3
    assert text.count("uses: actions/download-artifact@v4") == 3
    assert 'EXTRA_TESTS="$EXTRA_TESTS"' in text
    assert "make modelica-translation" in text
