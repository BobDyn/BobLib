import json

import ci_changes
import pytest


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
    assert out["smoke"] == "true"
    # The initialization suite runs in the smoke job, not the shards.
    assert out["component_tests"] == "Tests/test_fmu_events.py"


def test_nested_test_paths_are_not_test_modules():
    assert ci_changes.classify(["Tests/BobLibTest/test_fake.py"])["full"] == "true"


def test_deleted_test_modules_are_dropped(tmp_path):
    (tmp_path / "Tests").mkdir()
    (tmp_path / "Tests/test_kept.py").write_text("")
    out = ci_changes.classify(["Tests/test_kept.py", "Tests/test_gone.py"], tmp_path)
    assert out["component_tests"] == "Tests/test_kept.py"


def test_exclusions_match_the_makefile():
    makefile = (ci_changes.Path(__file__).resolve().parents[1] / "makefile").read_text()
    for test in ci_changes.NOT_IN_COMPONENT_SHARDS:
        assert test in makefile


def test_main_prints_github_outputs(capsys, monkeypatch, tmp_path):
    monkeypatch.setattr("sys.stdin", __import__("io").StringIO("README.md\n"))
    assert ci_changes.main(["--root", str(tmp_path)]) == 0
    assert "components=false" in capsys.readouterr().out
    assert ci_changes.main(["--full"]) == 0
    assert "full=true" in capsys.readouterr().out
