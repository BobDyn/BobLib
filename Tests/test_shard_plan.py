"""The CI shards split the work evenly and never drop or repeat a test."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

import shard_plan

DURATIONS = {"a.py": 10.0, "b.py": 9.0, "c.py": 5.0, "d.py": 4.0, "e.py": 1.0}


def loads(shards: list[set[str]], durations: dict[str, float]) -> list[float]:
    return [sum(durations[u] for u in shard) for shard in shards]


def test_heaviest_units_go_to_different_shards() -> None:
    shards = shard_plan.plan(DURATIONS, 2, DURATIONS)
    assert loads(shards, DURATIONS) == [15.0, 14.0]
    assert not {"a.py", "b.py"} <= shards[0] and not {"a.py", "b.py"} <= shards[1]


def test_every_unit_lands_in_exactly_one_shard() -> None:
    for count in (1, 2, 3, 4, 7):
        shards = shard_plan.plan(DURATIONS, count, DURATIONS)
        assert len(shards) == count
        assert sorted(u for shard in shards for u in shard) == sorted(DURATIONS)


def test_instant_units_spread_across_shards() -> None:
    durations = {"slow.py": 5.0, "x.py": 0.0, "y.py": 0.0, "z.py": 0.0}
    shards = shard_plan.plan(durations, 3, durations)
    assert [len(shard) for shard in shards] == [1, 2, 1]


def test_the_plan_does_not_depend_on_collection_order() -> None:
    forward = shard_plan.plan(list(DURATIONS), 3, DURATIONS)
    backward = shard_plan.plan(list(reversed(DURATIONS)), 3, DURATIONS)
    assert forward == backward


def test_unrecorded_units_count_as_the_median_of_their_kind() -> None:
    durations = {"a.py": 10.0, "b.py": 2.0, "c.py": 4.0, "p.py::t[1]": 30.0, "p.py::t[2]": 50.0}
    w = shard_plan.weights(["new.py", "p.py::t[3]"], durations)
    assert w == {"new.py": 4.0, "p.py::t[3]": 40.0}
    assert shard_plan.weights(["x.py"], {}) == {"x.py": 1.0}


def _junit(path: Path, cases: list[tuple[str, str, float]]) -> Path:
    body = "".join(f'<testcase classname="{c}" name="{n}" time="{t}"/>' for c, n, t in cases)
    path.write_text(f'<testsuites><testsuite name="pytest">{body}</testsuite></testsuites>')
    return path


@pytest.fixture
def fake_repo(tmp_path: Path) -> Path:
    tests = tmp_path / "Tests"
    tests.mkdir()
    (tests / "test_whole.py").write_text("def test_x(): pass\n")
    (tests / "test_split.py").write_text("BOBLIB_SHARD_BY_TEST = True\n")
    return tmp_path


def test_junit_reports_become_file_and_test_units(fake_repo: Path) -> None:
    report = _junit(
        fake_repo / "r.xml",
        [
            ("Tests.test_whole", "test_x", 3.0),
            ("Tests.test_whole.TestGroup", "test_y", 4.5),
            ("Tests.test_split", "test_case[one]", 7.0),
            ("Tests.test_gone", "test_z", 99.0),
        ],
    )
    assert shard_plan.measured([report], fake_repo) == {
        "Tests/test_whole.py": 7.5,
        "Tests/test_split.py::test_case[one]": 7.0,
    }


def test_refresh_keeps_old_entries_and_drops_deleted_files(fake_repo: Path) -> None:
    durations = fake_repo / "durations.json"
    durations.write_text(json.dumps({"Tests/test_split.py::test_old": 2.0, "Tests/test_deleted.py": 50.0}))
    report = _junit(fake_repo / "r.xml", [("Tests.test_whole", "test_x", 3.04)])
    merged = shard_plan.refresh([report], durations, fake_repo)
    assert merged == {"Tests/test_split.py::test_old": 2.0, "Tests/test_whole.py": 3.0}
    assert json.loads(durations.read_text()) == merged


def test_checked_in_durations_name_real_tests_and_balance_four_shards() -> None:
    durations = shard_plan.load_durations()
    assert durations, "Tests/shard_durations.json is missing or empty"
    for unit in durations:
        assert (shard_plan.ROOT / unit.split("::")[0]).is_file(), f"{unit} names a test file that is gone"
    per_shard = loads(shard_plan.plan(durations, 4, durations), durations)
    floor = max(sum(durations.values()) / 4, max(durations.values()))
    assert max(per_shard) <= 1.1 * floor, f"shards {per_shard} are unbalanced"


CHEAP_FILES = [
    "Tests/test_modelica_linter.py",
    "Tests/test_modelica_test_library.py",
    "Tests/test_native_artifact.py",
    "Tests/test_fmu_export.py",
    "Tests/test_modelica_fmu_source_edits.py",
]


def _collect(*extra: str) -> set[str]:
    out = subprocess.run(
        [sys.executable, "-m", "pytest", "--collect-only", "-q", "-p", "no:cacheprovider", *extra, *CHEAP_FILES],
        cwd=shard_plan.ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    return {line for line in out.splitlines() if "::" in line}


def test_conftest_shards_partition_the_collection() -> None:
    everything = _collect()
    shards = [_collect("--boblib-shard", f"{i}/3") for i in (1, 2, 3)]
    assert set().union(*shards) == everything
    assert sum(len(s) for s in shards) == len(everything), "a test ran in two shards"
    assert all(shards), "a shard got no tests"
