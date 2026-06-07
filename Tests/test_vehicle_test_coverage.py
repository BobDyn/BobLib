from __future__ import annotations

import re
import tomllib
from collections import Counter
from pathlib import Path
from typing import Any


VALID_STATUSES = {"direct", "parent", "partial", "deferred"}
MAX_DEFERRED_MODELS = 0


def _package_root() -> Path:
    cwd = Path.cwd()
    candidates = (
        cwd / "BobLib",
        cwd / "_0_Utils/external/BobLib/BobLib",
        Path(__file__).resolve().parents[1] / "BobLib",
    )
    for candidate in candidates:
        if (candidate / "package.mo").is_file():
            return candidate.resolve()
    raise FileNotFoundError("Could not locate BobLib/package.mo")


def _class_path(package_root: Path, modelica_file: Path) -> str:
    rel = modelica_file.relative_to(package_root).with_suffix("")
    return ".".join((package_root.name, *rel.parts))


def _class_file(package_root: Path, class_path: str) -> Path:
    parts = class_path.split(".")
    if parts[0] != package_root.name:
        raise AssertionError(f"{class_path} is not in {package_root.name}")
    return package_root.joinpath(*parts[1:]).with_suffix(".mo")


def _modelica_classes(root: Path, package_root: Path) -> list[str]:
    return sorted(
        _class_path(package_root, path)
        for path in root.rglob("*.mo")
        if path.name != "package.mo"
    )


def _load_manifest(path: Path) -> list[dict[str, Any]]:
    with path.open("rb") as handle:
        data = tomllib.load(handle)

    entries = data.get("model_tests")
    if not isinstance(entries, list):
        raise AssertionError(f"{path} must define a model_tests array")
    return entries


def _assert_entry_shape(entry: dict[str, Any]) -> None:
    model = entry.get("model")
    status = entry.get("status")
    reason = entry.get("reason")
    tests = entry.get("tests", [])

    assert isinstance(model, str) and model.startswith("BobLib.Vehicle."), entry
    assert status in VALID_STATUSES, entry
    assert isinstance(reason, str) and reason.strip(), entry
    assert isinstance(tests, list), entry
    assert all(isinstance(test, str) for test in tests), entry

    if status == "deferred":
        assert not tests, f"{model} is deferred but still lists tests"
    else:
        assert tests, f"{model} is {status} but does not list covering tests"


def test_vehicle_test_coverage_manifest_tracks_every_vehicle_class() -> None:
    package_root = _package_root()
    vehicle_models = _modelica_classes(package_root / "Vehicle", package_root)
    test_models = set(_modelica_classes(package_root / "Tests", package_root))
    manifest_path = package_root / "Tests" / "vehicle_test_coverage.toml"
    entries = _load_manifest(manifest_path)

    for entry in entries:
        _assert_entry_shape(entry)

    manifest_models = [entry["model"] for entry in entries]
    duplicate_models = [
        model for model, count in Counter(manifest_models).items() if count > 1
    ]
    assert duplicate_models == []

    missing_models = sorted(set(vehicle_models) - set(manifest_models))
    unknown_models = sorted(set(manifest_models) - set(vehicle_models))
    assert missing_models == []
    assert unknown_models == []

    deferred_models = [
        entry["model"] for entry in entries if entry["status"] == "deferred"
    ]
    assert len(deferred_models) <= MAX_DEFERRED_MODELS

    referenced_tests = sorted(
        {
            test
            for entry in entries
            for test in entry.get("tests", [])
        }
    )
    unknown_tests = sorted(set(referenced_tests) - test_models)
    assert unknown_tests == []

    for test in referenced_tests:
        test_file = _class_file(package_root, test)
        text = test_file.read_text(encoding="utf-8")
        assert re.search(r"\bexperiment\s*\(", text), (
            f"{test} should declare an experiment annotation"
        )
