from __future__ import annotations

import shutil
from pathlib import Path

import pytest

import modelica_initialization_checks as initialization_checks


def _configured_package_root(config: pytest.Config) -> Path:
    configured = config.getoption("--boblib-package-root")
    if configured:
        package_root = Path(configured)
        if not package_root.is_absolute():
            package_root = Path.cwd() / package_root
        return package_root.resolve()
    return initialization_checks.default_package_root().resolve()


def _baseline_path(config: pytest.Config) -> Path:
    configured = config.getoption("--boblib-initialization-baseline")
    if configured:
        path = Path(configured)
        if not path.is_absolute():
            path = Path.cwd() / path
        return path.resolve()
    return Path(__file__).with_name("modelica_initialization_baseline.csv")


def _timeout_s(config: pytest.Config) -> int:
    configured = config.getoption("--boblib-initialization-timeout-s")
    if configured is None:
        return initialization_checks.DEFAULT_TIMEOUT_S
    return int(configured)


def _fixture_models(package_root: Path) -> tuple[str, ...]:
    return initialization_checks.test_fixture_models(package_root)


def pytest_generate_tests(metafunc: pytest.Metafunc) -> None:
    if "model" not in metafunc.fixturenames:
        return

    package_root = _configured_package_root(metafunc.config)
    models = _fixture_models(package_root)
    metafunc.parametrize("model", models, ids=models)


@pytest.fixture(scope="session")
def initialization_baseline(
    pytestconfig: pytest.Config,
) -> dict[str, initialization_checks.InitializationMetrics]:
    return initialization_checks.read_baseline(_baseline_path(pytestconfig))


def test_modelica_initialization_baseline_tracks_fixture_models(
    pytestconfig: pytest.Config,
    initialization_baseline: dict[str, initialization_checks.InitializationMetrics],
) -> None:
    package_root = _configured_package_root(pytestconfig)
    models = set(_fixture_models(package_root))
    baseline_models = set(initialization_baseline)

    missing = sorted(models - baseline_models)
    stale = sorted(baseline_models - models)
    assert missing == []
    assert stale == []


def test_modelica_model_initializes_to_baseline(
    model: str,
    pytestconfig: pytest.Config,
    initialization_baseline: dict[str, initialization_checks.InitializationMetrics],
) -> None:
    if shutil.which("omc") is None:
        pytest.skip("OpenModelica omc is not installed")

    package_root = _configured_package_root(pytestconfig)
    actual = initialization_checks.initialize_model(
        package_root,
        model,
        timeout_s=_timeout_s(pytestconfig),
    )

    expected = initialization_baseline[model]
    initialization_checks.assert_matches_baseline(
        actual,
        expected,
        rtol=1e-9,
        atol=1e-8,
    )
