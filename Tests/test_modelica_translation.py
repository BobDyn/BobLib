from __future__ import annotations

import shutil
from pathlib import Path

import pytest

import modelica_translation_checks as translation_checks


def _configured_package_root(config: pytest.Config) -> Path:
    configured = config.getoption("--boblib-package-root")
    if configured:
        package_root = Path(configured)
        if not package_root.is_absolute():
            package_root = Path.cwd() / package_root
        return package_root.resolve()
    return translation_checks.default_package_root().resolve()


def _model_checks(package_root: Path) -> tuple[translation_checks.ModelCheck, ...]:
    checks = list(translation_checks.DEFAULT_CHECKS)
    checks.extend(translation_checks.test_fixture_checks(package_root))

    unique_checks: dict[str, translation_checks.ModelCheck] = {}
    for check in checks:
        unique_checks[check.name] = check
    return tuple(unique_checks.values())


def pytest_generate_tests(metafunc: pytest.Metafunc) -> None:
    if "model_check" not in metafunc.fixturenames:
        return

    package_root = _configured_package_root(metafunc.config)
    checks = _model_checks(package_root)
    metafunc.parametrize("model_check", checks, ids=[check.name for check in checks])


def test_modelica_model_translates(
    model_check: translation_checks.ModelCheck,
    pytestconfig: pytest.Config,
) -> None:
    if shutil.which("omc") is None:
        pytest.skip("OpenModelica omc is not installed")

    package_root = _configured_package_root(pytestconfig)
    translation_checks.run_check(
        package_root,
        model_check,
        enforce_counts=True,
    )
