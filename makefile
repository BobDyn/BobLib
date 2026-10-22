PYTHON ?= python3
PYTEST ?= $(PYTHON) -m pytest
PYTEST_FLAGS ?=
PACKAGE_ROOT ?= BobLib
MODELICA_INIT_TIMEOUT_S ?= 600
MODELICA_LINT_PATHS ?= BobLib Tests/BobLibTest
MODELICA_FMU_OUTPUT_DIR ?= build
MODELICA_FMU_TIMEOUT_S ?= 900

PYTHON_TESTS := \
	Tests/test_modelica_linter.py \
	Tests/test_modelica_fmu_source_edits.py \
	Tests/test_fmu_export.py \
	Tests/test_modelica_fmu_export_timeout.py \
	Tests/test_modelica_test_library.py

MODELICA_REGRESSION_TESTS := \
	Tests/test_modelica_regression.py \
	Tests/test_boblib_modelica.py

MODELICA_TRANSLATION_TESTS := \
	Tests/test_modelica_translation.py

MODELICA_INITIALIZATION_TESTS := \
	Tests/test_modelica_initialization.py

MODELICA_PHYSICS_TESTS := \
	Tests/test_modelica_physics_validation.py

MODELICA_FMU_TESTS := Tests/test_modelica_fmu_export.py

MODELICA_SMOKE_FLAGS ?= --no-test-fixtures --no-equation-counts

ALL_TESTS := Tests

# Keep the release steps disjoint while including newly added test modules.
ADDITIONAL_TESTS := $(filter-out \
	$(MODELICA_TRANSLATION_TESTS) $(MODELICA_INITIALIZATION_TESTS) \
	$(MODELICA_PHYSICS_TESTS) $(MODELICA_REGRESSION_TESTS) $(MODELICA_FMU_TESTS), \
	$(wildcard Tests/test_*.py))

.PHONY: help lint modelica-lint modelica-format test test-python test-pytest test-additional test-modelica modelica-deps modelica-smoke modelica-translation modelica-initialization modelica-physics modelica-regression modelica-fmu modelica-fmu-export ci check

help:
	@printf '%s\n' \
		'BobLib development targets:' \
		'  make lint                 Run Python lint checks' \
		'  make modelica-lint        Check Modelica formatting without rewriting files' \
		'  make modelica-format      Rewrite Modelica formatting in MODELICA_LINT_PATHS' \
		'  make test                 Run the full local pytest gate' \
		'  make test-python          Run pure Python pytest checks only' \
		'  make test-pytest          Run every pytest-collected check in one invocation' \
		'  make test-additional      Run component, contract and Python tests' \
		'  make modelica-deps        Install OpenModelica library dependencies' \
		'  make modelica-smoke       Translate default standards/regression models only' \
		'  make modelica-translation Translate standards and all BobLibTest fixtures via pytest' \
		'  make modelica-initialization Initialize calibrated fixtures and compare baselines' \
		'      MODELICA_INIT_TIMEOUT_S=600 controls the per-model OMC timeout' \
		'  make modelica-coverage    Check every model has a mirrored fixture' \
		'  make modelica-profile     Run mirrored fixtures and collect package timings' \
		'  make modelica-physics     Simulate physical validation baselines with scaled runtime budgets' \
		'  make modelica-regression  Simulate signal-level regressions and smoke-check BobLib/BobLibTest' \
		'  make modelica-fmu         Export and drive the FMUs' \
		'  make modelica-fmu-export  Export VehicleFMI as an FMU' \
		'  make test-modelica        Run all Modelica checks' \
		'  make ci                   Run the full local CI suite'

lint:
	$(PYTHON) -m ruff check Tests Tools

modelica-lint:
	$(PYTHON) Tests/modelica_linter.py check $(MODELICA_LINT_PATHS)

modelica-format:
	$(PYTHON) Tests/modelica_linter.py format $(MODELICA_LINT_PATHS)

test: test-pytest

test-python:
	$(PYTEST) $(PYTEST_FLAGS) $(PYTHON_TESTS)

modelica-deps:
	omc msl_setup.mos

modelica-smoke: modelica-deps
	$(PYTHON) Tests/modelica_translation_checks.py --package-root $(PACKAGE_ROOT) $(MODELICA_SMOKE_FLAGS)

test-pytest: modelica-deps
	$(PYTEST) $(PYTEST_FLAGS) $(ALL_TESTS) --boblib-package-root $(PACKAGE_ROOT) --boblib-initialization-timeout-s $(MODELICA_INIT_TIMEOUT_S) --boblib-fmu-timeout-s $(MODELICA_FMU_TIMEOUT_S)

test-additional: modelica-deps
	$(PYTEST) $(PYTEST_FLAGS) $(ADDITIONAL_TESTS) --boblib-package-root $(PACKAGE_ROOT) --boblib-initialization-timeout-s $(MODELICA_INIT_TIMEOUT_S)

modelica-translation: modelica-deps
	$(PYTEST) $(PYTEST_FLAGS) $(MODELICA_TRANSLATION_TESTS) --boblib-package-root $(PACKAGE_ROOT)

modelica-initialization: modelica-deps
	$(PYTEST) $(PYTEST_FLAGS) $(MODELICA_INITIALIZATION_TESTS) --boblib-package-root $(PACKAGE_ROOT) --boblib-initialization-timeout-s $(MODELICA_INIT_TIMEOUT_S)

modelica-physics: modelica-deps
	$(PYTEST) $(PYTEST_FLAGS) $(MODELICA_PHYSICS_TESTS) --boblib-package-root $(PACKAGE_ROOT)

modelica-regression: modelica-deps
	$(PYTEST) $(PYTEST_FLAGS) $(MODELICA_REGRESSION_TESTS)

modelica-fmu: modelica-deps
	$(PYTEST) $(PYTEST_FLAGS) $(MODELICA_FMU_TESTS) --boblib-package-root $(PACKAGE_ROOT) --boblib-fmu-timeout-s $(MODELICA_FMU_TIMEOUT_S)

modelica-fmu-export: modelica-deps
	$(PYTHON) Tests/modelica_fmu_export.py --package-root $(PACKAGE_ROOT) --output-dir $(MODELICA_FMU_OUTPUT_DIR) --timeout-s $(MODELICA_FMU_TIMEOUT_S)

test-modelica: modelica-fmu modelica-translation modelica-initialization modelica-physics modelica-regression modelica-components

ci: lint modelica-lint modelica-coverage test

check: ci

# PROFILE_DIR must be new; override it to preserve successive measurements.
PROFILE_DIR ?= /tmp/boblib-profile
.PHONY: modelica-coverage modelica-profile
modelica-coverage:
	$(PYTHON) Tests/modelica_test_library.py

modelica-profile:
	$(PYTHON) Tests/modelica_test_library.py --output $(PROFILE_DIR)

.PHONY: modelica-components
modelica-components: modelica-deps
	$(PYTEST) $(PYTEST_FLAGS) Tests/test_modelica_components.py
