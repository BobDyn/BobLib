PYTHON ?= python3
PYTEST ?= $(PYTHON) -m pytest
PYTEST_FLAGS ?=
PACKAGE_ROOT ?= BobLib
MODELICA_INIT_TIMEOUT_S ?= 600

PYTHON_TESTS := \
	Tests/test_vehicle_test_coverage.py

MODELICA_REGRESSION_TESTS := \
	Tests/test_modelica_regression.py \
	Tests/test_boblibvehicleinterfaces_modelica.py

MODELICA_TRANSLATION_TESTS := \
	Tests/test_modelica_translation.py

MODELICA_INITIALIZATION_TESTS := \
	Tests/test_modelica_initialization.py

MODELICA_TESTS := \
	$(MODELICA_TRANSLATION_TESTS) \
	$(MODELICA_INITIALIZATION_TESTS) \
	$(MODELICA_REGRESSION_TESTS)

ALL_TESTS := \
	$(PYTHON_TESTS) \
	$(MODELICA_TESTS)

.PHONY: help lint test test-python test-pytest test-modelica modelica-deps modelica-translation modelica-initialization modelica-regression ci check

help:
	@printf '%s\n' \
		'BobLib development targets:' \
		'  make lint                 Run Python lint checks' \
		'  make test                 Run the full local pytest gate' \
		'  make test-python          Run pure Python pytest checks only' \
		'  make test-pytest          Run every pytest-collected check in one invocation' \
		'  make modelica-deps        Install OpenModelica library dependencies' \
		'  make modelica-translation Translate standards and all BobLib.Tests fixtures via pytest' \
		'  make modelica-initialization Initialize all BobLib.Tests fixtures via pytest and compare baselines' \
		'      MODELICA_INIT_TIMEOUT_S=600 controls the per-model OMC timeout' \
		'  make modelica-regression  Simulate signal-level regressions and smoke-check BobLibVehicleInterfaces' \
		'  make test-modelica        Run all Modelica checks' \
		'  make ci                   Run the full local CI suite'

lint:
	$(PYTHON) -m ruff check Tests

test: test-python test-modelica

test-python:
	$(PYTEST) $(PYTEST_FLAGS) $(PYTHON_TESTS)

modelica-deps:
	omc msl_setup.mos

test-pytest: modelica-deps
	$(PYTEST) $(PYTEST_FLAGS) $(ALL_TESTS) --boblib-package-root $(PACKAGE_ROOT) --boblib-initialization-timeout-s $(MODELICA_INIT_TIMEOUT_S)

modelica-translation: modelica-deps
	$(PYTEST) $(PYTEST_FLAGS) $(MODELICA_TRANSLATION_TESTS) --boblib-package-root $(PACKAGE_ROOT)

modelica-initialization: modelica-deps
	$(PYTEST) $(PYTEST_FLAGS) $(MODELICA_INITIALIZATION_TESTS) --boblib-package-root $(PACKAGE_ROOT) --boblib-initialization-timeout-s $(MODELICA_INIT_TIMEOUT_S)

modelica-regression: modelica-deps
	$(PYTEST) $(PYTEST_FLAGS) $(MODELICA_REGRESSION_TESTS)

test-modelica: modelica-translation modelica-initialization modelica-regression

ci: lint test

check: ci
