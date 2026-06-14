PYTHON ?= python3
PACKAGE_ROOT ?= BobLib
MODELICA_INIT_TIMEOUT_S ?= 600

PYTHON_TESTS := \
	Tests/test_vehicle_test_coverage.py

MODELICA_REGRESSION_TESTS := \
	Tests/test_modelica_regression.py \
	Tests/test_boblibvehicleinterfaces_modelica.py

.PHONY: help lint test test-python test-modelica modelica-deps modelica-translation modelica-initialization modelica-regression ci check

help:
	@printf '%s\n' \
		'BobLib development targets:' \
		'  make lint                 Run Python lint checks' \
		'  make test                 Run Python tests and all Modelica regression checks' \
		'  make test-python          Run Python unit/coverage tests only' \
		'  make modelica-deps        Install OpenModelica library dependencies' \
		'  make modelica-translation Translate standards and all BobLib.Tests fixtures' \
		'  make modelica-initialization Initialize all BobLib.Tests fixtures and compare baselines' \
		'      MODELICA_INIT_TIMEOUT_S=600 controls the per-model OMC timeout' \
		'  make modelica-regression  Simulate signal-level regressions and smoke-check BobLibVehicleInterfaces' \
		'  make test-modelica        Run all Modelica checks' \
		'  make ci                   Run the full local CI suite'

lint:
	$(PYTHON) -m ruff check Tests

test: test-python test-modelica

test-python:
	$(PYTHON) -m pytest $(PYTHON_TESTS)

modelica-deps:
	omc msl_setup.mos

modelica-translation: modelica-deps
	$(PYTHON) Tests/modelica_translation_checks.py --package-root $(PACKAGE_ROOT)

modelica-initialization: modelica-deps
	$(PYTHON) Tests/modelica_initialization_checks.py --package-root $(PACKAGE_ROOT) --timeout-s $(MODELICA_INIT_TIMEOUT_S)

modelica-regression: modelica-deps
	$(PYTHON) -m pytest $(MODELICA_REGRESSION_TESTS)

test-modelica: modelica-translation modelica-initialization modelica-regression

ci: lint test

check: ci
