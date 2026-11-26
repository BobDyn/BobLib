# Development and validation

Commands run from the repository root.

## Local Health Checks

Run the full local gate before trusting a branch:

```sh
make ci
```

GitHub CI intentionally uses a lighter default gate for pushes and pull
requests: Python lint/tests, Modelica formatting, smoke translation,
initialization baselines, and mirrored fixture simulations/profiles.
Version tags use that same gate and also start the separate `Release Gate`
workflow. The full OpenModelica gate is a release activity: run it locally with
`make test`/`make ci`, or on a runner through that workflow (Actions -> Release
Gate -> Run workflow).

The release workflow separates lint, formatting, coverage, translation,
initialization, physics, signal regressions, and component/contract tests into
named steps and retains their individual test reports. Inspect skipped tests:
a passing default gate alone does not certify an FMU distribution or provide
rendered diagram sign-off. Use [export tools](../Tools/README.md) to build reusable native binaries and FMUs
with editable independent calibration. SS/GGV analysis remains excluded.
The FMU export/test support already merged in #30 is retained: use
`make modelica-fmu-export` to export VehicleFMI and `make modelica-fmu`
to run its checks. See Tests/modelica_fmu_export.py for runtime limitations.

Python tooling versions are pinned in `Tests/requirements-dev.txt` and the ruff
rule selection is pinned in `ruff.toml`, so a new ruff or pytest release cannot
turn CI red without a deliberate bump.

For narrower checks:

```sh
make test-python
make modelica-lint
make modelica-smoke
make test-pytest
make modelica-translation
make modelica-initialization
make modelica-physics
make modelica-regression
make modelica-fmu
```

The CI workflow runs on pull requests and on pushes to `main` and version tags.
Feature branches are validated through their pull request; open it as a draft if
you want signal before the branch is ready for review. Running on both events
for the same branch produced two runs per commit writing to the same commit SHA,
which made required status checks resolve unpredictably.

Modelica translation, initialization, and signal regressions are collected as
individual pytest cases. A failing model should appear directly in the pytest
node id, for example `Tests/test_modelica_translation.py::test_modelica_model_translates[BobLib.Experiments.Standards.VehicleSim]`.
Physical validation baselines also run through pytest and compare small
simulation outputs against checked-in signal baselines. Runtime budgets are
checked with a hardware-tolerant scale factor; override it with
`BOBLIB_RUNTIME_SCALE` or `--boblib-runtime-scale` for strict benchmarking.

## Adoption Guidance

For an FSAE or student engineering team, start by treating BobLib as a reference
and BobSim as the workflow entry point. A practical adoption path is:

1. Run the included baseline without modifying records.
2. Replace a small parameter set first: mass, CG, wheelbase, track, tire radius,
   simple aero, and basic powertrain limits.
3. Compare against simple known cases before trusting lap-time, damper,
   powertrain, tire, or aero conclusions.
4. Validate against your measured corner weights, inertias, suspension
   kinematics, damper curves, tire data, aero maps, drivetrain limits,
   controller behavior, and track-test signals.
5. Keep `make ci` passing as you adapt the model.

BobLib can support advanced vehicle dynamics and controls teams, but deep
customization assumes a modeling lead who is comfortable with Modelica package
debugging, OpenModelica diagnostics, and Python regression tooling.
