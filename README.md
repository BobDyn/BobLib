# BobLib

BobLib is the low-level Modelica vehicle modeling layer for BobDyn. It contains
the physical vehicle models, records, standard simulation entry points, and
regression fixtures used to develop detailed vehicle dynamics components.

The full workflow layer is BobSim. Use BobSim when you want repeatable study
execution, signal extraction, metrics, plots, reports, envelope maps, and
sensitivity sweeps. Use BobLib when you are developing or inspecting the
underlying Modelica models.

The authoritative documentation for this package version lives inside the
Modelica package at:

`BobLib.UsersGuide`

The UsersGuide is part of the versioned, tested package and should be treated
as the source of truth for current BobLib architecture, usage, extension, and
validation guidance. BobDocs may mirror or expand this material for web
reading, but when version-specific details differ, the in-package UsersGuide
wins.

The web documentation lives at:

https://bobdyn.com/boblib

## Release

Current package version: `0.1.0`, dated `2026-06-21`.

Release notes are maintained in `CHANGELOG.md` and in the `BobLib` package
revision annotation. The in-library tutorial lives at
`BobLib.UsersGuide`.

## License and Third-Party Notices

BobLib is licensed under GPL-3.0; see `LICENSE`.

BobLib depends on Modelica Standard Library 4.1.0 and VehicleInterfaces 2.0.2
through the package `uses(...)` annotation. The dependency notices are recorded
in `THIRD_PARTY_NOTICES.md`. If a release bundle vendors either dependency, the
corresponding full dependency license text should be distributed with that
bundle as well.

## Status

BobLib is active engineering infrastructure, not a finished general-purpose
vehicle standard library. It is suitable for Modelica-capable users who can run
OpenModelica checks, manage vehicle records carefully, and validate model
outputs against measured data.

The repository is useful as:

- a reference implementation for detailed FSAE-style vehicle modeling
- a development layer for BobDyn vehicle components
- a regression-tested Modelica package for advanced simulation teams

It is not yet:

- a plug-and-play workflow for a new team to model its car in one afternoon
- a substitute for vehicle-specific validation
- a safety-certified design or controls tool

## Supported Stack

Current checks target:

- Modelica Standard Library 4.1.0
- VehicleInterfaces 2.0.2 for `BobLib`
- OpenModelica via the CI container `openmodelica/openmodelica:v1.26.3-ompython`
- Python with `pytest` and `ruff`

Install Modelica dependencies with:

```sh
make modelica-deps
```

## Primary Entrypoints

- `BobLib.Experiments.Standards.VehicleFMI`: driver-input FMI/DIL/SIL vehicle boundary
- `BobLib.Experiments.Standards.VehicleSim`: full-vehicle maneuver benchmark
- `BobLib.Experiments.Standards.FourPostSim`: four-post benchmark

`BobLib` is the forward-looking standalone package. It builds
BobLib physics inside the public subsystem contracts from VehicleInterfaces
while keeping regression tests in the `BobLibTest` Modelica package under
the repository `Tests/` directory.

## Local Health Checks

Run the full local gate before trusting a branch:

```sh
make ci
```

GitHub CI intentionally uses a lighter default gate for pushes and pull
requests: Python lint/tests, Modelica formatting, and `make modelica-smoke`.
Version tags use the same lightweight gate so releases are not blocked by a
GitHub runner doing a full rebuild. The full OpenModelica gate is the local
release gate (`make test`/`make ci`) and can also be attempted manually through
workflow dispatch with the full Modelica option enabled.

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
```

The CI workflow runs on pull requests and on pushes to every branch so large
integration branches receive the same basic validation as `main`.

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
