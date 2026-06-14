# BobLib

BobLib is the low-level Modelica vehicle modeling layer for BobDyn. It contains
the physical vehicle models, records, standard simulation entry points, and
regression fixtures used to develop detailed vehicle dynamics components.

The full workflow layer is BobSim. Use BobSim when you want repeatable study
execution, signal extraction, metrics, plots, reports, envelope maps, and
sensitivity sweeps. Use BobLib when you are developing or inspecting the
underlying Modelica models.

The official documentation lives at:

https://bobdyn.com/boblib

Treat that site as the definitive source for installation, translation,
simulation, and OMEdit workflows.

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
- VehicleInterfaces 2.0.2 for `BobLibVehicleInterfaces`
- OpenModelica via the CI container `openmodelica/openmodelica:v1.26.3-ompython`
- Python with `pytest` and `ruff`

Install Modelica dependencies with:

```sh
make modelica-deps
```

## Primary Entrypoints

- `BobLib.Standards.VehicleSim`: legacy/current BobLib full-vehicle benchmark
- `BobLib.Standards.FourPostSim`: legacy/current four-post benchmark
- `BobLibVehicleInterfaces.Experiments.Standards.VehicleSim`: VehicleInterfaces-aligned full-vehicle benchmark
- `BobLibVehicleInterfaces.Experiments.Standards.FourPostSim`: VehicleInterfaces-aligned four-post benchmark

`BobLibVehicleInterfaces` is the forward-looking standalone package. It builds
BobLib physics inside the public subsystem contracts from VehicleInterfaces
while keeping regression tests in the sibling `BobLibVehicleInterfacesTests`
library.

## Local Health Checks

Run the full local gate before trusting a branch:

```sh
make ci
```

For narrower checks:

```sh
make test-python
make modelica-translation
make modelica-initialization
make modelica-regression
```

The CI workflow runs on pull requests and on pushes to every branch so large
integration branches receive the same basic validation as `main`.

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
