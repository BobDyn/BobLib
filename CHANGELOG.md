# Changelog

## 0.1.0 - 2026-06-21

Initial standalone BobLib release aligned with Modelica Standard Library 4.1.0
and VehicleInterfaces 2.0.2.

### Added

- Standalone `BobLib` package with public subsystem domains organized around
  VehicleInterfaces contracts.
- Standard runnable entry points:
  `BobLib.Experiments.Standards.VehicleSim`,
  `BobLib.Experiments.Standards.VehicleFMI`, and
  `BobLib.Experiments.Standards.FourPostSim`.
- Vehicle, FMI, and four-post template families for FSAE-style EV vehicle and
  double-wishbone suspension architecture comparisons.
- Publish/subscribe bus routing for driver intent, chassis measurements,
  battery and motor measurements, electric-drive commands, driveline commands,
  brake commands, and atmosphere-owned ambient signals.
- Standard EV stack with battery, inverter, motor, fixed-ratio transmission,
  rear final-drive differential, standard VCU, VCU-commanded mechanical brakes,
  CFD aero map, constant atmosphere, and detailed double-wishbone
  suspension/tire/contact models.
- `BobLib.UsersGuide` tutorial package covering template execution, vehicle
  construction, bus usage, lumped model authoring, core physics changes, and
  validation workflow.
- README guidance that makes `BobLib.UsersGuide` the authoritative
  version-specific documentation source, with BobDocs serving as a web mirror
  or extension.
- Third-party dependency notices for Modelica Standard Library 4.1.0 and
  VehicleInterfaces 2.0.2.
- Sibling `Tests/BobLibTest` Modelica test library and pytest-based checks for
  formatting, translation, initialization, physics validation, runtime, and
  regression behavior.

### Validation

- Release gate is `make test`, which runs Modelica formatting tests,
  translation checks, initialization baselines, physical validation baselines,
  regression simulations, and focused structural checks.
- The included vehicle and component models are regression-tested engineering
  baselines. They are not validated replicas of any specific vehicle.

### Known Limits

- Vehicle-specific records still require team measurement and validation before
  design decisions.
- BobSim remains the workflow layer for repeatable studies, plots, reports,
  envelope maps, and sensitivity sweeps.
