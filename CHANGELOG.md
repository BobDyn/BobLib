# Changelog

## 0.2.0 - 2026-08-09

Minor release focused on physically consistent vehicle initialization, correct
left/right and reverse-operation behavior, and stronger pull-request validation.

### Added

- Added a vehicle-owned quasi-steady-state initialization record populated from
  gravity-enabled four-post suspension solutions. The standard `VehicleSim`
  entry point now starts from the coupled chassis pose and four lower-arm
  coordinates, and exposes all four suspension spring lengths.
- Added complete symmetric-axle mass-property combination so reported vehicle
  mass, center of gravity, and inertia include both axle sides.
- Added focused Modelica regressions for rotated contact frames, motor direction,
  tire force frames, wheel mirroring and units, aerodynamic force direction,
  reverse transient slip, and signed vector angles.

### Fixed

- Resolved ground-contact connector forces in each connector's local frame and
  tire shear loads in an orthonormal road frame.
- Preserved motor mechanical-power sign during reverse motoring and regeneration.
- Preserved mirrored right-wheel toe and inclination through concrete chassis
  redeclarations, and made the degree-valued alignment API explicit.
- Applied aerodynamic drag opposite the complete vehicle-relative airflow vector.
- Preserved reverse longitudinal-slip sign and aligned transient-slip initial
  states with the same low-speed regularization used by their dynamics.
- Made signed vector-angle magnitude invariant to input scale and reference-axis
  skew while defining neutral behavior for degenerate inputs.
- Removed the artificial suspension settling transient at the start of the
  standard vehicle simulation. The rebased reference run reduced initial total
  tire-load error from 136.178 N to 0.04396 N and first-two-second RMS error from
  52.535 N to 6.133 N.

### Validation

- Pull-request CI now executes all initialization baselines in addition to
  formatting and smoke translation, preventing stale flattened-model baselines
  from merging unnoticed.
- The full OpenModelica release gate passed for the `v0.2.0` release branch,
  covering translation, 34 initialization fixtures, physical baselines, and
  signal-level regressions.
- The standard ramp-steer `VehicleSim` initialized without homotopy, produced
  finite recorded outputs, retained positive tire loads, and terminated normally
  at the QSS plateau.

## 0.1.1 - 2026-07-05

Patch release for four-post solver robustness.

### Fixed

- Held the four-post heave input at zero through the roll phase so the standard
  four-post template keeps a consistent time horizon for combined heave and
  roll evaluation.

### Validation

- GitHub push CI and the manual full OpenModelica release gate passed during
  the 0.1.1 release check, including translation, initialization baselines,
  physical validation baselines, and signal-level regressions.

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
