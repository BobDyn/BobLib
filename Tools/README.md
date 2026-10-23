# Reusable FMUs

Build a fixed vehicle architecture once, then set its independent calibration
before each new run. This includes suspension hardpoints, masses and inertias,
spring/damper curves, tire coefficients, aerodynamic maps, and electrical and
driveline calibration. Architecture selections and array capacities are fixed
at compilation. This is run-to-run calibration, not moving hardpoints during a run.

Requires OpenModelica 1.27.1, Modelica 4.1.0, VehicleInterfaces 2.0.2, Clang and
CMake. Install the Python test dependencies from `Tests/requirements-dev.txt`.
Binaries target the build host; build separately for each operating system and
CPU architecture.

```sh
python3 Tools/export_fmu.py --front stabar --rear stabar --rear-brakes Inboard --output build/fmu --jobs 4
```

Use `--all` for all nine suspension pairs. `--front-brakes`, `--rear-brakes`,
`--differential` and `--rotor-effects` select additional compiled variants.
The default brake selection is outboard at both axles; the example explicitly
selects rear inboard brakes. `Tools/architectures.json` defines the architecture
and table-capacity contract. `--driver` exports the existing maneuver driver.
No VehicleSimSS or GGV analysis is included.

Each artifact directory contains its Modelica wrapper, build logs,
`build-timings.json` and `manifest.json`. FMUs also embed provenance in
`resources/boblib-build.json`. Source and artifact hashes identify the exact
build, including uncommitted source changes; a source change during a build
fails the export.

## Calibration contract

Only `manifest.json`'s `parameters` entries are editable. Set these before FMI
initialization and create a fresh instance for each configuration. `derived`
entries are read-only diagnostics: combined mass/inertia, geometry aliases and
derived damping must recalculate from their independent inputs. `structural`
entries describe fixed choices, not setters. Export fails when a numeric
calibration is silently frozen, a known dependency becomes editable, or a
VehicleDefn field disappears from compiler metadata.

Spring and damper tables have 64 rows, with configurable active point counts.
Aero maps have 16 points per axis. Only the active rows/cells participate;
counts and ordered breakpoints are validated by the plant. Values in unused
capacity have no effect. Array dimensions cannot change without rebuilding.

## Runtime compatibility and validation

The exporter applies guarded workarounds to the generated OpenModelica runtime
copies for metadata compilation, tiny integration intervals, input changes,
homotopy initialization, FMI tolerance and state-event localization. It does
not modify the installed compiler. Unknown compiler source patterns fail the
build. Review these workarounds when upgrading OpenModelica.

```sh
BOBLIB_TEST_FMU_RUNTIME=1 python3 -m pytest Tests/test_fmu_events.py Tests/test_fmu_homotopy.py Tests/test_fmu_tolerance.py
```

FMI 2.0 Co-Simulation is the tested execution mode. Combined ME/CS export also
contains a Model Exchange interface, which is not certified here. Passing
metadata checks does not establish physical validity of arbitrary calibrations
or acceptance of all architecture combinations.

