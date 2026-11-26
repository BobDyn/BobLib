# BobLib

Modelica multibody vehicle models for BobDyn. BobLib provides the physical
components, calibration records, and vehicle and suspension test rigs.
[BobSim](https://github.com/BobDyn/BobSim) provides study orchestration,
configuration, analysis, and reporting.

[Documentation](https://bobdyn.com/boblib) · [Development](docs/development.md) ·
[Migration guide](docs/migration.md) · [Release status](docs/release-readiness.md)

## Get started

Install OpenModelica, then install the Modelica dependencies:

```sh
make modelica-deps
```

Open `BobLib/package.mo` in your Modelica environment. Start with
`BobLib.UsersGuide`, the authoritative guide for this package version.
Current checks target Modelica Standard Library **4.1.0**, VehicleInterfaces
**2.0.2**, and OpenModelica **1.27.1** in CI.

## Entry points

| Model | Purpose |
|---|---|
| `BobLib.Experiments.Standards.VehicleSim` | Full-vehicle maneuver simulations |
| `BobLib.Experiments.Standards.VehicleFMI` | Driver-input FMI boundary |
| `BobLib.Experiments.Standards.FourPostSim` | Four-post suspension studies |
| `BobLib.Experiments.Standards.KnCSteerable` / `KnCLocked` | Axle kinematics and compliance |

See [export tools](Tools/README.md) for reusable native binaries, FMUs, and cross-runtime
verification. The ownership and layout for native runtime adapters are documented
in [Tools/native/](Tools/native/README.md).

## Repository map

| Directory | Purpose |
|---|---|
| [BobLib/](BobLib/) | Modelica package and its UsersGuide |
| [Tools/](Tools/README.md) | Export/build entry points and native wrappers |
| [Tests/](Tests/) | BobLibTest fixtures, regression tests, and benchmarks |
| [docs/](docs/README.md) | Development, migration, and release notes |
| [development/](development/) | Compiler reproductions and visual design drafts |

Generated binaries and simulation outputs belong under the ignored `build/`
directory or an external output directory. Run `make help` for development
commands and [validation guidance](docs/development.md) before changing physics.

BobLib is active engineering infrastructure. Results need vehicle-specific
validation; experimental envelope benchmarks do not establish a production
workflow or measured vehicle fidelity.

Package version **0.2.0**. See [CHANGELOG.md](CHANGELOG.md) for release notes.
Licensed under [GPL-3.0](LICENSE); dependencies and attribution are recorded in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
