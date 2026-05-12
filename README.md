# BobLib

BobLib is a Modelica-based vehicle dynamics library for building modular multibody vehicle models, suspension assemblies, tire models, and standardized simulation workflows.

The codebase is organized as a reusable package tree with separate areas for vehicle structure, parameter records, utilities, standardized test models, and validation cases.

> Note: this repository should not be treated as a standard library until it implements [VehicleInterfaces](https://github.com/modelica/VehicleInterfaces/) and the latest version of the [Modelica Standard Library](https://github.com/modelica/ModelicaStandardLibrary). The current interfaces are a one-off layer targeted at BobDyn consumption.

## Highlights

- Modular vehicle architecture for chassis, suspension, powertrain, electronics, and aero
- Record-based parameterization for clean separation between model structure and data
- Standardized simulation entry points for steady-state and kinematics-and-compliance workflows
- Tire modeling support based on MF5.2-style templates and slip models
- Utility packages for math and multibody helpers

## Requirements

BobLib declares compatibility with:

- `Modelica 3.2.3`

A Modelica toolchain with OpenModelica scripting support is recommended if you want to use the provided build scripts in `Standards/`.

## Repository Layout

```text
BobLib/
├── Vehicle/        # Physical vehicle models and subsystem assemblies
├── Resources/      # Parameter records and visual/vehicle definitions
├── Standards/      # Standard simulation models and build scripts
├── Utilities/      # Shared math and multibody helpers
├── Tests/          # Validation and development test models
└── package.mo      # Root package definition
```

## Main Packages

### `Vehicle`

Contains the physical modeling layer for complete vehicles and subsystems.

Notable areas include:

- `Vehicle.Chassis`
- `Vehicle.Chassis.Body`
- `Vehicle.Chassis.Suspension`
- `Vehicle.Chassis.Suspension.Linkages`
- `Vehicle.Chassis.Suspension.Templates`
- `Vehicle.Powertrain`
- `Vehicle.Powertrain.Battery`
- `Vehicle.Powertrain.Drivetrain`
- `Vehicle.Powertrain.Electronics`
- `Vehicle.Electronics`
- `Vehicle.Aero`

### `Resources`

Holds the parameter data and visualization records used by the models.

Notable groups include:

- `Resources.VehicleDefn`
- `Resources.VehicleRecord`
- `Resources.StandardRecord`
- `Resources.VisualRecord`

These records are used to configure different vehicle combinations and test setups without hard-coding parameters into the model definitions.

### `Standards`

Contains standardized simulation models and automation scripts.

Key models:

- `Standards.VehicleSim`
- `Standards.VehicleFMI`
- `Standards.StandardSim.SteadyStateEval`
- `Standards.StandardSim.TransientEval`
- `Standards.StandardSim.FrKnC`
- `Standards.StandardSim.RrKnC`
- `Standards.StandardSim.Templates.KnC`

Build scripts:

- `Standards/BuildVehicleModel.mos`
- `Standards/BuildVehicleFMI.mos`
- `Standards/StandardSim/BuildKnC.mos`

### `Utilities`

Shared helper functions and components used throughout the library.

Notable groups include:

- `Utilities.Math.Vector`
- `Utilities.Math.Tensor`
- `Utilities.Mechanics.Multibody`
- `Utilities.FMI`

### `Tests`

Validation and development tests for individual subsystems and full vehicle configurations.

Examples:

- `Tests.TestVehicle.TestChassis.TestSuspension.TestFrAxleDW`
- `Tests.TestVehicle.TestChassis.TestSuspension.TestRrAxleDW`
- `Tests.TestVehicle.TestPowertrain.TestPowertrain`
- `Tests.TestVehicle.TestPowertrain.TestBatteryPack`
- `Tests.TestUtilities.TestMechanics.TestMultibody.TestGroundPhysics`

## Getting Started

### 1. Clone the repository

```bash
git clone <repository-url> BobLib
```

### 2. Load the package in OMEdit

Add the repository root to your Modelica library search path, then load `BobLib` from OMEdit.

If you prefer to keep the library in the default OpenModelica library directory, place it under:

```bash
~/.openmodelica/libraries/BobLib
```

### 3. Explore the package tree

Start with one of these entry points:

- `BobLib.Standards.VehicleSim`
- `BobLib.Standards.VehicleFMI`
- `BobLib.Standards.StandardSim.SteadyStateEval`
- `BobLib.Standards.StandardSim.FrKnC`
- `BobLib.Standards.StandardSim.RrKnC`

## Building and Running

The repository includes OpenModelica script files for common workflows.

### Build a vehicle model

Run `Standards/BuildVehicleModel.mos` to build the configured vehicle simulation target.

### Build an FMU

Run `Standards/BuildVehicleFMI.mos` to export `BobLib.Standards.VehicleFMI` as an FMU.

### Kinematics and compliance workflow

Run `Standards/StandardSim/BuildKnC.mos` for the standard K&C build flow.

## Model Structure

BobLib is intentionally split into a few layers:

- `Vehicle/` defines the model structure
- `Resources/` provides the data for those models
- `Standards/` wraps models in repeatable simulation and export workflows
- `Tests/` captures regression and development cases
- `Utilities/` contains reusable math and mechanics helpers

This separation makes it easier to swap vehicle definitions, reuse subsystem templates, and keep parameter data independent from the physical models.

## Notes

- The library is still evolving, so some subsystems are more complete than others.
- The chassis and suspension areas are the most mature parts of the tree.
- The build scripts are written for OpenModelica-style command line execution.

## Contributing

Contributions are welcome. Useful areas of work include:

- Model robustness
- Additional suspension and powertrain templates
- Test coverage
- Documentation improvements

## License

BobLib is distributed under the GNU General Public License v3.0 (GPLv3). See the [LICENSE](LICENSE) file for details.
