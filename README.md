# BobLib

**Note:** this repo should not be regarded as a standard library until it implements VehicleInterfaces and the latest version of the Modelica Standard Library. Current interfaces constitute a one-off layer targeted at BobDyn consumption.

This library plans to move forward with accepted open-source standards, making BobLib interchangeable with existing Modelica infrastructure.

---

**BobLib** is a physically grounded, open-source Modelica library for vehicle dynamics simulation.
It provides modular multibody vehicle models, tire models (MF5.2), and standardized test procedures for analyzing vehicle behavior.

This library is designed to support high-fidelity simulation workflows and serve as the core modeling layer for the broader **BobDyn** ecosystem.

---

## Overview

BobLib focuses on:

* First-principles multibody vehicle modeling
* Modular subsystem design (chassis, suspension, tire, powertrain)
* Standardized test procedures (ISO, K&C)
* Clean parameterization via records
* Compatibility with OpenModelica workflows

> ⚠️ **Current Status:**
> The **chassis subsystem** is the most mature component.
> Vehicle tests (SteadyStateEval, K&C) are functional. Other areas are under active development.

---

## Installation

Clone this repository into your OpenModelica libraries directory:

```bash
git clone https://github.com/BobDyn/BobLib.git ~/.openmodelica/libraries/BobLib
```

Alternatively, add the repo path manually in OMEdit:

* **Tools → Options → Libraries → Add Path**

---

## Getting Started

### Load the library

In OMEdit:

1. Open OMEdit
2. Load `BobLib`
3. Browse the package tree

---

### Run a test simulation

Good entry points:

* `Standards.SteadyStateEval`
* `Standards.FrKnC`
* `Standards.RrKnC`

Or explore:

```
Tests.TestVehicle.*
```

---

## Repository Structure

```
BobLib/
├── Vehicle/        # Physical system models (core library)
├── Resources/      # Parameter records and definitions
├── Standards/      # ISO and K&C test procedures
├── Utilities/      # Math and multibody utilities
├── Tests/          # Validation and development tests
```

---

### 🔧 Vehicle

Core physical models:

* `Chassis`

  * Double wishbone suspension
  * Linkages (bellcrank, rod, damper, spring)
  * Tire models (MF5.2 with combined slip)
* `Powertrain`
* `Electronics`
* `Aero` (placeholder)

This is where full vehicle assemblies are built.

---

### 📦 Resources

Parameter definitions and records:

* Vehicle definitions (`VehicleDefn`)
* Subsystem records (suspension, tire, etc.)
* MF5.2 parameter sets

These enable clean separation between:

* **structure (Vehicle/)**
* **data (Resources/)**

---

### 📏 Standards

Implements standardized vehicle tests:

* `SteadyStateEval` (steady-state cornering)
* `KnC` (Kinematics & Compliance)

Includes:

* `.mo` models (test definitions)
* `.mos` scripts (build/run automation)

---

### 🧪 Tests

Validation and development testing:

* Unit-style tests for subsystems
* Full vehicle test configurations
* Tire validation (MF5.2)

Examples:

```
Tests.TestVehicle.TestSuspension.*
Tests.TestVehicle.TestPowertrain.*
```

---

### 🧰 Utilities

Reusable components:

* Math (vector, tensor ops)
* Multibody utilities (constraints, actuators)
* FMI utilities (WIP)

---

## Modeling Philosophy

BobLib follows a strict design philosophy:

* **Physics-first**
  No black-box models unless explicitly encapsulated (e.g., MF5.2)

* **Modularity**
  Subsystems are composable and replaceable

* **Separation of concerns**

  * Structure → `Vehicle/`
  * Parameters → `Resources/`
  * Tests → `Standards/`, `Tests/`

* **DAE-consistent modeling**
  Designed for robust simulation with DAE solvers (IDA/DASSL)

---

## Known Limitations

* Chassis is the only fully mature subsystem
* Aero is not yet implemented
* Powertrain and electronics are early-stage
* Some tests are exploratory rather than production-ready

---

## Roadmap

* Expand suspension architectures (bellcrank variants, push/pullrod configs)
* Improve solver robustness and initialization
* Add transient tire dynamics (relaxation length)
* Integrate with BobSim for DOE workflows
* Improve FMU parameter exposure

---

## Contributing

Contributions are welcome.
Focus areas:

* Model robustness
* Additional templates
* Test coverage
* Documentation

---

## License

This project is licensed under the GNU General Public License v3.0 (GPLv3).

You are free to use, modify, and distribute this software, including for commercial purposes, provided that any distributed modifications or derivative works are also licensed under GPLv3 and include the corresponding source code.

See the [LICENSE](LICENSE) file for full details.

---

## Related Projects

* **BobSim** – simulation orchestration and DOE workflows
* **BobDocs** – documentation site (https://bobdyn.com)

---

## Author

Developed as part of the **BobDyn** project.
