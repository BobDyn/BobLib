# Agent Notes

These rules capture the current BobLib/BobLibVehicleInterfaces architecture so
future agents preserve the package boundaries.

## BobLibVehicleInterfaces Architecture

- Keep the first level of each public domain package aligned with the
  `VehicleInterfaces` contract. First-level models are the shared adapters or
  concrete insertion points used by experiments.
- Put BobLib physics, implementation details, templates, and helper models one
  level deeper than the public contract layer. Do not add duplicate interfaces
  or extra connectors when an existing `VehicleInterfaces` interface can be
  used directly.
- Domain-local implementation models should live in that domain's nested
  packages, usually `Internal`, `Templates`, `Actuators`, or another
  purpose-specific package. Avoid a second public access path for the same
  interface or physics model.
- Tires live under `Chassis.Suspension` because suspension axles own wheel
  centers, tire load paths, and raw contact-patch frames.
- Suspension axle and tire models should expose and connect raw contact-patch
  frames. MultiBody contact mechanics that close those frames to ground or
  chassis-level fixtures should not live under `Tires`; place reusable
  MultiBody contact/fixture utilities under `Utilities.Mechanics.MultiBody`.
- If a MultiBody helper is not naturally owned and used by the package exactly
  one level above it, prefer `Utilities.Mechanics.MultiBody` over burying it
  inside a deeper physics package. This includes reusable frame fixtures,
  rig actuators, and contact mechanics.
- General mechanics calculations that operate on records, arrays, or scalar
  quantities, such as mass-property combination or inertia translation, belong
  under `Utilities.Mechanics.Functions`. Frame-based helpers belong under
  `Utilities.Mechanics.MultiBody`.
- First-level packages under `Records.VehicleRecord` should mirror the public
  vehicle subsystem packages. Complete records under `Records.VehicleDefn`
  aggregate domain-owned records such as `pBattery`, `pVCU`, `pInverter`,
  `pMotor`, and `pDriveline`; avoid reintroducing a monolithic powertrain
  record as a second access path.
- Keep tests in the sibling `BobLibVehicleInterfacesTests` library. Production
  packages should not contain regression or component test models.
- Use Modelica Standard Library 4.1.0 and VehicleInterfaces 2.0.2 for this
  comparison library unless the user explicitly changes the target versions.
- The shared `bobdyn.png` logo should remain byte-identical across BobDyn
  repositories. Put theme/background treatment in Modelica annotations, not by
  editing the PNG asset.
