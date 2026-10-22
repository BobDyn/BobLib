# Third-Party Notices

BobLib declares dependencies on Modelica Standard Library 4.1.0 and
VehicleInterfaces 2.0.2 through the Modelica package `uses(...)` annotation.
The complete libraries are not vendored. Selected icon graphics are adapted in
BobLib package annotations; their license texts are included under
`BobLib/Resources/Licenses`.

If a binary, source archive, FMU-support bundle, container, or other release
artifact includes copies of these dependencies, include the corresponding full
dependency license text in that artifact.

## Modelica Standard Library 4.1.0

License: BSD 3-Clause License

Copyright (c) 1998-2025, Modelica Association and contributors.

The installed OpenModelica package carries the license text at:

`Modelica 4.1.0+maint.om/Resources/Licenses/LICENSE_ModelicaStandardLibrary.txt`

## VehicleInterfaces 2.0.2

License: Modelica License, Version 1.1

Copyright (C) 2005-2013, Dassault Systemes, DLR and Modelon.
Copyright (C) 2013-2025, Modelica Association.

The installed OpenModelica package carries the license text at:

`VehicleInterfaces 2.0.2/Resources/Licenses/LICENSE_VehicleInterfaces.txt`

## Adapted package graphics

On 2026-10-16, BobLib adapted VehicleInterfaces 2.0.2 icons for Atmospheres,
Brakes, Chassis, Controllers, Drivelines, DriverEnvironments, ElectricDrives,
EnergyStorage, Engines, and Transmissions. Instance-name text was removed and
the graphics were placed in package annotations. Each source package identifies
the original icon and copyright. The included license is
`BobLib/Resources/Licenses/LICENSE_VehicleInterfaces.txt`.

The MultiBody and Joints utility packages adapt MSL 4.1.0 package graphics.
Their original copyright is retained in source; the included license is
`BobLib/Resources/Licenses/LICENSE_ModelicaStandardLibrary.txt`.

## Notes for Release Bundles

- BobLib source releases must retain this notices file, icon source notices,
  and the included license files for the adapted graphics.
- Release artifacts that redistribute Modelica Standard Library or
  VehicleInterfaces should also redistribute the full license files from those
  packages.
- BobLib's own license remains GPL-3.0; see `LICENSE`.
