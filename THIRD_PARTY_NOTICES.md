# Third-Party Notices

BobLib declares dependencies on Modelica Standard Library 4.1.0 and
VehicleInterfaces 2.0.2 through the Modelica package `uses(...)` annotation.
Those libraries are not vendored in this repository.

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

## Notes for Release Bundles

- BobLib source releases that only reference these dependencies through
  `uses(...)` should keep this notices file.
- Release artifacts that redistribute Modelica Standard Library or
  VehicleInterfaces should also redistribute the full license files from those
  packages.
- BobLib's own license remains GPL-3.0; see `LICENSE`.
