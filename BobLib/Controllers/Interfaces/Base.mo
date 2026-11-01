within BobLib.Controllers.Interfaces;
partial model Base "Basic interface definition for a controller"

  // Adapted from VehicleInterfaces 2.0.2: typed BobLib control bus.
  // Copyright (C) 2005-2013 Dassault Systemes, DLR and Modelon;
  // (C) 2013-2025 Modelica Association. Modelica License 1.1:
  // Resources/Licenses/LICENSE_VehicleInterfaces.txt.

  BobLib.Utilities.Interfaces.ControlBus controlBus "Control signal bus"
    annotation(Placement(transformation(extent = {{-20, -80}, {20, -120}})));
  annotation(
    Documentation(info = "<html>
<p>
This partial model defines the interfaces required for a&nbsp;controller
subsystem. This class should be extended to form a&nbsp;particular
controller model. See the
<a href=\"modelica://VehicleInterfaces.Controllers\">documentation</a> and
<a href=\"modelica://VehicleInterfaces.Controllers.Tutorial\">tutorial</a>
for more information.
</p>
</html>"));
end Base;
