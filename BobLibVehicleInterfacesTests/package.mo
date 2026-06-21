within ;
package BobLibVehicleInterfacesTests

  "Tests for BobLibVehicleInterfaces"
  extends Modelica.Icons.ExamplesPackage;

  annotation(
    preferredView = "info",
    version = "0.1.0",
    uses(
      Modelica(version = "4.1.0"),
      VehicleInterfaces(version = "2.0.2"),
      BobLibVehicleInterfaces(version = "0.1.0")),
    Documentation(info = "<html>
<p>
Standalone regression and component tests for
<code>BobLibVehicleInterfaces</code>. Keeping this as a sibling library keeps
the production package tree focused on VehicleInterfaces-style subsystem
content.
</p>
</html>"));
end BobLibVehicleInterfacesTests;
