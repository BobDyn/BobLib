within BobLib;
package Engines

  "Engine models extending VehicleInterfaces engine contracts"
  extends Modelica.Icons.Package;

  annotation(Documentation(info = "<html>
<p>
Package <code>BobLib.Engines</code> contains engine subsystem
models exposed through the standard VehicleInterfaces engine boundary.
</p>
<p>
Direct models in this package are intended as vehicle-level insertion points.
Detailed internal-combustion engine physics should live one package level
deeper, then be adapted here through
<code>VehicleInterfaces.Engines.Interfaces.Base</code>.
</p>
<p>
<code>SimpleICEngine</code> is a small sample implementation for architecture
work. Inactive or omitted engine behavior should be represented by choosing an
architecture without an engine subsystem rather than by carrying a no-physics
adapter in the public package surface.
</p>
</html>"));
end Engines;
