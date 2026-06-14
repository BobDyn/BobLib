within BobLibVehicleInterfaces.Experiments.Standards;
package Architectures
  "Reusable vehicle architecture assemblies for standard experiments"
  extends Modelica.Icons.VariantsPackage;

  annotation(Documentation(info = "<html>
<p>
Package <code>BobLibVehicleInterfaces.Experiments.Standards.Architectures</code>
contains complete subsystem redeclare stacks for standard vehicle architectures.
</p>
<p>
The front-facing experiment models, such as
<code>Experiments.Standards.VehicleSim</code>, extend one of these architecture
models and add benchmark-specific experiment annotations. This keeps the
simulation entrypoint stable while giving battery-electric, conventional,
hybrid, and future architectures a dedicated place to live.
</p>
</html>"));
end Architectures;
