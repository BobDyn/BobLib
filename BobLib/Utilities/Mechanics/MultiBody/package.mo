within BobLib.Utilities.Mechanics;
package MultiBody

  "Reusable MultiBody helpers shared across vehicle domains"
  extends BobLib.Icons.BobLibPackageBackground;

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLib.Utilities.Mechanics.MultiBody</code>
contains reusable MultiBody helpers that are not owned by one vehicle subsystem.
</p>
<p>
Use this package for frame, fixture, actuator, and contact utilities that are
consumed by chassis-level assemblies or by more than one physics domain. Domain
models should expose raw frames and use these utilities at the assembly level
rather than burying shared MultiBody behavior inside tire, suspension, or
powertrain subpackages.
</p>
</html>"));
end MultiBody;
