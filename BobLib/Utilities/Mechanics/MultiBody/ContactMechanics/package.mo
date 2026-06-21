within BobLib.Utilities.Mechanics.MultiBody;
package ContactMechanics

  "MultiBody contact helpers for chassis-level contact-patch wiring"
  extends BobLib.Icons.BobLibPackageBackground;

  annotation(
    Documentation(info = "<html>
<p>
Package
<code>BobLib.Utilities.Mechanics.MultiBody.ContactMechanics</code>
contains reusable contact mechanics helpers for MultiBody contact-patch frames.
</p>
<p>
Suspension and tire models expose the raw contact-patch frames. Chassis-level
assemblies use this package to close those frames to ground or fixture models,
keeping contact mechanics out of the tire implementation package.
</p>
</html>"));
end ContactMechanics;
