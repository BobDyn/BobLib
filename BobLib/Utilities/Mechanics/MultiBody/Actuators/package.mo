within BobLib.Utilities.Mechanics.MultiBody;
package Actuators

  "Reusable MultiBody actuator and fixture helpers"
  extends BobLib.Icons.BobLibPackageBackground;

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLib.Utilities.Mechanics.MultiBody.Actuators</code>
contains reusable frame-based actuator and fixture models.
</p>
<p>
These models are not owned by a specific chassis, suspension, or tire
implementation. They provide rig-level plumbing for experiments and tests that
need to impose motion or loads through MultiBody frames.
</p>
</html>"));
end Actuators;
