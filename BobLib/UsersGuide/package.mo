within BobLib;
package UsersGuide

  "User guide and modeling tutorial for BobLib"
  extends Modelica.Icons.Information;

  annotation(
    preferredView = "info",
    Documentation(info = "<html>
<h4>BobLib Users Guide</h4>
<p>
This package is the in-library tutorial for using BobLib as an FSAE-style
vehicle modeling library. It explains how to run the included standard
templates, how the VehicleInterfaces assembly layer is wired, and how to build
new models that remain compatible with the surrounding architecture.
</p>
<p>
Start with <a href='modelica://BobLib.UsersGuide.Overview'>Overview</a>, then
choose the page that matches the work you are doing:
</p>
<ul>
<li><a href='modelica://BobLib.UsersGuide.RunningExistingTemplates'>Running Existing Templates</a> for the standard vehicle, FMI, and four-post entry points.</li>
<li><a href='modelica://BobLib.UsersGuide.ModifyingVehicleConstruction'>Modifying Vehicle Construction</a> for replacing a chassis, powertrain, VCU, brake system, or complete architecture.</li>
<li><a href='modelica://BobLib.UsersGuide.BusArchitecture'>Bus Architecture</a> for the publish/subscribe signal namespace used across subsystems.</li>
<li><a href='modelica://BobLib.UsersGuide.ModifyingLumpedModels'>Modifying Lumped Models</a> for diff, motor, inverter, aero, atmosphere, brakes, and VCU models.</li>
<li><a href='modelica://BobLib.UsersGuide.ModifyingCorePhysics'>Modifying Core Physics</a> for suspension, tires, contact mechanics, MultiBody state selection, and initialization.</li>
<li><a href='modelica://BobLib.UsersGuide.ValidationAndTesting'>Validation and Testing</a> for the local pytest gates and physical validation expectations.</li>
</ul>
<p>
BobLib is the Modelica model-development layer. Use BobSim for repeatable study
execution, signal extraction, metrics, plots, reports, envelope maps, and
sensitivity studies. Use BobLib directly when developing, replacing, or
inspecting subsystem models.
</p>
</html>"));
end UsersGuide;
