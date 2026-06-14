within ;
package BobLib

  annotation(
    uses(Modelica(version = "4.1.0")),
    Documentation(info = "<html>
<p>
<code>BobLib</code> is the low-level Modelica vehicle modeling layer for
BobDyn. It contains the physical vehicle models, records, standard simulation
entry points, utilities, and regression fixtures used to develop detailed
vehicle dynamics components.
</p>
<p>
This package is active engineering infrastructure rather than a finished
general-purpose vehicle standard library. It is intended for users who can run
OpenModelica translation and regression checks, manage vehicle records
carefully, and validate model outputs against measured vehicle data.
</p>
<p>
Use BobSim for full workflow execution, signal extraction, metrics, plots,
reports, envelope maps, and sensitivity studies. Use BobLib directly when
developing or inspecting the underlying Modelica models.
</p>
<p>
Current local and CI checks target Modelica Standard Library 4.1.0. The
forward-looking <code>BobLibVehicleInterfaces</code> package additionally uses
VehicleInterfaces 2.0.2 and keeps its tests in
<code>BobLibVehicleInterfacesTests</code>.
</p>
</html>"));
end BobLib;
