within BobLibVehicleInterfaces.Experiments;
package Standards

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLibVehicleInterfaces.Experiments.Standards</code> contains standard runnable experiments.
</p>
<p>
The models here are the primary entry points for driver-input vehicles, full-vehicle maneuvers, and four-post simulations assembled from one-level public subsystem models.
</p>
<p>
Reusable experiment assemblies live under <code>Templates</code>. The
<code>Templates.FMI</code> package contains FMU/export-oriented vehicle bases,
starting with the driver-input <code>BaseVehicleFMI</code>. The
<code>Templates.Vehicle</code> package contains the EV
battery-inverter-motor-differential maneuver simulation base and its nine
suspension-configuration specializations. The <code>Templates.FourPost</code>
package contains the PTN-agnostic four-post fixture base and its nine

suspension-configuration specializations.
</p>
</html>"));
end Standards;
